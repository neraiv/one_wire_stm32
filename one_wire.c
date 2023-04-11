/*
 * one_wire.c
 *
 *  Created on: Jan 17, 2023
 *      Author: neraiv
 */


#include "one_wire.h"

uint16_t OW_PIN;
GPIO_TypeDef* OW_PORT;
TIM_HandleTypeDef htim;
uint8_t DISABLE_INTERRUPTS_FLAG = 1;

DeviceAddress ROM_NO;
uint8_t LastDiscrepancy;
uint8_t LastFamilyDiscrepancy;
uint8_t LastDeviceFlag;

//
// Sets a timer to be used in delayMicroseconds function and defines GPIO pin.Also, starts
// given timer if its not started yet.
//
void one_wire_Init_Timer(TIM_HandleTypeDef *htimx,uint8_t CLOCK_LINE,GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin){
  OW_PIN = GPIO_Pin;
  OW_PORT = GPIOx;
  htim = *htimx;

  if(CLOCK_LINE == 1){
	  htim.Init.Prescaler = HAL_RCC_GetPCLK1Freq()*2/1000000 -1;
  }
  else if(CLOCK_LINE == 2){
	  htim.Init.Prescaler = HAL_RCC_GetPCLK2Freq()*2/1000000 -1;
  }

  HAL_TIM_Base_Init(&htim); //Yeni prescaler değerini timer a kaydet

  if(htim.State != HAL_TIM_STATE_BUSY){ // Timer başlatılmamışsa, başlat
	  HAL_TIM_Base_Start(&htim);
  }
}

//
// Disables/enables other interrupts block interfere delayMicroseconds function.
// 1 -> disable (DEFAULT)
// 0 -> enable
//
void one_wire_disableInterrupts(uint8_t disable_interrupts_flag){
	DISABLE_INTERRUPTS_FLAG = disable_interrupts_flag;
}

//
// Implements delay in microseconds.
//
void delayMicroseconds(uint32_t time){

	if(DISABLE_INTERRUPTS_FLAG){         // Disable other interrupts RECOMENDED
		__disable_irq();

		htim.Instance->CNT = 0;
		while(htim.Instance->CNT < time);

		__enable_irq();
	}
	else{
		htim.Instance->CNT = 0;
		while(htim.Instance->CNT < time);
	}
}

//
// Set given GPIO Pin as OUTPUT
//
void Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

//
// Set given GPIO Pin as INPUT
//
void Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

//
// Implement 1-Wire reset.
// RETURN;
// 1, if the reset was successful
// 0, if the reset wasn't successful
//
uint8_t onewire_reset(void)
{
	uint8_t r;
	uint8_t test;
	uint8_t retries = 20;

	do{
	Set_Pin_Input(OW_PORT, OW_PIN); // Read the line. Line must be at HIGH value,cuz
	do {                            // PULL-UP resistor
		if (--retries == 0) return 0;
		delayMicroseconds(2);
	} while (!HAL_GPIO_ReadPin(OW_PORT, OW_PIN));

	// Implementation of One-Wire reset
	Set_Pin_Output(OW_PORT, OW_PIN);    // Set the LOW for 480us
	HAL_GPIO_WritePin(OW_PORT, OW_PIN, GPIO_PIN_RESET);
	delayMicroseconds(480);

	Set_Pin_Input(OW_PORT, OW_PIN);     // Wait for PULL-UP resistor to do its job
	delayMicroseconds(70);

	r = !HAL_GPIO_ReadPin(OW_PORT, OW_PIN);
	delayMicroseconds(410);             // Complete the reset cycle
	test++;
	} while(!r && test < retries);
	return r;
}

//
// Write 1 bit data
//
void onewire_write_bit(uint8_t v)
{
	if(v & 1){                    // Send 1
		Set_Pin_Output(OW_PORT, OW_PIN);
		HAL_GPIO_WritePin(OW_PORT, OW_PIN, GPIO_PIN_RESET);
		delayMicroseconds(10);
		HAL_GPIO_WritePin(OW_PORT, OW_PIN, GPIO_PIN_SET);
		delayMicroseconds(55);
	}
	else{                        // Send 0
		Set_Pin_Output(OW_PORT, OW_PIN);
		HAL_GPIO_WritePin(OW_PORT, OW_PIN, GPIO_PIN_RESET);
		delayMicroseconds(65);
		HAL_GPIO_WritePin(OW_PORT, OW_PIN, GPIO_PIN_SET);
		delayMicroseconds(5);
	}
}

//
// Read 1 bit data
//
uint8_t onewire_read_bit(void)
{
	uint8_t r;

	Set_Pin_Output(OW_PORT, OW_PIN);
	HAL_GPIO_WritePin(OW_PORT, OW_PIN, GPIO_PIN_RESET);
	delayMicroseconds(3);
	Set_Pin_Input(OW_PORT, OW_PIN);
	delayMicroseconds(10);
	r = HAL_GPIO_ReadPin(OW_PORT, OW_PIN);
	delayMicroseconds(53);
	return r;
}

//
// Write 1 byte data
//
void onewire_write(uint8_t v)
{
	uint8_t bitMask;
   
   // Byte'ı bitlere bölerek gönder
	for (bitMask = 0x01; bitMask; bitMask <<= 1){
		onewire_write_bit( (bitMask & v)?1:0);
	}
}

//
// Write multiple bytes of data
//
void onewire_write_bytes(const uint8_t *buf, uint16_t count)
{
  for (uint16_t i = 0 ; i < count ; i++)
	  onewire_write(buf[i]);
}
//
// Read 1 byte data
//
uint8_t onewire_read(void)
{
   uint8_t bitMask;
   uint8_t r = 0;

   // Bit bit byte'ı oku
   for (bitMask = 0x01; bitMask; bitMask <<= 1) {
      if (onewire_read_bit()) r |= bitMask;
   }
   return r;
}

//
// Read multiple bytes of data
//
void onewire_read_bytes(uint8_t *buf, uint16_t count)
{
  for (uint16_t i = 0 ; i < count ; i++)
    buf[i] = onewire_read();
}

//
// Implement select ROM. After this function only the device at given
// deviceAddress will respond to next command
// 
void onewire_selectROM(DeviceAddress deviceAddress)
{
    uint8_t i;

    onewire_write(SELECT_ROM);           // ROM seçeceiğim komutu ver

    for (i = 0; i < 8; i++) onewire_write(deviceAddress[i]); // ROM adresini gönder
}

//
// Implement One-Wire skip ROM. This function can used in a sitation which
// there is only one device connected to line
//
void onewire_skipROM()
{
	onewire_write(SKIP_ROM);           // Skip ROM komutunu gönder
}


//
// Reset variables which used in onewire_search() function.
// This function can be used when the last onewire_search is incomplete
//
void onewire_reset_search()
{
  // reset the search state
  LastDiscrepancy = 0;
  LastDeviceFlag = 0;
  LastFamilyDiscrepancy = 0;
  for(int i = 7; ; i--) {
    ROM_NO[i] = 0;
    if ( i == 0) break;
  }
}

//
// Sets onewire_search() function variables to search for devices with given
// family_code. Family Code is the first byte of ROM address
//
void onewire_target_search(uint8_t family_code)
{
   // set the search state to find SearchFamily type devices
   ROM_NO[0] = family_code;
   for (uint8_t i = 1; i < 8; i++)
      ROM_NO[i] = 0;
   LastDiscrepancy = 64;
   LastFamilyDiscrepancy = 0;
   LastDeviceFlag = 0;
}

//
// Implements One-Wire search.  Returns 1 and ROM address if the search successful.
// Returns 0, if there is no device connected or the device found is same with the last device
//
uint8_t onewire_search(DeviceAddress newAddr, uint8_t search_mode /* = true */)
{
   uint8_t id_bit_number;
   uint8_t last_zero, rom_byte_number;
   uint8_t    search_result;
   uint8_t id_bit, cmp_id_bit;

   unsigned char rom_byte_mask, search_direction;

   id_bit_number = 1;
   last_zero = 0;
   rom_byte_number = 0;
   rom_byte_mask = 1;
   search_result = 0;

   // if not last device continue
   if (!LastDeviceFlag) {
      // 1-Wire reset
      if (!onewire_reset()) {
         // No device connected return 0
         LastDiscrepancy = 0;
         LastDeviceFlag = 0;
         LastFamilyDiscrepancy = 0;
         return 0;
      }

      // Send what search type
      if (search_mode == MODE_NORMAL) {
    	  onewire_write(NORMAL_SEARCH);   // NORMAL SEARCH
      } else {
    	  onewire_write(CONDITIONAL_SEARCH);   // CONDITIONAL SEARCH
      }

      // Continue until whole ROM address is read
      do
      {

         id_bit = onewire_read_bit();
         cmp_id_bit = onewire_read_bit();

         // if not this means device is not a One Wire device. So, break.
         if ((id_bit == 1) && (cmp_id_bit == 1)) {
            break;
         } else {
            // https://www.analog.com/en/app-notes/1wire-search-algorithm.html
            if (id_bit != cmp_id_bit) {
               search_direction = id_bit;  
            } else {

               if (id_bit_number < LastDiscrepancy) {
                  search_direction = ((ROM_NO[rom_byte_number] & rom_byte_mask) > 0);
               } else {

                  search_direction = (id_bit_number == LastDiscrepancy);
               }
             
               if (search_direction == 0) {
                  last_zero = id_bit_number;

   
                  if (last_zero < 9)
                     LastFamilyDiscrepancy = last_zero;
               }
            }

      
            if (search_direction == 1)
              ROM_NO[rom_byte_number] |= rom_byte_mask;
            else
              ROM_NO[rom_byte_number] &= ~rom_byte_mask;

      
            onewire_write_bit(search_direction);

         
            id_bit_number++;
            rom_byte_mask <<= 1;

         
            if (rom_byte_mask == 0) {
                rom_byte_number++;
                rom_byte_mask = 1;
            }
         }
      }
      while(rom_byte_number < 8);  

      // if search is successful
      if (!(id_bit_number < 65)) {
         // save to last device
         LastDiscrepancy = last_zero;

         // Check if the device is the last device
         if (LastDiscrepancy == 0) {
            LastDeviceFlag = 1;
         }
         search_result = 1;
      }
   }

   // if the search is not successful reset search variables
   if (!search_result || !ROM_NO[0]) {
      LastDiscrepancy = 0;
      LastDeviceFlag = 0;
      LastFamilyDiscrepancy = 0;
      search_result = 0;
   } else {
      for (int i = 0; i < 8; i++) newAddr[i] = ROM_NO[i];
   }
   return search_result;
}

uint8_t onewire_crc8(const uint8_t *addr,uint8_t len)
{
	uint8_t crc = 0;
	uint8_t inbyte = *addr++;
	for (uint8_t i = 8; i; i--) {
		uint8_t mix = (crc ^ inbyte) & 0x01;
		crc >>= 1;
		if (mix) crc ^= 0x8C;
			inbyte >>= 1;
	}
	return crc;
}


