/*
 * one_wire.h
 *
 *  Created on: Jan 17, 2023
 *      Author: neraiv
 */

#ifndef INC_ONE_WIRE_H_
#define INC_ONE_WIRE_H_

#ifndef __STM32F4xx_HAL_H
#include "stm32f4xx_hal.h"
#endif

#define DEFAULT_DELAY 0
#define USER_DEFINED_DELAY 1
// Change if u will pass ur own delay function.
#define DELAY_FUNCTION_TYPE DEFAULT_DELAY

typedef struct {
	uint16_t pin;
	GPIO_TypeDef* port;
	uint8_t lastDiscrepancy;
	uint8_t lastFamilyDiscrepancy;
	uint8_t lastDeviceFlag;
}OneWire_t;

typedef unsigned char DeviceAddress[8];

// One-Wire SEARCH MODES
#define MODE_NORMAL        1
#define MODE_CONDITIONAL   0

// One-Wire COMMANDS
#define SELECT_ROM         0x55
#define SKIP_ROM           0xCC
#define NORMAL_SEARCH      0xF0
#define CONDITIONAL_SEARCH 0xEC


typedef void (*DelayFunc)(uint32_t);
extern DelayFunc owDelayMicroSecs_func;

#if (DELAY_FUNCTION_TYPE == DEFAULT_DELAY)
extern TIM_HandleTypeDef owHtim;
void owDefaultDelayMicroSecs(uint32_t delay);
#endif

//
// Sets a timer to be used in owDelayMicroSecs_func function and defines GPIO pin.Also, starts
// given timer if its not started yet.
//
void one_wire_Init_Timer(TIM_HandleTypeDef *htimx, uint32_t source_clock, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

//
//Set a pin as output
//
void Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

//
//Set a pin as input
//
void Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

//Functions to communicate in one-wire protocol

//
// Implement 1-Wire reset.
// RETURN;
// 1, if the reset was successful
// 0, if the reset wasn't successful
//
uint8_t onewire_reset(OneWire_t* ow);

//
// Write 1 bit data
//
void onewire_write_bit(uint8_t v);

//
// Read 1 bit data
//
uint8_t onewire_read_bit(void);

//
// Write 1 byte data
//
void onewire_write(uint8_t v);

//
// Read 1 byte data
//
uint8_t onewire_read(void);

//
// Write multiple bytes of data
//
void onewire_write_bytes(const uint8_t *buf, uint16_t count);

//
// Read multiple bytes of data
//
void onewire_read_bytes(uint8_t *buf, uint16_t count);

//
// Implement select ROM. After this function only the device at given
// deviceAddress will respond to next command
//
void onewire_selectROM(DeviceAddress deviceAddress);

//
// Implement One-Wire skip ROM. This function can used in a sitation which
// there is only one device connected to line
//
void onewire_skipROM();

//
// Reset variables which used in onewire_search() function.
// This function can be used when the last onewire_search is incomplete
//
void onewire_reset_search();

//
// Sets onewire_search() function variables to search for devices with given
// family_code. Family Code is the first byte of ROM address
//
void onewire_target_search(uint8_t family_code);

//
// Implements One-Wire search.  Returns 1 and ROM address if the search successful.
// Returns 0, if there is no device connected or the device found is same with the last device
//
uint8_t onewire_search(uint8_t *newAddr, uint8_t search_mode);


uint8_t onewire_crc8(const uint8_t *addr,uint8_t len);

#endif /* INC_ONE_WIRE_H_ */

