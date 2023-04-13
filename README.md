# How to use
First add .c and .h files into ur project
1) To communicate in 1-Wire, we need a delay in microseconds. Since, HAL Libraries doesn't have delay in microseconds function, we need to create one. For now, there is only timer option. Select a timer (Library needs maximum of 470 microsecond of delay so u can choose whatever timer u want).

2) Choose a gpio pin for 1-Wire

3) Initilaze one_wire library using owInit_Timer function.
```
owInit_Timer(TIM_HandleTypeDef *htimx,uint8_t CLOCK_LINE,GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
(pointer to timer, timer_clock_line (1 for APB1,2 for APB2), GPIO_PORT, GPIO, Pin)
```
Thats it u can use the library funcitions now.
# Notes
Libabry has a type DeviceAddress. It can be useful to store mulitple ROM address in one array. There is an example in ds18xx library example (https://github.com/neraiv/ds18bxx_stm32).
