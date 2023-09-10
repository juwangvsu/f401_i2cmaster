
stm32 test proj to read adafruit power sensor ina260
Ju Wang, 2023

----9/9/23 -------------------
ina260 code from 
	https://github.com/ardnew/INA260-STM32-HAL
copy ina260.h .c file to core/include and core/src
note i2c address no need to <<1
sprintf %d. dont %u since negative value is possible and cause strange print value
juwangvsu git repo 
