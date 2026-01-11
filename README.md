## RTOS Based Grain Drier 
==============================

This project demonstrates the the use of RTOS in grain drying.The RTOS used is *RTL-RTX kernel* OS by *keil*.
It is demonstared using LPC1768 board.The sensores used are:-

1. DHT11 - To sense the room temperature.
2. HW080 - Capacitive moisture sensor.
3. A3144 - Hall effect sensor.

To work this you need three source files:

1. RTX_Config_CM.c
2. system_LPC177x.c
3. system_LPC177x.s

which are already provided in the repository.

The environment used is keil uvision 4. Follow the steps to get this work done.

1. Create a new project on keil uvision 4.You need to create a different project to know the working of each sensor.
2. Select the file you want and add it to the target with the source files.To work on LCD add the >>lcd.h and >>lcd.s files.
3. Go to options for target select the RTX Kernel.

# Below are the descriptions for each files:

1. DcMotor.c = To check DC Motor.
2. dht11_OS.c = To check temperature sensor in OS environment.
3. FanError_main.c = This is the main file.
4. hall_sensor_os.c = To check the Hall effect sensor in OS environment.
5. hall_sensor.c = To run the hall effect sensor in bare metal.
6. lcd_interface_main.c = Integration of lcd with the all three sensors.* (Still in working phase)
7. lcd.c = Source file to work on lcd.
8. main_lcd.c = Integartion of lcd with temperature and moisture sensor.
9. main.c = Integration of temperature and moisture sensor.
10. moistLM393.c = To run the moist sensor in bare metal.
11. RTX_Conf_CM.c = Source file for RTX kernel.
12. sensor_dht11.c = To run the temperature sensor in bare metal.
13. sensor_hw080_OS.c = To run the moist snsor in OS environment.
14. SensorStepperDc.c = Integration of DC motor with stepper and all three sensors (Unoptimized version of FanError_main.c).
15. stepper.c = To run the stepper motor in bare metal.
16. StepperAndSenor_main.c = Integration of Stepper and Sensors in oS environment.
17. system_LPC17xx.c = Source file for LPC1768.
18. ThreeSensors_main.c = Integration of all sensors.
19. system_LPC17xx.s = Source file for LPC1768.