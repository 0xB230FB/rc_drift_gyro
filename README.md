Device uses stm32F030 MCU. 2 PWM inputs are steering and gain channels from the hobby radio, output is 400Hz pwm to steering servo. 

Code uses simple digital filter and PID, with K and P factors adjustable from radio.

After powering up device performs gyro calibration procedure while red LED is on.

Filter code created in Winfilter software

Servo endpoints and input signal limits should be defined manually. Auto calibration have not finished yet.
