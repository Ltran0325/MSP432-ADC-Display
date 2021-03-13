# MSP432-ADC-Display
This program uses the Precision ADC module to display an analog voltage on a 7-segment display. (0-3.3V)

Setup:

![image](https://user-images.githubusercontent.com/62213019/111051088-60e11a00-8405-11eb-9c94-2412668cdf4c.png)

Demo:
https://youtu.be/P6bJy0kSj-o

Description:
To read the analog voltage at P5.1 and convert it to a digital value, we use the MSP432’s
Precision ADC module. To run this module at 48 MHz, we are required to adjust the clock
system (CS) and Power Supply System (PSS) settings. In the PCMCTL0 register, we set the
active mode request to AM_LDO_VCORE1. This core voltage supports the maximum CPU
frequency of 48 MHz. We then enable the DCO oscillator and source it to the MCLK. 

Program (Running):

![image](https://user-images.githubusercontent.com/62213019/111051126-abfb2d00-8405-11eb-8494-e5b02360934a.png)

