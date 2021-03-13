# MSP432-ADC-Display
This program uses the Precision ADC module to display an analog voltage on a 7-segment display.

Demo:
https://youtu.be/P6bJy0kSj-o

Description:
To read the analog voltage at P5.1 and convert it to a digital value, we use the MSP432’s
Precision ADC module. To run this module at 48 MHz, we are required to adjust the clock
system (CS) and Power Supply System (PSS) settings. In the PCMCTL0 register, we set the
active mode request to AM_LDO_VCORE1. This core voltage supports the maximum CPU
frequency of 48 MHz. We then enable the DCO oscillator and source it to the MCLK. 
