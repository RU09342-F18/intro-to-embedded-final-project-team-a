# Final Project: Team A
## Details
 * By Colby Clark and Scott Wood
 * Created: 8 December 2018
 * Last Updated: 14 December 2018 
## Purpose
  To create a rangefinder based on the intensity of reflected infrared waves. 
  The calculated distance would be displayed on an LCD screen. 

## Implementation
 The rangefinder was implemented using two microcontrollers that communicated over UART.
 One microcontroller reads the voltage generated by a photodetector and converts it to distance,
 and the other microcontroller is sent the distance over UART and displays it on an LCD 
 screen. 
 
 For more information about the key components of the rangefinder,
  open the individual files on Github.
  
### Specs
* The code is designed for the MSP430G2553 microprocessor
* A 16x2 LCD Display was used
* Derived equation to calculation distance can be found in the [datasheet](www.python-exemplary.com/download/GP2Y0A21YK.pdf)
