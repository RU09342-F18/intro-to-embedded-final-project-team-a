#include <msp430F5529.h>
#include <math.h>

float analogVoltage;
float adcReading;
int adcReady = 1;
int distance;

void calculateDistance();

int main(void)
{
  WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT
  REFCTL0 &= ~REFMSTR;                      // Reset REFMSTR to hand over control to
                                            // ADC12_A ref control registers

  //port 6.0 is analog input 0***************************************************************
  P6DIR &= ~BIT0;                           //set 6.0 to be input
  P6SEL |= BIT0;                            //set 6.0 to be A0 (input of A to D)

  //UART Setup*******************************************************************************
    P4SEL |= BIT5 + BIT4;                     //enable UART for these pins
    UCA1CTL1 |= UCSWRST;                      // **Put state machine in reset**
    UCA1CTL1 |= UCSSEL_2;
    // Use Table 24-5 in Family User Guide for BAUD rate calculation
    UCA1BR0 = 6;                              // 1MHz 9600 (see User's Guide)
    UCA1BR1 = 0;                              // 1MHz 9600
    UCA1MCTL = UCBRS_0 + UCBRF_13 + UCOS16;   // Modln UCBRSx=0, UCBRFx=0,
    UCA1CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
    UCA1IE |= UCRXIE;                         // Enable USCI_A0 RX interrupt
    UCA1TXBUF = 0;                            //set RX buffer to 0 for testing purposes



  ADC12CTL0 = ADC12SHT02 + ADC12ON;         // Sampling time, ADC12 on
  ADC12CTL1 = ADC12SHP;                     // Use sampling timer
  ADC12IE = 0x01;                           // Enable interrupt
  ADC12CTL0 |= ADC12ENC;                    //Enable conversion
  /*
   *   //ADC sampling******************************************************************************
  ADC12CTL0 = ADC12SHT02 + ADC12ON;         // Sampling time, ADC12 on
  ADC12CTL1 = ADC12SHP;                     // Use sampling timer
  ADC12IE = 0x01;                           // Enable interrupt
  ADC12CTL0 |= ADC12ENC;                    //Enable conversion
   */



  __bis_SR_register(/*LPM0 + */ GIE);        // LPM0 with interrupts enabled

  //polling for ADC values********************************************************************
  while(1)
  {
    ADC12CTL0 |= ADC12SC;                   // Sampling and conversion start
    while(adcReady == 0){};                 //run this loop while waiting for ADC to finish conversion
    calculateDistance();                               //set PWM values to change duty cycle
    adcReady = 0;                           //adc no longer ready

  }

}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = ADC12_VECTOR
__interrupt void ADC12_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(ADC12_VECTOR))) ADC12_ISR (void)
#else
#error Compiler not supported!
#endif
{
    //switch(__even_in_range(ADC12IV,34))


   switch(__even_in_range(ADC12IV,34))
  {
  case  0: break;                           // Vector  0:  No interrupt
  case  2: break;                           // Vector  2:  ADC overflow
  case  4: break;                           // Vector  4:  ADC timing overflow
  case  6:                                  // Vector  6:  ADC12IFG0
      adcReady = 1;                           //when this interrupt fire, ADC is ready
    adcReading = ADC12MEM0;                 //record ADC reading
    calculateDistance();
    __bic_SR_register_on_exit(LPM0_bits);   // Exit active CPU

  case  8: break;                           // Vector  8:  ADC12IFG1
  case 10: break;                           // Vector 10:  ADC12IFG2
  case 12: break;                           // Vector 12:  ADC12IFG3
  case 14: break;                           // Vector 14:  ADC12IFG4
  case 16: break;                           // Vector 16:  ADC12IFG5
  case 18: break;                           // Vector 18:  ADC12IFG6
  case 20: break;                           // Vector 20:  ADC12IFG7
  case 22: break;                           // Vector 22:  ADC12IFG8
  case 24: break;                           // Vector 24:  ADC12IFG9
  case 26: break;                           // Vector 26:  ADC12IFG10
  case 28: break;                           // Vector 28:  ADC12IFG11
  case 30: break;                           // Vector 30:  ADC12IFG12
  case 32: break;                           // Vector 32:  ADC12IFG13
  case 34: break;                           // Vector 34:  ADC12IFG14
  default: break;
  }

}

void calculateDistance()
{

    //calculations*********************************************************************************************
     analogVoltage = adcReading * (3.3 / 4096);    //convert digital reading back to analog voltage value, if using 5V as Vref


    // add condition to verify that division by zero does not occur
     distance = (6787/(analogVoltage -3) - 4);

     //printf("The distance: ", distance);
}
#pragma vector=USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void)

{
  switch(__even_in_range(UCA1IV,4))
  {
  case 0:break;                             // Vector 0 - no interrupt
  case 2:                                   // Vector 2 - RXIFG
    while (!(UCA1IFG & UCTXIFG));             // USCI_A0 TX buffer ready?
    UCA1TXBUF = UCA1RXBUF;                  // TX -> RXed character
    distance = UCA1RXBUF;
    break;

   // break;
  case 4:break;                             // Vector 4 - TXIFG
  default: break;
  }





}
