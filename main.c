/*******************************************************************************
*                       MSP432 ADC Display                                     *
*                                                                              *
* Author:  Long Tran                                                           *
* Device:  MSP432P401R LaunchPad                                               *
* Program: Display analog input on 7-segment display                           *
*                                                                              *
* Important Ports:                                                             *
* P4 is OUTPUT for 7-seg display digit pattern                                 *
* P5 is INPUT  from ADC14                                                      *
* P8 is OUTPUT to control active digits in row                                 *
* P9 is OUTPUT to drive potentiometer                                          *
*                                                                              *
* Demo: https://youtu.be/P6bJy0kSj-o                                           *
*******************************************************************************/

// Include header file(s) and define constants
#include "msp.h"

// Define digit-bit lookup table
const uint8_t look_up[17] = {
0b11000000,  // 0
0b11111001,  // 1
0b10100100,  // 2
0b10110000,  // 3
0b10011001,  // 4
0b10010010,  // 5
0b10000010,  // 6
0b11111000,  // 7
0b10000000,  // 8
0b10010000,  // 9
0b10001000,  // A
0b10000011,  // b
0b11000110,  // C
0b10100001,  // d
0b10000110,  // E
0b10001110,  // F
0b11111111,  // Blank Display
};

// Define prototypes
void init_clock(void);    // initialize PCM, FLCTL, CS
void init_A4_ADC(void);   // initialize 14-bit ADC
void init_GPIO_7seg_display(void);  // initialize GPIO for 7-segment display
void wait(uint32_t t);  // busy wait

uint16_t get_ADC_conversion_result(void);
void convert_ADC_result_to_Vin(uint16_t ADC_result);
void display_ADC_result_on_7seg_display(void);

// Define global variables
uint8_t ADC_display[4] = {0,0,0,0};
uint16_t ADC_result = 0x00;

void main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD; // disable watchdog
    init_clock();
    init_A4_ADC();
    init_GPIO_7seg_display();

    uint16_t temp = 0;
    uint8_t counter = 0;
    while(1){

        if(counter == 35){                      // read ADC data every 35 loops
        temp = get_ADC_conversion_result();
        convert_ADC_result_to_Vin(temp);
        counter = 0;
        }
        counter++;

        display_ADC_result_on_7seg_display();   // display ADC voltage on 7seg
    }

}

void init_clock(void){
    // MSP432 Technical Reference Manual
    // Set power level for the desired clock frequency (48 MHz)
    // Switches LDO VCORE0 to LDO VCORE1; mandatory for 48 MHz setting
    while((PCM->CTL1 & PCM_CTL1_PMR_BUSY));
    PCM->CTL0 = PCM_CTL0_KEY_VAL | PCM_CTL0_AMR_1;  // AM_LDO_VCORE1
    while((PCM->CTL1 & PCM_CTL1_PMR_BUSY));
    PCM->CTL0 &= 0x0000FFFF;    // lock PCM

    //  Flash read wait state number change
    FLCTL->BANK0_RDCTL &= ~(BIT(12) | BIT(13) | BIT(14) | BIT(15) ); // reset bits
    FLCTL->BANK0_RDCTL |=   BIT(12);                                 // set 1 wait state
    FLCTL->BANK1_RDCTL &= ~(BIT(12) | BIT(13) | BIT(14) | BIT(15) );
    FLCTL->BANK1_RDCTL |=   BIT(12);                                 // set 1 wait state

    // Enable DCO, set frequency to 48 MHz
    CS->KEY = 0x0000695A;           // unlock clock system registers (MSP432 Ref. Manual, pg.394)
    CS->CTL0 |= BIT(16)| BIT(18);   // set DCO frequency range to 48 MHz
    CS->CTL0 |= BIT(23);            // enable DCO oscillator

    // Select DCO as the source for MCLK
    CS->CTL1 |= BIT0 | BIT1;        // MCLK source: DCOCLK
    CS->CLKEN |= BIT1;              // enable MCLK
    CS->KEY =0x0;                   // lock CS registers

}

void init_A4_ADC(void){

    // CTL0 BIT1 ADC14ENC
    ADC14->CTL0 &=  ~BIT1;   // disable ADC14ENC to begin  ADC14 configuration

    // BIT 31-30 ADC14PDIV
    ADC14->CTL0 &= ~(BIT(30) | BIT(31));    //set ADC14 predivider to 32
    ADC14->CTL0 |= BIT(31);                 //set ADC14 predivider to 32

    // BIT 29-27 ADC14SHSx
    ADC14->CTL0 &= ~(BIT(27) | BIT(28) | BIT(29) ); // set ADC14 SH to ADC14SC bit

    // BIT 26 ADC14SHP
    ADC14->CTL0 |=   BIT(26);   // set source of sampling signal as sampling timer

    // BIT 24-22 ADC14DIVx
    ADC14->CTL0 &= ~(BIT(22) | BIT(23) | BIT(24) ); // set ADC14 clock divider to 4
    ADC14->CTL0 |=  (BIT(22) | BIT(23) );           // set ADC14 clock divider to 4

    // BIT 21-19 ADC14SSELx
    ADC14->CTL0 &= ~(BIT(19) | BIT(20) | BIT(21) ); // select MCLK as ADC14 clock source
    ADC14->CTL0 |=  (BIT(19) | BIT(20) );           // select MCLK as ADC14 clock source

    // BIT 17-18 ADC14CONSEQx
    ADC14->CTL0 &= ~(BIT(17) | BIT(18) ); // single channel, single conversion

    // BIT 11-8 ADC14SHT0x
    ADC14->CTL0 &= ~(BIT8 | BIT9 | BIT(10) | BIT(11) ); // set ADC14 sampling-hold time to 32 cycles per period
    ADC14->CTL0 |=   BIT8 | BIT9 ;                      // set ADC14 sampling-hold time to 32 cycles per period

    // BIT 4 ADC14ON
    ADC14->CTL0 |= BIT4;    // ADC14 on. ADC Core is ready for conversion.

    // BIT 20-16 ADC14CSTARTADDx
    ADC14->CTL1 &= ~(BIT(16) | BIT(17) | BIT(18) | BIT(19) | BIT(20) );  // conversion start address points to ADC14MEM0

    // Bit 4-5 ADC14RES
    ADC14->CTL1 |= (BIT4 | BIT5 );  // 14 bit conversion result resolution

    // BIT 3 ADC14DF
    ADC14->CTL1 &= ~BIT3;   // data read-back format is stroed as binary unsigned

    // BIT 0-4 ADC14INCHx
    ADC14->MCTL[0] &= ~(BIT0 | BIT1 | BIT2 | BIT3 | BIT4 ); // Select input channel A4
    ADC14->MCTL[0] |=   BIT2;                               // Select input channel A4

    // BIT 1 ADC14ENC
    ADC14->CTL0 |= BIT1;    // lock ADC14 configuration and enable ADC

    // ADC circuit setup
    P5->DIR &= ~(BIT1);     // set P5.1 as INPUT
    P5->SELC |=  BIT1;      // set P5.1 as ADC analog input A4  (MSP432P401R Texas Instruments data sheet, pg. 152)

    P9->DIR |= BIT4;        // set P9.4 as OUTPUT to drive voltage to potentiometer
    P9->OUT |= BIT4;        // set potentiometer voltage high
}

void init_GPIO_7seg_display(void){

    P4->DIR =  0xFF;  // P4 is LED output
    P8->DIR =  0xFF;  // P8 is display output

}

void wait(uint32_t t){
    while(t > 0){t--;}
}

uint16_t get_ADC_conversion_result(void){

    // BIT 0-1 ADC14SC and ADC14ENC
    ADC14->CTL0 |= ( BIT0|BIT1 );   // start ADC14 conversion by setting both registers

    while(ADC14->CTL0 & BIT(16));   // ADC14BUSY, wait while ADC14 is busy

    return ADC14->MEM[0];     // get ADC14 result
}

void convert_ADC_result_to_Vin(uint16_t ADC_result){

    // N_adc = 16384(Vin)/3.3
    uint32_t Vin = ADC_result*3300;    // conversion for single-ended mode
    Vin /= 16384;

    ADC_display[0] = Vin/1000;
    ADC_display[1] = (Vin/100)%10;
    ADC_display[2] = (Vin/10)%10;
    ADC_display[3] = Vin%10;
}

void display_ADC_result_on_7seg_display(void){

    static uint8_t k = 0;

    // Display digit-k
    P4->OUT = 0xFF;                      // blank 7-seg display
    P8->OUT = 0xFF & ~(BIT5 >> k);       // enable k-th digit in 7-seg display
    P4->OUT = look_up[ADC_display[k]];   // display k-th digit in 7-seg display
    if (k == 0){P4->OUT &= ~BIT7;}       // display decimal point at digit0

    // increment k index
    k++;
    if (k >= 4){k = 0;}

    // reduce flickering
    wait(15000);

}
