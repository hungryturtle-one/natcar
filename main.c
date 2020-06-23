

/**
 * main.c
 */

///////////////
//
// Include libraries that we need
//

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include "driverlib/rom.h"
#include <string.h>
#include "inc/hw_memmap.h"
#include "driverlib/adc.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "inc/hw_ints.h"
#include "driverlib/interrupt.h"
#include "inc/hw_nvic.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/pwm.h"




//
//
///////////////

volatile int SI = 0;                // SI signal to initiate a "line" of pixel
volatile int lineTimer;             // Timer signal to keep track of how long a "line" takes
volatile int CLK = 0;               // Clock signal that will tell the ADC to convert a value
uint32_t val;              // Variable that the ADC reads into
volatile uint32_t ping[128];        // Buffer #1 that will hold a "line" of ADC values
volatile uint32_t pong[128];        // Buffer #2 that will hold a "line" of ADC values
volatile int buffer = 1;            // Buffer pointer that tells the ADC where to store
                                    // values: 0 = ping, 1 = pong
volatile int indexping = 0;             // Index that tells the ADC where in ping/pong to store
volatile int index = 0;
volatile int indexpong = 0;                                     // the value
volatile int done = 0;              // Flag that lets the program know when the "line" is done
volatile int clkCount = 0;          // CLK counter that lets the ADC interrupt know when
                                    // 128 pixels have been converted
volatile int letter;                // Variable that the UART stores into
volatile int printFlag = 0;         // Flag to print something to UART
volatile int copyFlag = 0;          // Similar to above
volatile int SysFlag = 0;           //
volatile int fullPong = 0;
volatile int fullPing = 0;
volatile int letter = 0;
volatile int max =0;
volatile int min =0;
volatile int fullBuffer = 0;
volatile int maxIndex = 85;
volatile int ftflag = 0;
void ADCHandler();



int main(void)
{
    //
    // Initialize array for holding data from ADC
    //
    uint32_t pui32ADC0Value[1];

    //
    // Set SYSTEM clock to run at 50 MHz
    //
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                       SYSCTL_XTAL_16MHZ);

    //
    // Uncomment line below if use of the terminal is necessary
    //

    //*****
    InitConsole();
    //*****

    ///////////////
    //
    // Lines of Code that enable the GPIO pins we need
    //
    //

    //
    // Enable the C pins; there are output pins available on
    // peripheral C, which is why it was chosen.
    //
    // Also enable peripheral D for more GPIO pins
    //
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    GPIOPinConfigure(GPIO_PB6_M0PWM0);
    GPIOPinConfigure(GPIO_PB7_M0PWM1);



    //
    // Set PC6, PC7, and PD6 to be our GPIO outputs
    //

    GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_6);
    GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_7);
    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_4);

    //set INA INB
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_3);// INA
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_4); //INB
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, 0);

    // Set PE3 as our ADC input pin
    //

    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_7);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_UP_DOWN |
                      PWM_GEN_MODE_NO_SYNC);
    SysCtlPWMClockSet(SYSCTL_PWMDIV_64);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 15625);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 1172);
    // DC Motor Duty
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 4750);

    PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, true);
    PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, true);

    PWMGenEnable(PWM0_BASE, PWM_GEN_0);


    //
    // Enable sample sequence 3 (see lab 4 Part 1 code for more details
    //

    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);

    //
    // Configure step 0 on sequence 3. See lab 4 Part 1 code for more details
    //

    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH0 | ADC_CTL_IE |
                                 ADC_CTL_END);

    //
    // Enable ADC sequence 3
    //

    ADCSequenceEnable(ADC0_BASE, 3);

    //
    // Clear interrupt flag in case it start with an invalid flag
    //

    ADCIntClear(ADC0_BASE, 3);

    //
    //
    ///////////////

    ///////////////
    //
    // Configure SysTick, which will act as our timer interrupt
    //

    //
    // Set SysTick period to 100 Hz
    //

    SysTickPeriodSet(SysCtlClockGet()/100);

    //
    // Enable SysTick interrupt and SysTick in general
    //

    SysTickIntEnable();
    SysTickEnable();

    //
    //
    ///////////////

    ///////////////
    //
    // Enable ADC interrupt
    //

    ADCIntRegister(ADC0_BASE, 3, ADCHandler);
    ADCIntEnable(ADC0_BASE, 3);

    //
    //
    ///////////////


    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_PIN_4);

    ///////////////
    //
    // While(1) statement where the program chills out when not in an interrupt
    //

    int i = 0;

    while(1)
    {
        /*if (done) {
            done = 0;
            fullBuffer = 0;
        }*/


        if(fullBuffer && buffer) {
            if(UARTCharsAvail(UART0_BASE) == 0) {
                // calculate average
                printAvgHex2(pong);
            }
            else {
                letter = UARTCharGet(UART0_BASE);
                if(letter == 'p') {
                    //
                    //disable interupts
                    //
                    SysTickIntDisable();

                    // print the buffer
                    UARTprintf("Pong Buffer:\n");
                    for (i = 0; i < 128; i++) {
                        UARTprintf("%i,", pong[i]);
                    }
                    while(UARTCharsAvail(UART0_BASE) == 0) {

                    }
                    letter = UARTCharGet(UART0_BASE);
                    if(letter == 'c') {
                        // collect data again set shit = to 0
                        SysTickIntEnable();
                    }
                    else if (letter == 'q') {
                        UARTprintf("\n\n\n\n\n\n\n\n\n\nKa-CHOOOooow ;)");
                        exit(0);
                    }
                }
            }
        }
        else if(fullBuffer) {
            if(UARTCharsAvail(UART0_BASE) == 0) {
                // calculate average
                printAvgHex2(ping);
            }
            else {
                letter = UARTCharGet(UART0_BASE);
                if(letter == 'p') {
                    //
                    //disable interupts
                    //
                    SysTickIntDisable();

                    // print the buffer
                    UARTprintf("Ping Buffer:\n");
                    for (i = 0; i < 128; i++) {
                        UARTprintf("%i,", pong[i]);
                    }
                    while(UARTCharsAvail(UART0_BASE) == 0) {

                    }
                    letter = UARTCharGet(UART0_BASE);
                    if(letter == 'c') {
                        // collect data again set shit = to 0
                        SysTickIntEnable();
                    }
                    else if (letter == 'q') {
                        UARTprintf("\n\n\n\n\n\n\n\n\n\nKa-CHOOOooow ;)");
                        exit(0);
                    }
                }
            }

        }
        else {

        }


    }

}

void
InitConsole(void)
{
    //
    // Enable GPIO port A which is used for UART0 pins.
    // TODO: change this to whichever GPIO port you are using.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure the pin muxing for UART0 functions on port A0 and A1.
    // This step is not necessary if your part does not support pin muxing.
    // TODO: change this to select the port/pin you are using.
    //
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    //
    // Enable UART0 so that we can configure the clock.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Select the alternate (UART) function for these pins.
    // TODO: change this to select the port/pin you are using.
    //
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);

   // IntEnable(INT_UART0);
   //UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
}

void
UARTIntHandler(void)
{
    uint32_t ui32Status;

    //
    // Get the interrrupt status.
    //
    ui32Status = UARTIntStatus(UART0_BASE, true);

    //
    // Clear the asserted interrupts.
    //
    UARTIntClear(UART0_BASE, ui32Status);

    //
    // Loop while there are characters in the receive FIFO.
    //
    while(UARTCharsAvail(UART0_BASE))
    {
        //
        // Read the next character from the UART and write it back to the UART.
        //
        //ROM_UARTCharPutNonBlocking(UART0_BASE,letter);

        letter = UARTCharGet(UART0_BASE);
        //UARTSend((uint8_t *)"\033[2JEnter text: ", 16);
        UARTCharPutNonBlocking(UART0_BASE,letter);
        //ROM_UARTCharPutNonBlocking(UART1_BASE,letter);


        if (letter == ('p')){
            printFlag = 1;
            }
        else if (letter == ('c')){
            copyFlag = 1;
        }
        else if (letter == ('q')){
            exit(0);
        }
        //
        // Blink the LED to show a character transfer is occuring.
        //
        //GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);

        //
        // Delay for 1 millisecond.  Each SysCtlDelay is about 3 clocks.
        //
        //SysCtlDelay(SysCtlClockGet() / (100 * 3));

        //
        // Turn off the LED
        //
        //GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);


    }
}

///////////////
//
// Timer Interrupt Handler, moderated by SysTick
//
///////////////

void
SysTickHandler(void)
{
    //SysFlag = 1;

    //
    // Assert SI = HIGH
    //
    SI = 1;
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, GPIO_PIN_6);

    //
    // Assert conversion timer = HIGH
    //
    lineTimer = 1;
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_PIN_4);

    ///////////////
    // NOTE: SysTick timer interrupt request clears itself, so
    //       no line of code clearing timer interrupt request
    //       is included
    ///////////////

    //
    // Assert CLK signal = HIGH
    //
    CLK = 1;
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, GPIO_PIN_7);

    //
    // Begin CLK counter here
    //
    clkCount = 1;

    //
    // Reset done flag
    //

    //
    // Reset index
    //
    // indexping = 0;

    //
    // Change buffer pointer to the opposite, to switch between ping and pong
    //
    if (fullBuffer) {
        if (buffer == 1) {
            buffer = 0;
        }
        else {
            buffer = 1;
        }
    }


    //
    // Assert SI = LOW
    //
    SI = 0;
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0);

    //
    // Start ADC conversion
    //
    ADCProcessorTrigger(ADC0_BASE, 3);

    //
    // Assert CLK = LOW
    //
    CLK = 0;
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0);

}

void
ADCHandler(void)
{
    //
    // Assert CLK = HIGH
    //
    CLK = 1;
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, GPIO_PIN_7);

    //
    // Clear ADC interrupt
    //

    ADCIntClear(ADC0_BASE, 3);
    //
    // Read ADC value
    //
    ADCSequenceDataGet(ADC0_BASE, 3, &val);

    //
    // Store value in either ping or pong, and increment index If buffer is 1 store in pong.
    //

    if (buffer == 0) {
        ping[index] = val;
        index = index + 1;
    }
    else {
        pong[index] = val;
        index = index + 1;
    }

    //    if ((fullPong==0) && SI) {
    //    if (fullPong == 0) {
    //        pong[indexpong] = val;
    //        indexpong = indexpong + 1;
    //    }
    //    if(indexpong == 128) {
    //        fullPing = 0;
    //        fullPong = 1;
    //        indexpong = 0;
    //    }
    //    if(fullPing == 0 && fullPong==1) {
    //        ping[indexping] = val;
    //        indexping = indexping + 1;
    //    }
    //    if(indexping == 128) {
    //           fullPong = 0;
    //           fullPing = 1;
    //           indexping = 0;
    //    }

    //
    // Increment clock counter
    //
    clkCount++;

    //
    // Check if clock counter < 129
    // IF < 129: Start next ADC conversion (NOTE: ensure at least 120 ns delay since CLK set high)
    // ELSE: Assert lineTimer = LOW, Assert done = HIGH
    //

	//Adjust for middle. Higher max index means turn left 71-99
    if (clkCount < 129) {
        ADCProcessorTrigger(ADC0_BASE, 3);

    }
    else {
        lineTimer = 0;
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, 0);
        clkCount = 0;
        fullBuffer = 1;
        index = 0;
        done = 1;
        if(fullBuffer && buffer) {
            maxIndex = findMaxIndex(pong);
             /*
            if(brakeCheck(pong) && (ftflag == 0)) {
                            ftflag = 1;
            }
            */
            if (brakeCheck(pong) && (maxIndex < 80) ){
                PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 0); // DC shut off
            }

            // experimental
           /*f (turnRightSeeNothing(pong)){
                maxIndex = 130;
            }
*/
            if ((maxIndex < 80) && (maxIndex > 70)){ // originally 90 70
                maxIndex = 75; // originally 85
            }


            else if ((maxIndex < 129) && (maxIndex > 80)){ // Originally 90
                maxIndex = 130;
            }
        }
        else if(fullBuffer) {
            maxIndex = findMaxIndex(ping);
               /*
            if(brakeCheck(ping) && (ftflag == 0)) {
                ftflag = 1;
            }
            */
            if (brakeCheck(ping) && (maxIndex < 80)){
                PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 0); // DC shut off
            }
            // experimental
           /*f (turnRightSeeNothing(ping)){
                maxIndex = 130;
            }
*/
            if ((maxIndex < 80) && (maxIndex > 70)){
                maxIndex = 75;
            }
            else if ((maxIndex < 129) && (maxIndex > 80)){ // Originally 90
                maxIndex = 130;
            }
        }
        else {

        }

            // PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 1487-(maxIndex*5));
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 1487-(maxIndex*5));
            //        if(fullPing) {
            //            fullPing =0;
            //
//        }
//        else {
//            fullPong =0;
//        }


        }
    //
    // Set CLK = LOW
    //
    CLK = 0;
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0);

}

int findMin(uint32_t array[128])
{
    int y;
    int currMin = array[0];

    for (y = 1; y < 128; y++) {
        if (array[y] < currMin) {
            currMin = array[y];
        }
    }
    return currMin;
}

int findMax(uint32_t array[128])
{
    int y;
    int currMax = array[0];

    for (y = 1; y < 128; y++) {
        if (array[y] > currMax) {
            currMax = array[y];
        }
    }
    return currMax;
}

int findMaxIndex(uint32_t array[128])
{
    int y;
    int x;
    int currMax = array[0];

    for (y = 1; y < 128; y++) {
        if (array[y] > currMax) {
            currMax = array[y];
            x=y;
        }
    }
    return x;
}

int average(uint32_t array[128])
{
    int y;
    int funcAvg = 0;

    for (y = 0; y < 128; y++) {
        funcAvg = funcAvg + array[y];
    }
    funcAvg = funcAvg/128;
    return funcAvg;
}
int turnRightSeeNothing (uint32_t array[128]){
    int max_n = findMaxIndex(array);

    int avg_n = average(array);

    int result_n = 0;

            if (avg_n > ((.8)*array[max_n])){
                result_n = 1;
            }
            else {
                result_n = 0;
            }
    return result_n;
}
int brakeCheck(uint32_t array[128]){
    int max_i = findMaxIndex(array);

    int high1, high2;
    int low1, low2;
    int result = 0;

    high1 = array[max_i-13]; //Adjust 30 to camera data (left peak)
    high2 = array[max_i+13]; //Adjust 30 to camera data (right peak)

    low1 = array[max_i-6]; //Adjust 15 to camera data (left trough)
    low2 = array[max_i+6]; //Adjust 15 to camera data (right trough)

//    if ((high1 > ((3/4)*array[max_i])) || (high2 > ((3/4)*array[max_i]))){     //Adjust 1/3 if needed
//        if ((low1 < ((3/4)*array[max_i])) || (low2 < ((3/4)*array[max_i]))){
//            return 1;
//        }
//    }
//    else {
//        return 0;
//    }

    if (high1 > ((.75)*array[max_i])){
        if (low1 < ((.75)*array[max_i])){
            result = 1;
        }
    }

    if (high2 > ((.75)*array[max_i])){
        if (low2 < ((.75)*array[max_i])){
            result = 1;
        }
    }

    return result;
    /*int i, j, newMax, oldMax;
    newMax = 0;
    newMax2 = 0;
    oldMax = array[0];
    for(i = 1; i<max_;i++) {
        if(array[i]>oldMax) {
            oldMax = array[i];
            newMax = i;
        }
    }
    oldMax = array[128]
    for(j = 127; j > max_;j--) {
            if(array[j]>oldMax) {
                oldMax = array[j];
                newMax2 = j;
            }
        }

    */

}

void printAvgHex2(uint32_t array[128]) {
    int avg = average(array);
    int pp = findMax(array)-findMin(array);
    char avgHex[3];
    char ppHex[3];


    /*sprintf(avgHex,"%x",avg);
    sprintf(ppHex,"%x",findMax(array)-findMin(array));*/

    UARTprintf("%i/%i\n",avg,pp);
}

/*void printAvgHex(uint32_t array[128]) {
    int y;
    int avg = 0;
    int pp = 0;
    int remainder, quotient, quotient2;
    char avgHex[12];
    char ppHex[12];
    int i = 0;
    int temp = 0;
    avg = average(array);
    pp = findMax(array)-findMin(array);

    quotient = avg;
    while(quotient!=0) {
        temp = quotient % 16;
        //To convert integer into character
        if( temp < 10) {
                  temp = temp + 48;
        }
        else{
                 temp = temp + 55;
        }
        avgHex[i++]= temp;
        quotient = quotient / 16;
     }
    quotient2 = pp;
       while(quotient2!=0) {
           temp = quotient2 % 16;
           //To convert integer into character
           if( temp < 10) {
                     temp =temp + 48;
           }
           else{
                    temp = temp + 55;
           }
           ppHex[i++]= temp;
           quotient2 = quotient2 / 16;
        }
       UARTprintf("%s/%s",avgHex, ppHex);


}*/
