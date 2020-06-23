/*******************************************************************/ /*!
*    \file       main.c
*    \brief      See main.h for the description
*    \date       Created: 2020-06-17
*    \author     Karina Avalos <avaloskarina04@gmail.com>
***********************************************************************/

//=======================================================================
//=  MODULE INCLUDE FILE
#include "main.h"

//=======================================================================
//=  LIBRARY/OTHER INCLUDE FILES
#include "rom.h"
#include "hw_memmap.h"
#include "adc.h"
#include "gpio.h"
#include "pin_map.h"
#include "sysctl.h"
#include "uart.h"
#include "uartstdio.h"
#include "hw_ints.h"
#include "interrupt.h"
#include "hw_nvic.h"
#include "hw_types.h"
#include "hw_gpio.h"
#include "debug.h"
#include "fpu.h"
#include "pwm.h"

//=======================================================================
//=  SYSTEM/STANDARD INCLUDE FILES
//=    E.g. #include <stdbool.h>
#include <string.h>

//=======================================================================
//=  PRIVATE OBJECT-LIKE/FUNCTION-LIKE MACROS
//=    E.g. #define BUFFER_SIZE 1024
//=    E.g. #define SQR(s)  ((s) * (s))

//=======================================================================
//=  GLOBAL VARIABLES (extern)
//=    Declared with keyword `extern` in the .h file.
//=    E.g. int8_t g_error_variable;
uint32_t val;                // Variable that the ADC reads into
volatile uint32_t ping[128]; // Buffer #1 that will hold a "line" of ADC values
volatile uint32_t pong[128]; // Buffer #2 that will hold a "line" of ADC values
volatile uint8_t buffer = 1;     // Buffer pointer that tells the ADC where to store
                             // values: 0 = ping, 1 = pong
volatile uint16_t index_ping = 0;  // Index that tells the ADC where in ping/pong to store
volatile uint16_t index = 0;
volatile uint16_t index_pong = 0; // the value
volatile uint16_t clk_count = 0;  // CLK counter that lets the ADC interrupt know when
            
volatile uint16_t letter;        // Variable that the UART stores into
volatile uint16_t letter = 0;
volatile uint16_t max = 0;
volatile uint16_t min = 0;
volatile uint16_t max_index = 85;

//=======================================================================
//=  PRIVATE TYPE DEFINITIONS
//=    E.g. typedef private to this source file
//=    typedef struct
//=    {
//=       uint16_t count;
//=    } timer_t;

//=======================================================================
//=  PRIVATE VARIABLE DEFINITIONS (static)
//=    Definition of private constants, variables and structures at file scope
//=    E.g. static uint8_t variable_name;
//=    E.g. static const uint8_t variable_name_k;
static bool b_is_buffer_full = 0;
static bool print_flag = 0; // Flag to print something to UART
static bool copy_flag = 0;  // Similar to above
static bool SysFlag = 0;   //
static bool fullPong = 0;
static bool fullPing = 0;
static bool SI = 0;         // SI signal to initiate a "line" of pixel
static bool line_timer;      // Timer signal to keep track of how long a "line" takes
static bool CLK = 0;        // Clock signal that will tell the ADC to convert a value
static bool done = 0;      // Flag that lets the program know when the "line" is done

//=======================================================================
//=  PRIVATE FUNCTION DECLARATIONS (static)
//=    Local function prototypes
//=    E.g. static uint8_t private_function_name(uint8_t param);
static set_max_index(uint16_t buffer_max_index);
//=======================================================================
//=  PUBLIC FUNCTION DEFINITIONS
//=    Definition of the public functions declared first in the .h file
//=    All public functions in this file shall reside here and be placed in a logical order.

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

    InitConsole();

    
    // Enable the C pins; there are output pins available on
    // peripheral C, which is why it was chosen.
    // Also enable peripheral D for more GPIO pins

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    GPIOPinConfigure(GPIO_PB6_M0PWM0);
    GPIOPinConfigure(GPIO_PB7_M0PWM1);

    // Set PC6, PC7, and PD6 to be our GPIO outputs

    GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_6);
    GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_7);
    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_4);


    // Set PE3 as our ADC input pin
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_7);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    SysCtlPWMClockSet(SYSCTL_PWMDIV_64);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 15625);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 1172);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 1172);

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

    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END);

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

    SysTickPeriodSet(SysCtlClockGet() / 100);

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

    uint16_t i = 0;

    while (1)
    {
        /*if (done) {
            done = 0;
            b_is_buffer_full = 0;
        }*/

        if (b_is_buffer_full && buffer)
        {
            if (UARTCharsAvail(UART0_BASE) == 0)
            {
                // calculate average
                printAvgHex2(pong);
            }
            else
            {
                letter = UARTCharGet(UART0_BASE);
                if (letter == 'p')
                {
                    //
                    //disable interupts
                    //
                    SysTickIntDisable();

                    // print the buffer
                    UARTprintf("Pong Buffer:\n");
                    for (i = 0; i < 128; i++)
                    {
                        UARTprintf("%i,", pong[i]);
                    }
                    while (UARTCharsAvail(UART0_BASE) == 0)
                    {
                    }
                    letter = UARTCharGet(UART0_BASE);
                    if (letter == 'c')
                    {
                        // collect data again set shit = to 0
                        SysTickIntEnable();
                    }
                    else if (letter == 'q')
                    {
                        UARTprintf("\n\n\n\n\n\n\n\n\n\nKa-CHOOOooow ;)");
                        exit(0);
                    }
                }
            }
        }
        else if (b_is_buffer_full)
        {
            if (UARTCharsAvail(UART0_BASE) == 0)
            {
                // calculate average
                printAvgHex2(ping);
            }
            else
            {
                letter = UARTCharGet(UART0_BASE);
                if (letter == 'p')
                {
                    //
                    //disable interupts
                    //
                    SysTickIntDisable();

                    // print the buffer
                    UARTprintf("Ping Buffer:\n");
                    for (i = 0; i < 128; i++)
                    {
                        UARTprintf("%i,", pong[i]);
                    }
                    while (UARTCharsAvail(UART0_BASE) == 0)
                    {
                    }
                    letter = UARTCharGet(UART0_BASE);
                    if (letter == 'c')
                    {
                        // collect data again set shit = to 0
                        SysTickIntEnable();
                    }
                    else if (letter == 'q')
                    {
                        UARTprintf("\n\n\n\n\n\n\n\n\n\nKa-CHOOOooow ;)");
                        exit(0);
                    }
                }
            }
        }
        else
        {
        }
    }
}

void InitConsole(void)
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

void UARTIntHandler(void)
{
    uint32_t ui32Status;

    // Get the interrrupt status.
    ui32Status = UARTIntStatus(UART0_BASE, true);

    // Clear the asserted interrupts.
    UARTIntClear(UART0_BASE, ui32Status);


    while (UARTCharsAvail(UART0_BASE))
    {
        letter = UARTCharGet(UART0_BASE);
        UARTCharPutNonBlocking(UART0_BASE, letter);


        if (letter == ('p'))
        {
            print_flag = 1;
        }
        else if (letter == ('c'))
        {
            copy_flag = 1;
        }
        else if (letter == ('q'))
        {
            exit(0);
        }

    }
}


// Timer Interrupt Handler, moderated by SysTick
void SysTickHandler(void)
{

    // Assert SI = HIGH
    SI = 1;
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, GPIO_PIN_6);

    // Assert conversion timer = HIGH
    line_timer = 1;
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_PIN_4);

    // Assert CLK signal = HIGH
    CLK = 1;
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, GPIO_PIN_7);

    // Begin CLK counter here
    clk_count = 1;

    // Change buffer pointer to the opposite, to switch between ping and pong
    if (b_is_buffer_full)
    {
        if (buffer == 1)
        {
            buffer = 0;
        }
        else
        {
            buffer = 1;
        }
    }

    // Assert SI = LOW
    SI = 0;
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0);
    ADCProcessorTrigger(ADC0_BASE, 3);
    CLK = 0;
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0);

}

void ADCHandler(void)
{
  
    CLK = 1;
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, GPIO_PIN_7);
    ADCIntClear(ADC0_BASE, 3);
    ADCSequenceDataGet(ADC0_BASE, 3, &val);

    if (buffer == 0)
    {
        ping[index] = val;
        index = index + 1;
    }
    else
    {
        pong[index] = val;
        index = index + 1;
    }

    clk_count++;

    if (clk_count < 129)
    {
        ADCProcessorTrigger(ADC0_BASE, 3);
    }
    else
    {
       
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, 0);
        line_timer = 0;
        clk_count = 0;
        b_is_buffer_full = 1;
        index = 0;
        done = 1;

        if (b_is_buffer_full)
        {
            if(buffer)
            {
                max_index = find_max_index(pong);
                set_max_index(max_index);
                
            }
            else
            {
                max_index = find_max_index(ping);
                set_max_index(max_index);
            }
         
        }

        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 1487 - (max_index * 5));
    }
    
    CLK = 0;
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0);
}

// Adjusts the max index to set values based on range it is in 
static set_max_index(uint16_t buffer_max_index)
{
    if ((buffer_max_index < 109) && (buffer_max_index > 99))
    {
        max_index = 105;
    }
    if ((buffer_max_index < 118) && (buffer_max_index > 109))
    {
        max_index = 115;
    }
    if ((buffer_max_index < 71) && (buffer_max_index > 55))
    {
        max_index = 64;
    }
    if ((buffer_max_index < 56) && (buffer_max_index > 39))
    {
        max_index = 48;
    }
    if ((buffer_max_index < 40) && (buffer_max_index > 19))
    {
        max_index = 30;
    }
    if ((buffer_max_index < 20) && (buffer_max_index > 0))
    {
        max_index = 5;
    }
    if ((buffer_max_index < 129) && (buffer_max_index > 118))
    {
        max_index = 127;
    }
    if ((buffer_max_index < 100) && (buffer_max_index > 70))
    {
        max_index = 85;
    }
}