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
#include "calculations.h"

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
uint32_t          val;                 // Variable that the ADC reads into
volatile uint32_t ping[MAX_ARRAY_LEN]; // Buffer #1 that will hold a "line" of ADC values
volatile uint32_t pong[MAX_ARRAY_LEN]; // Buffer #2 that will hold a "line" of ADC values
volatile uint8_t  buffer = 1;          // Buffer pointer that tells the ADC where to store
                                       // values: 0 = ping, 1 = pong
volatile uint16_t index_ping = 0;      // Index that tells the ADC where in ping/pong to store
volatile uint16_t index      = 0;
volatile uint16_t index_pong = 0; // the value
volatile uint16_t clk_count  = 0; // CLK counter that lets the ADC interrupt know when

volatile uint16_t letter; // Variable that the UART stores into
volatile uint16_t letter    = 0;
volatile uint16_t max       = 0;
volatile uint16_t min       = 0;
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
static bool print_flag       = 0; // Flag to print something to UART
static bool copy_flag        = 0; // Similar to above
static bool sys_flag         = 0; //
static bool fullPong         = 0;
static bool fullPing         = 0;
static bool SI               = 0; // SI signal to initiate a "line" of pixel
static bool line_timer;           // Timer signal to keep track of how long a "line" takes
static bool CLK  = 0;             // Clock signal that will tell the ADC to convert a value
static bool done = 0;             // Flag that lets the program know when the "line" is done

void ADCHandler();
void hw_init();

int main(void)
{

    int i = 0;
    hw_init();

    while (1)
    {
        if (b_is_buffer_full && buffer)
        {
            if (UARTCharsAvail(UART0_BASE) == 0)
            {
                print_avg_Hex2(pong);
            }
            else
            {
                letter = UARTCharGet(UART0_BASE);
                if (letter == 'p')
                {

                    SysTickIntDisable();

                    // print the buffer
                    UARTprintf("Pong Buffer:\n");
                    for (i = 0; i < MAX_ARRAY_LEN; i++)
                    {
                        UARTprintf("%i,", pong[i]);
                    }
                    while (UARTCharsAvail(UART0_BASE) == 0)
                    {
                    }
                    letter = UARTCharGet(UART0_BASE);
                    if (letter == 'c')
                    {
                        SysTickIntEnable();
                    }
                    else if (letter == 'q')
                    {
                        UARTprintf("\n\n\n\n\n\n\n\n\n\nKa-CHOOOooow");
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
                print_avg_hex(ping);
            }
            else
            {
                letter = UARTCharGet(UART0_BASE);
                if (letter == 'p')
                {
                    //disable interupts
                    SysTickIntDisable();

                    // print the buffer
                    UARTprintf("Ping Buffer:\n");
                    for (i = 0; i < MAX_ARRAY_LEN; i++)
                    {
                        UARTprintf("%i,", pong[i]);
                    }
                    while (UARTCharsAvail(UART0_BASE) == 0)
                    {
                    }
                    letter = UARTCharGet(UART0_BASE);
                    if (letter == 'c')
                    {
                        SysTickIntEnable();
                    }
                    else if (letter == 'q')
                    {
                        UARTprintf("\n\n\n\n\n\n\n\n\n\nKa-CHOOOooow");
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
    // Enable GPIO port A which is used for UART0 pins.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Enable UART0 so that we can configure the clock.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTStdioConfig(0, 115200, 16000000);
}

void UARTIntHandler(void)
{
    uint32_t ui32Status;

    // Get the interrrupt status.
    ui32Status = UARTIntStatus(UART0_BASE, true);

    // Clear the asserted interrupts.
    UARTIntClear(UART0_BASE, ui32Status);

    // Loop while there are characters in the receive FIFO.
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

void SysTickHandler(void)
{

    SI = 1;
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, GPIO_PIN_6);

    // Assert conversion timer = HIGH
    line_timer = 1;
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_PIN_4);

    // Assert CLK signal = HIGH
    CLK = 1;
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, GPIO_PIN_7);

    clk_count = 1;

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

    SI = 0;
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0);
    ADCProcessorTrigger(ADC0_BASE, 3);

    CLK = 0;
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0);
}

void ADCHandler(void)
{
    // Assert CLK = HIGH
    CLK = 1;
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, GPIO_PIN_7);

    // Clear ADC interrupt
    ADCIntClear(ADC0_BASE, 3);

    // Read ADC value
    ADCSequenceDataGet(ADC0_BASE, 3, &val);

    // Store value in either ping or pong, and increment index If buffer is 1 store in pong.
    if (buffer == 0)
    {
        ping[index] = val;
        index       = index + 1;
    }
    else
    {
        pong[index] = val;
        index       = index + 1;
    }

    clk_count++;

    //Adjust for middle. Higher max index means turn left 71-99
    if (clk_count < 129)
    {
        ADCProcessorTrigger(ADC0_BASE, 3);
    }
    else
    {
        line_timer = 0;
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, 0);
        clk_count        = 0;
        b_is_buffer_full = 1;
        index            = 0;
        done             = 1;

        if (b_is_buffer_full && buffer)
        {
            max_index = find_max_Index(pong);

            if (brake_check(pong) && (max_index < 80))
            {
                PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 0); // DC shut off
            }

            if ((max_index < 80) && (max_index > 70))
            {                   // originally 90 70
                max_index = 75; // originally 85
            }

            else if ((max_index < 129) && (max_index > 80))
            { // Originally 90
                max_index = 130;
            }
        }
        else if (b_is_buffer_full)
        {
            max_index = find_max_index(ping);

            if (brake_check(ping) && (max_index < 80))
            {
                PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 0); // DC shut off
            }

            if ((max_index < 80) && (max_index > 70))
            {
                max_index = 75;
            }
            else if ((max_index < 129) && (max_index > 80))
            { // Originally 90
                max_index = 130;
            }
        }
        else
        {
        }

        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 1487 - (max_index * 5));
    }

    CLK = 0;
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0);
}

int turn_right_clear(uint32_t array[MAX_ARRAY_LEN])
{
    int max_n = find_max_index(array);

    int avg_n = average(array);

    int result_n = 0;

    if (avg_n > ((.8) * array[max_n]))
    {
        result_n = 1;
    }
    else
    {
        result_n = 0;
    }
    return result_n;
}

int brake_check(uint32_t array[MAX_ARRAY_LEN])
{
    int max_i = find_max_index(array);

    int high1, high2;
    int low1, low2;
    int result = 0;

    high1 = array[max_i - 13]; //Adjust 30 to camera data (left peak)
    high2 = array[max_i + 13]; //Adjust 30 to camera data (right peak)

    low1 = array[max_i - 6]; //Adjust 15 to camera data (left trough)
    low2 = array[max_i + 6]; //Adjust 15 to camera data (right trough)

    if (high1 > ((.75) * array[max_i]))
    {
        if (low1 < ((.75) * array[max_i]))
        {
            result = 1;
        }
    }

    if (high2 > ((.75) * array[max_i]))
    {
        if (low2 < ((.75) * array[max_i]))
        {
            result = 1;
        }
    }

    return result;
}

void hw_init()
{
    uint32_t pui32ADC0Value[1];

    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                   SYSCTL_XTAL_16MHZ);

    InitConsole();

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
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

    //set INA INB
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_3); // INA
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_4); //INB
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, 0);

    // Set PE3 as our ADC input pin
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_7);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    SysCtlPWMClockSet(SYSCTL_PWMDIV_64);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 15625);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 1172);
    // DC Motor Duty
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 4750);

    PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, true);
    PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, true);

    PWMGenEnable(PWM0_BASE, PWM_GEN_0);

    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, 3);
    ADCIntClear(ADC0_BASE, 3);
    SysTickPeriodSet(SysCtlClockGet() / 100);
    SysTickIntEnable();
    SysTickEnable();

    ADCIntRegister(ADC0_BASE, 3, ADCHandler);
    ADCIntEnable(ADC0_BASE, 3);

    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_PIN_4);
}