#include <stdbool.h>
#include <stdint.h>
#include "inc/tm4c1294ncpdt.h" // CMSIS-Core
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h" // driverlib
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/systick.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"

#define Ticks_Hz 12e6
#define Ticks_kHz 20e3
#define F_CLK 24e6


uint32_t g_ui32SysClock;

void
ConfigureUART(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART0.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, g_ui32SysClock);
}

int SysTick_count = 0;
int freq;
int freqAcquired = 0;

void SysTick_Handler() 
{
  SysTick_count++;
  if(SysTick_count >= 2 * (g_ui32SysClock/24e6))
  {
    SysTick_count = 0;
    TimerDisable(TIMER0_BASE, TIMER_A); 
    freq = 0xFFFFFF - TimerValueGet(TIMER0_BASE, TIMER_A);
    freqAcquired = 1;
    HWREG(TIMER0_BASE+0x50)=0xFFFFFF; // reset timer
    TimerPrescaleSet(TIMER0_BASE, TIMER_A, 0xff);
     
  }
  
}



static void
PortJ_IntHandler(void)
{
    //
    // Go into an infinite loop.
    //
    
      int x=10;
      x=x+10;
      GPIOIntClear(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1);
}

void ConfigurePWM0()
{
  //
  // Enable the PWM0 peripheral
  //
  SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
  //
  // Wait for the PWM0 module to be ready.
  //
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0))
  { }
  //
  // Configure the PWM generator for count down mode with immediate updates
  // to the parameters.
  //
  PWMGenConfigure(PWM0_BASE, PWM_GEN_0,
  PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
  //
  // Set the period. For a 50 KHz frequency, the period = 1/50,000, or 20
  // microseconds. For a 20 MHz clock, this translates to 400 clock ticks.
  // Use this value to set the period.
  //
  PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 20);
  //
  // Set the pulse width of PWM0 for a 25% duty cycle.
  //
  PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 20/2);
  //
  // Set the pulse width of PWM1 for a 75% duty cycle.
  //
  PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 1);
  //
  // Start the timers in generator 0.
  //
  PWMGenEnable(PWM0_BASE, PWM_GEN_0);
  //
  // Enable the outputs.
  //
  PWMOutputState(PWM0_BASE, (PWM_OUT_0_BIT | PWM_OUT_1_BIT), true);
}



void main(void){
  uint32_t ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                              SYSCTL_OSC_MAIN |
                                              SYSCTL_USE_PLL |
                                              SYSCTL_CFG_VCO_480),
                                              F_CLK); // PLL em 24MHz
  g_ui32SysClock = ui32SysClock;
  
  
   // GPIO initialization
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD); 
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD)); // Aguarda final da habilita��o
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); 
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF)); // Aguarda final da habilita��o
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ); 
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOJ)); // Aguarda final da habilita��o
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL); 
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOL)); // Aguarda final da habilita��o
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION); 
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPION)); // Aguarda final da habilita��o
  
  GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0);
  GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3); 
  
  GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_0);
  GPIOPinConfigure(GPIO_PF0_M0PWM0);
  
  GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1);
  GPIOPadConfigSet(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU); // configure button pins with pull ups7
  GPIOIntTypeSet(GPIO_PORTJ_BASE, GPIO_INT_PIN_0 | GPIO_INT_PIN_1, GPIO_BOTH_EDGES);
  GPIOIntEnable(GPIO_PORTJ_BASE, GPIO_INT_PIN_0 | GPIO_INT_PIN_1);
  GPIOIntRegister(GPIO_PORTJ_BASE, PortJ_IntHandler);
  
  GPIOPinConfigure(GPIO_PL4_T0CCP0);
  GPIOPinTypeTimer(GPIO_PORTL_BASE, GPIO_PIN_4);
  
  
  
  //Timer initialization iarde wor
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0); 
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0)); // Aguarda final da habilita��o
  TimerConfigure(TIMER0_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_COUNT);
  TimerControlEvent(TIMER0_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);
  TimerPrescaleSet(TIMER0_BASE, TIMER_A, 0xff);
  TimerEnable(TIMER0_BASE, TIMER_A);
  
  // SysTick Initialization
  SysTickPeriodSet(Ticks_Hz);
  SysTickIntEnable();
  SysTickEnable();
  
  
  ConfigurePWM0();
  ConfigureUART();
  while (1)
  {
    if(freqAcquired)
    {
      UARTprintf("Frequência: %d Hz\n", freq);
      freqAcquired = 0;
      TimerEnable(TIMER0_BASE, TIMER_A);
    }
    
  }
  
//   int numamostras = AMOSTRAUMSEG;
//  while(1)
//  {
//    int contagem = 0;
//    int i;
//    int leitura = GPIOPinRead(GPIO_PORTJ_BASE, GPIO_PIN_1 | GPIO_PIN_0);
//    switch(leitura) 
//    {
//      case GPIO_PIN_0: numamostras = AMOSTRAUMSEG;
//      break;
//      
//      case GPIO_PIN_1: numamostras = AMOSTRAUMMILI;
//      break;
//    }
//    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3 , GPIO_PIN_3); 
//    int leituraAnterior = GPIOPinRead(GPIO_PORTM_BASE, GPIO_PIN_3);
//    for(i = 0; i < numamostras; i++)
//    {
//      int a = GPIOPinRead(GPIO_PORTM_BASE, GPIO_PIN_3);
//      if (a != leituraAnterior && a == GPIO_PIN_3)
//      {
//        contagem++;
//      }
//      leituraAnterior = a;
//    }
//    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3 , 0); 
//    
//    UARTprintf("%d\n", contagem);
//  }
}