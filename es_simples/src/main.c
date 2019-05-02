#include <stdbool.h>
#include <stdint.h>
#include "inc/tm4c1294ncpdt.h" // CMSIS-Core
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h" // driverlib
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/systick.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"

#define AMOSTRAUMSEG 1262559 // 631711 * 1,7262 *1,15
#define AMOSTRAUMMILI 1261 // 632 * 1,99 * 1,053


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


void main(void){
  uint32_t ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                              SYSCTL_OSC_MAIN |
                                              SYSCTL_USE_PLL |
                                              SYSCTL_CFG_VCO_480),
                                              24000000); // PLL em 24MHz
  g_ui32SysClock = ui32SysClock;
   //iartheworld
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION); 
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPION)); // Aguarda final da habilita��o
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM); 
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOM)); // Aguarda final da habilita��o
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); 
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF)); // Aguarda final da habilita��o
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ); 
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOJ)); // Aguarda final da habilita��o
  
  GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0); 
  GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3); 
  GPIOPinTypeGPIOInput(GPIO_PORTM_BASE, GPIO_PIN_3); 
  GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1); 
  GPIOPadConfigSet(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
  ConfigureUART();
  
   int numamostras = AMOSTRAUMSEG;
  while(1)
  {
    int contagem = 0;
    int i;
    int leitura = GPIOPinRead(GPIO_PORTJ_BASE, GPIO_PIN_1 | GPIO_PIN_0);
    switch(leitura) 
    {
      case GPIO_PIN_0: numamostras = AMOSTRAUMSEG;
      break;
      
      case GPIO_PIN_1: numamostras = AMOSTRAUMMILI;
      break;
    }
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3 , GPIO_PIN_3); 
    int leituraAnterior = GPIOPinRead(GPIO_PORTM_BASE, GPIO_PIN_3);
    for(i = 0; i < numamostras; i++)
    {
      int a = GPIOPinRead(GPIO_PORTM_BASE, GPIO_PIN_3);
      if (a != leituraAnterior && a == GPIO_PIN_3)
      {
        contagem++;
      }
      leituraAnterior = a;
    }
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3 , 0); 
    
    UARTprintf("%d\n", contagem);
  }
}