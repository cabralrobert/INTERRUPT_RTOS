/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>


/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

#include <stdio.h>

/* TI-RTOS Header files */
#include <ti/drv/gpio/GPIO.h>
#include <ti/drv/gpio/soc/GPIO_soc.h>

#include "GPIO_log.h"
#include "GPIO_board.h"

#include <ti/board/board.h>

#include <ti/sysbios/family/arm/a8/intcps/Hwi.h>

#include <ti/drv/uart/UART.h>
#include <ti/drv/uart/UART_stdio.h>
#include <ADC.h>
#include <GPIO_robert.h>

/**********************************************************************
 ************************** Macros ************************************
 **********************************************************************/
#define DELAY_VALUE       (0x6FFFFFU)

uint32_t value = 0;

/**********************************************************************
 ************************** Internal functions ************************
 **********************************************************************/

Task_Handle main_task;

#define GPIO_LED_USER0 21
#define GPIO_LED_USER1 22
#define GPIO_LED_USER2 23
#define GPIO_LED_USER3 24


/* Delay function */
void AppDelay(unsigned int delayVal);

/* Callback function */
void AppGpioCallbackFxn(void);
/*
 *  ======== Board_initI2C ========
 */
static void Board_initGPIO(void)
{
    Board_initCfg boardCfg;

    boardCfg = BOARD_INIT_PINMUX_CONFIG |
            BOARD_INIT_MODULE_CLOCK |
            BOARD_INIT_UART_STDIO;

    Board_init(boardCfg);
}

/**********************************************************************
 ************************** Global Variables **************************
 **********************************************************************/
volatile uint32_t gpio_intr_triggered = 0;
uint32_t gpioBaseAddr;
uint32_t gpioPin;

/*
 *  ======== test function ========
 */

void gpioLedUser0(UArg arg0, UArg arg1){
  while(1){
  GPIOPinWrite(GPIO_BASE_ADDR, GPIO_LED_USER0, GPIO_PIN_HIGH);
  AppDelay(2*DELAY_VALUE);
  GPIOPinWrite(GPIO_BASE_ADDR, GPIO_LED_USER0, GPIO_PIN_LOW);
  AppDelay(2*DELAY_VALUE);
  Task_sleep(1);
  }

  Task_exit();
}

void gpioLedUser1(UArg arg0, UArg arg1){
  while(1){
  GPIOPinWrite(GPIO_BASE_ADDR, GPIO_LED_USER1, GPIO_PIN_HIGH);
  AppDelay(3*DELAY_VALUE);
  GPIOPinWrite(GPIO_BASE_ADDR, GPIO_LED_USER1, GPIO_PIN_LOW);
  AppDelay(3*DELAY_VALUE);
  Task_sleep(1);
  }

  Task_exit();
}

void gpioLedUser2(UArg arg0, UArg arg1){
  while(1){
  GPIOPinWrite(GPIO_BASE_ADDR, GPIO_LED_USER2, GPIO_PIN_HIGH);
  AppDelay(4*DELAY_VALUE);
  GPIOPinWrite(GPIO_BASE_ADDR, GPIO_LED_USER2, GPIO_PIN_LOW);
  AppDelay(4*DELAY_VALUE);
  Task_sleep(1);
  }

  Task_exit();
}

void gpioLedUser3(UArg arg0, UArg arg1){
  while(1){
  GPIOPinWrite(GPIO_BASE_ADDR, GPIO_LED_USER3, GPIO_PIN_HIGH);
  AppDelay(5*DELAY_VALUE);
  GPIOPinWrite(GPIO_BASE_ADDR, GPIO_LED_USER3, GPIO_PIN_LOW);
  AppDelay(5*DELAY_VALUE);
  Task_sleep(1);
  }

  Task_exit();
}

void adcRead(UArg arg0, UArg arg1){
  while(1){
    value = TSCADCFIFOADCDataRead(SOC_ADC_TSC_0_REGS, TSCADC_FIFO_0);
    UART_printf("Valor do sensor: %d\n", value);
    AppDelay(DELAY_VALUE);
  }

  Task_exit();
}


/*
 *  ======== main ========
 */

Void myBegin1(Hwi_Handle hwi) {
    UART_printf("OI\n");
    HWREG(SOC_GPIO_1_REGS + 0x2c) |= (1 << 28);
}


int main(void){

    /* Call board init functions */
    Board_initGPIO();

    ledInit(GPIO21, MODULE1);

    Task_Params taskParams;    
    Task_Params_init(&taskParams);
    taskParams.priority = 1;
    taskParams.stackSize = 0x1400;



    Hwi_Params hwiParams;
    Hwi_Handle myHwi;
    Error_Block eb;

    Error_init(&eb);

    Hwi_Params_init(&hwiParams);
    hwiParams.enableInt = FALSE;

    myHwi = Hwi_create(98, (Hwi_FuncPtr)myBegin1, &hwiParams, &eb);

    if(myHwi == NULL)
        UART_printf("Erro ao criar\n");
    else
        UART_printf("Criou a interrupção\n");

    Hwi_enableInterrupt(98);

    GPIODirModeSet(SOC_GPIO_1_REGS,GPIO28,DIR_INPUT);
    GPIOIntTypeSet(SOC_GPIO_1_REGS, GPIO28, GPIO_INT_TYPE_RISE_EDGE);
    GPIOPinIntEnable(SOC_GPIO_1_REGS, GPIO_INT_LINE_1, GPIO28);

    main_task = Task_create (gpioLedUser0, &taskParams, NULL);

    /* Start BIOS */
    BIOS_start();
    return (0);
}

/*
 *  ======== AppDelay ========
 */
void AppDelay(unsigned int delayVal){
    while(delayVal)
        delayVal--;
}

