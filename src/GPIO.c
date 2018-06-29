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

#include <ti/starterware/include/hw/hw_control_am335x.h>
#include <ti/starterware/include/hw/soc_am335x.h>
#include <ti/starterware/include/hw/hw_cm_wkup.h>
#include <ti/starterware/include/hw/hw_cm_per.h>
#include <ti/starterware/include/hw/hw_types.h>
#include <ti/csl/src/ip/gpio/V1/gpio_v2.h>
#include <GPIO_robert.h>

void GPIOPinMuxSetup(unsigned int offsetAddr, unsigned int padConfValue){
    HWREG(SOC_CONTROL_REGS + offsetAddr) = (padConfValue);
}

void GPIO3_ModuleClkConfig(void){

    HWREG(SOC_CM_PER_REGS + CM_PER_GPIO3_CLKCTRL) |=
            CM_PER_GPIO3_CLKCTRL_MODULEMODE_ENABLE;

    HWREG(SOC_CM_PER_REGS + CM_PER_GPIO3_CLKCTRL) |=
            CM_PER_GPIO3_CLKCTRL_OPTFCLKEN_GPIO_3_GDBCLK;

}

void GPIO2_ModuleClkConfig(void){

    HWREG(SOC_CM_PER_REGS + CM_PER_GPIO2_CLKCTRL) |=
            CM_PER_GPIO2_CLKCTRL_MODULEMODE_ENABLE;

    HWREG(SOC_CM_PER_REGS + CM_PER_GPIO2_CLKCTRL) |=
            CM_PER_GPIO2_CLKCTRL_OPTFCLKEN_GPIO_2_GDBCLK;

}

void GPIO1_ModuleClkConfig(void){

    HWREG(SOC_CM_PER_REGS + CM_PER_GPIO1_CLKCTRL) |=
            CM_PER_GPIO1_CLKCTRL_MODULEMODE_ENABLE;

    HWREG(SOC_CM_PER_REGS + CM_PER_GPIO1_CLKCTRL) |=
            CM_PER_GPIO1_CLKCTRL_OPTFCLKEN_GPIO_1_GDBCLK;

}


void GPIO0_ModuleClkConfig(void){

    HWREG(SOC_CM_WKUP_REGS + CM_WKUP_GPIO0_CLKCTRL) |=
            CM_WKUP_GPIO0_CLKCTRL_MODULEMODE_ENABLE;

    HWREG(SOC_CM_WKUP_REGS + CM_WKUP_GPIO0_CLKCTRL) |=
            CM_WKUP_GPIO0_CLKCTRL_OPTFCLKEN_GPIO0_GDBCLK;

}

void gpioModuleClk(int moduleClk){

    switch(moduleClk){
    case MODULE0:
        GPIO0_ModuleClkConfig();
        break;
    case MODULE1:
        GPIO1_ModuleClkConfig();
        break;
    case MODULE2:
        GPIO2_ModuleClkConfig();
        break;
    case MODULE3:
        GPIO3_ModuleClkConfig();
        break;
    }

}

void ledInit(int nGpio, int GPIOModule){

    gpioModuleClk(GPIOModule);

    //CONFIGURAR O PINO gpioPinSelect.c
    switch(GPIOModule){
    case MODULE0:
        modulo0(nGpio);
        break;
    case MODULE1:
        modulo1(nGpio);
        break;
    case MODULE2:
        modulo2(nGpio);
        break;
    case MODULE3:
        modulo3(nGpio);
        break;
    }

    GPIOModuleEnable(GPIO_INSTANCE_ADDRESS(GPIOModule));

    GPIODirModeSet(GPIO_INSTANCE_ADDRESS(GPIOModule),
                   GPIO_INSTANCE_PIN_NUMBER(nGpio),
                   DIR_OUTPUT);
}


void modulo0(int nGpio){
    int num = 0;
    switch(nGpio){
    case GPIO0 ... GPIO1:
    selectMDIO(nGpio);
    break;

    case GPIO2 ... GPIO6:
    selectSPI0(nGpio);
    break;

    case GPIO7:
        GPIOPinMuxSetup(CONTROL_CONF_ECAP0_IN_PWM0_OUT, CONTROL_CONF_MUXMODE(7));
        break;

    case GPIO8 ... GPIO11:
    num = nGpio + 4;
    GPIOPinMuxSetup(CONTROL_CONF_LCD_DATA(num), CONTROL_CONF_MUXMODE(7));
    break;

    case GPIO12 ... GPIO15:
    selectUART(nGpio);
    break;

    case GPIO16:
        selectMII(nGpio);
        break;

    case GPIO17:
        selectMII(nGpio);
        break;

    case GPIO21:
        selectMII(nGpio);
        break;

    case GPIO28:
        selectMII(nGpio);
        break;

    case GPIO22 ... GPIO23:
    num = nGpio - 14;
    GPIOPinMuxSetup(CONTROL_CONF_GPMC_AD(num), CONTROL_CONF_MUXMODE(7));
    break;

    case GPIO26 ... GPIO27:
    num = nGpio - 16;
    GPIOPinMuxSetup(CONTROL_CONF_GPMC_AD(num), CONTROL_CONF_MUXMODE(7));
    break;

    case GPIO18:
        GPIOPinMuxSetup(CONTROL_CONF_USB_DRVVBUS(0), CONTROL_CONF_MUXMODE(7));
        break;

    case GPIO19 ... GPIO20:
    selectXDMA(nGpio);
    break;

    case GPIO29:
        GPIOPinMuxSetup(CONTROL_CONF_RMII1_REFCLK, CONTROL_CONF_MUXMODE(7));
        break;

    case GPIO30:
        GPIOPinMuxSetup(CONTROL_CONF_GPMC_WAIT0, CONTROL_CONF_MUXMODE(7));
        break;

    case GPIO31:
        GPIOPinMuxSetup(CONTROL_CONF_GPMC_WPN, CONTROL_CONF_MUXMODE(7));
        break;

    }
}

void modulo1(int nGpio){
    int num = 0;
    switch(nGpio){
    case GPIO0 ... GPIO7:
    GPIOPinMuxSetup(CONTROL_CONF_GPMC_AD(nGpio), CONTROL_CONF_MUXMODE(7));
    break;

    case GPIO12 ... GPIO15:
    GPIOPinMuxSetup(CONTROL_CONF_GPMC_AD(nGpio), CONTROL_CONF_MUXMODE(7));
    break;

    case GPIO8 ... GPIO11:
    selectUART(nGpio);
    break;

    case GPIO16 ... GPIO27:
    num = nGpio - 16;
    GPIOPinMuxSetup(CONTROL_CONF_GPMC_A(num), CONTROL_CONF_MUXMODE(7));
    break;

    case GPIO28:
        GPIOPinMuxSetup(CONTROL_CONF_GPMC_BE1N, CONTROL_CONF_MUXMODE(7));
        break;

    case GPIO29 ... GPIO31:
    selectCSN(nGpio);
    break;
    }
}

void modulo2(int nGpio){
    int num = 0;
    switch(nGpio){
    case GPIO0 ... GPIO5:
    selectGPMC(nGpio);
    break;

    case GPIO6 ... GPIO17:
    num = nGpio - 6;
    GPIOPinMuxSetup(CONTROL_CONF_LCD_DATA(num), CONTROL_CONF_MUXMODE(7));
    break;

    case GPIO18 ... GPIO21:
    num = nGpio - 18;
    selectMII1RXD(nGpio);
    break;

    case GPIO22 ... GPIO25:
    selectLCD(nGpio);
    break;

    case GPIO26 ... GPIO31:
    selectMMC0(nGpio);
    break;
    }

}

void modulo3(int nGpio){
    switch(nGpio){
    case GPIO0 ... GPIO4:
    selectMII1(nGpio);
    break;

    case GPIO9 ... GPIO10:
    selectMII1(nGpio);
    break;

    case GPIO5 ... GPIO6:
    selectI2C0(nGpio);
    break;

    case GPIO7 ... GPIO8:
    GPIOPinMuxSetup(CONTROL_CONF_EMU(0), CONTROL_CONF_MUXMODE(7));
    break;

    case GPIO13:
        GPIOPinMuxSetup(CONTROL_CONF_USB_DRVVBUS(1), CONTROL_CONF_MUXMODE(7));
        break;

    case GPIO14 ... GPIO21:
    selectMCASPO(nGpio);
    break;
    }

}

void selectUART(int nGpio){
    switch(nGpio){
    //MODULO 1
    case GPIO8:
        GPIOPinMuxSetup(CONTROL_CONF_UART_CTSN(0), CONTROL_CONF_MUXMODE(7));
        break;
    case GPIO9:
        GPIOPinMuxSetup(CONTROL_CONF_UART_RTSN(0), CONTROL_CONF_MUXMODE(7));
        break;
    case GPIO10:
        GPIOPinMuxSetup(CONTROL_CONF_UART_RXD(0), CONTROL_CONF_MUXMODE(7));
        break;
    case GPIO11:
        GPIOPinMuxSetup(CONTROL_CONF_UART_TXD(0), CONTROL_CONF_MUXMODE(7));
        break;
        //FIM MODULO 1

        //MODULO 0
    case GPIO12:
        GPIOPinMuxSetup(CONTROL_CONF_UART_CTSN(1), CONTROL_CONF_MUXMODE(7));
        break;
    case GPIO13:
        GPIOPinMuxSetup(CONTROL_CONF_UART_RTSN(1), CONTROL_CONF_MUXMODE(7));
        break;
    case GPIO14:
        GPIOPinMuxSetup(CONTROL_CONF_UART_RXD(1), CONTROL_CONF_MUXMODE(7));
        break;
    case GPIO15:
        GPIOPinMuxSetup(CONTROL_CONF_UART_TXD(1), CONTROL_CONF_MUXMODE(7));
        break;
        //FIM MODULO 0

    }
}

//MODULO 1
void selectCSN(int nGpio){
    switch(nGpio){
    case GPIO29:
        GPIOPinMuxSetup(CONTROL_CONF_GPMC_CSN(0), CONTROL_CONF_MUXMODE(7));
        break;
    case GPIO30:
        GPIOPinMuxSetup(CONTROL_CONF_GPMC_CSN(1), CONTROL_CONF_MUXMODE(7));
        break;
    case GPIO31:
        GPIOPinMuxSetup(CONTROL_CONF_GPMC_CSN(2), CONTROL_CONF_MUXMODE(7));
        break;
    }
}

// MODULO 0
void selectMII(int nGpio){
    switch(nGpio){
    case GPIO16:
        GPIOPinMuxSetup(CONTROL_CONF_MII1_TXD3, CONTROL_CONF_MUXMODE(7));
        break;
    case GPIO17:
        GPIOPinMuxSetup(CONTROL_CONF_MII1_TXD2, CONTROL_CONF_MUXMODE(7));
        break;
    case GPIO21:
        GPIOPinMuxSetup(CONTROL_CONF_MII1_TXD1, CONTROL_CONF_MUXMODE(7));
        break;
    case GPIO28:
        GPIOPinMuxSetup(CONTROL_CONF_MII1_TXD0, CONTROL_CONF_MUXMODE(7));
        break;
    }

}


//MODULO 0
void selectXDMA(int nGpio){
    switch(nGpio){
    case GPIO19:
        GPIOPinMuxSetup(CONTROL_CONF_XDMA_EVENT_INTR(0), CONTROL_CONF_MUXMODE(7));
        break;
    case GPIO20:
        GPIOPinMuxSetup(CONTROL_CONF_XDMA_EVENT_INTR(1), CONTROL_CONF_MUXMODE(7));
        break;
    }
}


//MODULO 0
void selectMDIO(int nGpio){
    switch(nGpio){
    case GPIO0:
        GPIOPinMuxSetup(CONTROL_CONF_MDIO_DATA, CONTROL_CONF_MUXMODE(7));
        break;
    case GPIO1:
        GPIOPinMuxSetup(CONTROL_CONF_MDIO_CLK, CONTROL_CONF_MUXMODE(7));
        break;
    }
}


//MODULO 0
void selectSPI0(int nGpio){
    switch(nGpio){
    case GPIO2:
        GPIOPinMuxSetup(CONTROL_CONF_SPI0_SCLK, CONTROL_CONF_MUXMODE(7));
        break;
    case GPIO3:
        GPIOPinMuxSetup(CONTROL_CONF_SPI0_D0, CONTROL_CONF_MUXMODE(7));
        break;
    case GPIO4:
        GPIOPinMuxSetup(CONTROL_CONF_SPI0_D1, CONTROL_CONF_MUXMODE(7));
        break;
    case GPIO5:
        GPIOPinMuxSetup(CONTROL_CONF_SPI0_CS0, CONTROL_CONF_MUXMODE(7));
        break;
    case GPIO6:
        GPIOPinMuxSetup(CONTROL_CONF_SPI0_CS1, CONTROL_CONF_MUXMODE(7));
        break;
    }
}

//MODULO 2
void selectGPMC(int nGpio){
    switch(nGpio){
    case GPIO0:
        GPIOPinMuxSetup(CONTROL_CONF_GPMC_CSN(3), CONTROL_CONF_MUXMODE(7));
        break;

    case GPIO1:
        GPIOPinMuxSetup(CONTROL_CONF_GPMC_CLK, CONTROL_CONF_MUXMODE(7));
        break;

    case GPIO2:
        GPIOPinMuxSetup(CONTROL_CONF_GPMC_ADVN_ALE, CONTROL_CONF_MUXMODE(7));
        break;

    case GPIO3:
        GPIOPinMuxSetup(CONTROL_CONF_GPMC_OEN_REN, CONTROL_CONF_MUXMODE(7));
        break;

    case GPIO4:
        GPIOPinMuxSetup(CONTROL_CONF_GPMC_WEN, CONTROL_CONF_MUXMODE(7));
        break;

    case GPIO5:
        GPIOPinMuxSetup(CONTROL_CONF_GPMC_BE0N_CLE, CONTROL_CONF_MUXMODE(7));
        break;
    }
}

//MODULO 2
void selectMII1RXD(int nGpio){
    switch(nGpio){
    case GPIO18:
        GPIOPinMuxSetup(CONTROL_CONF_MII1_RXD3, CONTROL_CONF_MUXMODE(7));
        break;

    case GPIO19:
        GPIOPinMuxSetup(CONTROL_CONF_MII1_RXD2, CONTROL_CONF_MUXMODE(7));
        break;

    case GPIO20:
        GPIOPinMuxSetup(CONTROL_CONF_MII1_RXD1, CONTROL_CONF_MUXMODE(7));
        break;

    case GPIO21:
        GPIOPinMuxSetup(CONTROL_CONF_MII1_RXD0, CONTROL_CONF_MUXMODE(7));
        break;
    }
}

//MODULO 2
void selectLCD(int nGpio){
    switch(nGpio){
    case GPIO22:
        GPIOPinMuxSetup(CONTROL_CONF_LCD_VSYNC, CONTROL_CONF_MUXMODE(7));
        break;

    case GPIO23:
        GPIOPinMuxSetup(CONTROL_CONF_LCD_HSYNC, CONTROL_CONF_MUXMODE(7));
        break;

    case GPIO24:
        GPIOPinMuxSetup(CONTROL_CONF_LCD_PCLK, CONTROL_CONF_MUXMODE(7));
        break;

    case GPIO25:
        GPIOPinMuxSetup(CONTROL_CONF_LCD_AC_BIAS_EN, CONTROL_CONF_MUXMODE(7));
        break;
    }
}

//MODULO 2
void selectMMC0(int nGpio){
    switch(nGpio){
    case GPIO26:
        GPIOPinMuxSetup(CONTROL_CONF_MMC0_DAT3 , CONTROL_CONF_MUXMODE(7));
        break;

    case GPIO27:
        GPIOPinMuxSetup(CONTROL_CONF_MMC0_DAT2 , CONTROL_CONF_MUXMODE(7));
        break;

    case GPIO28:
        GPIOPinMuxSetup(CONTROL_CONF_MMC0_DAT1, CONTROL_CONF_MUXMODE(7));
        break;

    case GPIO29:
        GPIOPinMuxSetup(CONTROL_CONF_MMC0_DAT0 , CONTROL_CONF_MUXMODE(7));
        break;

    case GPIO30:
        GPIOPinMuxSetup(CONTROL_CONF_MMC0_CLK, CONTROL_CONF_MUXMODE(7));
        break;

    case GPIO31:
        GPIOPinMuxSetup(CONTROL_CONF_MMC0_CMD, CONTROL_CONF_MUXMODE(7));
        break;
    }
}

//MODULO 3
void selectMII1(int nGpio){
    switch(nGpio){
    case GPIO0:
        GPIOPinMuxSetup(CONTROL_CONF_MII1_COL, CONTROL_CONF_MUXMODE(7));
        break;

    case GPIO1:
        GPIOPinMuxSetup(CONTROL_CONF_MII1_CRS , CONTROL_CONF_MUXMODE(7));
        break;

    case GPIO2:
        GPIOPinMuxSetup(CONTROL_CONF_MII1_RXERR, CONTROL_CONF_MUXMODE(7));
        break;

    case GPIO3:
        GPIOPinMuxSetup(CONTROL_CONF_MII1_TXEN, CONTROL_CONF_MUXMODE(7));
        break;

    case GPIO4:
        GPIOPinMuxSetup(CONTROL_CONF_MII1_RXDV, CONTROL_CONF_MUXMODE(7));
        break;

    case GPIO9:
        GPIOPinMuxSetup(CONTROL_CONF_MII1_TXCLK, CONTROL_CONF_MUXMODE(7));
        break;

    case GPIO10:
        GPIOPinMuxSetup(CONTROL_CONF_MII1_RXCLK, CONTROL_CONF_MUXMODE(7));
        break;
    }
}

//MODULO 3
void selectI2C0(int nGpio){
    switch(nGpio){
    case GPIO7:
        GPIOPinMuxSetup(CTRL_CONF_I2C0_SDA, CONTROL_CONF_MUXMODE(7));
        break;

    case GPIO8:
        GPIOPinMuxSetup(CONTROL_CONF_I2C0_SCL, CONTROL_CONF_MUXMODE(7));
        break;

    }
}

//MODULO 3
void selectMCASPO(int nGpio){
    switch(nGpio){
    case GPIO14:
        GPIOPinMuxSetup(CONTROL_CONF_MCASP0_ACLKX, CONTROL_CONF_MUXMODE(7));
        break;

    case GPIO15:
        GPIOPinMuxSetup( CONTROL_CONF_MCASP0_FSX, CONTROL_CONF_MUXMODE(7));
        break;

    case GPIO16:
        GPIOPinMuxSetup(CONTROL_CONF_MCASP0_AXR0, CONTROL_CONF_MUXMODE(7));
        break;

    case GPIO17:
        GPIOPinMuxSetup(CONTROL_CONF_MCASP0_AHCLKR, CONTROL_CONF_MUXMODE(7));
        break;

    case GPIO18:
        GPIOPinMuxSetup(CONTROL_CONF_MCASP0_ACLKR, CONTROL_CONF_MUXMODE(7));
        break;

    case GPIO19:
        GPIOPinMuxSetup(CONTROL_CONF_MCASP0_FSR, CONTROL_CONF_MUXMODE(7));
        break;

    case GPIO20:
        GPIOPinMuxSetup(CONTROL_CONF_MCASP0_AXR1, CONTROL_CONF_MUXMODE(7));
        break;

    case GPIO21:
        GPIOPinMuxSetup(CONTROL_CONF_MCASP0_AHCLKX, CONTROL_CONF_MUXMODE(7));
        break;
    }
}

void ledToggle(int nGpio, int GPIOModule){

    flagToggle^=TOGGLE;

    if(flagToggle){
        /* Driving a logic HIGH on the GPIO pin. */
        GPIOPinWrite(GPIO_INSTANCE_ADDRESS(GPIOModule),
                     GPIO_INSTANCE_PIN_NUMBER(nGpio),
                     GPIO_PIN_HIGH);
    }else{
        /* Driving a logic LOW on the GPIO pin. */
        GPIOPinWrite(GPIO_INSTANCE_ADDRESS(GPIOModule),
                     GPIO_INSTANCE_PIN_NUMBER(nGpio),
                     GPIO_PIN_LOW);
    }
}
