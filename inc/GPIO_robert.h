#ifndef GPIO_H_
#define GPIO_H_

/* This is used to configure a GPIO pin as an input pin. */
#define DIR_INPUT                1
/* This is used to configure a GPIO pin as an output pin.*/
#define DIR_OUTPUT               0

/* This is used to write a logic 0 to a pin.*/
#define PIN_LOW                  0

/* This is used to write a logic 1 to a pin.*/
#define PIN_HIGH                 1

static int flagToggle = 0;

#define GPIO_INSTANCE_PIN_NUMBER(n)     (n)

#define GPIO_INSTANCE_ADDRESS(n)    ( (n==0) ? SOC_GPIO_0_REGS : (n==1) ? SOC_GPIO_1_REGS : (n==2) ? SOC_GPIO_2_REGS: SOC_GPIO_3_REGS)

#define TOGGLE                          (0x01u)

enum moduleClk{MODULE0, MODULE1, MODULE2, MODULE3}moduleClk;

enum pin{GPIO0, GPIO1, GPIO2, GPIO3, GPIO4, GPIO5, GPIO6,
    GPIO7, GPIO8, GPIO9, GPIO10, GPIO11, GPIO12, GPIO13,
    GPIO14, GPIO15, GPIO16, GPIO17, GPIO18, GPIO19, GPIO20,
    GPIO21, GPIO22, GPIO23, GPIO24, GPIO25, GPIO26, GPIO27,
    GPIO28, GPIO29, GPIO30, GPIO31}pin;

#define HWREGS(x) (*((volatile unsigned int *)(x)))

    enum data{D0, D1, D2, D3, D4, D5, D6, D7}data;

    void GPIOPinMuxSetup(unsigned int offsetAddr, unsigned int padConfValue);
    void GPIO3_ModuleClkConfig(void);
    void GPIO2_ModuleClkConfig(void);
    void GPIO1_ModuleClkConfig(void);
    void GPIO0_ModuleClkConfig(void);
    void gpioModuleClk(int moduleClk);
    void ledInit(int nGpio, int GPIOModule);
    void modulo0(int nGpio);
    void modulo1(int nGpio);
    void modulo2(int nGpio);
    void modulo3(int nGpio);
    void selectUART(int nGpio);
    void selectCSN(int nGpio);
    void selectMII(int nGpio);
    void selectXDMA(int nGpio);
    void selectMDIO(int nGpio);
    void selectSPI0(int nGpio);
    void selectGPMC(int nGpio);
    void selectMII1RXD(int nGpio);
    void selectLCD(int nGpio);
    void selectMMC0(int nGpio);
    void selectMII1(int nGpio);
    void selectI2C0(int nGpio);
    void selectMCASPO(int nGpio);
    void ledToggle(int nGpio, int GPIOModule);

#endif
