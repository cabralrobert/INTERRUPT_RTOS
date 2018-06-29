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


#include <ti/board/board.h>

#include <ti/drv/uart/UART.h>
#include <ti/drv/uart/UART_stdio.h>
#include <ADC.h>

void TSCADCModuleClkConfig(void)
{
    /* Configuring L3 Interface Clocks. */

    /* Writing to MODULEMODE field of CM_PER_L3_CLKCTRL register. */
    HWREG(SOC_CM_PER_REGS + CM_PER_L3_CLKCTRL) |=
          CM_PER_L3_CLKCTRL_MODULEMODE_ENABLE;

    /* Waiting for MODULEMODE field to reflect the written value. */
    while(CM_PER_L3_CLKCTRL_MODULEMODE_ENABLE !=
          (HWREG(SOC_CM_PER_REGS + CM_PER_L3_CLKCTRL) &
           CM_PER_L3_CLKCTRL_MODULEMODE));

    /* Writing to MODULEMODE field of CM_PER_L3_INSTR_CLKCTRL register. */
    HWREG(SOC_CM_PER_REGS + CM_PER_L3_INSTR_CLKCTRL) |=
          CM_PER_L3_INSTR_CLKCTRL_MODULEMODE_ENABLE;

    /* Waiting for MODULEMODE field to reflect the written value. */
    while(CM_PER_L3_INSTR_CLKCTRL_MODULEMODE_ENABLE !=
          (HWREG(SOC_CM_PER_REGS + CM_PER_L3_INSTR_CLKCTRL) &
           CM_PER_L3_INSTR_CLKCTRL_MODULEMODE));

    /* Writing to CLKTRCTRL field of CM_PER_L3_CLKSTCTRL register. */
    HWREG(SOC_CM_PER_REGS + CM_PER_L3_CLKSTCTRL) |=
          CM_PER_L3_CLKSTCTRL_CLKTRCTRL_SW_WKUP;

    /* Waiting for CLKTRCTRL field to reflect the written value. */
    while(CM_PER_L3_CLKSTCTRL_CLKTRCTRL_SW_WKUP !=
          (HWREG(SOC_CM_PER_REGS + CM_PER_L3_CLKSTCTRL) &
           CM_PER_L3_CLKSTCTRL_CLKTRCTRL));

    /* Writing to CLKTRCTRL field of CM_PER_OCPWP_L3_CLKSTCTRL register. */
    HWREG(SOC_CM_PER_REGS + CM_PER_OCPWP_L3_CLKSTCTRL) |=
          CM_PER_OCPWP_L3_CLKSTCTRL_CLKTRCTRL_SW_WKUP;

    /*Waiting for CLKTRCTRL field to reflect the written value. */
    while(CM_PER_OCPWP_L3_CLKSTCTRL_CLKTRCTRL_SW_WKUP !=
          (HWREG(SOC_CM_PER_REGS + CM_PER_OCPWP_L3_CLKSTCTRL) &
           CM_PER_OCPWP_L3_CLKSTCTRL_CLKTRCTRL));

    /* Writing to CLKTRCTRL field of CM_PER_L3S_CLKSTCTRL register. */
    HWREG(SOC_CM_PER_REGS + CM_PER_L3S_CLKSTCTRL) |=
          CM_PER_L3S_CLKSTCTRL_CLKTRCTRL_SW_WKUP;

    /*Waiting for CLKTRCTRL field to reflect the written value. */
    while(CM_PER_L3S_CLKSTCTRL_CLKTRCTRL_SW_WKUP !=
          (HWREG(SOC_CM_PER_REGS + CM_PER_L3S_CLKSTCTRL) &
           CM_PER_L3S_CLKSTCTRL_CLKTRCTRL));

    /* Checking fields for necessary values.  */

    /* Waiting for IDLEST field in CM_PER_L3_CLKCTRL register to be set to 0x0. */
    while((CM_PER_L3_CLKCTRL_IDLEST_FUNC << CM_PER_L3_CLKCTRL_IDLEST_SHIFT)!=
          (HWREG(SOC_CM_PER_REGS + CM_PER_L3_CLKCTRL) & CM_PER_L3_CLKCTRL_IDLEST));

    /*
    ** Waiting for IDLEST field in CM_PER_L3_INSTR_CLKCTRL register to attain the
    ** desired value.
    */
    while((CM_PER_L3_INSTR_CLKCTRL_IDLEST_FUNC <<
           CM_PER_L3_INSTR_CLKCTRL_IDLEST_SHIFT)!=
          (HWREG(SOC_CM_PER_REGS + CM_PER_L3_INSTR_CLKCTRL) &
           CM_PER_L3_INSTR_CLKCTRL_IDLEST));

    /*
    ** Waiting for CLKACTIVITY_L3_GCLK field in CM_PER_L3_CLKSTCTRL register to
    ** attain the desired value.
    */
    while(CM_PER_L3_CLKSTCTRL_CLKACTIVITY_L3_GCLK !=
          (HWREG(SOC_CM_PER_REGS + CM_PER_L3_CLKSTCTRL) &
           CM_PER_L3_CLKSTCTRL_CLKACTIVITY_L3_GCLK));

    /*
    ** Waiting for CLKACTIVITY_OCPWP_L3_GCLK field in CM_PER_OCPWP_L3_CLKSTCTRL
    ** register to attain the desired value.
    */
    while(CM_PER_OCPWP_L3_CLKSTCTRL_CLKACTIVITY_OCPWP_L3_GCLK !=
          (HWREG(SOC_CM_PER_REGS + CM_PER_OCPWP_L3_CLKSTCTRL) &
           CM_PER_OCPWP_L3_CLKSTCTRL_CLKACTIVITY_OCPWP_L3_GCLK));

    /*
    ** Waiting for CLKACTIVITY_L3S_GCLK field in CM_PER_L3S_CLKSTCTRL register
    ** to attain the desired value.
    */
    while(CM_PER_L3S_CLKSTCTRL_CLKACTIVITY_L3S_GCLK !=
          (HWREG(SOC_CM_PER_REGS + CM_PER_L3S_CLKSTCTRL) &
          CM_PER_L3S_CLKSTCTRL_CLKACTIVITY_L3S_GCLK));


    /* Configuring registers related to Wake-Up region. */

    /* Writing to MODULEMODE field of CM_WKUP_CONTROL_CLKCTRL register. */
    HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CONTROL_CLKCTRL) |=
          CM_WKUP_CONTROL_CLKCTRL_MODULEMODE_ENABLE;

    /* Waiting for MODULEMODE field to reflect the written value. */
    while(CM_WKUP_CONTROL_CLKCTRL_MODULEMODE_ENABLE !=
          (HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CONTROL_CLKCTRL) &
           CM_WKUP_CONTROL_CLKCTRL_MODULEMODE));

    /* Writing to CLKTRCTRL field of CM_PER_L3S_CLKSTCTRL register. */
    HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CLKSTCTRL) |=
          CM_WKUP_CLKSTCTRL_CLKTRCTRL_SW_WKUP;

    /*Waiting for CLKTRCTRL field to reflect the written value. */
    while(CM_WKUP_CLKSTCTRL_CLKTRCTRL_SW_WKUP !=
          (HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CLKSTCTRL) &
           CM_WKUP_CLKSTCTRL_CLKTRCTRL));

    /* Writing to CLKTRCTRL field of CM_L3_AON_CLKSTCTRL register. */
    HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_L3_AON_CLKSTCTRL) |=
          CM_WKUP_CM_L3_AON_CLKSTCTRL_CLKTRCTRL_SW_WKUP;

    /*Waiting for CLKTRCTRL field to reflect the written value. */
    while(CM_WKUP_CM_L3_AON_CLKSTCTRL_CLKTRCTRL_SW_WKUP !=
          (HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_L3_AON_CLKSTCTRL) &
           CM_WKUP_CM_L3_AON_CLKSTCTRL_CLKTRCTRL));

    /* Writing to MODULEMODE field of CM_WKUP_TSC_CLKCTRL register. */
    HWREG(SOC_CM_WKUP_REGS + CM_WKUP_ADC_TSC_CLKCTRL) |=
          CM_WKUP_ADC_TSC_CLKCTRL_MODULEMODE_ENABLE;

    /* Waiting for MODULEMODE field to reflect the written value. */
    while(CM_WKUP_ADC_TSC_CLKCTRL_MODULEMODE_ENABLE !=
          (HWREG(SOC_CM_WKUP_REGS + CM_WKUP_ADC_TSC_CLKCTRL) &
           CM_WKUP_ADC_TSC_CLKCTRL_MODULEMODE));

    /* Verifying if the other bits are set to required settings. */

    /*
    ** Waiting for IDLEST field in CM_WKUP_CONTROL_CLKCTRL register to attain
    ** desired value.
    */
    while((CM_WKUP_CONTROL_CLKCTRL_IDLEST_FUNC <<
           CM_WKUP_CONTROL_CLKCTRL_IDLEST_SHIFT) !=
          (HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CONTROL_CLKCTRL) &
           CM_WKUP_CONTROL_CLKCTRL_IDLEST));

    /*
    ** Waiting for CLKACTIVITY_L3_AON_GCLK field in CM_L3_AON_CLKSTCTRL
    ** register to attain desired value.
    */
    while(CM_WKUP_CM_L3_AON_CLKSTCTRL_CLKACTIVITY_L3_AON_GCLK !=
          (HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_L3_AON_CLKSTCTRL) &
           CM_WKUP_CM_L3_AON_CLKSTCTRL_CLKACTIVITY_L3_AON_GCLK));

    /*
    ** Waiting for IDLEST field in CM_WKUP_L4WKUP_CLKCTRL register to attain
    ** desired value.
    */
    while((CM_WKUP_L4WKUP_CLKCTRL_IDLEST_FUNC <<
           CM_WKUP_L4WKUP_CLKCTRL_IDLEST_SHIFT) !=
          (HWREG(SOC_CM_WKUP_REGS + CM_WKUP_L4WKUP_CLKCTRL) &
           CM_WKUP_L4WKUP_CLKCTRL_IDLEST));

    /*
    ** Waiting for CLKACTIVITY_L4_WKUP_GCLK field in CM_WKUP_CLKSTCTRL register
    ** to attain desired value.
    */
    while(CM_WKUP_CLKSTCTRL_CLKACTIVITY_L4_WKUP_GCLK !=
          (HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CLKSTCTRL) &
           CM_WKUP_CLKSTCTRL_CLKACTIVITY_L4_WKUP_GCLK));

    /*
    ** Waiting for CLKACTIVITY_L4_WKUP_AON_GCLK field in CM_L4_WKUP_AON_CLKSTCTRL
    ** register to attain desired value.
    */
    while(CM_WKUP_CM_L4_WKUP_AON_CLKSTCTRL_CLKACTIVITY_L4_WKUP_AON_GCLK !=
          (HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_L4_WKUP_AON_CLKSTCTRL) &
           CM_WKUP_CM_L4_WKUP_AON_CLKSTCTRL_CLKACTIVITY_L4_WKUP_AON_GCLK));

    /*
    ** Waiting for CLKACTIVITY_ADC_FCLK field in CM_WKUP_CLKSTCTRL
    ** register to attain desired value.
    */
    while(CM_WKUP_CLKSTCTRL_CLKACTIVITY_ADC_FCLK !=
          (HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CLKSTCTRL) &
           CM_WKUP_CLKSTCTRL_CLKACTIVITY_ADC_FCLK));

    /*
    ** Waiting for IDLEST field in CM_WKUP_ADC_TSC_CLKCTRL register to attain
    ** desired value.
    */
    while((CM_WKUP_ADC_TSC_CLKCTRL_IDLEST_FUNC <<
           CM_WKUP_ADC_TSC_CLKCTRL_IDLEST_SHIFT) !=
          (HWREG(SOC_CM_WKUP_REGS + CM_WKUP_ADC_TSC_CLKCTRL) &
           CM_WKUP_ADC_TSC_CLKCTRL_IDLEST));
}


unsigned int TSCADCPinMuxSetUp(void)
{

    HWREG( SOC_CONTROL_REGS + CONTROL_CONF_AIN0) = CONTROL_CONF_AIN0_CONF_AIN0_RXACTIVE;

    HWREG( SOC_CONTROL_REGS + CONTROL_CONF_AIN1) = CONTROL_CONF_AIN1_CONF_AIN1_RXACTIVE;

    HWREG( SOC_CONTROL_REGS +  CONTROL_CONF_AIN2)= CONTROL_CONF_AIN2_CONF_AIN2_RXACTIVE;

    HWREG( SOC_CONTROL_REGS + CONTROL_CONF_AIN3) = CONTROL_CONF_AIN3_CONF_AIN3_RXACTIVE;

    HWREG( SOC_CONTROL_REGS + CONTROL_CONF_AIN4) = CONTROL_CONF_AIN4_CONF_AIN4_RXACTIVE;

    HWREG( SOC_CONTROL_REGS + CONTROL_CONF_AIN5) = CONTROL_CONF_AIN5_CONF_AIN5_RXACTIVE;

    HWREG( SOC_CONTROL_REGS + CONTROL_CONF_AIN6) = CONTROL_CONF_AIN6_CONF_AIN6_RXACTIVE;

    HWREG( SOC_CONTROL_REGS + CONTROL_CONF_AIN7) = CONTROL_CONF_AIN7_CONF_AIN7_RXACTIVE;

    HWREG( SOC_CONTROL_REGS + CONTROL_CONF_VREFP)= CONTROL_CONF_VREFP_CONF_VREFP_RXACTIVE;

    HWREG( SOC_CONTROL_REGS +  CONTROL_CONF_VREFN)= CONTROL_CONF_VREFN_CONF_VREFN_RXACTIVE;
    return TRUE;
}

void TSCADCConfigureAFEClock(unsigned int baseAdd, unsigned int moduleClk,
                          unsigned int inputClk)
{
    unsigned int clkDiv;

    clkDiv = moduleClk / inputClk;

    HWREG(baseAdd + TSC_ADC_SS_ADC_CLKDIV) &=
                        ~TSC_ADC_SS_ADC_CLKDIV_ADC_CLK_DIV;

    HWREG(baseAdd + TSC_ADC_SS_ADC_CLKDIV) = (clkDiv - 1);
}

void TSCADCTSTransistorConfig(unsigned int baseAdd, unsigned int enableTSTransistor)
{
    HWREG(baseAdd + TSC_ADC_SS_CTRL) &= ~TSC_ADC_SS_CTRL_TOUCH_SCREEN_ENABLE;

    HWREG(baseAdd + TSC_ADC_SS_CTRL) |= enableTSTransistor <<
                                   TSC_ADC_SS_CTRL_TOUCH_SCREEN_ENABLE_SHIFT;
}

void TSCADCStepIDTagConfig(unsigned int baseAdd, unsigned int enableStepIDTag)
{
    HWREG(baseAdd + TSC_ADC_SS_CTRL) &= ~TSC_ADC_SS_CTRL_STERP_ID_TAG;
    HWREG(baseAdd + TSC_ADC_SS_CTRL) |= enableStepIDTag <<
                                     TSC_ADC_SS_CTRL_STERP_ID_TAG_SHIFT;
}

void TSCADCStepConfigProtectionDisable(unsigned int baseAdd)
{
    HWREG(baseAdd + TSC_ADC_SS_CTRL) |=
                      TSC_ADC_SS_CTRL_STEPCONFIG_WRITEPROTECT_N;
}

void TSCADCTSModeConfig(unsigned int baseAdd, unsigned int tsMode)
{
    HWREG(baseAdd + TSC_ADC_SS_CTRL) &= ~TSC_ADC_SS_CTRL_AFE_PEN_CTRL;

    HWREG(baseAdd + TSC_ADC_SS_CTRL) |= tsMode <<
                                   TSC_ADC_SS_CTRL_AFE_PEN_CTRL_SHIFT;
}

void TSCADCConfigureStepEnable(unsigned int baseAdd, unsigned int stepSel,
                        unsigned int stepEn_Dis)
{
    HWREG(baseAdd + TSC_ADC_SS_STEPENABLE) &= ~(1 << stepSel);
    HWREG(baseAdd + TSC_ADC_SS_STEPENABLE) |= stepEn_Dis << stepSel;
}

void TSCADCModuleStateSet(unsigned int baseAdd, unsigned int enableModule)
{
    HWREG(baseAdd + TSC_ADC_SS_CTRL) &= ~TSC_ADC_SS_CTRL_ENABLE;
    HWREG(baseAdd + TSC_ADC_SS_CTRL) |= enableModule;
}

void TSCADCTSStepOperationModeControl(unsigned int baseAdd, unsigned int mode,
                                   unsigned int stepSelect)
{
    if(mode)
    {
         HWREG(baseAdd + TSC_ADC_SS_STEPCONFIG(stepSelect)) |=
                                      TSC_ADC_SS_STEPCONFIG_DIFF_CNTRL;
    }
    else
    {
         HWREG(baseAdd + TSC_ADC_SS_STEPCONFIG(stepSelect)) &=
                                    ~TSC_ADC_SS_STEPCONFIG_DIFF_CNTRL;
    }
}

void TSCADCTSStepConfig(unsigned int baseAdd, unsigned int stepSelect,
                unsigned int adcNegativeRef, unsigned int adcPositiveInp,
                        unsigned int adcNegativeInp, unsigned int adcPositiveRef)
{

    HWREG(baseAdd + TSC_ADC_SS_STEPCONFIG(stepSelect)) &=
                                   ~TSC_ADC_SS_STEPCONFIG_SEL_RFM_SWC;

    HWREG(baseAdd + TSC_ADC_SS_STEPCONFIG(stepSelect)) |=
                   adcNegativeRef << TSC_ADC_SS_STEPCONFIG_SEL_RFM_SWC_SHIFT;

    HWREG(baseAdd + TSC_ADC_SS_STEPCONFIG(stepSelect)) &=
                               ~TSC_ADC_SS_STEPCONFIG_SEL_INP_SWC;

    HWREG(baseAdd + TSC_ADC_SS_STEPCONFIG(stepSelect)) |=
           adcPositiveInp << TSC_ADC_SS_STEPCONFIG_SEL_INP_SWC_SHIFT;

    HWREG(baseAdd + TSC_ADC_SS_STEPCONFIG(stepSelect)) &=
                                   ~TSC_ADC_SS_STEPCONFIG_SEL_INM_SWM;

    HWREG(baseAdd + TSC_ADC_SS_STEPCONFIG(stepSelect)) |=
                         adcNegativeInp << TSC_ADC_SS_STEPCONFIG_SEL_INM_SWM_SHIFT;

    HWREG(baseAdd + TSC_ADC_SS_STEPCONFIG(stepSelect)) &=
                               ~TSC_ADC_SS_STEPCONFIG_SEL_RFP_SWC;

    HWREG(baseAdd + TSC_ADC_SS_STEPCONFIG(stepSelect)) |=
                          adcPositiveRef << TSC_ADC_SS_STEPCONFIG_SEL_RFP_SWC_SHIFT;
}

void TSCADCTSStepFIFOSelConfig(unsigned int baseAdd, unsigned int stepSel,
                        unsigned int FIFOSel)
{
    HWREG(baseAdd + TSC_ADC_SS_STEPCONFIG(stepSel)) &=
                          ~TSC_ADC_SS_STEPCONFIG_FIFO_SELECT;

    HWREG(baseAdd + TSC_ADC_SS_STEPCONFIG(stepSel)) |=
                           FIFOSel << TSC_ADC_SS_STEPCONFIG_FIFO_SELECT_SHIFT;
}

void TSCADCTSStepModeConfig(unsigned int baseAdd, unsigned int stepSel,
                     unsigned int mode)
{
    HWREG(baseAdd + TSC_ADC_SS_STEPCONFIG(stepSel))
           &= ~TSC_ADC_SS_STEPCONFIG_MODE;
    HWREG(baseAdd + TSC_ADC_SS_STEPCONFIG(stepSel))
           |= mode << TSC_ADC_SS_STEPCONFIG_MODE_SHIFT;
}

unsigned int TSCADCFIFOADCDataRead(unsigned int baseAdd, unsigned int FIFOSel)
{
    return (HWREG(baseAdd + TSC_ADC_SS_FIFODATA(FIFOSel)) &
                        TSC_ADC_SS_FIFODATA_ADC_DATA);
}

void ADCConfigure(void) {
    /* Enable the clock for touch screen */
        TSCADCModuleClkConfig();

        TSCADCPinMuxSetUp();

        /* Configures ADC to 3Mhz */
        TSCADCConfigureAFEClock(SOC_ADC_TSC_0_REGS, 24000000, 3000000);

        /* Enable Transistor bias */
        TSCADCTSTransistorConfig(SOC_ADC_TSC_0_REGS, TSCADC_TRANSISTOR_ENABLE);

        TSCADCStepIDTagConfig(SOC_ADC_TSC_0_REGS, 1);

        /* Disable Write Protection of Step Configuration regs*/
        TSCADCStepConfigProtectionDisable(SOC_ADC_TSC_0_REGS);

        /* Configure step 1 for channel 1(AN0)*/
        StepConfigure(0, TSCADC_FIFO_0, TSCADC_POSITIVE_INP_CHANNEL1);

        /* General purpose inputs */
        TSCADCTSModeConfig(SOC_ADC_TSC_0_REGS, TSCADC_GENERAL_PURPOSE_MODE);

        /* Enable step 1 */
        TSCADCConfigureStepEnable(SOC_ADC_TSC_0_REGS, 1, 1);

        /* Enable the TSC_ADC_SS module*/
        TSCADCModuleStateSet(SOC_ADC_TSC_0_REGS, TSCADC_MODULE_ENABLE);
}

void StepConfigure(unsigned int stepSel, unsigned int fifo,
                   unsigned int positiveInpChannel){
        /* Configure ADC to Single ended operation mode */
        TSCADCTSStepOperationModeControl(SOC_ADC_TSC_0_REGS,
                                  TSCADC_SINGLE_ENDED_OPER_MODE , stepSel);

        /* Configure step to select Channel, refernce voltages */
        TSCADCTSStepConfig(SOC_ADC_TSC_0_REGS, stepSel, TSCADC_NEGATIVE_REF_VSSA,
                    positiveInpChannel, TSCADC_NEGATIVE_INP_CHANNEL1, TSCADC_POSITIVE_REF_VDDA);

    /* XPPSW Pin is on, Which pull up the AN0 line*/
        //TSCADCTSStepAnalogSupplyConfig(SOC_ADC_TSC_0_REGS, TSCADC_XPPSW_PIN_ON, TSCADC_XNPSW_PIN_OFF,
        //                        TSCADC_YPPSW_PIN_OFF, stepSel);

        /* XNNSW Pin is on, Which pull down the AN1 line*/
        //TSCADCTSStepAnalogGroundConfig(SOC_ADC_TSC_0_REGS, TSCADC_XNNSW_PIN_ON, TSCADC_YPNSW_PIN_OFF,
        //                        TSCADC_YNNSW_PIN_OFF,  TSCADC_WPNSW_PIN_OFF, stepSel);

        /* select fifo 0 or 1*/
        TSCADCTSStepFIFOSelConfig(SOC_ADC_TSC_0_REGS, stepSel, fifo);

        /* Configure ADC to one short mode */
        TSCADCTSStepModeConfig(SOC_ADC_TSC_0_REGS, stepSel, TSCADC_CONTINIOUS_SOFTWARE_ENABLED);
}
