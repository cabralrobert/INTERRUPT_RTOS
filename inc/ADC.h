#ifndef ADC_H_
#define ADC_H_

#include <ti/starterware/include/hw/hw_types.h>
#include <ti/starterware/include/hw/soc_am335x.h>
#include <ti/starterware/include/hw/hw_control_am335x.h>
#include <ti/starterware/include/hw/hw_cm_per.h>
#include <ti/starterware/include/hw/hw_cm_wkup.h>
#include <ti/starterware/include/hw/hw_tsc_adc_ss.h>
#include <ti/starterware/include/tsc_adc.h>

#define     RESOL_VREF                  (4095)

#define     TIMER_INITIAL_COUNT            (0xFF000000u)
#define     TIMER_RLD_COUNT                (0xFFFFFF83u) //(0xFF000000u)

#define     OVERFLOW                        (0xFFFFFFFFu)
#define     TIMER_1MS_COUNT                 (0x5DC0u)

#define HWREGS(x) (*((volatile unsigned int *)(x)))

void TSCADCModuleClkConfig(void);
unsigned int TSCADCPinMuxSetUp(void);
void TSCADCConfigureAFEClock(unsigned int baseAdd, unsigned int moduleClk,unsigned int inputClk);
void TSCADCTSTransistorConfig(unsigned int baseAdd, unsigned int enableTSTransistor);
void TSCADCStepIDTagConfig(unsigned int baseAdd, unsigned int enableStepIDTag);
void TSCADCStepConfigProtectionDisable(unsigned int baseAdd);
void TSCADCTSModeConfig(unsigned int baseAdd, unsigned int tsMode);
void TSCADCConfigureStepEnable(unsigned int baseAdd, unsigned int stepSel,unsigned int stepEn_Dis);
void TSCADCModuleStateSet(unsigned int baseAdd, unsigned int enableModule);
void TSCADCTSStepOperationModeControl(unsigned int baseAdd, unsigned int mode,unsigned int stepSelect);
void TSCADCTSStepConfig(unsigned int baseAdd, unsigned int stepSelect,
                unsigned int adcNegativeRef, unsigned int adcPositiveInp,
                        unsigned int adcNegativeInp, unsigned int adcPositiveRef);
void TSCADCTSStepFIFOSelConfig(unsigned int baseAdd, unsigned int stepSel,
                        unsigned int FIFOSel);
void TSCADCTSStepModeConfig(unsigned int baseAdd, unsigned int stepSel, unsigned int mode);
unsigned int TSCADCFIFOADCDataRead(unsigned int baseAdd, unsigned int FIFOSel);
void ADCConfigure(void);
void StepConfigure(unsigned int, unsigned int, unsigned int);

#endif
