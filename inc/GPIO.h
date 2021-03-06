/*
 * Copyright (c) 2014-2015, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/** ============================================================================
 *  @file       GPIO.h
 *
 *  @brief      GPIO driver
 *
 *  The GPIO header file should be included in an application as follows:
 *  @code
 *  #include <ti/drv/gpio/GPIO.h>
 *  @endcode
 *
 *  # Operation #
 *
 *  The GPIO module allows you to manage General Purpose I/O pins via
 *  simple and portable APIs. GPIO pin behaviour can be configured completely
 *  statically, or dynamically defined at runtime.
 *
 *  The application is required to supply a device
 *  specific GPIOXXX_Config structure to the module. This structure
 *  communicates to the GPIO module how to configure the pins that will be used
 *  by the application (See the description of GPIO_PinConfig).
 *
 *  The application is required to call GPIO_init(). This function will
 *  initialize all the GPIO pins defined in the GPIO_PinConfig table to the
 *  configurations specified. Once that is completed the other APIs can be
 *  used to access the pins.
 *
 *  Asserts are used to verify that the driver has been initialized and
 *  to validate pin indexes within the various APIs.
 *
 *  See the device specific GPIO header file for configuration details.
 *
 *  ============================================================================
 */

#ifndef GPIO__H
#define GPIO__H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*!
 *  @brief  GPIO pin configuration settings
 *
 *  The upper 16 bits of the 32 bit PinConfig is reserved
 *  for pin configuration settings.
 *
 *  The lower 16 bits are reserved for device-specific
 *  port/pin identifications
 */
typedef uint32_t GPIO_PinConfig;

/*!
 *  @cond NODOC
 *  Internally used configuration bit access macros.
 */
#define GPIO_CFG_IO_MASK           0x00ff0000U
#define GPIO_CFG_IO_LSB            16U
#define GPIO_CFG_OUT_TYPE_MASK     0x00060000U
#define GPIO_CFG_OUT_TYPE_LSB      17U
#define GPIO_CFG_IN_TYPE_MASK      0x00060000U
#define GPIO_CFG_IN_TYPE_LSB       17U
#define GPIO_CFG_OUT_STRENGTH_MASK 0x00f00000U
#define GPIO_CFG_OUT_STRENGTH_LSB  20U
#define GPIO_CFG_INT_MASK          0x07000000U
#define GPIO_CFG_INT_LSB           24U
#define GPIO_CFG_OUT_BIT           19U



/*! @endcond */

/** @name GPIO_PinConfig pin direction configuration macros
 *  @{
 */
#define GPIO_CFG_OUTPUT            (((uint32_t) 0) << GPIO_CFG_IO_LSB) /*! Pin is an output. */
#define GPIO_CFG_OUT_STD           (((uint32_t) 0) << GPIO_CFG_IO_LSB) /*! Output pin is not Open Drain */
#define GPIO_CFG_OUT_OD_NOPULL     (((uint32_t) 2) << GPIO_CFG_IO_LSB) /*! Output pin is Open Drain */
#define GPIO_CFG_OUT_OD_PU         (((uint32_t) 4) << GPIO_CFG_IO_LSB) /*! Output pin is Open Drain w/ pull up */
#define GPIO_CFG_OUT_OD_PD         (((uint32_t) 6) << GPIO_CFG_IO_LSB) /*! Output pin is Open Drain w/ pull dn */

#define GPIO_CFG_OUT_HIGH          (((uint32_t) 1) << GPIO_CFG_OUT_BIT) /*! Set pin's output to 1. */
#define GPIO_CFG_OUT_LOW           (((uint32_t) 0) << GPIO_CFG_OUT_BIT) /*! Set pin's output to 0. */

#define GPIO_CFG_OUT_STR_LOW       (((uint32_t) 0) << GPIO_CFG_OUT_STRENGTH_LSB)
#define GPIO_CFG_OUT_STR_MED       (((uint32_t) 1) << GPIO_CFG_OUT_STRENGTH_LSB)
#define GPIO_CFG_OUT_STR_HIGH      (((uint32_t) 2) << GPIO_CFG_OUT_STRENGTH_LSB)

#define GPIO_CFG_INPUT             (((uint32_t) 1) << GPIO_CFG_IO_LSB) /*! Pin is an input. */
#define GPIO_CFG_IN_NOPULL         (((uint32_t) 1) << GPIO_CFG_IO_LSB) /*! Input pin has no PU/PD */
#define GPIO_CFG_IN_PU             (((uint32_t) 3) << GPIO_CFG_IO_LSB) /*! Input pin has Pullup */
#define GPIO_CFG_IN_PD             (((uint32_t) 5) << GPIO_CFG_IO_LSB) /*! Input pin has Pulldown */
/** @} */

/** @name GPIO_PinConfig pin interrupt configuration macros
 *  @{
 */
#define GPIO_CFG_IN_INT_NONE       (((uint32_t) 0) << GPIO_CFG_INT_LSB)    /*! No Interrupt */
#define GPIO_CFG_IN_INT_FALLING    (((uint32_t) 1) << GPIO_CFG_INT_LSB)    /*! Interrupt on falling edge */
#define GPIO_CFG_IN_INT_RISING     (((uint32_t) 2) << GPIO_CFG_INT_LSB)    /*! Interrupt on rising edge */
#define GPIO_CFG_IN_INT_BOTH_EDGES (((uint32_t) 3) << GPIO_CFG_INT_LSB)    /*! Interrupt on both edges */
#define GPIO_CFG_IN_INT_LOW        (((uint32_t) 4) << GPIO_CFG_INT_LSB)    /*! Interrupt on low level */
#define GPIO_CFG_IN_INT_HIGH       (((uint32_t) 5) << GPIO_CFG_INT_LSB)    /*! Interrupt on high level */
/** @} */

/** @name Special GPIO_PinConfig configuration macros
 *  @{
 */

/*!
 *  \brief 'Or' in this GPIO_PinConfig definition to inform GPIO_setConfig()
 *  to only configure the interrupt attributes of a GPIO input pin.
 */
#define GPIO_CFG_IN_INT_ONLY       (((uint32_t) 1) << 27)                  /*! configure interrupt only */

/*!
 *  \brief Use this GPIO_PinConfig definition to inform GPIO_init()
 *  NOT to configure the corresponding pin
 */
#define GPIO_DO_NOT_CONFIG         0x7fff0001U                             /*! Do not configure this Pin */

/** @} */

/*!
 *  @brief  GPIO callback function type
 */
typedef void (*GPIO_CallbackFxn)(void);

/*!
 *  @brief  Initializes the GPIO module
 */
typedef void        (*GPIO_InitFxn)           (void);

/*!
 *  @brief      Reads the value of a GPIO pin
 */
typedef uint32_t    (*GPIO_ReadFxn)           (uint32_t index);

/*!
 *  @brief     Writes the value to a GPIO pin
 */
typedef void        (*GPIO_WriteFxn)          (uint32_t index,
                                               uint32_t value);

/*!
 *  @brief      Clear a GPIO pin interrupt flag
 */
typedef void        (*GPIO_clearIntFxn)       (uint32_t index);

/*!
 *  @brief      Disable a GPIO pin interrupt
 */
typedef void        (*GPIO_disableIntFxn)      (uint32_t index);

/*!
 *  @brief      Enable a GPIO pin interrupt
 */
typedef void        (*GPIO_enableIntFxn)       (uint32_t index);

/*!
 *  @brief      Bind a callback function to a GPIO pin interrupt
 */
typedef void        (*GPIO_setCallbackFxn)     (uint32_t index,
                                                GPIO_CallbackFxn callback);

/*!
 *  @brief      Configure the gpio pin
 */
typedef void        (*GPIO_setConfigFxn)       (uint32_t index,
                                                GPIO_PinConfig pinConfig);
/*!
 *  @brief      Toggles the current state of a GPIO
 *
 *  @param      index  GPIO index
 */
typedef void        (*GPIO_toggleFxn)          (uint32_t index);

/*!
 *  @brief      The definition of a GPIO function table that contains the
 *              required set of functions to control a specific GPIO driver
 *              implementation.
 */
 typedef struct GPIO_FxnTable_s {
    /*! Function to initialize the given data object */
    GPIO_InitFxn            initFxn;
    /*! Function to read from the specified peripheral */
    GPIO_ReadFxn            readFxn;
    /*! Function to write from the specified peripheral */
    GPIO_WriteFxn           writeFxn;
    /*! Function to clear a GPIO pin interrupt flag */
    GPIO_clearIntFxn        clearIntFxn;
    /*! Function to disable a GPIO pin interrupt flag */
    GPIO_disableIntFxn      disableIntFxn;
    /*! Function to enable a GPIO pin interrupt flag */
    GPIO_enableIntFxn       enableIntFxn;
    /*! Function to bind a callback function to a GPIO pin interrupt */
    GPIO_setCallbackFxn     setCallbackFxn;
    /*! Function to configure the gpio pin */
    GPIO_setConfigFxn       setConfigFxn;
    /*! Function to toggles the current state of a GPIO */
    GPIO_toggleFxn          toggleFxn;

} GPIO_FxnTable;


/*! @brief GPIO Global configuration */
typedef struct GPIO_Config_s {
    /*! Pointer to a table of a driver-specific implementation of GPIO functions */
    GPIO_FxnTable const    *fxnTablePtr;

    /*! Pointer to a driver specific data object */
    void                   *object;

    /*! Pointer to a driver specific hardware attributes structure */
    void          const    *hwAttrs;
} GPIO_Config;

/* Avoid Misra warning "MISRA.DECL.ARRAY_SIZE" by pairing config array type 
 * with its array size to avoid externs with [] (no size) */
#define GPIO_MAX_CONFIG_CNT       (3U) 
typedef GPIO_Config GPIOConfigList[GPIO_MAX_CONFIG_CNT];

/*!
 *  @brief      Clear a GPIO pin interrupt flag
 *
 *  Clears the GPIO interrupt for the specified index.
 *
 *  Note: It is not necessary to call this API within a
 *  callback assigned to a pin.
 *
 *  @param      index       GPIO index
 */
extern void GPIO_clearInt(uint32_t index);

/*!
 *  @brief      Disable a GPIO pin interrupt
 *
 *  Disables interrupts for the specified GPIO index.
 *
 *  @param      index       GPIO index
 */
extern void GPIO_disableInt(uint32_t index);

/*!
 *  @brief      Enable a GPIO pin interrupt
 *
 *  Enables GPIO interrupts for the selected index to occur.
 *
 *  Note:  Prior to enabling a GPIO pin interrupt, make sure
 *  that a corresponding callback function has been provided.
 *  Use the GPIO_setCallback() API for this purpose at runtime.
 *  Alternatively, the callback function can be statically
 *  configured in the GPIO_CallbackFxn array provided.
 *
 *  @param      index       GPIO index
 */
extern void GPIO_enableInt(uint32_t index);

/*!
 *  @brief  Initializes the GPIO module
 *
 *  The pins defined in the application-provided *GPIOXXX_config* structure
 *  are initialized accordingly.
 *
 *  @pre    The GPIO_config structure must exist and be persistent before this
 *          function can be called. This function must also be called before
 *          any other GPIO driver APIs.
 */
extern void GPIO_init(void);

/*!
 *  @brief      Reads the value of a GPIO pin
 *
 *  The value returned will either be zero or one depending on the
 *  state of the pin.
 *
 *  @param      index  GPIO index
 *
 *  @return     0 or 1, depending on the state of the pin.
 */
extern uint32_t GPIO_read(uint32_t index);

/*!
 *  @brief      Bind a callback function to a GPIO pin interrupt
 *
 *  Associate a callback function with a particular GPIO pin interrupt.
 *
 *  Callbacks can be changed at any time, making it easy to switch between
 *  efficient, state-specific interrupt handlers.
 *
 *  Note: The callback function is called within the context of an interrupt
 *  handler.
 *
 *  Note: This API does not enable the GPIO pin interrupt.
 *  Use GPIO_enableInt() and GPIO_disableInt() to enable
 *  and disable the pin interrupt as necessary.
 *
 *  Note: it is not necessary to call GPIO_clearInt() within a callback.
 *  That operation is performed internally before the callback is invoked.
 *
 *  @param      index       GPIO index
 *  @param      callback    address of the callback function
 */
extern void GPIO_setCallback(uint32_t index, GPIO_CallbackFxn callback);

/*!
 *  @brief      Configure the gpio pin
 *
 *  Dynamically configure a gpio pin to a device specific setting.
 *  For many applications, the pin configurations provided in the static
 *  GPIO_PinConfig array is sufficient.
 *
 *  For input pins with interrupt configurations, a corresponding interrupt
 *  object will be created as needed.
 *
 *  @param      index       GPIO index
 *  @param      pinConfig   device specific pin configuration settings
 */
extern void GPIO_setConfig(uint32_t index, GPIO_PinConfig pinConfig);

/*!
 *  @brief      Toggles the current state of a GPIO
 *
 *  @param      index  GPIO index
 */
extern void GPIO_toggle(uint32_t index);

/*!
 *  @brief     Writes the value to a GPIO pin
 *
 *  @param      index    GPIO index
 *  @param      value    must be either 0 or 1
 */
extern void GPIO_write(uint32_t index, uint32_t value);

#ifdef __cplusplus
}
#endif

#endif /* _GPIO_H */
