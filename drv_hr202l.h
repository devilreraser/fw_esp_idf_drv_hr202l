/* *****************************************************************************
 * File:   drv_hr202l.h
 * Author: XX
 *
 * Created on YYYY MM DD
 * 
 * Description: ...
 * 
 **************************************************************************** */
#pragma once

#ifdef __cplusplus
extern "C"
{
#endif /* __cplusplus */


/* *****************************************************************************
 * Header Includes
 **************************************************************************** */
#include "driver/gpio.h"
    
/* *****************************************************************************
 * Configuration Definitions
 **************************************************************************** */

/* *****************************************************************************
 * Constants and Macros Definitions
 **************************************************************************** */

/* *****************************************************************************
 * Enumeration Definitions
 **************************************************************************** */

/* *****************************************************************************
 * Type Definitions
 **************************************************************************** */

/* *****************************************************************************
 * Function-Like Macro
 **************************************************************************** */

/* *****************************************************************************
 * Variables External Usage
 **************************************************************************** */ 

/* *****************************************************************************
 * Function Prototypes
 **************************************************************************** */
int drv_hr202l_get_last_measurement_raw(void);
void drv_hr202l_dbg_pin_setup(gpio_num_t pin);
void drv_hr202l_a1_pin_setup(gpio_num_t pin);
void drv_hr202l_a2_pin_setup(gpio_num_t pin);
void drv_hr202l_ain_pin_setup(gpio_num_t pin);
void drv_hr202l_init(gpio_num_t pin_A1, gpio_num_t pin_A2, gpio_num_t pin_AIN, gpio_num_t pin_DBG);
void drv_hr202l_trigger_measurement(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */


