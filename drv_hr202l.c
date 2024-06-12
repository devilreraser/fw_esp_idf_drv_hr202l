/* *****************************************************************************
 * File:   drv_hr202l.c
 * Author: XX
 *
 * Created on YYYY MM DD
 * 
 * Description: ...
 * 
 **************************************************************************** */

/* *****************************************************************************
 * Header Includes
 **************************************************************************** */
#include "drv_hr202l.h"
#include "cmd_hr202l.h"

#include "freertos/FreeRTOS.h"

#include "driver/gpio.h"
#include "driver/timer.h"

#include "drv_adc.h"

/* *****************************************************************************
 * Configuration Definitions
 **************************************************************************** */
#define TIMER_RESOLUTION_HZ   1000000 // 1MHz resolution
#define TIMER_ALARM_PERIOD_S  0.00025     // Alarm period resolution 1 second



/* *****************************************************************************
 * Constants and Macros Definitions
 **************************************************************************** */
#define TAG "drv_hr202l"

/* *****************************************************************************
 * Enumeration Definitions
 **************************************************************************** */

/* *****************************************************************************
 * Type Definitions
 **************************************************************************** */

/* *****************************************************************************
 * Function-Like Macros
 **************************************************************************** */

/* *****************************************************************************
 * Variables Definitions
 **************************************************************************** */


/* *****************************************************************************
 * Prototype of functions definitions
 **************************************************************************** */
gpio_num_t pin_A1 = GPIO_NUM_NC;
gpio_num_t pin_A2 = GPIO_NUM_NC;
gpio_num_t pin_DBG = GPIO_NUM_NC;

bool b_init_passes = false;

/* *****************************************************************************
 * Functions
 **************************************************************************** */

static bool IRAM_ATTR timer_group_isr_callback(void *args)
{
    BaseType_t high_task_awoken = pdFALSE;
    static uint32_t loop_counter = 0;

    int time_chunk = loop_counter & 0x03;

    switch(time_chunk)
    {
        case 0:
            gpio_set_level(pin_A1, 0);
            gpio_set_level(pin_A2, 1);
            drv_adc_sample_channel(DRV_ADC_AIN_0);
            break;
        case 1:
            drv_adc_sample_channel(DRV_ADC_AIN_1);
            break;
        case 2:
            gpio_set_level(pin_A1, 1);
            gpio_set_level(pin_A2, 0);
            drv_adc_sample_channel(DRV_ADC_AIN_2);
            break;
        case 3:
            gpio_set_level(pin_DBG, 1);
            drv_adc_sample_channel(DRV_ADC_AIN_3);
            gpio_set_level(pin_DBG, 0);
            break;
    }
    loop_counter++;

    // Send the event data back to the main program task
    //xQueueSendFromISR(user_data->user_queue, &evt, &high_task_awoken);

    return high_task_awoken == pdTRUE; // return whether a task switch is needed
}

void setup_timer(void)
{
    timer_config_t config = {
        .clk_src = TIMER_SRC_CLK_DEFAULT,
        .divider = APB_CLK_FREQ / TIMER_RESOLUTION_HZ,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = TIMER_AUTORELOAD_EN,
    };
    int group = TIMER_GROUP_0;
    int timer = TIMER_0;
    uint64_t alarm_value = TIMER_ALARM_PERIOD_S * TIMER_RESOLUTION_HZ;
    ESP_ERROR_CHECK(timer_init(group, timer, &config));
    // For the timer counter to a initial value
    ESP_ERROR_CHECK(timer_set_counter_value(group, timer, 0));
    // Set alarm value and enable alarm interrupt
    ESP_ERROR_CHECK(timer_set_alarm_value(group, timer, alarm_value));
    ESP_ERROR_CHECK(timer_enable_intr(group, timer));
    // Hook interrupt callback
    ESP_ERROR_CHECK(timer_isr_callback_add(group, timer, timer_group_isr_callback, NULL, 0));
    // Start timer
    ESP_ERROR_CHECK(timer_start(group, timer));
}


void setup_gpio_output(gpio_num_t pin) 
{
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set
    io_conf.pin_bit_mask = (1ULL<<pin);
    //disable pull-down mode
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    //disable pull-up mode
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
}


void drv_hr202l_dbg_pin_setup(gpio_num_t pin)
{
    pin_DBG = pin;
    setup_gpio_output(pin); 
}
void drv_hr202l_a1_pin_setup(gpio_num_t pin)
{
    pin_A1 = pin;
    setup_gpio_output(pin); 
}
void drv_hr202l_a2_pin_setup(gpio_num_t pin)
{
    pin_A2 = pin;
    setup_gpio_output(pin); 
}
void drv_hr202l_init(gpio_num_t pin_A1, gpio_num_t pin_A2, gpio_num_t pin_DBG)
{
    drv_hr202l_dbg_pin_setup(pin_DBG);
    drv_hr202l_a1_pin_setup(pin_A1);
    drv_hr202l_a2_pin_setup(pin_A2);

    if (b_init_passes == false)
    {
        b_init_passes = true;
        cmd_hr202l_register();
        setup_timer();
    }
}

