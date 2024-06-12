/* *****************************************************************************
 * File:   cmd_hr202l.c
 * Author: Dimitar Lilov
 *
 * Created on 2022 06 18
 * 
 * Description: ...
 * 
 **************************************************************************** */

/* *****************************************************************************
 * Header Includes
 **************************************************************************** */
#include "cmd_hr202l.h"
#include "drv_hr202l.h"

#include <string.h>

#include "esp_log.h"
#include "esp_console.h"
#include "esp_system.h"

#include "driver/gpio.h"

#include "argtable3/argtable3.h"

/* *****************************************************************************
 * Configuration Definitions
 **************************************************************************** */
#define TAG "cmd_hr202l"

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
 * Function-Like Macros
 **************************************************************************** */

/* *****************************************************************************
 * Variables Definitions
 **************************************************************************** */

static struct {
    struct arg_str *pin;
    struct arg_str *cmd;
    struct arg_end *end;
} hr202l_args;


/* *****************************************************************************
 * Prototype of functions definitions
 **************************************************************************** */

/* *****************************************************************************
 * Functions
 **************************************************************************** */
static int command_hr202l(int argc, char **argv)
{
    ESP_LOGI(__func__, "argc=%d", argc);
    for (int i = 0; i < argc; i++)
    {
        ESP_LOGI(__func__, "argv[%d]=%s", i, argv[i]);
    }

    int nerrors = arg_parse(argc, argv, (void **)&hr202l_args);
    if (nerrors != ESP_OK)
    {
        arg_print_errors(stderr, hr202l_args.end, argv[0]);
        return ESP_FAIL;
    }

    const char* hr202l_number = hr202l_args.pin->sval[0];

    gpio_num_t hr202l_pin = atoi(hr202l_number);
    ESP_LOGI(TAG, "hr202l_pin=%d", hr202l_pin);
    
    //int gpio_pin = drv_hr202l_get_gpio_num_from_configuration(hr202l_pin);
    //ESP_LOGI(TAG, "gpio_pin=%d", gpio_pin);

    const char* hr202l_command = hr202l_args.cmd->sval[0];

    if (strcmp(hr202l_command,"debug") == 0)
    {
        drv_hr202l_dbg_pin_setup(hr202l_pin);
    }
    else if (strcmp(hr202l_command,"a1") == 0)
    {
        drv_hr202l_a1_pin_setup(hr202l_pin);
    }
    else if (strcmp(hr202l_command,"a2") == 0)
    {
        drv_hr202l_a2_pin_setup(hr202l_pin);
    }
    else
    {
        ESP_LOGE(TAG, "Unknown command %s", hr202l_command);
        return ESP_FAIL;
    }
    return 0;
}

static void register_hr202l(void)
{
    hr202l_args.pin = arg_strn("p", "pin",      "<pin number>",         1, 1, "Specify pin number : hr202l -p {0|1|2|3|4|5|6|7|8}");
    hr202l_args.cmd = arg_strn("c", "command",  "<command>",            1, 1, "Command can be     : hr202l -c {debug|a1|a2}");
    hr202l_args.end = arg_end(2);

    const esp_console_cmd_t cmd_hr202l = {
        .command = "hr202l",
        .help = "HR202L Command Request",
        .hint = NULL,
        .func = &command_hr202l,
        .argtable = &hr202l_args,
    };

    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd_hr202l));
}


void cmd_hr202l_register(void)
{
    register_hr202l();
}
