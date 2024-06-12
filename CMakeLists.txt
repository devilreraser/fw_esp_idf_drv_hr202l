idf_component_register(SRCS "cmd_hr202l.c" "drv_hr202l.c" 
                    INCLUDE_DIRS "." 
                    REQUIRES 
                            "driver"
                            "console"
                            "drv_adc"
                                      )
                 

