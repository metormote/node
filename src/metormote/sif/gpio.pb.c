/* Automatically generated nanopb constant definitions */
/* Generated by nanopb-0.1.9-dev at Wed Apr 10 12:01:54 2013. */

#include "gpio.pb.h"



const pb_field_t gpio_State_fields[3] = {
  {1, (pb_type_t) ((int) PB_HTYPE_OPTIONAL | (int) PB_LTYPE_VARINT),
    offsetof(gpio_State, IO_DAC0),
    pb_delta(gpio_State, has_IO_DAC0, IO_DAC0),
  pb_membersize(gpio_State, IO_DAC0), 0, 0},

  {2, (pb_type_t) ((int) PB_HTYPE_OPTIONAL | (int) PB_LTYPE_VARINT),
    pb_delta_end(gpio_State, IO_DAC1, IO_DAC0),
    pb_delta(gpio_State, has_IO_DAC1, IO_DAC1),
  pb_membersize(gpio_State, IO_DAC1), 0, 0},

  PB_LAST_FIELD
};

