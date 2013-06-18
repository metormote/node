/* Automatically generated nanopb constant definitions */
#include "potentiometer.pb.h"



const pb_field_t potentiometer_In_fields[2] = {
    {1, PB_HTYPE_REQUIRED | PB_LTYPE_VARINT,
    offsetof(potentiometer_In, value), 0,
    pb_membersize(potentiometer_In, value), 0, 0},

    PB_LAST_FIELD
};

const pb_field_t potentiometer_Out_fields[3] = {
    {1, PB_HTYPE_REQUIRED | PB_LTYPE_VARINT,
    offsetof(potentiometer_Out, value), 0,
    pb_membersize(potentiometer_Out, value), 0, 0},

    {2, PB_HTYPE_OPTIONAL | PB_LTYPE_VARINT,
    pb_delta_end(potentiometer_Out, temperature, value),
    pb_delta(potentiometer_Out, has_temperature, temperature),
    pb_membersize(potentiometer_Out, temperature), 0, 0},

    PB_LAST_FIELD
};

