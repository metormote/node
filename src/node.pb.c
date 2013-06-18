/* Automatically generated nanopb constant definitions */
/* Generated by nanopb-0.1.9-dev at Fri Jan 18 14:24:39 2013. */

#include "node.pb.h"



const pb_field_t node_Out_fields[3] = {
    {1, (pb_type_t) ((int) PB_HTYPE_OPTIONAL | (int) PB_LTYPE_VARINT),
    offsetof(node_Out, batteryLevel),
    pb_delta(node_Out, has_batteryLevel, batteryLevel),
    pb_membersize(node_Out, batteryLevel), 0, 0},

    {2, (pb_type_t) ((int) PB_HTYPE_OPTIONAL | (int) PB_LTYPE_VARINT),
    pb_delta_end(node_Out, temperature, batteryLevel),
    pb_delta(node_Out, has_temperature, temperature),
    pb_membersize(node_Out, temperature), 0, 0},

    PB_LAST_FIELD
};

const pb_field_t node_In_fields[6] = {
    {1, (pb_type_t) ((int) PB_HTYPE_OPTIONAL | (int) PB_LTYPE_VARINT),
    offsetof(node_In, transferInterval),
    pb_delta(node_In, has_transferInterval, transferInterval),
    pb_membersize(node_In, transferInterval), 0, 0},

    {2, (pb_type_t) ((int) PB_HTYPE_OPTIONAL | (int) PB_LTYPE_VARINT),
    pb_delta_end(node_In, leds, transferInterval),
    pb_delta(node_In, has_leds, leds),
    pb_membersize(node_In, leds), 0, 0},

    {3, (pb_type_t) ((int) PB_HTYPE_OPTIONAL | (int) PB_LTYPE_VARINT),
    pb_delta_end(node_In, pollingInterval, leds),
    pb_delta(node_In, has_pollingInterval, pollingInterval),
    pb_membersize(node_In, pollingInterval), 0, 0},

    {4, (pb_type_t) ((int) PB_HTYPE_OPTIONAL | (int) PB_LTYPE_VARINT),
    pb_delta_end(node_In, timeout, pollingInterval),
    pb_delta(node_In, has_timeout, timeout),
    pb_membersize(node_In, timeout), 0, 0},

    {5, (pb_type_t) ((int) PB_HTYPE_OPTIONAL | (int) PB_LTYPE_VARINT),
    pb_delta_end(node_In, noOfPolls, timeout),
    pb_delta(node_In, has_noOfPolls, noOfPolls),
    pb_membersize(node_In, noOfPolls), 0, 0},

    PB_LAST_FIELD
};
