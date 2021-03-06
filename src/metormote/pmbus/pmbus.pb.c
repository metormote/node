/* Automatically generated nanopb constant definitions */
/* Generated by nanopb-0.1.9-dev at Fri Jan 18 14:24:46 2013. */

#include "pmbus.pb.h"



const pb_field_t pmbus_In_fields[3] = {
    {1, (pb_type_t) ((int) PB_HTYPE_REQUIRED | (int) PB_LTYPE_VARINT),
    offsetof(pmbus_In, pmbusAddress), 0,
    pb_membersize(pmbus_In, pmbusAddress), 0, 0},

    {2, (pb_type_t) ((int) PB_HTYPE_CALLBACK | (int) PB_LTYPE_SUBMESSAGE),
    pb_delta_end(pmbus_In, command, pmbusAddress), 0,
    pb_membersize(pmbus_In, command), 0,
    &pmbus_Command_fields},

    PB_LAST_FIELD
};

const pb_field_t pmbus_Out_fields[3] = {
    {1, (pb_type_t) ((int) PB_HTYPE_REQUIRED | (int) PB_LTYPE_VARINT),
    offsetof(pmbus_Out, pmbusAddress), 0,
    pb_membersize(pmbus_Out, pmbusAddress), 0, 0},

    {2, (pb_type_t) ((int) PB_HTYPE_CALLBACK | (int) PB_LTYPE_SUBMESSAGE),
    pb_delta_end(pmbus_Out, response, pmbusAddress), 0,
    pb_membersize(pmbus_Out, response), 0,
    &pmbus_Response_fields},

    PB_LAST_FIELD
};

const pb_field_t pmbus_Command_fields[5] = {
    {1, (pb_type_t) ((int) PB_HTYPE_REQUIRED | (int) PB_LTYPE_VARINT),
    offsetof(pmbus_Command, type), 0,
    pb_membersize(pmbus_Command, type), 0, 0},

    {2, (pb_type_t) ((int) PB_HTYPE_REQUIRED | (int) PB_LTYPE_VARINT),
    pb_delta_end(pmbus_Command, code, type), 0,
    pb_membersize(pmbus_Command, code), 0, 0},

    {3, (pb_type_t) ((int) PB_HTYPE_REQUIRED | (int) PB_LTYPE_VARINT),
    pb_delta_end(pmbus_Command, responseLen, code), 0,
    pb_membersize(pmbus_Command, responseLen), 0, 0},

    {4, (pb_type_t) ((int) PB_HTYPE_CALLBACK | (int) PB_LTYPE_BYTES),
    pb_delta_end(pmbus_Command, data, responseLen), 0,
    pb_membersize(pmbus_Command, data), 0, 0},

    PB_LAST_FIELD
};

const pb_field_t pmbus_Response_fields[4] = {
    {1, (pb_type_t) ((int) PB_HTYPE_REQUIRED | (int) PB_LTYPE_VARINT),
    offsetof(pmbus_Response, code), 0,
    pb_membersize(pmbus_Response, code), 0, 0},

    {2, (pb_type_t) ((int) PB_HTYPE_CALLBACK | (int) PB_LTYPE_BYTES),
    pb_delta_end(pmbus_Response, data, code), 0,
    pb_membersize(pmbus_Response, data), 0, 0},

    {3, (pb_type_t) ((int) PB_HTYPE_OPTIONAL | (int) PB_LTYPE_VARINT),
    pb_delta_end(pmbus_Response, error, data),
    pb_delta(pmbus_Response, has_error, error),
    pb_membersize(pmbus_Response, error), 0, 0},

    PB_LAST_FIELD
};

