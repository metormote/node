/* Automatically generated nanopb header */
#ifndef _PB_POTENTIOMETER_PB_H_
#define _PB_POTENTIOMETER_PB_H_
#include <pb.h>

/* Enum definitions */
/* Struct definitions */
typedef struct {
    uint32_t value;
} potentiometer_In;

typedef struct {
    uint32_t value;
    bool has_temperature;
    uint32_t temperature;
} potentiometer_Out;

/* Default values for struct fields */

/* Struct field encoding specification for nanopb */
extern const pb_field_t potentiometer_In_fields[2];
extern const pb_field_t potentiometer_Out_fields[3];

#endif
