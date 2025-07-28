#ifndef Label_tic_h
#define Label_tic_h

#include <stdint.h>
#include <stdbool.h>


typedef enum {
    LABEL_TYPE_ASCII = 0,
    LABEL_TYPE_U16,
    LABEL_TYPE_U32,
    LABEL_TYPE_FLOAT
} label_value_type_t;


uint8_t find_label_id(const char* label, bool is_standard);
const char* find_label_str(uint8_t id, bool is_standard);

#endif