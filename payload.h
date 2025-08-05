#ifndef PAYLOAD_H
#define PAYLOAD_H

#include <stdint.h>
#include <stdbool.h>

#define PAYLOAD_VALUE_LEN 13  // 12 caractères utiles + '\0'

enum _sender_id {
  LORALINK_H = 1,
  LORALINK_S,
  LORAGAZ,
  TEMPOLINK,
  TEMPOVIEW
};

typedef struct __attribute__((packed)) {
    uint8_t sender_id;
    uint8_t counter_addr[5];
    uint8_t label_id;          // identifiant d’étiquette TIC
    char value[PAYLOAD_VALUE_LEN]; // chaîne ASCII terminée par '\0'
    uint8_t crc;
} PayloadData;


// Fonctions d’accès
void payload_set_value_str(PayloadData* p, const char* str);
void payload_get_value_str(const PayloadData* p, char* out);

void payload_set_counter_addr(PayloadData* p, const char* str);
void payload_get_counter_addr_toString(const PayloadData* p, char* out, size_t max_len);
uint64_t payload_get_counter_addr_toUint64(const PayloadData* p);

// CRC
uint8_t payload_compute_crc(const PayloadData* p);
bool payload_check_crc(const PayloadData* p);
void payload_finalize(PayloadData* p);

// Sérialisation
int payload_serialize(const PayloadData* p, uint8_t* buffer);
bool payload_deserialize(PayloadData* p, const uint8_t* buffer, int length);

// Divers
void printUint64(uint64_t value);
bool is_valid_sender_id(uint8_t id);

#endif // PAYLOAD_H