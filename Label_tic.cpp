#include "Label_tic.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>



LabelMap tic_standard_labels[] = {
    { "ADSC", 0x01, "", ""},
    { "VTIC", 0x02, "", ""},
    { "DATE", 0x03, "", "date"},
    { "NGTF", 0x04, "", ""},
    { "LTARF", 0x05, "", ""},
    { "EAST", 0x06, "Wh", "energy"},
    { "EASF01", 0x07, "Wh", "energy"},
    { "EASF02", 0x08, "Wh", "energy"},
    { "EASF03", 0x09, "Wh", "energy"},
    { "EASF04", 0x0A, "Wh", "energy"},
    { "EASF05", 0x0B, "Wh", "energy"},
    { "EASF06", 0x0C, "Wh", "energy"},
    { "EASF07", 0x0D, "Wh", "energy"},
    { "EASF08", 0x0E, "Wh", "energy"},
    { "EASF09", 0x0F, "Wh", "energy"},
    { "EASF10", 0x10, "Wh", "energy"},
    { "EASD01", 0x11, "Wh", "energy"},
    { "EASD02", 0x12, "Wh", "energy"},
    { "EASD03", 0x13, "Wh", "energy"},
    { "EASD04", 0x14, "Wh", "energy"},
    { "EAIT", 0x15, "Wh", "energy"},
    { "ERQ1", 0x16, "VArh", "energy"},
    { "ERQ2", 0x17, "VArh", "energy"},
    { "ERQ3", 0x18, "VArh", "energy"},
    { "ERQ4", 0x19, "VArh", "energy"},
    { "IRMS1", 0x1A, "A", "current"},
    { "IRMS2", 0x1B, "A", "current"},
    { "IRMS3", 0x1C, "A", "current"},
    { "URMS1", 0x1D, "V", "voltage"},
    { "URMS2", 0x1E, "V", "voltage"},
    { "URMS3", 0x1F, "V", "voltage"},
    { "PREF", 0x20, "kVA", "apparent_power"},
    { "PCOUP", 0x21, "kVA", "apparent_power"},
    { "SINSTS", 0x22, "VA", "apparent_power"},
    { "SINSTS1", 0x23, "VA", "apparent_power"},
    { "SINSTS2", 0x24, "VA", "apparent_power"},
    { "SINSTS3", 0x25, "VA", "apparent_power"},
    { "SMAXSN", 0x26, "VA", "apparent_power"},
    { "SMAXSN1", 0x27, "VA", "apparent_power"},
    { "SMAXSN2", 0x28, "VA", "apparent_power"},
    { "SMAXSN3", 0x29, "VA", "apparent_power"},
    { "SMAXSN-1", 0x2A, "VA", "apparent_power"},
    { "SMAXSN1-1", 0x2B, "VA", "apparent_power"},
    { "SMAXSN2-1", 0x2C, "VA", "apparent_power"},
    { "SMAXSN3-1", 0x2D, "VA", "apparent_power"},
    { "SINSTI", 0x2E, "VA", "apparent_power"},
    { "SMAXIN", 0x2F, "VA", "apparent_power"},
    { "SMAXIN-1", 0x30, "VA", "apparent_power"},
    { "CCASN", 0x31, "W", "energy"},
    { "CCASN-1", 0x32, "W", "energy"},
    { "CCAIN", 0x33, "W", "energy"},
    { "CCAIN-1", 0x34, "W", "energy"},
    { "UMOY1", 0x35, "V", "voltage"},
    { "UMOY2", 0x36, "V", "voltage"},
    { "UMOY3", 0x37, "V", "voltage"},
    { "STGE", 0x38, "", ""},
    { "DPM1", 0x39, "", ""},
    { "FPM1", 0x3A, "", ""},
    { "DPM2", 0x3B, "", ""},
    { "FPM2", 0x3C, "", ""},
    { "DPM3", 0x3D, "", ""},
    { "FPM3", 0x3E, "", ""},
    { "MSG1", 0x3F, "", ""},
    { "MSG2", 0x40, "", ""},
    { "PMR", 0x41, "", ""},
    { "RELAIS", 0x42, "", ""},
    { "NTARF", 0x43, "", ""},
    { "NJOURF", 0x44, "", ""},
    { "NJOURF+1", 0x45, "", ""},
    { "PJOURF+1", 0x46, "", ""},
    { "PPOINTE", 0x47, "", ""},
    { "BOOT", 0xFF, "", ""}
};
const size_t tic_standard_count = sizeof(tic_standard_labels) / sizeof(LabelMap);

LabelMap tic_historical_labels[] = {
    { "ADCO", 0x01, "", ""},
    { "OPTARIF", 0x02, "", ""},
    { "ISOUSC", 0x03, "A", "current"},
    { "BASE", 0x04, "Wh", "energy"},
    { "HCHC", 0x05, "Wh", "energy"},
    { "HCHP", 0x06, "Wh", "energy"},
    { "EJPHN", 0x07, "Wh", "energy"},
    { "EJPHPM", 0x08, "Wh", "energy"},
    { "BBRHCJB", 0x09, "Wh", "energy"},
    { "BBRHPJB", 0x0A, "Wh", "energy"},
    { "BBRHCJW", 0x0B, "Wh", "energy"},
    { "BBRHPJW", 0x0C, "Wh", "energy"},
    { "BBRHCJR", 0x0D, "Wh", "energy"},
    { "BBRHPJR", 0x0E, "Wh", "energy"},
    { "PEJP", 0x0F, "mn", "duration"},
    { "PTEC", 0x10, "", ""},
    { "DEMAIN", 0x11, "", ""},
    { "IINST", 0x12, "A", "current"},
    { "IINST1", 0x13, "A", "current"},
    { "IINST2", 0x14, "A", "current"},
    { "IINST3", 0x15, "A", "current"},
    { "IMAX", 0x16, "A", "current"},
    { "IMAX1", 0x17, "A", "current"},
    { "IMAX2", 0x18, "A", "current"},
    { "IMAX3", 0x19, "A", "current"},
    { "PAPP", 0x1A, "VA", "apparent_power"},
    { "PMAX", 0x1B, "VA", "apparent_power"},
    { "ADPS", 0x1C, "A", "current"},
    { "HHPHC", 0x1D, "", ""},
    { "MOTDETAT", 0x1E, "", ""},
    { "PPOT", 0x1F, "", ""},
    { "BOOT", 0xFF, "", ""}
};
const size_t tic_historical_count = sizeof(tic_historical_labels) / sizeof(LabelMap);

// Trouve l’ID d’un label
uint8_t find_label_id(const char* label, bool is_standard) {
    LabelMap* map = is_standard ? tic_standard_labels : tic_historical_labels;
    size_t count = is_standard ? tic_standard_count : tic_historical_count;

    for (size_t i = 0; i < count; i++) {
        if (strcmp(map[i].label, label) == 0) {
            return map[i].id;
        }
    }
    return 0; // 0 représente un ID invalide
}

// Retourne le nom d’étiquette à partir de son ID
const char* find_label_str(uint8_t id, bool is_standard) {
    LabelMap* map = is_standard ? tic_standard_labels : tic_historical_labels;
    size_t count = is_standard ? tic_standard_count : tic_historical_count;

    for (size_t i = 0; i < count; i++) {
        if (map[i].id == id) {
            return map[i].label;
        }
    }
    return NULL; // NULL si non trouvé
}

const LabelMap* find_label_by_id(uint8_t id, bool is_standard) {
    LabelMap* map = is_standard ? tic_standard_labels : tic_historical_labels;
    size_t count = is_standard ? tic_standard_count : tic_historical_count;
    
    for (size_t i = 0; i < count; i++) {
        if (map[i].id == id) {
            return &map[i];
        }
    }
    return NULL;  // NULL si non trouvé
}

