//--------------------------------------------------------------------
// ███████╗██████╗ ███████╗
// ██╔════╝██╔══██╗██╔════╝
// █████╗  ██████╔╝███████╗
// ██╔══╝  ██╔══██╗╚════██║
// ██║     ██████╔╝███████║
// ╚═╝     ╚═════╝ ╚══════╝
// FuméeBleueSolutions
//--------------------------------------------------------------------
// Code pour le module LoraLink de la société FBS. 
//--------------------------------------------------------------------
// Copyright (C) 2025 FBS. Tous droits réservés.
//
//    Ce programme ne peut être modifié ou redistribué sans 
//     l'accord explicite de la société FBS.
//
//--------------------------------------------------------------------
// 2025/04/12 - FBS V1.00 - LoraLink
// 2025/07/31 - FBS V1.01 - Ajout calcul Toa (Time On Air)
//--------------------------------------------------------------------
#include <math.h>
#include <SPI.h>
#include <LoRa.h>
#include <jled.h>
#include <LibTeleinfo.h>
#include "payload.h"
#include "label_tic.h"

#define VERSION "1.01"

//#define DEBUG_TIC

#define tic   Serial1
#define debug Serial

#define CAPACITANCE	1 // Capacité du supercondensateur 
#define VOLTAGE_CAPA_MAX	4.8
#define VOLTAGE_CAPA_MIN	4.5

#define LORA_SF					7
#define LORA_BW					125E3		
#define LORA_CR					4
#define LORA_PREAMBLE_LENGTH	8
#define LORA_POWER				5


const unsigned long SEND_FREQUENCY_FULL = 180000; // 3mn, Minimum time between send (in milliseconds).
unsigned long lastTime_LoraSend = 0;
unsigned long lastTime_full = 0;
const int ledTicPin = PIN_PD1;
const int ledInfoPin = PIN_PD2;
const int vcapPin = PIN_PD0;

const long frequency = 868E6; // LoRa Frequency
const int csPin = PIN_PA7; //7;          // LoRa radio chip select
const int resetPin = PIN_PF6; //2;       // LoRa radio reset
const int irqPin = PIN_PA3; //3;         // change for your board; must be a hardware interrupt pin

const char char_ADCO[] PROGMEM = "ADCO";
const char char_ADSC[] PROGMEM = "ADSC";

bool flag_counter_addr = false;
bool flag_tic_standard = false;
bool flag_first_run = true;
bool full_tic = true;

//char cpt_id[PAYLOAD_VALUE_LEN];
uint32_t LoraToA = 0;

_Mode_e mode_tic;
TInfo tinfo;
PayloadData payload_data;
auto led = JLed(PIN_PD2).Breathe(2000).DelayAfter(300).Forever();


// ---------------------------------------------------------------- 
// calculLoRaToA
// Calcule le temps entre deux émissions (en ms) en fonction des paramètres LoRa
// ----------------------------------------------------------------
uint32_t calculLoRaToA (
  uint8_t payloadLength,     // longueur du payload (en octets)
  uint8_t spreadingFactor,   // SF7 à SF12
  uint32_t bandwidth,        // en Hz (ex : 125000)
  uint8_t codingRate,        // CR = 1 pour 4/5, 2 pour 4/6, etc.
  uint8_t preambleLength,    // en symboles (souvent 8)
  bool crcOn,                // true si CRC activé
  bool headerOn,             // true si header explicite
  float dutyCyclePercent     // ex: 1.0 pour 1% duty cycle
) {
  float tSym = pow(2, spreadingFactor) / (float)bandwidth * 1000.0; // ms

  // Low data rate optimization
  bool lowDataRate = (spreadingFactor >= 11);
  uint8_t DE = lowDataRate ? 1 : 0;

  uint8_t IH = headerOn ? 0 : 1; // 0 = header explicite

  // Calcul de la taille du payload en symboles
  float payloadSymbNb = 8.0 +
    max(
      ceil(
        (8.0 * payloadLength + (crcOn ? 16 : 0) + (IH ? 0 : 20) - 4.0 * spreadingFactor) /
        (4.0 * (spreadingFactor - 2.0 * DE))
      ) * (codingRate + 4),
      0.0
    );

  // Temps du préambule
  float tPreamble = (preambleLength + 4.25) * tSym;

  // Temps total d’émission (ToA)
  float tOnAir = tPreamble + payloadSymbNb * tSym;

  // Calcul du délai minimum entre deux transmissions (en ms)
  float interval = tOnAir / (dutyCyclePercent / 100.0);

  return (uint32_t)ceil(interval); // en ms
}


// ---------------------------------------------------------------- 
// rtrim
// ---------------------------------------------------------------- 
void rtrim(char *str) {
    int len = strlen(str);
    while (len > 0 && isspace((unsigned char)str[len - 1])) {
        str[--len] = '\0';
    }
}

// ---------------------------------------------------------------- 
// calculEnergy
// ---------------------------------------------------------------- 
float calculEnergy(float tensionIntitiale, float tensionFinale) {
  debug.print("Tinit:");
  debug.print(tensionIntitiale, 3);
  debug.print("V, Tend:");
  debug.print(tensionFinale, 3);
  debug.println("V.");
  return 0.5 * CAPACITANCE * (sq(tensionIntitiale) - sq(tensionFinale));
}

// ---------------------------------------------------------------- 
// lectureTensionCapa
// ---------------------------------------------------------------- 
float lectureTensionCapa() {
	int valeurBrute = analogRead(vcapPin); 
    float tensionCapa = (valeurBrute / 1023.0) * 6.6; 
	
	return tensionCapa;
}

// ---------------------------------------------------------------- 
// verifSuperCapa
// ---------------------------------------------------------------- 
bool verifSuperCapa(float voltageRef) {
  bool rc = false;
  static unsigned long tempo = 0;
  
  float tensionCapa = lectureTensionCapa();
  if (tensionCapa > voltageRef) {
      rc = true;
  }
  if (millis() - tempo > 5000) {  // affiche tension toutes les 5 secondes
    tempo = millis();

	  debug.print("Tension capa:");
	  debug.println(tensionCapa, 3);
  }
  led.Update();

  if (rc) led.On();
  
  return rc;
}


// ---------------------------------------------------------------- 
// clignoteLed
// ---------------------------------------------------------------- 
void clignoteLed(uint8_t led, uint8_t nbr, int16_t delai)
{
int led_state=0;

  for (int i=0; i<nbr*2; i++) {
    led_state = !led_state;
    digitalWrite(led, led_state);
    delay(delai);
  }
  digitalWrite(led, LOW);
}

// ---------------------------------------------------------------- 
// changeEtatLed
// ---------------------------------------------------------------- 
void changeEtatLed(uint8_t led)
{
  static uint8_t led_state;

  led_state = !led_state;
  digitalWrite(led, led_state);

}

// ---------------------------------------------------------------- 
// initSpeedTIC
// ---------------------------------------------------------------- 
_Mode_e initSpeedTIC()
{
boolean flag_timeout = false;
boolean flag_found_speed = false;
uint32_t currentTime = millis();
unsigned step = 0;
int nbc_etiq=0;
int nbc_val=0;
_Mode_e mode;

 
  digitalWrite(ledInfoPin, LOW);
  
  // Test en mode historique
  // Recherche des éléments de début, milieu et fin de trame (0x0A, 0x20, 0x20, 0x0D)
  tic.begin(1200); // mode historique
  debug.println(F("Recherche mode TIC"));

  while (!flag_timeout && !flag_found_speed) {
    if (tic.available()>0) {
      char in = (char)tic.read() & 127;  // seulement sur 7 bits
       
      digitalWrite(ledTicPin, HIGH);
    
      #ifdef DEBUG_TIC
      debug.print(in, HEX);
      debug.print(".");
      #endif
        
      // début trame
      if (in == 0x0A) {
        step = 1;
        nbc_etiq = -1;
        nbc_val = 0;
        #ifdef DEBUG_TIC
          debug.println(F("Deb 0x0A"));
        #endif
      }
      
      // premier milieu de trame, étiquette
      if (step == 1) {
        if (in == 0x20) {
          #ifdef DEBUG_TIC
            debug.print(F("Etq 0x20:"));
            debug.println(nbc_etiq);
          #endif
          if (nbc_etiq > 3 && nbc_etiq < 10) step = 2;
            else step = 0;
        }
        else nbc_etiq++; // recupère nombre caractères de l'étiquette
      }
      else {
        // deuxième milieu de trame, valeur
        if (step == 2) {
          if (in == 0x20) {
            #ifdef DEBUG_TIC
              debug.print(F("Val 0x20:"));
              debug.println(nbc_val);
            #endif
            if (nbc_val > 0 && nbc_val < 13) step = 3;
              else step = 0;
          }
          else nbc_val++; // recupère nombre caractères de la valeur
        }
      }

      // fin trame
      if (step == 3 && in == 0x0D) {
        #ifdef DEBUG_TIC
          debug.println(F("Fin 0x0D"));
        #endif
        flag_found_speed = true;
        step = 0;
      }
    }
    if (currentTime + 6000 <  millis()) {
      flag_timeout = true; // 6s de timeout
    }
  }

  if (flag_timeout == true && flag_found_speed == false) { // trame avec vistesse histo non trouvée donc passage en mode standard par defaut
     mode = TINFO_MODE_STANDARD;
     tic.end();
     tic.begin(9600); // mode standard
     
     debug.println(F(">> TIC mode standard <<"));
     flag_tic_standard = true;
     payload_data.sender_id = LORALINK_S;
     clignoteLed(ledTicPin, 3, 500);
  }
  else {
    mode = TINFO_MODE_HISTORIQUE;
    debug.println(F(">> TIC mode historique <<"));
    flag_tic_standard = false;
    payload_data.sender_id = LORALINK_H;
    clignoteLed(ledTicPin, 6, 500);
  }
  
  digitalWrite(ledTicPin, HIGH);
  digitalWrite(ledInfoPin, HIGH);

  return mode;
}

// ---------------------------------------------------------------- 
// sendMessageLora
//    Envoi trame de teleinfo
// ---------------------------------------------------------------- 
void sendMessageLora(char *name, char *value) {
  uint8_t buffer[sizeof(payload_data)]; 

  digitalWrite(ledTicPin, LOW);

  rtrim(name); // enlève les éventuels caractères blancs en fin de chaîne

  payload_data.label_id = find_label_id(name, flag_tic_standard);
  if (payload_data.label_id != 0) { // test validité id

	if (millis()-lastTime_LoraSend < LoraToA) {
		debug.print(F("wait Toa - "));
		debug.print(LoraToA);
		debug.print(F("ms."));
	}
	while (millis()-lastTime_LoraSend < LoraToA) {}; // Attendre pour être conforme à la réglementation Time Of Air LoRa.
	debug.println(F(" - ok."));
	
	debug.print(F("check Capa"));
    while (verifSuperCapa(VOLTAGE_CAPA_MIN) == false) {}
	debug.println(F(" - ok."));

    payload_set_value_str(&payload_data, value);
    payload_finalize(&payload_data);
    payload_serialize(&payload_data, buffer);
	
	lastTime_LoraSend = millis();
	
	debug.print(F("sendMessageLora:"));
	debug.print(name);
	debug.print(F("-"));
	debug.print(payload_data.label_id);
	debug.print(F("-"));
	debug.println(value);

	LoRa.beginPacket();                   // start packet
	LoRa.write(buffer, sizeof(payload_data));
	LoRa.endPacket();                     // finish packet and send it
	delay(10);
	LoRa.idle();

  }
  else {
    digitalWrite(ledInfoPin, HIGH);
    debug.print(F("❌ id invalide: "));
    debug.println(name);
    delay(100);
    digitalWrite(ledInfoPin, LOW);
  }

  digitalWrite(ledTicPin, HIGH);
}

// ---------------------------------------------------------------- 
// sendTeleinfo
//    Envoi trame de teleinfo
// ---------------------------------------------------------------- 
void sendTeleinfo(ValueList *vl_tic, bool all_tic)
{
	
  if (vl_tic) {

    //debug.print("sendTeleinfo:");
    //debug.println(vl_tic->flags);
	
    // parcours liste chainée vl_tic
    while (vl_tic->next) {
      vl_tic = vl_tic->next;
      // uniquement sur les nouvelles valeurs ou celles modifiées ou toutes
      if ( all_tic || ( vl_tic->flags & (TINFO_FLAGS_UPDATED | TINFO_FLAGS_ADDED) )) {
        //if (vl_tic->name && strlen(vl_tic->name) > 2 && vl_tic->value && strlen(vl_tic->value) > 1) {
          debug.print(F("Send "));
          debug.print(vl_tic->name);
          debug.print(F(":"));
          debug.println(vl_tic->value);
          sendMessageLora(vl_tic->name, vl_tic->value);
        //}
      }
    }
  }
}

// ---------------------------------------------------------------- 
// search_adress_teleinfo
//    Envoi trame de teleinfo
// ---------------------------------------------------------------- 
bool search_adress_teleinfo(ValueList *vl_tic)
{
bool rc = false;

  if (vl_tic) {
    // parcours liste chainée vl_tic
    while (vl_tic->next) {
      vl_tic = vl_tic->next;
      if (vl_tic->name && strlen(vl_tic->name) && vl_tic->value && strlen(vl_tic->value)) {
        if (strstr_P(vl_tic->name, char_ADCO) == 0 || strstr_P(vl_tic->name, char_ADSC) == 0) {
          
          debug.print(F("Adr cpt: "));
          debug.println(vl_tic->value);
          payload_set_counter_addr(&payload_data, vl_tic->value);
          rc = true;
          break;
         }
      }
    }
  }
  return rc;
}

// ---------------------------------------------------------------- 
// NewFrame 
// ---------------------------------------------------------------- 
void newFrame(ValueList *vl_tic)
{
  /*debug.print("newFrame:");
  debug.print(flag_counter_addr);
  debug.print("-");
  debug.println(full_tic);*/

  if (flag_counter_addr == false) flag_counter_addr = search_adress_teleinfo(vl_tic);
    else sendTeleinfo(vl_tic, full_tic);
    
  if (full_tic) full_tic = false;
}

// ---------------------------------------------------------------- 
// updatedFrame 
// ---------------------------------------------------------------- 
void updatedFrame(ValueList *vl_tic)
{

  /*debug.print("updatedFrame:");
  debug.print(flag_counter_addr);
  debug.print("-");
  debug.println(full_tic);*/

  if (flag_counter_addr == false) flag_counter_addr = search_adress_teleinfo(vl_tic);
    else sendTeleinfo(vl_tic, full_tic);
    
  if (full_tic) full_tic = false;
}


// ---------------------------------------------------------------- 
// signaleDeuxLeds
// ---------------------------------------------------------------- 
void signaleDeuxLeds(int countLed)
{
	for (int i=0; i<countLed; i++) {
		digitalWrite(ledTicPin, LOW);
    digitalWrite(ledInfoPin, LOW);
		delay(500);
		digitalWrite(ledTicPin, HIGH);
    digitalWrite(ledInfoPin, HIGH);
		delay(500);
	}
}

// ---------------------------------------------------------------- 
// setup
// ---------------------------------------------------------------- 
void setup() {
  unsigned int blinkCount = 0;
  uint8_t resetFlags = RSTCTRL.RSTFR;
   
  pinMode(ledTicPin, OUTPUT);
  pinMode(ledInfoPin, OUTPUT);

  digitalWrite(ledTicPin, HIGH);
  digitalWrite(ledInfoPin, HIGH);
  
  //debug.begin(115200);
  debug.begin(9600);
  
  debug.println(F(" ███████╗██████╗ ███████╗"));
  debug.println(F(" ██╔════╝██╔══██╗██╔════╝"));
  debug.println(F(" █████╗  ██████╔╝███████╗"));
  debug.println(F(" ██╔══╝  ██╔══██╗╚════██║"));
  debug.println(F(" ██║     ██████╔╝███████║"));
  debug.println(F(" ╚═╝     ╚═════╝ ╚══════╝"));
  debug.print(F(" LoraLink  v"));
  debug.println(VERSION);

  // Attente charge super capa -----------
  debug.print(F("Charge capa : "));
  while (verifSuperCapa(VOLTAGE_CAPA_MAX) == false) {}
  debug.println(F("ok"));
    
  // Lire le registre des causes de reset. ------------
  debug.print(F("Raison du reset : "));
  if (resetFlags & RSTCTRL_PORF_bm) {
    debug.println(F("Power-On Reset (POR)"));
    blinkCount = 2;
  }
  else if (resetFlags & RSTCTRL_BORF_bm) {
    debug.println(F("Brown-Out Reset (BOD)"));
    blinkCount = 3;
  } else if (resetFlags & RSTCTRL_EXTRF_bm) {
    debug.println(F("Reset externe"));
    blinkCount = 4;
  } else if (resetFlags & RSTCTRL_WDRF_bm) {
    debug.println(F("Reset du Watchdog (WDT)"));
    blinkCount = 5;
  } else if (resetFlags & RSTCTRL_SWRF_bm) {
    debug.println(F("Reset logiciel"));
    blinkCount = 6;
  } else {
	  debug.println("Inconnu");
  }
  // Effacer les flags pour éviter une détection erronée au prochain reset
  RSTCTRL.RSTFR = 0xFF;
  
  signaleDeuxLeds(blinkCount);
  
  // Demarrage module Lora ------
  debug.print(F("Init LoRa : "));
  LoRa.setPins(csPin, resetPin, irqPin);
  if (LoRa.begin(frequency)) {
	LoRa.setSpreadingFactor(LORA_SF);
    LoRa.setSignalBandwidth(LORA_BW);
    LoRa.setCodingRate4(LORA_CR);
    LoRa.setPreambleLength(LORA_PREAMBLE_LENGTH);
    LoRa.enableCrc();
    LoRa.setTxPower(LORA_POWER);
	
	LoRa.idle();
	
	debug.println(F("ok"));
  
	LoraToA = calculLoRaToA(sizeof(PayloadData), LORA_SF, LORA_BW, LORA_CR, LORA_PREAMBLE_LENGTH, true, true, 1.0);
	debug.print(F("Lora ToA="));
	debug.print(LoraToA);
	debug.println(F("ms."));
  }
  else {
    debug.println(F("ko!"));
    while (1) {
      digitalWrite(ledTicPin, LOW);
      digitalWrite(ledInfoPin, HIGH);
      delay(300);
      digitalWrite(ledTicPin, HIGH);
      digitalWrite(ledInfoPin, LOW);
		  delay(300);
	  }
  }

  debug.print(F("Init TIC : "));
  mode_tic = initSpeedTIC();

  //tinfo.attachData(dataCallback);
  tinfo.attachNewFrame(newFrame);
  tinfo.attachUpdatedFrame(updatedFrame);
 
  debug.println(F("ok"));

  digitalWrite(ledTicPin, HIGH);
  digitalWrite(ledInfoPin, HIGH);
  
  debug.println(F("-- Fin setup --"));
}


// ---------------------------------------------------------------- 
// loop
// ---------------------------------------------------------------- 
void loop() {
  uint32_t currentTime = millis();
  static char c;

  
  if (tic.available()) {
    c = tic.read();
    if (c!=TINFO_STX && c!=TINFO_ETX) debug.print((char)(c & 127));
    tinfo.process(c);
  }

  if (flag_first_run == true) {
      if (flag_counter_addr == true) {
        flag_first_run = false;
        sendMessageLora("BOOT", VERSION);
      }
  }
  else {
    if (currentTime - lastTime_full > SEND_FREQUENCY_FULL) {
      full_tic = true;
      lastTime_full = currentTime;
    }
  }
}