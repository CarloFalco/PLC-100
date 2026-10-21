// =============================================================================
// ESP32-PLC 100 (ESP32-C6) – Zigbee End Device
// Rele    : EP 1–6   → ZigbeeLight (on/off da zigbee2mqtt)
// DIN 1–4 : EP 11–14 → ZigbeeLight (sola lettura, stato imposto dal poll)
// AIN 1–4 : EP 21–24 → ZigbeeLight (sola lettura, soglia 5 V)
//
// Prerequisiti Arduino IDE:
//   Board           : esp32 by Espressif >= 3.x
//   Tools Zigbee    : Zigbee ZCZR (Router)
//   Partition Scheme: Zigbee 4MB with spiffs
//   Librerie        : RTClib
// =============================================================================

#ifndef ZIGBEE_MODE_ZCZR
#error "Seleziona 'Zigbee ZCZR' in Tools -> Zigbee mode"
#endif

#include "Zigbee.h"
#include <Wire.h>
#include <RTClib.h>

// ---------------------------------------------------------------------------
// PCA9557 – expander I2C per i rele
// ---------------------------------------------------------------------------
class PCA9557 {
public:
  PCA9557(uint8_t address = 0x18, TwoWire *bus = &Wire)
    : _addr(address), _wire(bus) {}

  bool read_register(uint8_t reg, uint8_t *value) {
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    if (_wire->endTransmission(false) != 0) return false;
    if (_wire->requestFrom(_addr, (uint8_t)1) != 1) return false;
    *value = _wire->read();
    return true;
  }
  bool write_register(uint8_t reg, uint8_t value) {
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    _wire->write(value);
    return (_wire->endTransmission() == 0);
  }
  bool digitalWrite(uint8_t pin, int value) {
    uint8_t out;
    if (!read_register(0x01, &out)) return false;
    if (value == HIGH) out |= (1 << pin);
    else               out &= ~(1 << pin);
    return write_register(0x01, out);
  }
  int digitalRead(uint8_t pin) {
    uint8_t in;
    if (!read_register(0x00, &in)) return -1;
    return (in & (1 << pin)) ? HIGH : LOW;
  }

private:
  uint8_t  _addr;
  TwoWire *_wire;
};

// ---------------------------------------------------------------------------
// Hardware
// ---------------------------------------------------------------------------
#define GPIO_EXPANDER_ADDR  0x1A
#define SDA_PIN             6
#define SCL_PIN             7

#define DIN1_PIN  4
#define DIN2_PIN  5
#define DIN3_PIN  10
#define DIN4_PIN  11

#define AIN1_PIN  0
#define AIN2_PIN  1
#define AIN3_PIN  2
#define AIN4_PIN  3

#ifndef BOOT_PIN
  #define BOOT_PIN 9
#endif

// Ingressi analogici: scala ADC (0-3300 mV) -> tensione reale sul pin
// Modifica AIN_SCALE_FACTOR per il range effettivo della scheda (es. 0-10 V)
#define AIN_SCALE_FACTOR  (10000.0f / 3300.0f)
#define AIN_THRESHOLD_MV  5000   // soglia 5 V
#define AIN_HYST_MV       200    // isteresi +/- 200 mV

#define INPUT_POLL_MS  100

// ---------------------------------------------------------------------------
// Endpoint Zigbee – tutti ZigbeeLight per compatibilita' z2m multi-endpoint
//   Rele  : EP 1-6
//   DIN   : EP 11-14  (sola lettura: lo stato e' imposto dal firmware)
//   AIN   : EP 21-24  (sola lettura: lo stato e' imposto dal firmware)
// ---------------------------------------------------------------------------

// Rele
ZigbeeLight zbRelay1(1);
ZigbeeLight zbRelay2(2);
ZigbeeLight zbRelay3(3);
ZigbeeLight zbRelay4(4);
ZigbeeLight zbRelay5(5);
ZigbeeLight zbRelay6(6);

// Ingressi digitali
ZigbeeLight zbDin1(11);
ZigbeeLight zbDin2(12);
ZigbeeLight zbDin3(13);
ZigbeeLight zbDin4(14);

// Ingressi analogici
ZigbeeLight zbAin1(21);
ZigbeeLight zbAin2(22);
ZigbeeLight zbAin3(23);
ZigbeeLight zbAin4(24);

// ---------------------------------------------------------------------------
// Periferiche
// ---------------------------------------------------------------------------
PCA9557    io(GPIO_EXPANDER_ADDR, &Wire);
RTC_DS3231 rtc;

// ---------------------------------------------------------------------------
// Pending relay (thread-safe: i callback Zigbee arrivano da altro task RTOS)
// ---------------------------------------------------------------------------
volatile bool g_relayPending[7] = {};
volatile bool g_relayTarget[7]  = {};

void writeRelay(uint8_t relay, bool state) {
  if (relay < 1 || relay > 6) return;
  io.digitalWrite(relay, state ? HIGH : LOW);
}

void onRelay1Change(bool s) { g_relayTarget[1]=s; g_relayPending[1]=true; }
void onRelay2Change(bool s) { g_relayTarget[2]=s; g_relayPending[2]=true; }
void onRelay3Change(bool s) { g_relayTarget[3]=s; g_relayPending[3]=true; }
void onRelay4Change(bool s) { g_relayTarget[4]=s; g_relayPending[4]=true; }
void onRelay5Change(bool s) { g_relayTarget[5]=s; g_relayPending[5]=true; }
void onRelay6Change(bool s) { g_relayTarget[6]=s; g_relayPending[6]=true; }

// Callback vuoti per gli endpoint di ingresso:
// z2m potrebbe inviare comandi, li ignoriamo (il poll sovrascrive lo stato)
void onDinChange(bool) {}
void onAinChange(bool) {}

// ---------------------------------------------------------------------------
// Helper analogici
// ---------------------------------------------------------------------------
inline int adcToRealMv(uint32_t adcMv) {
  return (int)(adcMv * AIN_SCALE_FACTOR);
}
bool analogThreshold(bool prev, int realMv) {
  if (!prev && realMv >= AIN_THRESHOLD_MV + AIN_HYST_MV) return true;
  if ( prev && realMv <  AIN_THRESHOLD_MV - AIN_HYST_MV) return false;
  return prev;
}

// ---------------------------------------------------------------------------
// setup()
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== ESP32-PLC Zigbee ===");

  Wire.begin(SDA_PIN, SCL_PIN);
  io.write_register(0x03, 0b10000001);  // bit1-6 output (rele)
  io.write_register(0x01, 0b00000000);  // tutti i rele OFF

  if (!rtc.begin()) {
    Serial.println("[WARN] RTC non trovato");
  } else if (rtc.lostPower()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  pinMode(DIN1_PIN, INPUT);
  pinMode(DIN2_PIN, INPUT);
  pinMode(DIN3_PIN, INPUT);
  pinMode(DIN4_PIN, INPUT);
  pinMode(BOOT_PIN, INPUT_PULLUP);

  // -- Rele --
  zbRelay1.setManufacturerAndModel("ESP32-PLC", "Relay-1"); zbRelay1.onLightChange(onRelay1Change); Zigbee.addEndpoint(&zbRelay1);
  zbRelay2.setManufacturerAndModel("ESP32-PLC", "Relay-2"); zbRelay2.onLightChange(onRelay2Change); Zigbee.addEndpoint(&zbRelay2);
  zbRelay3.setManufacturerAndModel("ESP32-PLC", "Relay-3"); zbRelay3.onLightChange(onRelay3Change); Zigbee.addEndpoint(&zbRelay3);
  zbRelay4.setManufacturerAndModel("ESP32-PLC", "Relay-4"); zbRelay4.onLightChange(onRelay4Change); Zigbee.addEndpoint(&zbRelay4);
  zbRelay5.setManufacturerAndModel("ESP32-PLC", "Relay-5"); zbRelay5.onLightChange(onRelay5Change); Zigbee.addEndpoint(&zbRelay5);
  zbRelay6.setManufacturerAndModel("ESP32-PLC", "Relay-6"); zbRelay6.onLightChange(onRelay6Change); Zigbee.addEndpoint(&zbRelay6);

  // -- Ingressi digitali --
  zbDin1.setManufacturerAndModel("ESP32-PLC", "DIN-1"); zbDin1.onLightChange(onDinChange); Zigbee.addEndpoint(&zbDin1);
  zbDin2.setManufacturerAndModel("ESP32-PLC", "DIN-2"); zbDin2.onLightChange(onDinChange); Zigbee.addEndpoint(&zbDin2);
  zbDin3.setManufacturerAndModel("ESP32-PLC", "DIN-3"); zbDin3.onLightChange(onDinChange); Zigbee.addEndpoint(&zbDin3);
  zbDin4.setManufacturerAndModel("ESP32-PLC", "DIN-4"); zbDin4.onLightChange(onDinChange); Zigbee.addEndpoint(&zbDin4);

  // -- Ingressi analogici --
  zbAin1.setManufacturerAndModel("ESP32-PLC", "AIN-1"); zbAin1.onLightChange(onAinChange); Zigbee.addEndpoint(&zbAin1);
  zbAin2.setManufacturerAndModel("ESP32-PLC", "AIN-2"); zbAin2.onLightChange(onAinChange); Zigbee.addEndpoint(&zbAin2);
  zbAin3.setManufacturerAndModel("ESP32-PLC", "AIN-3"); zbAin3.onLightChange(onAinChange); Zigbee.addEndpoint(&zbAin3);
  zbAin4.setManufacturerAndModel("ESP32-PLC", "AIN-4"); zbAin4.onLightChange(onAinChange); Zigbee.addEndpoint(&zbAin4);

  // -- Zigbee stack --
  Serial.println("[INFO] Avvio Zigbee stack...");
  if (!Zigbee.begin(ZIGBEE_ROUTER)) {
    Serial.println("[ERROR] Zigbee.begin() fallito - reboot");
    delay(1000);
    ESP.restart();
  }

  Serial.print("[INFO] Connessione alla rete Zigbee");
  uint32_t t0 = millis();
  while (!Zigbee.connected()) {
    Serial.print(".");
    delay(200);
    if (millis() - t0 > 30000) {
      Serial.println("\n[WARN] Timeout - reboot");
      delay(500);
      ESP.restart();
    }
  }
  Serial.println("\n[INFO] Connesso!");
}

// ---------------------------------------------------------------------------
// loop()
// ---------------------------------------------------------------------------
void loop() {
  // 1. Scritture rele pendenti
  for (uint8_t i = 1; i <= 6; i++) {
    if (g_relayPending[i]) {
      g_relayPending[i] = false;
      bool s = g_relayTarget[i];
      writeRelay(i, s);
      Serial.printf("[RELAY] %d -> %s\n", i, s ? "ON" : "OFF");
    }
  }

  // 2. Polling ingressi
  static bool     lastDin[5] = {};
  static bool     lastAin[5] = {};
  static uint32_t lastPoll   = 0;

  if (millis() - lastPoll >= INPUT_POLL_MS) {
    lastPoll = millis();

    // Digitali
    bool din[5];
    din[1] = (digitalRead(DIN1_PIN) == HIGH);
    din[2] = (digitalRead(DIN2_PIN) == HIGH);
    din[3] = (digitalRead(DIN3_PIN) == HIGH);
    din[4] = (digitalRead(DIN4_PIN) == HIGH);

    if (din[1] != lastDin[1]) { lastDin[1]=din[1]; zbDin1.setLight(din[1]); Serial.printf("[DIN] 1->%d\n", din[1]); }
    if (din[2] != lastDin[2]) { lastDin[2]=din[2]; zbDin2.setLight(din[2]); Serial.printf("[DIN] 2->%d\n", din[2]); }
    if (din[3] != lastDin[3]) { lastDin[3]=din[3]; zbDin3.setLight(din[3]); Serial.printf("[DIN] 3->%d\n", din[3]); }
    if (din[4] != lastDin[4]) { lastDin[4]=din[4]; zbDin4.setLight(din[4]); Serial.printf("[DIN] 4->%d\n", din[4]); }

    // Analogici
    int ain_mv[5];
    ain_mv[1] = adcToRealMv(analogReadMilliVolts(AIN1_PIN));
    ain_mv[2] = adcToRealMv(analogReadMilliVolts(AIN2_PIN));
    ain_mv[3] = adcToRealMv(analogReadMilliVolts(AIN3_PIN));
    ain_mv[4] = adcToRealMv(analogReadMilliVolts(AIN4_PIN));

    bool ain[5];
    ain[1] = analogThreshold(lastAin[1], ain_mv[1]);
    ain[2] = analogThreshold(lastAin[2], ain_mv[2]);
    ain[3] = analogThreshold(lastAin[3], ain_mv[3]);
    ain[4] = analogThreshold(lastAin[4], ain_mv[4]);

    if (ain[1] != lastAin[1]) { lastAin[1]=ain[1]; zbAin1.setLight(ain[1]); Serial.printf("[AIN] 1->%d (%d mV)\n", ain[1], ain_mv[1]); }
    if (ain[2] != lastAin[2]) { lastAin[2]=ain[2]; zbAin2.setLight(ain[2]); Serial.printf("[AIN] 2->%d (%d mV)\n", ain[2], ain_mv[2]); }
    if (ain[3] != lastAin[3]) { lastAin[3]=ain[3]; zbAin3.setLight(ain[3]); Serial.printf("[AIN] 3->%d (%d mV)\n", ain[3], ain_mv[3]); }
    if (ain[4] != lastAin[4]) { lastAin[4]=ain[4]; zbAin4.setLight(ain[4]); Serial.printf("[AIN] 4->%d (%d mV)\n", ain[4], ain_mv[4]); }
  }

  // 3. Factory reset BOOT > 3s
  if (digitalRead(BOOT_PIN) == LOW) {
    delay(100);
    uint32_t t0 = millis();
    while (digitalRead(BOOT_PIN) == LOW) {
      delay(50);
      if ((millis() - t0) > 3000) {
        Serial.println("[INFO] Factory reset Zigbee...");
        delay(500);
        Zigbee.factoryReset();
      }
    }
  }

  // 4. Debug ogni 5s
  static uint32_t lastDbg = 0;
  if (millis() - lastDbg >= 5000) {
    lastDbg = millis();
    char ts[24] = "RTC-ERR";
    if (rtc.begin()) {
      DateTime now = rtc.now();
      snprintf(ts, sizeof(ts), "%04d-%02d-%02d %02d:%02d:%02d",
               now.year(), now.month(), now.day(),
               now.hour(), now.minute(), now.second());
    }
    Serial.printf("[DBG] %s DIN:%d%d%d%d AIN:%d%d%d%d RLY:%d%d%d%d%d%d\n", ts,
      lastDin[1],lastDin[2],lastDin[3],lastDin[4],
      lastAin[1],lastAin[2],lastAin[3],lastAin[4],
      (int)g_relayTarget[1],(int)g_relayTarget[2],
      (int)g_relayTarget[3],(int)g_relayTarget[4],
      (int)g_relayTarget[5],(int)g_relayTarget[6]);
  }
}
