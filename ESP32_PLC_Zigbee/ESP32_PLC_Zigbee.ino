// =============================================================================
// ESP32-PLC 100 (ESP32-C6) – Zigbee Router
// Rele    : EP 1–6   → ZigbeeLight (on/off da zigbee2mqtt)
// DIN 1–4 : EP 11–14 → ZigbeeLight (sola lettura, stato imposto dal poll)
// AIN 1–4 : EP 21–24 → ZigbeeLight (sola lettura, soglia 5 V)
// Watchdog: EP 31 Heartbeat (HA lo commuta ogni minuto)
//           EP 32 WD-Status (ON = watchdog scattato, HA lo riporta a OFF)
//           EP 33 WD-Enable (OFF = modalita' ombra, ON = riavvii reali)
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
#include <Preferences.h>
#include <esp_system.h>

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
bool       g_rtcOk = false;

// ---------------------------------------------------------------------------
// Watchdog di rete (heartbeat da Home Assistant)
//
//  - HA commuta l'endpoint Heartbeat (EP 31) ogni minuto. Qualunque comando
//    ricevuto (heartbeat o rele') vale come "segnale di vita".
//  - Il controllo si ARMA solo al primo heartbeat ricevuto (salvato in NVS):
//    finche' HA non e' configurato la scheda non si riavvia per mancanza di
//    heartbeat.
//  - Scatta se: (armato e nessun segnale da troppo tempo) oppure
//    (non connessa alla rete da troppo tempo).
//  - Timeout crescenti: 15, 30, 60 min, poi un solo tentativo ogni 6 ore.
//    Il contatore sta in NVS e si azzera al primo segnale di vita.
//  - MODALITA' OMBRA (default): invece di riavviare accende WD-Status
//    (EP 32). I riavvii reali si abilitano da HA con WD-Enable (EP 33),
//    senza ricompilare.
// ---------------------------------------------------------------------------
#define WD_LEVEL1_MS  (15UL * 60UL * 1000UL)
#define WD_LEVEL2_MS  (30UL * 60UL * 1000UL)
#define WD_LEVEL3_MS  (60UL * 60UL * 1000UL)
#define WD_LONG_MS    (6UL * 60UL * 60UL * 1000UL)
#define WD_TIME_DIV   1    // prove al banco: 60 => i minuti diventano secondi

ZigbeeLight zbHeartbeat(31);
ZigbeeLight zbWdStatus(32);
ZigbeeLight zbWdEnable(33);

Preferences prefs;

// Stato persistente (NVS, namespace "plcwd")
bool    g_armed        = false;
bool    g_wdEnabled    = false;   // false = modalita' ombra
uint8_t g_restartCount = 0;
bool    g_wdFlag       = false;   // evento non ancora confermato da HA

// Stato runtime (i flag volatile arrivano dai callback del task Zigbee)
volatile uint32_t g_lastLifeMs  = 0;
volatile bool     g_lifePending = false;
volatile bool     g_hbPending   = false;
volatile bool     g_enPending   = false;
volatile bool     g_enTarget    = false;
volatile bool     g_ackPending  = false;
uint32_t g_lastConnectedMs = 0;
bool     g_wasConnected    = false;
bool     g_shadowFired     = false;

volatile uint32_t g_rxCount = 0;   // comandi ricevuti da Zigbee (diagnostica)

inline void lifeSignal() { g_lastLifeMs = millis(); g_rxCount = g_rxCount + 1; g_lifePending = true; }

// ---------------------------------------------------------------------------
// Pending relay (thread-safe: i callback Zigbee arrivano da altro task RTOS)
// ---------------------------------------------------------------------------
volatile bool g_relayPending[7] = {};
volatile bool g_relayTarget[7]  = {};

// ---------------------------------------------------------------------------
// Uscite rele' tramite espansore I2C (PCA9557), scrittura "a prova di glitch"
//
// Se l'espansore subisce un disturbo (rele' che commutano, brownout) puo'
// perdere la configurazione e tornare con tutti i pin in ingresso: i rele'
// smettono di rispondere mentre il resto della scheda (Zigbee compreso)
// continua a funzionare. Per questo:
//  - lo stato desiderato dei 6 rele' viene riscritto per intero ogni
//    RELAY_REFRESH_MS, insieme alla configurazione dei pin;
//  - prima della riscrittura periodica si legge la configurazione: se e'
//    cambiata lo segnala sul seriale (contatore expReset);
//  - dopo 3 scritture consecutive fallite si prova a liberare il bus I2C.
// ---------------------------------------------------------------------------
#define PCA_CONFIG_VALUE  0b10000001   // bit0 e bit7 ingressi, bit1..6 uscite (rele')
#define RELAY_REFRESH_MS  2000

uint32_t g_i2cErrTotal   = 0;
uint32_t g_expResetTotal = 0;
uint8_t  g_i2cErrRun     = 0;

void i2cRecover() {
  Serial.println("[I2C] scritture fallite di seguito: recupero del bus");
  Wire.end();
  pinMode(SDA_PIN, INPUT_PULLUP);
  pinMode(SCL_PIN, OUTPUT);
  for (int i = 0; i < 9; i++) {          // 9 impulsi di clock sbloccano uno slave bloccato
    digitalWrite(SCL_PIN, LOW);  delayMicroseconds(10);
    digitalWrite(SCL_PIN, HIGH); delayMicroseconds(10);
  }
  Wire.begin(SDA_PIN, SCL_PIN);
}

bool applyRelayOutputs(bool verifyConfig) {
  if (verifyConfig) {
    uint8_t cfg = 0;
    if (io.read_register(0x03, &cfg) && cfg != PCA_CONFIG_VALUE) {
      g_expResetTotal++;
      Serial.printf("[I2C] configurazione espansore cambiata (0x%02X): riconfiguro\n", cfg);
    }
  }
  uint8_t mask = 0;
  for (uint8_t i = 1; i <= 6; i++) {
    if (g_relayTarget[i]) mask |= (1 << i);
  }
  // prima il valore delle uscite, poi la direzione dei pin: niente impulsi sui rele'
  bool ok = io.write_register(0x01, mask);
  ok = io.write_register(0x03, PCA_CONFIG_VALUE) && ok;
  return ok;
}

void onRelay1Change(bool s) { g_relayTarget[1]=s; g_relayPending[1]=true; lifeSignal(); }
void onRelay2Change(bool s) { g_relayTarget[2]=s; g_relayPending[2]=true; lifeSignal(); }
void onRelay3Change(bool s) { g_relayTarget[3]=s; g_relayPending[3]=true; lifeSignal(); }
void onRelay4Change(bool s) { g_relayTarget[4]=s; g_relayPending[4]=true; lifeSignal(); }
void onRelay5Change(bool s) { g_relayTarget[5]=s; g_relayPending[5]=true; lifeSignal(); }
void onRelay6Change(bool s) { g_relayTarget[6]=s; g_relayPending[6]=true; lifeSignal(); }

// Callback watchdog (chiamati dal task Zigbee: solo flag, niente NVS/I2C qui)
void onHeartbeat(bool)        { g_hbPending = true; lifeSignal(); }
void onWdStatusChange(bool s) { if (!s) g_ackPending = true; }      // HA conferma -> OFF
void onWdEnableChange(bool s) { g_enTarget = s; g_enPending = true; }

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
// Factory reset Zigbee: tieni premuto BOOT per piu' di 3 s.
// Chiamata sia nel loop sia durante l'attesa della rete in setup, cosi' si
// puo' fare il reset anche quando la scheda non riesce a connettersi.
// ---------------------------------------------------------------------------
void checkFactoryReset() {
  if (digitalRead(BOOT_PIN) != LOW) return;
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

// ---------------------------------------------------------------------------
// Watchdog: funzioni
// ---------------------------------------------------------------------------
uint32_t wdTimeoutMs() {
  uint32_t ms;
  switch (g_restartCount) {
    case 0:  ms = WD_LEVEL1_MS; break;
    case 1:  ms = WD_LEVEL2_MS; break;
    case 2:  ms = WD_LEVEL3_MS; break;
    default: ms = WD_LONG_MS;   break;
  }
  return ms / WD_TIME_DIV;
}

void setWdFlag(bool v) {
  g_wdFlag = v;
  prefs.putBool("flag", v);
  zbWdStatus.setLight(v);
}

// Da chiamare spesso (loop e attesa connessione in setup)
void wdService() {
  // 1. Eventi arrivati dai callback, gestiti qui fuori dal task Zigbee
  if (g_lifePending) {
    g_lifePending = false;
    g_shadowFired = false;
    if (g_restartCount != 0) {           // scrive in NVS solo se serve
      g_restartCount = 0;
      prefs.putUChar("rcnt", 0);
      Serial.println("[WD] segnale di vita: contatore riavvii azzerato");
    }
  }
  if (g_hbPending) {
    g_hbPending = false;
    if (!g_armed) {
      g_armed = true;
      prefs.putBool("armed", true);
      Serial.println("[WD] ARMATO (primo heartbeat ricevuto)");
    }
  }
  if (g_enPending) {
    g_enPending = false;
    bool t = g_enTarget;
    if (t != g_wdEnabled) {
      g_wdEnabled = t;
      prefs.putBool("en", t);
      Serial.printf("[WD] riavvii reali %s\n", t ? "ABILITATI" : "disabilitati (modalita' ombra)");
    }
  }
  if (g_ackPending) {
    g_ackPending = false;
    if (g_wdFlag) { g_wdFlag = false; prefs.putBool("flag", false); }
  }

  // 2. Stato di rete
  bool connected = Zigbee.connected();
  uint32_t life  = g_lastLifeMs;         // letto PRIMA di millis(): niente sottrazioni negative
  uint32_t now   = millis();
  if (connected) {
    g_lastConnectedMs = now;
    if (!g_wasConnected) {               // (ri)connessione: ripubblica lo stato
      zbWdEnable.setLight(g_wdEnabled);
      zbWdStatus.setLight(g_wdFlag);
    }
  }
  g_wasConnected = connected;

  // 3. Condizioni di scatto
  uint32_t to = wdTimeoutMs();
  bool noLife = g_armed && (now - life > to);
  bool noNet  = (now - g_lastConnectedMs > to);
  if (!noLife && !noNet) return;

  const char *why = noNet ? "rete assente" : "heartbeat assente";

  if (!g_wdEnabled) {                    // MODALITA' OMBRA: segnala e basta
    if (!g_shadowFired) {
      g_shadowFired = true;
      Serial.printf("[WD] (ombra) sarebbe scattato: %s da oltre %lu s\n", why, (unsigned long)(to / 1000UL));
      setWdFlag(true);
    }
    return;
  }

  // RIAVVIO REALE (con backoff: il contatore sale a ogni riavvio)
  if (g_restartCount < 250) g_restartCount++;
  prefs.putUChar("rcnt", g_restartCount);
  prefs.putBool("flag", true);
  Serial.printf("[WD] RIAVVIO #%u: %s\n", (unsigned)g_restartCount, why);
  Serial.flush();
  delay(300);
  ESP.restart();
}

// ---------------------------------------------------------------------------
// setup()
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== ESP32-PLC Zigbee ===");

  // Watchdog: stato persistente
  prefs.begin("plcwd", false);
  g_armed        = prefs.getBool("armed", false);
  g_wdEnabled    = prefs.getBool("en", false);      // default: modalita' ombra
  g_restartCount = prefs.getUChar("rcnt", 0);
  g_wdFlag       = prefs.getBool("flag", false);
  // motivo reset: 1=power-on 3=riavvio sw 4=panic 5/6/7=watchdog 9=brownout
  Serial.printf("[BOOT] motivo_reset=%d armato=%d riavvii_reali=%d contatore=%u\n",
                (int)esp_reset_reason(), g_armed, g_wdEnabled, (unsigned)g_restartCount);

  Wire.begin(SDA_PIN, SCL_PIN);
  io.write_register(0x03, 0b10000001);  // bit1-6 output (rele)
  io.write_register(0x01, 0b00000000);  // tutti i rele OFF

  g_rtcOk = rtc.begin();                  // una sola volta: non va richiamato nel loop
  if (!g_rtcOk) {
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

  // -- Watchdog (heartbeat / stato / abilitazione) --
  zbHeartbeat.setManufacturerAndModel("ESP32-PLC", "WD-Heartbeat"); zbHeartbeat.onLightChange(onHeartbeat);       Zigbee.addEndpoint(&zbHeartbeat);
  zbWdStatus.setManufacturerAndModel("ESP32-PLC", "WD-Status");     zbWdStatus.onLightChange(onWdStatusChange);   Zigbee.addEndpoint(&zbWdStatus);
  zbWdEnable.setManufacturerAndModel("ESP32-PLC", "WD-Enable");     zbWdEnable.onLightChange(onWdEnableChange);   Zigbee.addEndpoint(&zbWdEnable);

  // -- Zigbee stack --
  // Zigbee2MQTT fa un bind (genOnOff) per ogni endpoint. La tabella di binding
  // della scheda ha 16 posti di default: con 17 endpoint il 17esimo bind fallisce
  // con TABLE_FULL e il dispositivo resta configurato a meta'.
  // Va chiamata PRIMA di Zigbee.begin() (che esegue esp_zb_init).
  // Se non compila o z2m continua a dare TABLE_FULL: togli questa riga e riduci
  // il numero di endpoint a 16 o meno.
  esp_zb_aps_src_binding_table_size_set(32);

  Serial.println("[INFO] Avvio Zigbee stack...");
  if (!Zigbee.begin(ZIGBEE_ROUTER)) {
    Serial.println("[ERROR] Zigbee.begin() fallito - reboot");
    delay(1000);
    ESP.restart();
  }

  Serial.print("[INFO] Connessione alla rete Zigbee");
  g_lastConnectedMs = millis();
  g_lastLifeMs      = millis();
  while (!Zigbee.connected()) {
    Serial.print(".");
    delay(200);
    checkFactoryReset();   // BOOT > 3 s: reset Zigbee anche se non si connette
    wdService();           // riavvia (con backoff) solo se i riavvii reali sono abilitati
  }
  Serial.println("\n[INFO] Connesso!");
  g_lastConnectedMs = millis();
  g_lastLifeMs      = millis();
}

// ---------------------------------------------------------------------------
// loop()
// ---------------------------------------------------------------------------
void loop() {
  // 1. Rele': applica subito le variazioni e riscrivi tutto ogni RELAY_REFRESH_MS
  bool relayChanged = false;
  for (uint8_t i = 1; i <= 6; i++) {
    if (g_relayPending[i]) {
      g_relayPending[i] = false;
      relayChanged = true;
      Serial.printf("[RELAY] %d -> %s\n", i, g_relayTarget[i] ? "ON" : "OFF");
    }
  }
  static uint32_t lastRelayRefresh = 0;
  if (relayChanged || millis() - lastRelayRefresh >= RELAY_REFRESH_MS) {
    lastRelayRefresh = millis();
    if (applyRelayOutputs(!relayChanged)) {
      g_i2cErrRun = 0;
    } else {
      g_i2cErrTotal++;
      if (++g_i2cErrRun >= 3) { i2cRecover(); g_i2cErrRun = 0; }
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
  checkFactoryReset();

  // 4. Watchdog di rete
  wdService();

  // 5. Debug ogni 5s
  static uint32_t lastDbg = 0;
  if (millis() - lastDbg >= 5000) {
    lastDbg = millis();
    char ts[24] = "RTC-ERR";
    if (g_rtcOk) {
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
    Serial.printf("[WD] armato=%d riavvii_reali=%d contatore=%u ultimo_segnale=%lus fa\n",
      g_armed, g_wdEnabled, (unsigned)g_restartCount,
      (unsigned long)((millis() - g_lastLifeMs) / 1000UL));
    // diagnostica: rx = comandi Zigbee ricevuti, heap = memoria libera (se scende sempre: perdita)
    Serial.printf("[SYS] uptime=%lus heap=%u minheap=%u rx=%lu i2cErr=%lu expReset=%lu\n",
      (unsigned long)(millis() / 1000UL),
      (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMinFreeHeap(),
      (unsigned long)g_rxCount, (unsigned long)g_i2cErrTotal, (unsigned long)g_expResetTotal);
  }
}
