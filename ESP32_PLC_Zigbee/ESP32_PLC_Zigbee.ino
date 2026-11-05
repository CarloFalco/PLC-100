// =============================================================================
// ESP32-PLC 100 (ESP32-C6) – Zigbee Router
//
// Rele    : EP 1–6   → ZigbeeLight (on/off da zigbee2mqtt)
// DIN 1–4 : EP 11–14 → ZigbeeLight (sola lettura, stato imposto dal poll)
// AIN 1–4 : EP 21–24 → ZigbeeLight (sola lettura, soglia 5 V con isteresi)
// Watchdog: EP 31 Heartbeat (HA lo commuta ogni minuto)
//           EP 32 WD-Status (ON = watchdog scattato, HA lo riporta a OFF)
//           EP 33 WD-Enable (OFF = modalita' ombra, ON = riavvii reali)
//
// Prerequisiti Arduino IDE:
//   Board           : esp32 by Espressif >= 3.x
//   Tools Zigbee    : Zigbee ZCZR (Router)
//   Partition Scheme: Zigbee 4MB with spiffs
//   Librerie        : RTClib
//
// REVISIONE "I2C ROBUSTO"
//  Il guasto da risolvere: i rele' smettono di rispondere mentre Zigbee continua
//  a funzionare. La causa piu' probabile e' l'espansore PCA9557 che perde la
//  configurazione (pin tornati ingressi) o un bus I2C bloccato. Per questo:
//   1. stato DESIDERATO (da Zigbee) e stato APPLICATO (confermato dall'espansore)
//      sono separati; il loop li riconcilia di continuo, quindi un comando non
//      va mai perso se una scrittura fallisce;
//   2. ogni RELAY_REFRESH_MS si LEGGONO output, polarita' e configurazione
//      dell'espansore, si registra se sono cambiate (reset dell'espansore) e si
//      riscrive tutto; poi si rilegge per verificare TUTTI e tre i registri;
//   3. il bus viene sbloccato a livello di pin (9 impulsi di clock + STOP) prima
//      di ogni Wire.begin(), non basta Wire.end()/Wire.begin();
//   4. piu' rele' da cambiare vengono commutati uno alla volta, distanziati,
//      per non scaricare l'alimentazione con le correnti di spunto delle bobine;
//   5. se Zigbee e' vivo ma il bus non si riprende per I2C_FAIL_RESTART_MS,
//      riavvio controllato (massimo I2C_MAX_RESTARTS di fila, contati in NVS);
//   6. se la scrittura fallisce a lungo, lo stato pubblicato su Zigbee torna a
//      quello reale, cosi' Home Assistant vede la discrepanza e puo' reagire.
//
//  NB: la rilettura dell'espansore verifica i REGISTRI, non i contatti
//  meccanici dei rele' ne' la posizione delle valvole.
// =============================================================================

#ifndef ZIGBEE_MODE_ZCZR
#error "Seleziona 'Zigbee ZCZR' in Tools -> Zigbee mode"
#endif

#include "Zigbee.h"
#include <Wire.h>
#include <RTClib.h>
#include <Preferences.h>
#include <esp_system.h>
#include "freertos/FreeRTOS.h"

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

#define INPUT_POLL_MS       100
#define INPUT_REPUBLISH_MS  60000   // ripubblica gli ingressi anche se invariati
#define DEBUG_INTERVAL_MS   5000
#define RTC_PRINT_MS        60000

// ---------------------------------------------------------------------------
// Parametri I2C / espansore
// ---------------------------------------------------------------------------
#define I2C_CLOCK_HZ          100000   // 100 kHz: piu' tollerante ai disturbi di 400 kHz
#define I2C_TIMEOUT_MS        50

#define PCA_REG_INPUT         0x00
#define PCA_REG_OUTPUT        0x01
#define PCA_REG_POLARITY      0x02
#define PCA_REG_CONFIG        0x03

#define PCA_RELAY_MASK        0x7E     // bit 1..6 = rele'
#define PCA_CONFIG_VALUE      0x81     // 1 = ingresso: bit0 e bit7 ingressi, bit1..6 uscite

#define RELAY_REFRESH_MS      2000     // lettura+riscrittura+verifica periodica
#define RELAY_STAGGER_MS      80       // pausa tra due rele' che cambiano insieme
#define RELAY_SETTLE_MS       30       // attesa dopo la commutazione prima di rileggere

#define I2C_ATTEMPTS          3        // tentativi per ciclo di scrittura
#define I2C_PUBLISH_TRUTH_AFTER 2      // cicli falliti prima di correggere lo stato Zigbee
#define I2C_FAIL_RESTART_MS   60000UL  // I2C morto per tanto => riavvio controllato
#define I2C_MAX_RESTARTS      3        // riavvii consecutivi massimi per I2C
#define I2C_STABLE_CLEAR_MS   (10UL * 60UL * 1000UL)

// ---------------------------------------------------------------------------
// Endpoint Zigbee – tutti ZigbeeLight per compatibilita' z2m multi-endpoint
// ---------------------------------------------------------------------------
ZigbeeLight zbRelay1(1);
ZigbeeLight zbRelay2(2);
ZigbeeLight zbRelay3(3);
ZigbeeLight zbRelay4(4);
ZigbeeLight zbRelay5(5);
ZigbeeLight zbRelay6(6);

ZigbeeLight zbDin1(11);
ZigbeeLight zbDin2(12);
ZigbeeLight zbDin3(13);
ZigbeeLight zbDin4(14);

ZigbeeLight zbAin1(21);
ZigbeeLight zbAin2(22);
ZigbeeLight zbAin3(23);
ZigbeeLight zbAin4(24);

ZigbeeLight zbHeartbeat(31);
ZigbeeLight zbWdStatus(32);
ZigbeeLight zbWdEnable(33);

ZigbeeLight *const zbRelays[7] = { nullptr, &zbRelay1, &zbRelay2, &zbRelay3,
                                   &zbRelay4, &zbRelay5, &zbRelay6 };

// ---------------------------------------------------------------------------
// Periferiche
// ---------------------------------------------------------------------------
class PCA9557 {
public:
  PCA9557(uint8_t address, TwoWire *bus) : _addr(address), _wire(bus) {}

  uint8_t lastErr = 0;   // 0 ok | 1..5 codici Wire | 0xFE requestFrom | 0xFD read

  bool readRegister(uint8_t reg, uint8_t *value) {
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    lastErr = _wire->endTransmission(false);
    if (lastErr != 0) return false;
    if (_wire->requestFrom(_addr, (uint8_t)1) != 1) { lastErr = 0xFE; return false; }
    int v = _wire->read();
    if (v < 0) { lastErr = 0xFD; return false; }
    *value = (uint8_t)v;
    return true;
  }
  bool writeRegister(uint8_t reg, uint8_t value) {
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    _wire->write(value);
    lastErr = _wire->endTransmission();
    return (lastErr == 0);
  }

private:
  uint8_t  _addr;
  TwoWire *_wire;
};

PCA9557    io(GPIO_EXPANDER_ADDR, &Wire);
RTC_DS3231 rtc;
bool       g_rtcOk = false;
Preferences prefs;

// ---------------------------------------------------------------------------
// Watchdog di rete (heartbeat da Home Assistant)
//
//  - HA commuta l'endpoint Heartbeat (EP 31) ogni minuto. Qualunque comando
//    ricevuto (heartbeat o rele') vale come "segnale di vita".
//  - Il controllo si ARMA solo al primo heartbeat ricevuto (salvato in NVS).
//  - Scatta se: (armato e nessun segnale da troppo tempo) oppure
//    (non connessa alla rete da troppo tempo).
//  - Timeout crescenti: 15, 30, 60 min, poi un solo tentativo ogni 6 ore.
//    Il contatore sta in NVS e si azzera al primo segnale di vita.
//  - MODALITA' OMBRA (default): invece di riavviare accende WD-Status
//    (EP 32). I riavvii reali si abilitano da HA con WD-Enable (EP 33).
// ---------------------------------------------------------------------------
#define WD_LEVEL1_MS  (15UL * 60UL * 1000UL)
#define WD_LEVEL2_MS  (30UL * 60UL * 1000UL)
#define WD_LEVEL3_MS  (60UL * 60UL * 1000UL)
#define WD_LONG_MS    (6UL * 60UL * 60UL * 1000UL)
#define WD_TIME_DIV   1    // prove al banco: 60 => i minuti diventano secondi

// Stato persistente (NVS, namespace "plcwd")
bool    g_armed        = false;
bool    g_wdEnabled    = false;   // false = modalita' ombra
uint8_t g_restartCount = 0;
bool    g_wdFlag       = false;   // evento non ancora confermato da HA
uint8_t g_i2cRestarts  = 0;       // riavvii consecutivi causati dall'I2C

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
// Rele': stato desiderato / applicato / pubblicato
//  - g_relayTarget  : ultimo valore COMANDATO da Zigbee (desiderato)
//  - g_relayApplied : ultimo valore CONFERMATO dalla rilettura dell'espansore
//  - g_relayReported: valore che l'attributo Zigbee contiene ora
// I callback Zigbee girano in un altro task RTOS: solo flag, mai I2C.
// ---------------------------------------------------------------------------
volatile bool g_relayPending[7]  = {};
volatile bool g_relayTarget[7]   = {};
volatile bool g_relayReported[7] = {};
volatile bool g_selfPublish      = false;   // true mentre chiamiamo noi setLight()
portMUX_TYPE  g_relayMux         = portMUX_INITIALIZER_UNLOCKED;

uint8_t  g_relayApplied   = 0;
bool     g_i2cReady       = false;
uint32_t g_lastRelayActMs = 0;     // ultima commutazione fisica

// Contatori diagnostici
uint32_t g_syncOkTotal     = 0;
uint32_t g_i2cErrTotal     = 0;
uint32_t g_i2cRecoverTotal = 0;
uint32_t g_expResetTotal   = 0;    // configurazione espansore trovata cambiata
uint32_t g_outMismatchTotal= 0;    // registro uscite diverso dall'atteso
uint32_t g_verifyFailTotal = 0;    // rilettura dopo scrittura non coincide
uint32_t g_inputPubFail    = 0;
uint8_t  g_failRun         = 0;    // cicli di scrittura falliti consecutivi
uint32_t g_failSinceMs     = 0;
uint32_t g_okSinceMs       = 0;

void queueRelayCommand(uint8_t relay, bool state) {
  if (g_selfPublish) return;               // eco di una setLight() fatta da noi
  if (relay < 1 || relay > 6) return;
  portENTER_CRITICAL(&g_relayMux);
  g_relayTarget[relay]   = state;
  g_relayReported[relay] = state;          // l'attributo Zigbee ora vale questo
  g_relayPending[relay]  = true;
  portEXIT_CRITICAL(&g_relayMux);
  lifeSignal();
}
void onRelay1Change(bool s) { queueRelayCommand(1, s); }
void onRelay2Change(bool s) { queueRelayCommand(2, s); }
void onRelay3Change(bool s) { queueRelayCommand(3, s); }
void onRelay4Change(bool s) { queueRelayCommand(4, s); }
void onRelay5Change(bool s) { queueRelayCommand(5, s); }
void onRelay6Change(bool s) { queueRelayCommand(6, s); }

// Callback watchdog (task Zigbee: solo flag, niente NVS/I2C qui)
void onHeartbeat(bool)        { g_hbPending = true; lifeSignal(); }
void onWdStatusChange(bool s) { if (!s) g_ackPending = true; }      // HA conferma -> OFF
void onWdEnableChange(bool s) { g_enTarget = s; g_enPending = true; }

// Ingressi: z2m potrebbe inviare comandi, li ignoriamo (il poll riscrive lo stato)
void onDinChange(bool) {}
void onAinChange(bool) {}

// ---------------------------------------------------------------------------
// Bus I2C: sblocco a livello di pin + avvio
// ---------------------------------------------------------------------------
// Uno slave che ha perso un clock a meta' byte tiene SDA bassa: Wire.end() e
// Wire.begin() non lo liberano. Servono fino a 9 impulsi di clock a mano e uno
// STOP. Va fatto con il driver I2C spento, quindi dopo Wire.end().
void i2cBusRelease() {
  Wire.end();
  pinMode(SDA_PIN, INPUT_PULLUP);
  pinMode(SCL_PIN, OUTPUT_OPEN_DRAIN);
  digitalWrite(SCL_PIN, HIGH);
  delayMicroseconds(10);

  bool sdaStuck = (digitalRead(SDA_PIN) == LOW);
  bool sclStuck = (digitalRead(SCL_PIN) == LOW);   // SCL tenuta bassa da uno slave/corto
  uint8_t pulses = 0;
  while (digitalRead(SDA_PIN) == LOW && pulses < 9) {
    digitalWrite(SCL_PIN, LOW);  delayMicroseconds(10);
    digitalWrite(SCL_PIN, HIGH); delayMicroseconds(10);
    pulses++;
  }
  // condizione di STOP: SDA sale mentre SCL e' alta
  pinMode(SDA_PIN, OUTPUT_OPEN_DRAIN);
  digitalWrite(SDA_PIN, LOW);  delayMicroseconds(10);
  digitalWrite(SCL_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(SDA_PIN, HIGH); delayMicroseconds(10);
  pinMode(SDA_PIN, INPUT_PULLUP);

  if (sdaStuck || sclStuck || pulses) {
    Serial.printf("[I2C] sblocco bus: SDA bloccata=%d SCL bloccata=%d impulsi=%u SDA ora=%s\n",
                  sdaStuck, sclStuck, pulses,
                  digitalRead(SDA_PIN) == HIGH ? "alta (ok)" : "BASSA (hardware?)");
  }
}

bool i2cStart() {
  i2cBusRelease();
  if (!Wire.begin(SDA_PIN, SCL_PIN, I2C_CLOCK_HZ)) {
    Serial.println("[I2C] Wire.begin FALLITO");
    return false;
  }
  Wire.setClock(I2C_CLOCK_HZ);
  Wire.setTimeOut(I2C_TIMEOUT_MS);
  return true;
}

void i2cRestartBus() {
  g_i2cRecoverTotal++;
  g_i2cReady = i2cStart();
}

// ---------------------------------------------------------------------------
// Espansore PCA9557: sincronizzazione completa con verifica
// ---------------------------------------------------------------------------
// 1) legge config, polarita', uscite e confronta con quanto atteso (diagnosi);
// 2) scrive uscite, polarita', configurazione (in quest'ordine: il valore delle
//    uscite e' gia' pronto quando i pin diventano uscite, niente impulsi);
// 3) dopo l'assestamento rilegge TUTTI e tre i registri.
// Aggiorna g_relayApplied solo se la verifica riesce.
bool pcaSync(uint8_t mask) {
  static bool firstSync = true;
  uint8_t o = 0, p = 0, c = 0;

  if (!io.readRegister(PCA_REG_CONFIG, &c) ||
      !io.readRegister(PCA_REG_POLARITY, &p) ||
      !io.readRegister(PCA_REG_OUTPUT, &o)) {
    return false;
  }
  if (!firstSync) {
    if (c != PCA_CONFIG_VALUE || p != 0x00) {
      g_expResetTotal++;
      Serial.printf("[I2C] ESPANSORE RESETTATO? cfg=0x%02X (atteso 0x%02X) pol=0x%02X out=0x%02X (atteso 0x%02X) | ultima commutazione %lu s fa\n",
                    c, PCA_CONFIG_VALUE, p, o, g_relayApplied,
                    (unsigned long)((millis() - g_lastRelayActMs) / 1000UL));
    } else if (o != g_relayApplied) {
      g_outMismatchTotal++;
      Serial.printf("[I2C] registro uscite 0x%02X diverso dall'atteso 0x%02X\n", o, g_relayApplied);
    }
  }
  firstSync = false;

  if (!io.writeRegister(PCA_REG_OUTPUT, mask) ||
      !io.writeRegister(PCA_REG_POLARITY, 0x00) ||
      !io.writeRegister(PCA_REG_CONFIG, PCA_CONFIG_VALUE)) {
    return false;
  }

  if (mask != o) delay(RELAY_SETTLE_MS);   // lascia assestare le bobine, poi rilegge

  uint8_t o2 = 0, p2 = 0, c2 = 0;
  if (!io.readRegister(PCA_REG_OUTPUT, &o2) ||
      !io.readRegister(PCA_REG_POLARITY, &p2) ||
      !io.readRegister(PCA_REG_CONFIG, &c2)) {
    return false;
  }
  if (o2 != mask || p2 != 0x00 || c2 != PCA_CONFIG_VALUE) {
    g_verifyFailTotal++;
    Serial.printf("[I2C] verifica FALLITA: out=0x%02X/0x%02X pol=0x%02X cfg=0x%02X/0x%02X\n",
                  o2, mask, p2, c2, PCA_CONFIG_VALUE);
    return false;
  }

  if (mask != g_relayApplied) g_lastRelayActMs = millis();
  g_relayApplied = mask;
  g_syncOkTotal++;
  return true;
}

// Porta l'espansore a 'target' cambiando un rele' alla volta
bool applyMask(uint8_t target) {
  uint8_t diff = (uint8_t)((g_relayApplied ^ target) & PCA_RELAY_MASK);
  if (diff == 0) return pcaSync(target);          // solo refresh + verifica
  while (diff) {
    uint8_t bit  = (uint8_t)(diff & (uint8_t)(~diff + 1));   // bit piu' basso
    uint8_t step = (uint8_t)(g_relayApplied ^ bit);
    if (!pcaSync(step)) return false;
    diff = (uint8_t)((g_relayApplied ^ target) & PCA_RELAY_MASK);
    if (diff) delay(RELAY_STAGGER_MS);
  }
  return true;
}

// Raccoglie i comandi arrivati da Zigbee (sotto sezione critica)
uint8_t desiredMask(uint8_t *pendingMask) {
  uint8_t m = 0, pend = 0;
  portENTER_CRITICAL(&g_relayMux);
  for (uint8_t i = 1; i <= 6; i++) {
    if (g_relayTarget[i])  m    |= (uint8_t)(1U << i);
    if (g_relayPending[i]) { pend |= (uint8_t)(1U << i); g_relayPending[i] = false; }
  }
  portEXIT_CRITICAL(&g_relayMux);
  *pendingMask = pend;
  return m;
}

// Allinea l'attributo Zigbee allo stato reale dei rele'
void publishRelayTruth() {
  for (uint8_t i = 1; i <= 6; i++) {
    bool actual = ((g_relayApplied >> i) & 1U) != 0;
    bool rep, pend;
    portENTER_CRITICAL(&g_relayMux);
    rep  = g_relayReported[i];
    pend = g_relayPending[i];
    portEXIT_CRITICAL(&g_relayMux);
    if (pend || rep == actual) continue;
    g_selfPublish = true;                    // il callback di setLight() va ignorato
    zbRelays[i]->setLight(actual);
    g_selfPublish = false;
    portENTER_CRITICAL(&g_relayMux);
    g_relayReported[i] = actual;
    portEXIT_CRITICAL(&g_relayMux);
    Serial.printf("[RELAY] %u: stato Zigbee riallineato a %s (reale)\n", i, actual ? "ON" : "OFF");
  }
}

void serviceRelays() {
  static uint32_t lastSync = 0;
  static bool     limitLogged = false;

  uint8_t pend = 0;
  uint8_t want = desiredMask(&pend);
  uint32_t now = millis();

  if (want == g_relayApplied && !pend && (uint32_t)(now - lastSync) < RELAY_REFRESH_MS) return;
  lastSync = now;

  if (pend) {
    for (uint8_t i = 1; i <= 6; i++)
      if (pend & (1U << i)) Serial.printf("[RELAY] %u -> %s\n", i, ((want >> i) & 1U) ? "ON" : "OFF");
  }

  bool ok = false;
  for (uint8_t attempt = 1; attempt <= I2C_ATTEMPTS && !ok; attempt++) {
    if (!g_i2cReady) i2cRestartBus();
    ok = g_i2cReady && applyMask(want);
    if (!ok) {
      g_i2cErrTotal++;
      Serial.printf("[I2C][ERRORE] tentativo %u/%u (wire err=%u, applicato=0x%02X, voluto=0x%02X)\n",
                    attempt, I2C_ATTEMPTS, io.lastErr, g_relayApplied, want);
      if (attempt == 1) delay(5);          // un primo errore isolato: riprova subito
      else              i2cRestartBus();   // dal secondo: sblocca e riavvia il bus
    }
  }

  if (ok) {
    if (g_failRun) {
      Serial.printf("[I2C] ripreso dopo %u cicli falliti\n", g_failRun);
      g_okSinceMs = millis();
    }
    g_failRun = 0;
    limitLogged = false;
    publishRelayTruth();                   // dopo un periodo di errori riallinea HA
    if (g_i2cRestarts && (uint32_t)(millis() - g_okSinceMs) >= I2C_STABLE_CLEAR_MS) {
      g_i2cRestarts = 0;
      prefs.putUChar("irst", 0);
    }
    return;
  }

  if (g_failRun == 0) g_failSinceMs = millis();
  if (g_failRun < 250) g_failRun++;
  if (g_failRun >= I2C_PUBLISH_TRUTH_AFTER) publishRelayTruth();

  if ((uint32_t)(millis() - g_failSinceMs) >= I2C_FAIL_RESTART_MS) {
    if (g_i2cRestarts < I2C_MAX_RESTARTS) {
      g_i2cRestarts++;
      prefs.putUChar("irst", g_i2cRestarts);
      Serial.printf("[I2C] bus non recuperabile da %lu s: RIAVVIO #%u/%u\n",
                    (unsigned long)(I2C_FAIL_RESTART_MS / 1000UL), g_i2cRestarts, I2C_MAX_RESTARTS);
      Serial.flush();
      delay(300);
      ESP.restart();
    } else if (!limitLogged) {
      limitLogged = true;
      Serial.println("[I2C] limite riavvii raggiunto: resto acceso (Zigbee attivo), controllare l'hardware");
    }
  }
}

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
// Ingressi: poll e pubblicazione. Pubblica al primo giro, a ogni variazione e
// ogni INPUT_REPUBLISH_MS (z2m potrebbe aver sovrascritto l'attributo).
// ---------------------------------------------------------------------------
void updateInput(ZigbeeLight &ep, bool &last, bool value, bool force,
                 const char *tag, uint8_t n, int mv) {
  if (!force && value == last) return;
  if (!ep.setLight(value)) {
    g_inputPubFail++;
    Serial.printf("[ZB][WARN] pubblicazione %s%u fallita\n", tag, n);
  }
  if (value != last) {   // log solo sulle variazioni vere, non sui ripubblicamenti
    if (mv >= 0) Serial.printf("[%s] %u->%d (%d mV)\n", tag, n, value, mv);
    else         Serial.printf("[%s] %u->%d\n", tag, n, value);
  }
  last = value;
}

void pollInputs() {
  static bool     lastDin[5] = {};
  static bool     lastAin[5] = {};
  static bool     first      = true;
  static uint32_t lastPoll   = 0;
  static uint32_t lastForce  = 0;

  uint32_t now = millis();
  if ((uint32_t)(now - lastPoll) < INPUT_POLL_MS) return;
  lastPoll = now;

  bool force = first;
  if ((uint32_t)(now - lastForce) >= INPUT_REPUBLISH_MS) { force = true; lastForce = now; }

  bool din[5];
  din[1] = (digitalRead(DIN1_PIN) == HIGH);
  din[2] = (digitalRead(DIN2_PIN) == HIGH);
  din[3] = (digitalRead(DIN3_PIN) == HIGH);
  din[4] = (digitalRead(DIN4_PIN) == HIGH);

  int mv[5];
  mv[1] = adcToRealMv(analogReadMilliVolts(AIN1_PIN));
  mv[2] = adcToRealMv(analogReadMilliVolts(AIN2_PIN));
  mv[3] = adcToRealMv(analogReadMilliVolts(AIN3_PIN));
  mv[4] = adcToRealMv(analogReadMilliVolts(AIN4_PIN));

  bool ain[5];
  for (uint8_t i = 1; i <= 4; i++) ain[i] = analogThreshold(lastAin[i], mv[i]);

  updateInput(zbDin1, lastDin[1], din[1], force, "DIN", 1, -1);
  updateInput(zbDin2, lastDin[2], din[2], force, "DIN", 2, -1);
  updateInput(zbDin3, lastDin[3], din[3], force, "DIN", 3, -1);
  updateInput(zbDin4, lastDin[4], din[4], force, "DIN", 4, -1);
  updateInput(zbAin1, lastAin[1], ain[1], force, "AIN", 1, mv[1]);
  updateInput(zbAin2, lastAin[2], ain[2], force, "AIN", 2, mv[2]);
  updateInput(zbAin3, lastAin[3], ain[3], force, "AIN", 3, mv[3]);
  updateInput(zbAin4, lastAin[4], ain[4], force, "AIN", 4, mv[4]);

  first = false;
}

// ---------------------------------------------------------------------------
// Factory reset Zigbee: BOOT premuto per piu' di 3 s (non bloccante).
// Chiamata anche durante l'attesa della rete in setup().
// ---------------------------------------------------------------------------
void pollBootButton() {
  static bool     wasPressed   = false;
  static bool     resetFired   = false;
  static uint32_t pressStart   = 0;

  bool pressed = (digitalRead(BOOT_PIN) == LOW);
  if (pressed && !wasPressed) {
    wasPressed = true; resetFired = false; pressStart = millis();
  } else if (!pressed && wasPressed) {
    wasPressed = false; resetFired = false;
  }
  if (pressed && !resetFired && (uint32_t)(millis() - pressStart) >= 3000) {
    resetFired = true;
    Serial.println("[INFO] Factory reset Zigbee...");
    delay(300);
    Zigbee.factoryReset();
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
// Diagnostica periodica
// ---------------------------------------------------------------------------
void printDiagnostics() {
  static uint32_t lastDbg = 0;
  static uint32_t lastRtc = 0;
  uint32_t now = millis();
  if ((uint32_t)(now - lastDbg) < DEBUG_INTERVAL_MS) return;
  lastDbg = now;

  uint8_t tgt = 0;
  portENTER_CRITICAL(&g_relayMux);
  for (uint8_t i = 1; i <= 6; i++) if (g_relayTarget[i]) tgt |= (uint8_t)(1U << i);
  portEXIT_CRITICAL(&g_relayMux);

  Serial.printf("[SYS] up=%lus zb=%s heap=%u min=%u rx=%lu | I2C %s voluto=0x%02X applicato=0x%02X ok=%lu err=%lu rec=%lu expReset=%lu outMis=%lu verFail=%lu\n",
    (unsigned long)(now / 1000UL),
    Zigbee.connected() ? "OK" : "NO",
    (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMinFreeHeap(),
    (unsigned long)g_rxCount,
    g_i2cReady ? "ok" : "ERR", tgt, g_relayApplied,
    (unsigned long)g_syncOkTotal, (unsigned long)g_i2cErrTotal, (unsigned long)g_i2cRecoverTotal,
    (unsigned long)g_expResetTotal, (unsigned long)g_outMismatchTotal, (unsigned long)g_verifyFailTotal);
  Serial.printf("[WD] armato=%d riavvii_reali=%d contatore=%u ultimo_segnale=%lus fa i2cRiavvii=%u\n",
    g_armed, g_wdEnabled, (unsigned)g_restartCount,
    (unsigned long)((now - g_lastLifeMs) / 1000UL), (unsigned)g_i2cRestarts);

  if (g_rtcOk && (lastRtc == 0 || (uint32_t)(now - lastRtc) >= RTC_PRINT_MS)) {
    lastRtc = now ? now : 1;
    DateTime t = rtc.now();
    Serial.printf("[RTC] %04d-%02d-%02d %02d:%02d:%02d\n",
                  t.year(), t.month(), t.day(), t.hour(), t.minute(), t.second());
  }
}

// ---------------------------------------------------------------------------
// setup()
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== ESP32-PLC Zigbee (I2C robusto) ===");

  // Stato persistente
  prefs.begin("plcwd", false);
  g_armed        = prefs.getBool("armed", false);
  g_wdEnabled    = prefs.getBool("en", false);      // default: modalita' ombra
  g_restartCount = prefs.getUChar("rcnt", 0);
  g_wdFlag       = prefs.getBool("flag", false);
  g_i2cRestarts  = prefs.getUChar("irst", 0);
  // motivo reset: 1=power-on 3=riavvio sw 4=panic 5/6/7=watchdog 9=brownout
  Serial.printf("[BOOT] motivo_reset=%d armato=%d riavvii_reali=%d contatore=%u i2cRiavvii=%u\n",
                (int)esp_reset_reason(), g_armed, g_wdEnabled,
                (unsigned)g_restartCount, (unsigned)g_i2cRestarts);

  // I2C + espansore: tutti i rele' OFF all'avvio. Se l'espansore non risponde
  // NON si riavvia in loop (la scheda non entrerebbe mai in rete): si prosegue e
  // serviceRelays() continuera' a riprovare.
  g_i2cReady = i2cStart();
  bool expOk = false;
  for (uint8_t a = 1; a <= 3 && !expOk; a++) {
    expOk = g_i2cReady && pcaSync(0);
    if (!expOk) {
      Serial.printf("[I2C] inizializzazione espansore, tentativo %u fallito (wire err=%u)\n", a, io.lastErr);
      delay(100);
      i2cRestartBus();
    }
  }
  if (expOk) Serial.printf("[I2C] PCA9557 0x%02X pronto (cfg=0x%02X, rele' tutti OFF)\n",
                           GPIO_EXPANDER_ADDR, PCA_CONFIG_VALUE);
  else       Serial.println("[I2C][ERRORE] espansore non risponde: continuo, riprovo nel loop");
  g_okSinceMs = millis();

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
  zbHeartbeat.setManufacturerAndModel("ESP32-PLC", "WD-Heartbeat"); zbHeartbeat.onLightChange(onHeartbeat);     Zigbee.addEndpoint(&zbHeartbeat);
  zbWdStatus.setManufacturerAndModel("ESP32-PLC", "WD-Status");     zbWdStatus.onLightChange(onWdStatusChange); Zigbee.addEndpoint(&zbWdStatus);
  zbWdEnable.setManufacturerAndModel("ESP32-PLC", "WD-Enable");     zbWdEnable.onLightChange(onWdEnableChange); Zigbee.addEndpoint(&zbWdEnable);

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
    pollBootButton();      // BOOT > 3 s: reset Zigbee anche se non si connette
    serviceRelays();       // tiene vivo l'espansore anche senza rete
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
  serviceRelays();       // 1. rele': riconcilia desiderato/applicato, verifica espansore
  pollInputs();          // 2. ingressi
  pollBootButton();      // 3. factory reset
  wdService();           // 4. watchdog di rete
  printDiagnostics();    // 5. diagnostica
  delay(1);              // cede il tempo agli altri task
}
