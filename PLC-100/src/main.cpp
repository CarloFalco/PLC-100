/**
 * Firmware ESP32-PLC100 basato su ESP32‑C6 con stack Zigbee (Arduino Zigbee).
 *
 * - Ogni relè è esposto come endpoint Zigbee On/Off (cluster OnOff)
 * - Ogni ingresso digitale è esposto come sensore di contatto (IAS Zone / contact switch)
 * - Il core Zigbee è gestito da `ZigbeeManager`
 * - La gestione dell’hardware (relè + ingressi) è incapsulata in `IOManager`
 *
 * Questo file si limita a comporre i due manager e a gestire il ciclo principale.
 */

#include <Arduino.h>
#include "io_manager.h"
#include "zigbee_manager.h"

// Istanza globale dei manager
static IOManager     *g_io  = nullptr;
static ZigbeeManager *g_zb  = nullptr;

// Configurazione hardware specifica per la tua scheda PLC‑100
static IOManager::Config make_io_config() {
    IOManager::Config cfg{};

    // Bus I2C per PCA9557 (relè)
    cfg.i2c_sda         = GPIO_NUM_6;
    cfg.i2c_scl         = GPIO_NUM_7;
    cfg.i2c_frequency_hz= 400000;          // 400 kHz, tipico per expander IO
    cfg.pca9557_address = 0x1A;            // documentato

    // Mappatura relè -> bit del PCA9557 (1‑based a livello di API)
    cfg.relay_count     = 6;
    cfg.relay_bits[0]   = 1;
    cfg.relay_bits[1]   = 2;
    cfg.relay_bits[2]   = 3;
    cfg.relay_bits[3]   = 4;
    cfg.relay_bits[4]   = 5;
    cfg.relay_bits[5]   = 6;

    // Ingressi digitali collegati direttamente ai GPIO dell’ESP32‑C6
    cfg.input_count     = 4;
    cfg.input_pins[0]   = GPIO_NUM_4;
    cfg.input_pins[1]   = GPIO_NUM_5;
    cfg.input_pins[2]   = GPIO_NUM_10;
    cfg.input_pins[3]   = GPIO_NUM_11;

    // Parametri di debounce (ms)
    cfg.debounce_ms     = 30;

    return cfg;
}

// Configurazione Zigbee (ruolo, PAN ID, canale, mapping relè/ingressi)
static ZigbeeManager::Config make_zigbee_config(const IOManager::Config &io_cfg) {
    ZigbeeManager::Config cfg{};

    cfg.relay_count     = io_cfg.relay_count;
    cfg.input_count     = io_cfg.input_count;

    // Parametri di rete: possono essere lasciati a 0 per auto‑join,
    // oppure fissati per avere un comportamento deterministico.
    cfg.pan_id          = 0x1A62;
    cfg.channel         = 20;        // tipicamente 11‑26; assicurati che Zigbee2MQTT lo usi o che sia in 'channel scan'

    // La PLC‑100 è alimentata e stabile -> meglio come Router Zigbee
    cfg.device_role     = ZigbeeManager::DeviceRole::Router;

    // Endpoint base per relè e sensori (puoi cambiarli se necessario)
    cfg.base_relay_endpoint  = 10;   // relè -> 10,11,12,...
    cfg.base_input_endpoint  = 30;   // ingressi -> 30,31,32,...

    return cfg;
}

void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println();
    Serial.println("=== ESP32‑PLC100 Zigbee firmware start ===");

    // --- IO Manager ---
    IOManager::Config io_cfg = make_io_config();
    static IOManager io(io_cfg);
    if (!io.init()) {
        Serial.println("[MAIN] ERRORE: IOManager init() failed, riavvio...");
        delay(2000);
        ESP.restart();
    }
    g_io = &io;

    // --- Zigbee Manager ---
    ZigbeeManager::Config zb_cfg = make_zigbee_config(io_cfg);
    static ZigbeeManager zb(zb_cfg);
    g_zb = &zb;

    // Callback: Zigbee -> Relè
    zb.setRelayCallbacks(
        // set_cb
        [](uint8_t relay_index_1based, bool on) {
            if (g_io) {
                g_io->setRelay(relay_index_1based, on);
            }
        },
        // get_cb
        [](uint8_t relay_index_1based) -> bool {
            if (!g_io) return false;
            return g_io->getRelayState(relay_index_1based);
        }
    );

    // Callback: IO -> Zigbee (publish stato ingressi)
    zb.setInputCallback(
        [](uint8_t input_index_0based) -> bool {
            if (!g_io) return false;
            return g_io->getInputState(input_index_0based);
        }
    );

    if (!zb.init()) {
        Serial.println("[MAIN] ERRORE: ZigbeeManager init() failed, riavvio...");
        delay(2000);
        ESP.restart();
    }
    if (!zb.start()) {
        Serial.println("[MAIN] ERRORE: ZigbeeManager start() failed, riavvio...");
        delay(2000);
        ESP.restart();
    }

    Serial.println("[MAIN] Inizializzazione completata, entro nel loop principale.");
}

void loop() {
    if (!g_io || !g_zb) {
        // Non dovrebbe succedere, ma evitiamo crash
        delay(1000);
        return;
    }

    // 1) Poll + debounce degli ingressi digitali
    g_io->pollInputsAndApplyDebounce();

    // 2) Controlla variazioni di stato e pubblica via Zigbee
    for (uint8_t i = 0; i < g_io->getInputCount(); ++i) {
        IOManager::InputChange change;
        if (g_io->getLastChange(i, change)) {
            Serial.printf("[MAIN] Input %u changed -> %u (debounced)\n", i, change.current_state);
            g_zb->publishInputState(i, change.current_state);
        }
    }

    // 3) Altre attività periodiche (se servono)
    g_zb->loop();

    delay(20); // ~50 Hz di polling: sufficiente con debounce a 30 ms
}