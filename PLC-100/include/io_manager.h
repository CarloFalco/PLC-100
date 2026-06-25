/**
 * IOManager
 *
 * Gestisce:
 *  - Relè pilotati da expander I2C PCA9557
 *  - Ingressi digitali con debounce software
 *
 * L’implementazione è pensata per essere indipendente da Zigbee:
 *  - Zigbee chiede di leggere/settare relè tramite le API pubbliche
 *  - Gli ingressi sono letti in polling e resi disponibili con uno storico di cambi
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "driver/gpio.h"

class IOManager {
public:
    struct Config {
        // --- Bus I2C per PCA9557 (solo un expander in questo esempio) ---
        gpio_num_t  i2c_sda         = GPIO_NUM_NC;
        gpio_num_t  i2c_scl         = GPIO_NUM_NC;
        uint32_t    i2c_frequency_hz= 400000;
        uint8_t     pca9557_address = 0x1A;

        // --- Relè (mappati su bit dell’expander PCA9557) ---
        uint8_t     relay_count     = 0;         // numero di relè gestiti
        uint8_t     relay_bits[8]   = {0};       // bit 0..7 dell’expander associati ai relè (1‑based a livello di API)

        // --- Ingressi digitali (collegati ai GPIO nativi dell’ESP32‑C6) ---
        uint8_t     input_count     = 0;
        gpio_num_t  input_pins[8]   = {GPIO_NUM_NC};

        // --- Debounce ---
        uint32_t    debounce_ms     = 30;        // soglia minima di stabilità prima di accettare il cambio
    };

    struct InputChange {
        bool        current_state = false;
        uint32_t    changed_at_ms = 0;   // timestamp (millis) dell’ultimo cambio stabile
    };

    explicit IOManager(const Config &cfg);

    // Inizializza I2C + PCA9557 + GPIO input
    // Ritorna true se tutto ok, false in caso di errore (vedi log seriale)
    bool init();

    // Numero di ingressi gestiti
    uint8_t getInputCount() const { return _cfg.input_count; }

    // --- Relè ---
    void setRelay(uint8_t relay_index_1based, bool on);
    bool getRelayState(uint8_t relay_index_1based) const;

    // --- Ingressi digitali ---
    // Polling + debounce da chiamare regolarmente (es. ogni 20 ms)
    void pollInputsAndApplyDebounce();

    // Ritorna true se l’ingresso `index` ha cambiato stato dall’ultima lettura
    bool getLastChange(uint8_t input_index_0based, InputChange &out_change);

    // Stato attuale (debounced) di un ingresso
    bool getInputState(uint8_t input_index_0based) const;

private:
    Config      _cfg;

    // Stato corrente dei relè (bitmask sugli 8 bit del PCA9557)
    uint8_t     _relay_mask = 0;

    // Stato istantaneo e debounced degli ingressi
    bool        _input_raw[8]      = {false};
    bool        _input_stable[8]   = {false};
    uint32_t    _last_edge_ms[8]   = {0};   // ultimo istante in cui il valore raw è cambiato
    bool        _has_change[8]     = {false};
    InputChange _last_change[8];

    // Funzioni interne
    bool init_i2c();
    bool init_pca9557();
    bool write_pca9557_outputs();
};

