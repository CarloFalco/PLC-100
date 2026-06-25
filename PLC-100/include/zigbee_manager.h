/**
 * ZigbeeManager
 *
 * Incapsula lo stack Zigbee Arduino‑ESP32:
 *  - Crea endpoint OnOff (ZigbeeLight) per i relè
 *  - Crea endpoint Contact Switch (ZigbeeContactSwitch) per gli ingressi digitali
 *  - Espone callback verso il codice applicativo per pilotare l’hardware reale
 *
 * Obiettivo: fornire una API semplice e modulare a `main.cpp` e agli altri moduli,
 * mantenendo tutta la logica Zigbee confinata qui.
 */

#pragma once

#include <stdint.h>
#include <functional>

class ZigbeeLight;
class ZigbeeContactSwitch;

class ZigbeeManager {
public:
    enum class DeviceRole {
        EndDevice,
        Router,
    };

    struct Config {
        uint8_t     relay_count         = 0;
        uint8_t     input_count         = 0;

        // Endpoint di base per la creazione dinamica
        uint8_t     base_relay_endpoint = 10;
        uint8_t     base_input_endpoint = 30;

        // Parametri di rete (opzionali, gestiti per lo più dallo stack)
        uint16_t    pan_id              = 0;     // 0 = auto
        uint8_t     channel             = 0;     // 0 = auto (11‑26)

        DeviceRole  device_role         = DeviceRole::Router;

        // Identificativi opzionali per migliorare l’esperienza in Zigbee2MQTT
        const char* manufacturer        = "Custom";
        const char* model               = "ESP32-PLC100";
    };

    using RelaySetFn  = std::function<void(uint8_t relay_index_1based, bool on)>;
    using RelayGetFn  = std::function<bool(uint8_t relay_index_1based)>;
    using InputGetFn  = std::function<bool(uint8_t input_index_0based)>;

    explicit ZigbeeManager(const Config &cfg);

    // Da chiamare una volta a startup, dopo aver istanziato l’oggetto
    bool init();
    bool start();

    // Da chiamare periodicamente dal loop principale (gestione interna, se necessario)
    void loop();

    // Wiring verso l’IO fisico
    void setRelayCallbacks(RelaySetFn set_cb, RelayGetFn get_cb);
    void setInputCallback(InputGetFn get_cb);

    // API per notificare Zigbee di un cambio ingresso digitale
    void publishInputState(uint8_t input_index_0based, bool is_active);

private:
    Config          _cfg;

    // Array dinamico di endpoint per relè e ingressi
    ZigbeeLight         **_relay_eps  = nullptr;
    ZigbeeContactSwitch **_input_eps  = nullptr;

    RelaySetFn      _relay_set_cb;
    RelayGetFn      _relay_get_cb;
    InputGetFn      _input_get_cb;

    // Creazione e registrazione degli endpoint
    bool create_relay_endpoints();
    bool create_input_endpoints();

    // Non copiabile
    ZigbeeManager(const ZigbeeManager&) = delete;
    ZigbeeManager& operator=(const ZigbeeManager&) = delete;
};

