#pragma once

#include "esp_log.h"
#include "esp_zigbee_core.h"
#include "esp_zigbee_zcl.h"
#include <functional>
#include <cstdint>

class ZigbeeManager {
public:
    static constexpr const char *TAG = "ZIGBEE_MANAGER";

    // Callback verso IOManager: set relay / leggi input / publish state
    using RelaySetCallback   = std::function<void(uint8_t relay_index_1based, bool on)>;
    using RelayGetCallback   = std::function<bool(uint8_t relay_index_1based)>;
    using InputGetCallback   = std::function<bool(uint8_t input_index_0based)>;

    struct Config {
        uint8_t  relay_count       = 6;
        uint8_t  input_count       = 4;
        uint16_t pan_id            = 0x1A62;
        uint8_t  channel           = 20;   // Zigbee canale 20 tipico
        bool     is_router         = true; // PLC alimentato -> router
    };

    explicit ZigbeeManager(const Config &cfg);

    void setRelayCallbacks(RelaySetCallback set_cb, RelayGetCallback get_cb);
    void setInputCallback(InputGetCallback input_cb);

    esp_err_t init();
    void      start();

    // Chiamabile da IOManager quando cambia un ingresso
    void publishInputState(uint8_t input_index_0based, bool state);

private:
    Config          _cfg;
    RelaySetCallback _relay_set_cb;
    RelayGetCallback _relay_get_cb;
    InputGetCallback _input_get_cb;

    static esp_err_t zclAttributeHandler(esp_zb_zcl_cmd_t *cmd_info);
    static void      zbAppSignalHandler(esp_zb_app_signal_t *signal_struct);

    static ZigbeeManager *s_instance;

    void handleOnOffServerCommand(esp_zb_zcl_cmd_t *cmd_info);
    void updateInputBinaryCluster(uint8_t input_index_0based, bool state);

    uint8_t relayEndpointFromIndex(uint8_t relay_index_1based) const;
    uint8_t inputEndpointFromIndex(uint8_t input_index_0based) const;
};