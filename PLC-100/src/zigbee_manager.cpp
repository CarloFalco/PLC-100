#include "zigbee_manager.h"

#include <Arduino.h>
#include "Zigbee.h"

// Espressif Zigbee endpoint types
#include "ZigbeeLight.h"
#include "ZigbeeContactSwitch.h"

ZigbeeManager::ZigbeeManager(const Config &cfg)
    : _cfg(cfg) {
}

void ZigbeeManager::setRelayCallbacks(RelaySetFn set_cb, RelayGetFn get_cb) {
    _relay_set_cb = std::move(set_cb);
    _relay_get_cb = std::move(get_cb);
}

void ZigbeeManager::setInputCallback(InputGetFn get_cb) {
    _input_get_cb = std::move(get_cb);
}

bool ZigbeeManager::init() {
    Serial.println("[ZB] Init ZigbeeManager");

    if (_cfg.relay_count == 0 && _cfg.input_count == 0) {
        Serial.println("[ZB] ERRORE: nessun endpoint configurato (relay_count e input_count sono zero)");
        return false;
    }

    // Alloca array di puntatori agli endpoint
    if (_cfg.relay_count > 0) {
        _relay_eps = new ZigbeeLight*[_cfg.relay_count]();
    }
    if (_cfg.input_count > 0) {
        _input_eps = new ZigbeeContactSwitch*[_cfg.input_count]();
    }

    if (!create_relay_endpoints()) {
        Serial.println("[ZB] ERRORE: create_relay_endpoints() fallita");
        return false;
    }
    if (!create_input_endpoints()) {
        Serial.println("[ZB] ERRORE: create_input_endpoints() fallita");
        return false;
    }

    Serial.println("[ZB] Tutti gli endpoint Zigbee sono stati creati e registrati.");
    return true;
}

bool ZigbeeManager::start() {
    Serial.println("[ZB] Avvio dello stack Zigbee...");

    // Impostazioni opzionali di rete (PAN ID, canale, ecc.) sono attualmente gestite
    // dalla configurazione di default del core Zigbee Arduino‑ESP32. Al momento
    // non ci sono API pubbliche per forzare PAN ID/canale da qui in modo semplice,
    // ma Zigbee2MQTT gestirà l’associazione normalmente.

    bool ok = false;
    if (_cfg.device_role == DeviceRole::Router) {
        ok = Zigbee.begin(ZIGBEE_ROUTER);
    } else {
        ok = Zigbee.begin(ZIGBEE_END_DEVICE);
    }

    if (!ok) {
        Serial.println("[ZB] ERRORE: Zigbee.begin() fallita");
        return false;
    }

    Serial.println("[ZB] Zigbee avviato, connessione alla rete in corso...");
    while (!Zigbee.connected()) {
        Serial.print(".");
        delay(100);
    }
    Serial.println();
    Serial.println("[ZB] Connesso alla rete Zigbee.");

    return true;
}

void ZigbeeManager::loop() {
    // Per ora non c’è logica periodica specifica: il core Zigbee Arduino‑ESP32
    // viene gestito internamente. Questo metodo è un hook per future estensioni
    // (es. heartbeat, diagnostica, gestione OTA, ecc.).
}

bool ZigbeeManager::create_relay_endpoints() {
    if (_cfg.relay_count == 0) {
        return true;
    }

    for (uint8_t i = 0; i < _cfg.relay_count; ++i) {
        uint8_t ep = _cfg.base_relay_endpoint + i;
        auto *light = new ZigbeeLight(ep);
        if (!light) {
            Serial.println("[ZB] ERRORE: allocazione ZigbeeLight fallita");
            return false;
        }

        // Nome e modello personalizzati per Zigbee2MQTT
        light->setManufacturerAndModel(_cfg.manufacturer, _cfg.model);

        // Callback di cambio stato: chiama l’hardware reale tramite _relay_set_cb
        light->onLightChange([this, i](bool on) {
            uint8_t relay_index_1based = static_cast<uint8_t>(i + 1);
            Serial.printf("[ZB] Relay endpoint %u -> %s (relay index %u)\n",
                          _cfg.base_relay_endpoint + i,
                          on ? "ON" : "OFF",
                          relay_index_1based);
            if (_relay_set_cb) {
                _relay_set_cb(relay_index_1based, on);
            }
        });

        // Registra endpoint nel core Zigbee
        Zigbee.addEndpoint(light);

        _relay_eps[i] = light;
    }

    return true;
}

bool ZigbeeManager::create_input_endpoints() {
    if (_cfg.input_count == 0) {
        return true;
    }

    for (uint8_t i = 0; i < _cfg.input_count; ++i) {
        uint8_t ep = _cfg.base_input_endpoint + i;
        auto *cs = new ZigbeeContactSwitch(ep);
        if (!cs) {
            Serial.println("[ZB] ERRORE: allocazione ZigbeeContactSwitch fallita");
            return false;
        }

        cs->setManufacturerAndModel(_cfg.manufacturer, _cfg.model);

        // In questo esempio non utilizziamo le API avanzate IAS Zone (enroll/persistenza)
        // perché Zigbee2MQTT gestisce comunque correttamente il cluster del contact switch.
        // Se necessario, puoi estendere qui con logica di enroll simile all’esempio ufficiale.

        Zigbee.addEndpoint(cs);
        _input_eps[i] = cs;
    }

    return true;
}

void ZigbeeManager::publishInputState(uint8_t input_index_0based, bool is_active) {
    if (input_index_0based >= _cfg.input_count) {
        Serial.printf("[ZB] publishInputState: indice %u fuori range\n", input_index_0based);
        return;
    }

    auto *cs = _input_eps[input_index_0based];
    if (!cs) {
        Serial.printf("[ZB] publishInputState: endpoint mancante per input %u\n", input_index_0based);
        return;
    }

    // Convenzione: HIGH = aperto, LOW = chiuso (o viceversa a seconda del cablaggio).
    // Qui esponiamo semplicemente true = OPEN, false = CLOSED.
    if (is_active) {
        cs->setOpen();
    } else {
        cs->setClosed();
    }

    Serial.printf("[ZB] publishInputState: input %u -> %s\n",
                  input_index_0based, is_active ? "OPEN" : "CLOSED");
}

#include "zigbee_manager.h"
#include "esp_zigbee_core.h"
#include "esp_zigbee_zcl.h"
#include "esp_zigbee_zcl_on_off.h"
#include "esp_zigbee_zcl_binary_input.h"

ZigbeeManager *ZigbeeManager::s_instance = nullptr;

ZigbeeManager::ZigbeeManager(const Config &cfg)
    : _cfg(cfg) {
    s_instance = this;
}

void ZigbeeManager::setRelayCallbacks(RelaySetCallback set_cb, RelayGetCallback get_cb) {
    _relay_set_cb = std::move(set_cb);
    _relay_get_cb = std::move(get_cb);
}

void ZigbeeManager::setInputCallback(InputGetCallback input_cb) {
    _input_get_cb = std::move(input_cb);
}

uint8_t ZigbeeManager::relayEndpointFromIndex(uint8_t relay_index_1based) const {
    // Endpoint 1..N per relè
    return relay_index_1based;
}

uint8_t ZigbeeManager::inputEndpointFromIndex(uint8_t input_index_0based) const {
    // Input: endpoint 0x20 + index (es. 32..35)
    return 0x20 + input_index_0based;
}

esp_err_t ZigbeeManager::init() {
    ESP_LOGI(TAG, "Initializing Zigbee stack");

    esp_zb_platform_config_t platform_config = {};
    platform_config.radio_config.radio_mode = ESP_ZB_RADIO_MODE_NATIVE;
    platform_config.host_config.host_connection_mode = ESP_ZB_HOST_CONNECTION_MODE_NONE;
    ESP_ERROR_CHECK(esp_zb_platform_config(&platform_config));

    esp_zb_cfg_t zb_nwk_cfg = {};
    zb_nwk_cfg.esp_zb_role = _cfg.is_router ? ESP_ZB_DEVICE_TYPE_ROUTER : ESP_ZB_DEVICE_TYPE_ED;
    zb_nwk_cfg.install_code_policy = false;
    zb_nwk_cfg.nwk_cfg.zigbee_config.channel = _cfg.channel;
    zb_nwk_cfg.nwk_cfg.zigbee_config.pan_id = _cfg.pan_id;

    ESP_ERROR_CHECK(esp_zb_init(&zb_nwk_cfg));

    // Endpoint + cluster config
    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();

    // Crea endpoint per ogni relè con cluster OnOff server
    for (uint8_t i = 1; i <= _cfg.relay_count; ++i) {
        uint8_t ep_id = relayEndpointFromIndex(i);

        esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();
        // OnOff server
        esp_zb_on_off_cluster_cfg_t onoff_cfg = {};
        onoff_cfg.on_off = ZB_ZCL_ON_OFF_IS_OFF;
        esp_zb_cluster_list_add_on_off_server(cluster_list, &onoff_cfg, ESP_ZB_ZCL_CLUSTER_ID_ON_OFF);

        esp_zb_ep_list_add_ep(ep_list, cluster_list, ep_id, ESP_ZB_AF_HA_PROFILE_ID,
                              ESP_ZB_HA_ON_OFF_OUTPUT_DEVICE_ID);
    }

    // Endpoint per ingressi (Binary Input Basic server)
    for (uint8_t i = 0; i < _cfg.input_count; ++i) {
        uint8_t ep_id = inputEndpointFromIndex(i);

        esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();
        esp_zb_binary_input_cluster_cfg_t bin_cfg = {};
        bin_cfg.out_of_service = false;
        bin_cfg.present_value = 0;
        bin_cfg.status_flags = 0;

        esp_zb_cluster_list_add_binary_input_server(cluster_list, &bin_cfg, ESP_ZB_ZCL_CLUSTER_ID_BINARY_INPUT_BASIC);

        esp_zb_ep_list_add_ep(ep_list, cluster_list, ep_id, ESP_ZB_AF_HA_PROFILE_ID,
                              ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID);
    }

    esp_zb_device_register(ep_list);

    // Registra handler generali ZCL / segnali di stack
    esp_zb_zcl_set_attribute_handler(&ZigbeeManager::zclAttributeHandler);
    esp_zb_set_signal_handler(&ZigbeeManager::zbAppSignalHandler);

    ESP_LOGI(TAG, "Zigbee initialized");
    return ESP_OK;
}

void ZigbeeManager::start() {
    ESP_LOGI(TAG, "Starting Zigbee task");
    esp_zb_app_register_cli(); // opzionale, per debug
    esp_zb_start(true);        // start e join automatico
}

esp_err_t ZigbeeManager::zclAttributeHandler(esp_zb_zcl_cmd_t *cmd_info) {
    if (!s_instance) return ESP_FAIL;

    if (cmd_info->cluster_id == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF &&
        cmd_info->cmd_direction == ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV &&
        cmd_info->cmd_id != ESP_ZB_ZCL_CMD_READ_ATTRIBUTES &&
        cmd_info->cmd_id != ESP_ZB_ZCL_CMD_WRITE_ATTRIBUTES) {
        s_instance->handleOnOffServerCommand(cmd_info);
    }

    return ESP_OK;
}

void ZigbeeManager::handleOnOffServerCommand(esp_zb_zcl_cmd_t *cmd_info) {
    uint8_t endpoint = cmd_info->endpoint;
    uint8_t relay_index_1based = endpoint; // 1:1 mapping

    if (!_relay_set_cb || !_relay_get_cb) {
        ESP_LOGW(TAG, "Relay callbacks not set");
        return;
    }

    bool new_state = _relay_get_cb(relay_index_1based);

    switch (cmd_info->cmd_id) {
    case ESP_ZB_ZCL_CMD_ON_OFF_ON_ID:
        new_state = true;
        break;
    case ESP_ZB_ZCL_CMD_ON_OFF_OFF_ID:
        new_state = false;
        break;
    case ESP_ZB_ZCL_CMD_ON_OFF_TOGGLE_ID:
        new_state = !new_state;
        break;
    default:
        ESP_LOGW(TAG, "Unhandled OnOff cmd_id=%u", cmd_info->cmd_id);
        return;
    }

    _relay_set_cb(relay_index_1based, new_state);

    // Aggiorna attributo OnOff cluster
    uint8_t onoff_attr = new_state ? ZB_ZCL_ON_OFF_IS_ON : ZB_ZCL_ON_OFF_IS_OFF;
    esp_zb_zcl_status_t status = esp_zb_zcl_set_attribute_val(
        endpoint,
        ESP_ZB_ZCL_CLUSTER_ID_ON_OFF,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID,
        &onoff_attr,
        false
    );
    ESP_LOGI(TAG, "Set OnOff attr for ep %u => %u (status=%d)", endpoint, onoff_attr, status);
}

void ZigbeeManager::zbAppSignalHandler(esp_zb_app_signal_t *signal_struct) {
    esp_zb_app_signal_type_t sig_type = *signal_struct->p_app_signal;
    esp_err_t status = signal_struct->esp_err_status;

    ESP_LOGI(TAG, "Zigbee signal: %d, status: %d", sig_type, status);

    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Stack ready, starting network steering");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        break;
    default:
        break;
    }
}

void ZigbeeManager::publishInputState(uint8_t input_index_0based, bool state) {
    updateInputBinaryCluster(input_index_0based, state);
}

void ZigbeeManager::updateInputBinaryCluster(uint8_t input_index_0based, bool state) {
    uint8_t endpoint = inputEndpointFromIndex(input_index_0based);
    uint8_t present_value = state ? 1 : 0;

    esp_zb_zcl_status_t status = esp_zb_zcl_set_attribute_val(
        endpoint,
        ESP_ZB_ZCL_CLUSTER_ID_BINARY_INPUT_BASIC,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_BINARY_INPUT_PRESENT_VALUE_ID,
        &present_value,
        false
    );

    ESP_LOGI(TAG, "Updated BinaryInput ep %u -> %u (status=%d)", endpoint, present_value, status);
}