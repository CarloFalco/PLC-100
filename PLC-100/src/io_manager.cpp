#include "io_manager.h"

#include <Arduino.h>
#include "driver/i2c.h"

// I2C hardware configuration per ESP32‑C6
static constexpr i2c_port_t IO_I2C_PORT = I2C_NUM_0;

// Registri principali del PCA9557 (semplificati per uso come expander out)
static constexpr uint8_t PCA9557_REG_OUTPUT     = 0x01;
static constexpr uint8_t PCA9557_REG_CONFIG     = 0x03;

IOManager::IOManager(const Config &cfg)
    : _cfg(cfg) {
}

bool IOManager::init() {
    Serial.println("[IO] Init IOManager");

    if (_cfg.relay_count > 8 || _cfg.input_count > 8) {
        Serial.println("[IO] ERRORE: massimo 8 relè e 8 ingressi supportati nell’implementazione corrente.");
        return false;
    }

    if (!init_i2c()) {
        Serial.println("[IO] ERRORE: init_i2c() fallita");
        return false;
    }
    if (!init_pca9557()) {
        Serial.println("[IO] ERRORE: init_pca9557() fallita");
        return false;
    }

    // Configura i GPIO degli ingressi con pull‑up interno e lettura iniziale
    for (uint8_t i = 0; i < _cfg.input_count; ++i) {
        gpio_num_t pin = _cfg.input_pins[i];
        if (pin == GPIO_NUM_NC) continue;

        pinMode(pin, INPUT_PULLUP);
        bool level = (digitalRead(pin) == HIGH);
        _input_raw[i]    = level;
        _input_stable[i] = level;
        _last_edge_ms[i] = millis();
        _has_change[i]   = false;
        _last_change[i]  = { level, _last_edge_ms[i] };
    }

    Serial.println("[IO] Init completata");
    return true;
}

bool IOManager::init_i2c() {
    if (_cfg.i2c_sda == GPIO_NUM_NC || _cfg.i2c_scl == GPIO_NUM_NC) {
        Serial.println("[IO] I2C non configurato (SDA/SCL = NC), nessun relè verrà gestito.");
        return true; // non è un errore fatale se vuoi solo gli ingressi
    }

    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = _cfg.i2c_sda;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = _cfg.i2c_scl;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = _cfg.i2c_frequency_hz;

    if (i2c_param_config(IO_I2C_PORT, &conf) != ESP_OK) {
        Serial.println("[IO] i2c_param_config fallita");
        return false;
    }
    if (i2c_driver_install(IO_I2C_PORT, conf.mode, 0, 0, 0) != ESP_OK) {
        Serial.println("[IO] i2c_driver_install fallita");
        return false;
    }

    Serial.printf("[IO] I2C inizializzato (port=%d, SDA=%d, SCL=%d, freq=%lu Hz)\n",
                  IO_I2C_PORT, _cfg.i2c_sda, _cfg.i2c_scl,
                  static_cast<unsigned long>(_cfg.i2c_frequency_hz));
    return true;
}

bool IOManager::init_pca9557() {
    if (_cfg.relay_count == 0) {
        Serial.println("[IO] Nessun relè configurato, salto init PCA9557.");
        return true;
    }

    // Tutti i bit come output (0 = output, 1 = input)
    uint8_t cfg_dir = 0x00;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_cfg.pca9557_address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, PCA9557_REG_CONFIG, true);
    i2c_master_write_byte(cmd, cfg_dir, true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(IO_I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    if (err != ESP_OK) {
        Serial.printf("[IO] ERRORE: scrittura CONFIG PCA9557 fallita (err=%d)\n", err);
        return false;
    }

    // Porta inizialmente tutti i relè a OFF
    _relay_mask = 0x00;
    return write_pca9557_outputs();
}

bool IOManager::write_pca9557_outputs() {
    if (_cfg.relay_count == 0 || _cfg.i2c_sda == GPIO_NUM_NC || _cfg.i2c_scl == GPIO_NUM_NC) {
        return true; // niente da fare
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_cfg.pca9557_address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, PCA9557_REG_OUTPUT, true);
    i2c_master_write_byte(cmd, _relay_mask, true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(IO_I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    if (err != ESP_OK) {
        Serial.printf("[IO] ERRORE: scrittura OUTPUT PCA9557 fallita (err=%d)\n", err);
        return false;
    }
    return true;
}

void IOManager::setRelay(uint8_t relay_index_1based, bool on) {
    if (relay_index_1based == 0 || relay_index_1based > _cfg.relay_count) {
        Serial.printf("[IO] setRelay: indice %u fuori range\n", relay_index_1based);
        return;
    }

    uint8_t bit = _cfg.relay_bits[relay_index_1based - 1];
    if (bit > 7) {
        Serial.printf("[IO] setRelay: bit %u fuori range (0..7)\n", bit);
        return;
    }

    if (on) {
        _relay_mask |= (1u << bit);
    } else {
        _relay_mask &= ~(1u << bit);
    }

    if (!write_pca9557_outputs()) {
        Serial.println("[IO] setRelay: errore nella scrittura al PCA9557");
    } else {
        Serial.printf("[IO] Relay %u -> %s (mask=0x%02X)\n",
                      relay_index_1based, on ? "ON" : "OFF", _relay_mask);
    }
}

bool IOManager::getRelayState(uint8_t relay_index_1based) const {
    if (relay_index_1based == 0 || relay_index_1based > _cfg.relay_count) {
        return false;
    }

    uint8_t bit = _cfg.relay_bits[relay_index_1based - 1];
    if (bit > 7) {
        return false;
    }
    return (_relay_mask & (1u << bit)) != 0;
}

void IOManager::pollInputsAndApplyDebounce() {
    uint32_t now = millis();

    for (uint8_t i = 0; i < _cfg.input_count; ++i) {
        gpio_num_t pin = _cfg.input_pins[i];
        if (pin == GPIO_NUM_NC) {
            continue;
        }

        bool level = (digitalRead(pin) == HIGH);

        if (level != _input_raw[i]) {
            // Nuovo fronte grezzo -> reset timer di debounce
            _input_raw[i]    = level;
            _last_edge_ms[i] = now;
        }

        // Se il valore raw è diverso da quello stabile da almeno debounce_ms,
        // accettiamo il cambio come stabile
        if (_input_stable[i] != _input_raw[i] &&
            (now - _last_edge_ms[i]) >= _cfg.debounce_ms) {

            _input_stable[i] = _input_raw[i];
            _has_change[i]   = true;
            _last_change[i].current_state = _input_stable[i];
            _last_change[i].changed_at_ms = now;

            Serial.printf("[IO] Input %u debounced -> %u\n", i, _input_stable[i]);
        }
    }
}

bool IOManager::getLastChange(uint8_t input_index_0based, InputChange &out_change) {
    if (input_index_0based >= _cfg.input_count) {
        return false;
    }

    if (!_has_change[input_index_0based]) {
        return false;
    }

    _has_change[input_index_0based] = false;
    out_change = _last_change[input_index_0based];
    return true;
}

bool IOManager::getInputState(uint8_t input_index_0based) const {
    if (input_index_0based >= _cfg.input_count) {
        return false;
    }
    return _input_stable[input_index_0based];
}

#include "io_manager.h"
#include "esp_timer.h"

// ---------------- PCA9557 ----------------

PCA9557::PCA9557(uint8_t address, i2c_port_t i2c_port)
    : _addr(address), _port(i2c_port) {}

bool PCA9557::readRegister(uint8_t reg, uint8_t &value) {
    esp_err_t err = i2c_master_write_read_device(
        _port, _addr, &reg, 1, &value, 1, pdMS_TO_TICKS(100));
    return (err == ESP_OK);
}

bool PCA9557::writeRegister(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    esp_err_t err = i2c_master_write_to_device(
        _port, _addr, buf, 2, pdMS_TO_TICKS(100));
    return (err == ESP_OK);
}

bool PCA9557::pinMode(uint8_t pin, gpio_mode_t mode) {
    uint8_t cfg = 0;
    if (!readRegister(0x03, cfg)) return false;

    if (mode == GPIO_MODE_OUTPUT) {
        cfg &= ~(1 << pin);
    } else {
        cfg |= (1 << pin);
    }
    return writeRegister(0x03, cfg);
}

bool PCA9557::digitalWrite(uint8_t pin, bool value) {
    uint8_t out = 0;
    if (!readRegister(0x01, out)) return false;

    if (value) {
        out |= (1 << pin);
    } else {
        out &= ~(1 << pin);
    }
    return writeRegister(0x01, out);
}

int PCA9557::digitalRead(uint8_t pin) {
    uint8_t in = 0;
    if (!readRegister(0x00, in)) return -1;
    return (in & (1 << pin)) ? 1 : 0;
}

// ---------------- IOManager ----------------

IOManager::IOManager(const Config &cfg)
    : _cfg(cfg), _expander(cfg.pca9557_address, cfg.i2c_port) {}

esp_err_t IOManager::init() {
    ESP_LOGI(TAG, "Initializing I2C and IO");

    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = _cfg.i2c_sda;
    conf.scl_io_num = _cfg.i2c_scl;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 100000;

    ESP_ERROR_CHECK(i2c_param_config(_cfg.i2c_port, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(_cfg.i2c_port, conf.mode, 0, 0, 0));

    // Configura PCA9557
    for (uint8_t i = 0; i < _cfg.relay_count; ++i) {
        _expander.pinMode(_cfg.relay_bits[i], GPIO_MODE_OUTPUT);
        _expander.digitalWrite(_cfg.relay_bits[i], false);
        _relay_states[i] = false;
    }

    // Configura ingressi diretti
    for (uint8_t i = 0; i < _cfg.input_count; ++i) {
        gpio_config_t io_conf = {};
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.intr_type = GPIO_INTR_DISABLE; // debounce via polling
        io_conf.pin_bit_mask = 1ULL << _cfg.input_pins[i];
        io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        ESP_ERROR_CHECK(gpio_config(&io_conf));

        bool level = gpio_get_level(_cfg.input_pins[i]);
        _input_raw[i] = level;
        _input_debounced[i] = level;
        _input_last_change_ms[i] = getMillis();
    }

    ESP_LOGI(TAG, "IO initialization done");
    return ESP_OK;
}

esp_err_t IOManager::setRelay(uint8_t relay_index_1based, bool on) {
    if (relay_index_1based == 0 || relay_index_1based > _cfg.relay_count) {
        ESP_LOGE(TAG, "Invalid relay index: %u", relay_index_1based);
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t idx = relay_index_1based - 1;
    uint8_t bit = _cfg.relay_bits[idx];

    if (!_expander.digitalWrite(bit, on)) {
        ESP_LOGE(TAG, "Failed to write relay %u (bit %u)", relay_index_1based, bit);
        return ESP_FAIL;
    }

    _relay_states[idx] = on;
    ESP_LOGI(TAG, "Relay %u set to %s", relay_index_1based, on ? "ON" : "OFF");
    return ESP_OK;
}

esp_err_t IOManager::toggleRelay(uint8_t relay_index_1based) {
    if (relay_index_1based == 0 || relay_index_1based > _cfg.relay_count) {
        return ESP_ERR_INVALID_ARG;
    }
    bool new_state = !_relay_states[relay_index_1based - 1];
    return setRelay(relay_index_1based, new_state);
}

bool IOManager::getRelayState(uint8_t relay_index_1based) const {
    if (relay_index_1based == 0 || relay_index_1based > _cfg.relay_count) {
        return false;
    }
    return _relay_states[relay_index_1based - 1];
}

bool IOManager::getInputState(uint8_t input_index_0based) const {
    if (input_index_0based >= _cfg.input_count) return false;
    return _input_debounced[input_index_0based];
}

void IOManager::pollInputsAndApplyDebounce() {
    uint32_t now = getMillis();

    for (uint8_t i = 0; i < _cfg.input_count; ++i) {
        bool level = gpio_get_level(_cfg.input_pins[i]);
        if (level != _input_raw[i]) {
            _input_raw[i] = level;
            _input_last_change_ms[i] = now;
        }
        if ((now - _input_last_change_ms[i]) >= _cfg.debounce_ms_inputs &&
            _input_debounced[i] != _input_raw[i]) {
            _input_debounced[i] = _input_raw[i];
            ESP_LOGI(TAG, "Input %u debounced -> %u", i, _input_debounced[i]);
        }
    }
}

uint32_t IOManager::getMillis() const {
    return static_cast<uint32_t>(esp_timer_get_time() / 1000ULL);
}