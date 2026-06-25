#pragma once

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstdint>

class PCA9557 {
public:
    PCA9557(uint8_t address, i2c_port_t i2c_port);

    bool pinMode(uint8_t pin, gpio_mode_t mode);
    bool digitalWrite(uint8_t pin, bool value);
    int  digitalRead(uint8_t pin);

private:
    bool readRegister(uint8_t reg, uint8_t &value);
    bool writeRegister(uint8_t reg, uint8_t value);

    uint8_t     _addr;
    i2c_port_t  _port;
};

class IOManager {
public:
    static constexpr const char *TAG = "IO_MANAGER";

    struct Config {
        // I2C per PCA9557
        i2c_port_t i2c_port          = I2C_NUM_0;
        gpio_num_t i2c_sda           = GPIO_NUM_6;
        gpio_num_t i2c_scl           = GPIO_NUM_7;
        uint8_t    pca9557_address   = 0x1A;

        // Numero relè gestiti
        uint8_t relay_count          = 6;

        // Mappatura: relè 1..N -> bit del PCA9557 (1..6 nel tuo esempio)
        uint8_t relay_bits[6]        = {1, 2, 3, 4, 5, 6};

        // Ingressi digitali diretti (GPIO su ESP32‑C6)
        uint8_t  input_count         = 4;
        gpio_num_t input_pins[4]     = {GPIO_NUM_4, GPIO_NUM_5, GPIO_NUM_10, GPIO_NUM_11};

        // Debounce in millisecondi
        uint32_t debounce_ms_inputs  = 50;
        uint32_t debounce_ms_relays  = 20;
    };

    explicit IOManager(const Config &cfg);

    esp_err_t init();

    // Relè
    esp_err_t setRelay(uint8_t relay_index_1based, bool on);
    esp_err_t toggleRelay(uint8_t relay_index_1based);
    bool      getRelayState(uint8_t relay_index_1based) const;

    // Ingressi
    bool      getInputState(uint8_t input_index_0based) const;
    void      pollInputsAndApplyDebounce();

private:
    Config    _cfg;
    PCA9557   _expander;

    bool      _relay_states[6]      = {false};
    bool      _input_raw[4]         = {false};
    bool      _input_debounced[4]   = {false};
    uint32_t  _input_last_change_ms[4] = {0};

    uint32_t  getMillis() const;
};