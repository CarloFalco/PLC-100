#include <Wire.h>
// #include <PCA9557.h> // Include the I/O expander library
#include <RTClib.h> // Include the RTC library


class PCA9557 {
private:
    TwoWire *_wire;
    uint8_t _addr;



public:
    PCA9557(int address = 0x18, TwoWire *bus = &Wire) {
        _addr = address;
        _wire = bus;
    }
    bool read_register(uint8_t reg, uint8_t *value) {
        _wire->beginTransmission(_addr);
        _wire->write(reg);
        if (_wire->endTransmission(false) != 0) return false;

        if (_wire->requestFrom(_addr, (uint8_t)1) != 1) return false;
        *value = _wire->read();
        return true;
    }
    // <<< RESO PUBBLICO
    bool write_register(uint8_t reg, uint8_t value) {
        _wire->beginTransmission(_addr);
        _wire->write(reg);
        _wire->write(value);
        return (_wire->endTransmission() == 0);
    }

    bool pinMode(int pin, int mode) {
        uint8_t config;
        if (!read_register(0x03, &config)) return false;

        if (mode == OUTPUT)
            config &= ~(1 << pin);
        else
            config |= (1 << pin);

        return write_register(0x03, config);
    }

    bool digitalWrite(int pin, int value) {
        uint8_t out;
        if (!read_register(0x01, &out)) return false;

        if (value == HIGH)
            out |= (1 << pin);
        else
            out &= ~(1 << pin);

        return write_register(0x01, out);
    }

    int digitalRead(int pin) {
        uint8_t in;
        if (!read_register(0x00, &in)) return -1;
        return (in & (1 << pin)) ? HIGH : LOW;
    }
};




#define GPIO_EXPANDER_ADDRESS 0x1A   // CORRETTO
#define SDA_PIN 6
#define SCL_PIN 7





PCA9557 io(GPIO_EXPANDER_ADDRESS, &Wire);
RTC_DS3231 rtc; // Create an instance of the RTC object

void setup() {
  Serial.begin(9600);
  Wire.begin(SDA_PIN, SCL_PIN);

  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }

  if (rtc.lostPower()) {
    // Set the RTC time to the compile time
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
 
  // Configure digital pins as inputs
  pinMode(4, INPUT);
  pinMode(5, INPUT);
  pinMode(10, INPUT);
  pinMode(11, INPUT);

  // Configure GPIO pins as outputs for relay control
  /*
  for (int bit = 1; bit <= 8; bit++) {
      io.pinMode(bit, OUTPUT);   // bit1..bit6
  }
  */  
  delay(2000); // Adjust delay as needed
  io.write_register(0x03, 0b10000001);
  uint8_t cfg;
  io.read_register(0x03, &cfg);
  Serial.println(cfg, BIN);

}

void loop() {
  DateTime now = rtc.now(); // Get current time from RTC

  // Read digital inputs and activate corresponding relays
  if (digitalRead(4) == HIGH) {
    writeRelay(1, HIGH);
  } else {
    writeRelay(1, LOW);
  }
  if (digitalRead(5) == HIGH) {
    writeRelay(2, HIGH);
  } else {
    writeRelay(2, LOW);
  }
  if (digitalRead(10) == HIGH) {
    writeRelay(3, HIGH);
  } else {
    writeRelay(3, LOW);
  }
  if (digitalRead(11) == HIGH) {
    writeRelay(4, HIGH);
  } else {
    writeRelay(4, LOW);
  }

  // Read analog inputs and activate relays 5 and 6 accordingly
  int analogValue1 = analogRead(0);  // OK → GPIO0
  int analogValue2 = analogRead(1);  // OK → GPIO1
  int analogValue3 = analogRead(2);  // OK → GPIO2
  int analogValue4 = analogRead(3);  // OK → GPIO3

  if (analogValue1 >= 1023 || analogValue2 >= 1023 || analogValue3 >= 1023 || analogValue4 >= 1023) {
    writeRelay(5, HIGH); // Activate relay 5
  } else {
    writeRelay(5, LOW); // Deactivate relay 5
  }

  if (analogValue1 >= 2047 || analogValue2 >= 2047 || analogValue3 >= 2047 || analogValue4 >= 2047) {
    writeRelay(6, HIGH); // Activate relay 6
  } else {
    writeRelay(6, LOW); // Deactivate relay 6
  }

  // Set PWM outputs based on analog input values
  analogWrite(18, calculatePWM(analogValue1)); // Analog Output 1
  analogWrite(19, calculatePWM(analogValue2)); // Analog Output 2
  analogWrite(20, calculatePWM(analogValue3)); // Analog Output 3
  analogWrite(21, calculatePWM(analogValue4)); // Analog Output 4

  // Prepare the timestamp
  char timestamp[20];
  sprintf(timestamp, "%04d-%02d-%02d %02d:%02d:%02d", now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());

  // Enhanced debug output with actual time/date stamp
  Serial.print(timestamp); Serial.print(" | ");
  Serial.print(digitalRead(4)); Serial.print("  ");
  Serial.print(digitalRead(5)); Serial.print("  ");
  Serial.print(digitalRead(10)); Serial.print("  ");
  Serial.print(digitalRead(11)); Serial.print("  "); Serial.print(" | ");
  Serial.print(io.digitalRead(1)); Serial.print("  ");
  Serial.print(io.digitalRead(2)); Serial.print("  ");
  Serial.print(io.digitalRead(3)); Serial.print("  ");
  Serial.print(io.digitalRead(4)); Serial.print("  ");
  Serial.print(io.digitalRead(5)); Serial.print("  ");
  Serial.print(io.digitalRead(6)); Serial.print("  "); Serial.print(" | ");
  Serial.print(analogRead(0)); Serial.print("  ");
  Serial.print(analogRead(1)); Serial.print("  ");
  Serial.print(analogRead(2)); Serial.print("  ");
  Serial.println(analogRead(3));

  delay(200); // Adjust delay as needed
}

// Function to calculate PWM duty cycle based on analog reading
int calculatePWM(int analogValue) {
  // Calculate PWM duty cycle using linear interpolation
  int pwm = map(analogValue, 0, 3300, 0, 255); // Adjusted range for analog input values

  // Ensure PWM duty cycle does not exceed 255
  return min(pwm, 255);
}
// Funzione wrapper per correggere la mappatura reale della board
void writeRelay(uint8_t relay, bool state) {
  // relay = 1..6
  uint8_t bit = relay;  // bit1..bit6
  io.digitalWrite(bit, state ? HIGH : LOW);
}