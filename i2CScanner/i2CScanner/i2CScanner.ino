#include <Wire.h>

#define SDA_PIN 6
#define SCL_PIN 7
#define RTC_ADDR 0x68

uint8_t bcdToDec(uint8_t val) {
  return (val / 16 * 10) + (val % 16);
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  Wire.begin(SDA_PIN, SCL_PIN);

  Serial.println("Leggo RTC DS3231M...");
}

void loop() {
  Wire.beginTransmission(RTC_ADDR);
  Wire.write(0x00);  // registro dei secondi
  Wire.endTransmission();

  Wire.requestFrom(RTC_ADDR, 3);
  if (Wire.available() == 3) {
    uint8_t sec = bcdToDec(Wire.read());
    uint8_t min = bcdToDec(Wire.read());
    uint8_t hour = bcdToDec(Wire.read());

    Serial.printf("Ora RTC: %02d:%02d:%02d\n", hour, min, sec);
  } else {
    Serial.println("RTC non risponde");
  }

  delay(1000);
}
