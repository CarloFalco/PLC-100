#include <Wire.h>

#define SDA_PIN   6      // quelli che hai usato nel test che funziona
#define SCL_PIN   7
#define PCA_ADDR  0x1A

uint8_t relayState = 0x00;  // 1 = ON, 0 = OFF

void pcaWriteOutputs(uint8_t value) {
  Wire.beginTransmission(PCA_ADDR);
  Wire.write(0x01);      // OUTPUT register
  Wire.write(value);
  Wire.endTransmission();
}

void setRelay(uint8_t relayIndex, bool on) {
  // relayIndex: 1..6
  if (relayIndex < 1 || relayIndex > 6) return;

  uint8_t bit = relayIndex;      // bit1..bit6

  if (on) relayState |=  (1 << bit);
  else    relayState &= ~(1 << bit);

  pcaWriteOutputs(relayState);
}


void setup() {
  Serial.begin(115200);
  delay(2000);
  Wire.begin(SDA_PIN, SCL_PIN);

  // tutti output
  Wire.beginTransmission(PCA_ADDR);
  Wire.write(0x03);   // CONFIG
  Wire.write(0x00);   // 0 = output
  Wire.endTransmission();

  Serial.println("Relè pronti.");
}

void loop() {
  // Accendi i relè uno alla volta
  for (uint8_t i = 1; i < 7; i++) {
    Serial.printf("Relay %u ON\n", i);
    setRelay(i, true);
    delay(500);

    Serial.printf("Relay %u OFF\n", i);
    setRelay(i, false);
    delay(200);
  }
}
