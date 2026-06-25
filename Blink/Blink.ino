/*
  Blink

  Turns an LED on for one second, then off for one second, repeatedly.

  Most Arduinos have an on-board LED you can control. On the UNO, MEGA and ZERO
  it is attached to digital pin 13, on MKR1000 on pin 6. LED_BUILTIN is set to
  the correct LED pin independent of which board is used.
  If you want to know what pin the on-board LED is connected to on your Arduino
  model, check the Technical Specs of your board at:
  https://docs.arduino.cc/hardware/

  modified 8 May 2014
  by Scott Fitzgerald
  modified 2 Sep 2016
  by Arturo Guadalupi
  modified 8 Sep 2016
  by Colby Newman

  This example code is in the public domain.

  https://docs.arduino.cc/built-in-examples/basics/Blink/
*/
//#define RGB_BUILTIN 15   // esempio
#define LED_BUILTIN 39

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  Serial1.begin(115200);
  Serial.begin(115200);

  Serial0.begin(115200);     // UART esterna su GPIO16/17

  Serial.println("USB CDC OK");
  Serial0.println("UART esterna OK");
  Serial1.println("BOH");
}

// the loop function runs over and over again forever
void loop() {
  Serial.println("Hello World");
  Serial1.print("Ciao Mondo");
  neopixelWrite(LED_BUILTIN,64,64,64);
  delay(1000);                      // wait for a second
  //neopixelWrite(LED_BUILTIN,64,64,0);
  delay(1000);                      // wait for a second
  //neopixelWrite(LED_BUILTIN,0,64,0);
  delay(1000);                      // wait for a second
  neopixelWrite(LED_BUILTIN,0,0,0);
  delay(1000);                      // wait for a second
}
