/* This example shows how to use the SX1272 chip
 * (part of Murata module) without any external stack.
 * To achieve this, we must setup the modem in dumb mode
 * and use LORA_IRQ_DUMB pin as chip select and SPI1 as communication port.
 * 
 * The example is based on LoraSender by @sandeepmistry arduino-LoRa library
 * https://github.com/sandeepmistry/arduino-LoRa
 * 
 * Since there is no IRQ pin available the host must poll for data (unfortunately)
 * 
 */

#include <SPI.h>
#include <LoRa.h>
#include <MKRWAN.h>

int counter = 0;

LoRaModem modem;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  modem.dumb();

  Serial.println(F("LoRa Sender"));

  // override the default CS, reset, and IRQ pins (optional)
  LoRa.setPins(LORA_IRQ_DUMB, 6, 1); // set CS, reset, IRQ pin
  LoRa.setSPIFrequency(100000);

  if (!LoRa.begin(SPI1, 915E6)) {
    Serial.println(F("Starting LoRa failed!"));
    while (1);
  }
}

void loop() {
  Serial.print(F("Sending packet: "));
  Serial.println(counter);

  // send packet
  LoRa.beginPacket();
  LoRa.print(F("hello "));
  LoRa.print(counter);
  LoRa.endPacket();

  counter++;

  delay(5000);
}
