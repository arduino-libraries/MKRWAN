/* This example shows how to use the SX1272 chip
 * (part of Murata module) without any external stack.
 * To achieve this, we must setup the modem in dumb mode
 * and use LORA_IRQ_DUMB pin as chip select and SPI1 as communication port.
 * 
 * The example is based on LoraSender by @sandeepmistry arduino-LoRa library
 * https://github.com/sandeepmistry/arduino-LoRa
 *
 * Starting from https://github.com/sandeepmistry/arduino-LoRa/commit/5f62ed2ce9d1623bfc12f468b8152ba1878b5b16,
 * LoRa library knows about MKRWAN1300/1310 and automatically restarts the module in dumb mode, uses SPI1 and the correct gpios.
 * 
 * Since there is no IRQ pin available the host must poll for data (unfortunately)
 * 
 */

#include <SPI.h>
#include <LoRa.h>
//#include <MKRWAN.h>

int counter = 0;

//LoRaModem modem;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  // No need to call modem.dumb() with arduino-LoRa >= 0.5.0
  //modem.dumb();

  Serial.println("LoRa Sender");

  // Configure LoRa module to transmit and receive at 915MHz (915*10^6) 
  // Replace 915E6 with the frequency you need (eg. 433E6 for 433MHz)
  if (!LoRa.begin(915E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
}

void loop() {
  Serial.print("Sending packet: ");
  Serial.println(counter);

  // send packet
  LoRa.beginPacket();
  LoRa.print("hello ");
  LoRa.print(counter);
  LoRa.endPacket();

  counter++;

  delay(5000);
}
