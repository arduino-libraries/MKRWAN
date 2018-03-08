/*
  First Configuration
  This sketch demonstrates the usage of MKR WAN 1300 LoRa module.
  This example code is in the public domain.
*/

#include <MKRWAN.h>

LoRaModem modem;

// Uncomment if using the Murata chip as a module
// LoRaModem modem(Serial1);

String appEui;
String appKey;
String devAddr;
String nwkSKey;
String appSKey;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial);
  Serial.println(F("Welcome to MKRWAN1300 first configuration sketch"));
  Serial.println(F("Register to your favourite LoRa network and we are ready to go!"));
  // change this to your regional band (eg. US915, AS923, ...)
  if (!modem.begin(EU868)) {
    Serial.println(F("Failed to start module"));
    while (1) {}
  };
  Serial.print(F("Your module version is: "));
  Serial.println(modem.version());
  Serial.print(F("Your device EUI is: "));
  Serial.println(modem.deviceEUI());

  int mode = 0;
  while (mode != 1 && mode != 2) {
    Serial.println(F("Are you connecting via OTAA (1) or ABP (2)?"));
    while (!Serial.available());
    mode = Serial.readStringUntil('\n').toInt();
  }

  int connected;
  if (mode == 1) {
    Serial.println(F("Enter your APP EUI"));
    while (!Serial.available());
    appEui = Serial.readStringUntil('\n');

    Serial.println(F("Enter your APP KEY"));
    while (!Serial.available());
    appKey = Serial.readStringUntil('\n');

    appKey.trim();
    appEui.trim();

    connected = modem.joinOTAA(appEui, appKey);
  } else if (mode == 2) {

    Serial.println(F("Enter your Device Address"));
    while (!Serial.available());
    devAddr = Serial.readStringUntil('\n');

    Serial.println(F("Enter your NWS KEY"));
    while (!Serial.available());
    nwkSKey = Serial.readStringUntil('\n');

    Serial.println(F("Enter your APP SKEY"));
    while (!Serial.available());
    appSKey = Serial.readStringUntil('\n');

    devAddr.trim();
    nwkSKey.trim();
    appSKey.trim();

    connected = modem.joinABP(devAddr, nwkSKey, appSKey);
  }

  if (!connected) {
    Serial.println(F("Something went wrong; are you indoor? Move near a window and retry"));
    while (1) {}
  }

  delay(5000);

  int err;
  modem.setPort(3);
  modem.beginPacket();
  modem.print("HeLoRA world!");
  err = modem.endPacket(true);
  if (err > 0) {
    Serial.println(F("Message sent correctly!"));
  } else {
    Serial.println(F("Error sending message :("));
  }
}

void loop() {
  while (modem.available()) {
    Serial.write(modem.read());
  }
  modem.poll();
}
