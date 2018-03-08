/*
  Lora Send And Receive
  This sketch demonstrates how to send and receive data with the MKR WAN 1300 LoRa module.
  This example code is in the public domain.
*/

#include <MKRWAN.h>

LoRaModem modem;

// Uncomment if using the Murata chip as a module
// LoRaModem modem(Serial1);

#include "arduino_secrets.h" 
// Please enter your sensitive data in the Secret tab or arduino_secrets.h
String appEui = SECRET_APP_EUI;
String appKey = SECRET_APP_KEY;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial);
  // change this to your regional band (eg. US915, AS923, ...)
  if (!modem.begin(EU868)) {
    Serial.println(F("Failed to start module"));
    while (1) {}
  };
  Serial.print(F("Your module version is: "));
  Serial.println(modem.version());
  Serial.print(F("Your device EUI is: "));
  Serial.println(modem.deviceEUI());

  int connected = modem.joinOTAA(appEui, appKey);
  if (!connected) {
    Serial.println(F("Something went wrong; are you indoor? Move near a window and retry"));
    while (1) {}
  }

  // Set poll interval to 60 secs.
  modem.minPollInterval(60);
  // NOTE: independently by this setting the modem will
  // not allow to send more than one message every 2 minutes,
  // this is enforced by firmware and can not be changed.
}

void loop() {
  Serial.println();
  Serial.println(F("Enter a message to send to network"));
  Serial.println(F("(make sure that end-of-line 'NL' is enabled)"));

  while (!Serial.available());
  String msg = Serial.readStringUntil('\n');

  Serial.println();
  Serial.print("Sending: " + msg + " - ");
  for (unsigned int i = 0; i < msg.length(); i++) {
    Serial.print(msg[i] >> 4, HEX);
    Serial.print(msg[i] & 0xF, HEX);
    Serial.print(F(" "));
  }
  Serial.println();

  int err;
  modem.beginPacket();
  modem.print(msg);
  err = modem.endPacket(true);
  if (err > 0) {
    Serial.println(F("Message sent correctly!"));
  } else {
    Serial.println(F("Error sending message :("));
    Serial.println(F("(you may send a limited amount of messages per minute, depending on the signal strength"));
    Serial.println(F("it may vary from 1 message every couple of seconds to 1 message every minute)"));
  }
  delay(1000);
  if (!modem.available()) {
    Serial.println(F("No downlink message received at this time."));
    return;
  }
  String rcv;
  rcv.reserve(64);
  while (modem.available()) {
    rcv += (char)modem.read();
  }
  Serial.print("Received: " + rcv + " - ");
  for (unsigned int i = 0; i < rcv.length(); i++) {
    Serial.print(rcv[i] >> 4, HEX);
    Serial.print(rcv[i] & 0xF, HEX);
    Serial.print(F(" "));
  }
  Serial.println();
}
