/*
* Helium Send And Receive
*
* For Helium and Adafruit.io http integrations sensor data is Base 64 encoded
*
* Run the MKRWAN example MKRWANFWUpdate_standalone to make sure the LoRa module is reset
*
* In Helium "functions" section use this simple base64 decoder
*
////////////////////////////////////////////////////////////
 function Decoder(bytes, port, uplink_info) {
  var decoded={};
  try{
  var result = String.fromCharCode.apply(null, bytes);
  decoded.value = result;
  return decoded;
  } catch (err) {
  return 'Decoder: ' + err.name + " : " + err.message;;
  }
 }
////////////////////////////////////////////////////////////
*
*
*
* And in the integration section of helium use this JSON
*
////////////////////////////////////////////////////////////
 { "value": {{decoded.payload.value}} }
////////////////////////////////////////////////////////////
*
*
* 
* This sketch demonstrates how to send and receive data with the PortentaH7 LoRa module.
* This example code is in the public domain.
* By Jeremy Ellis Twitter @rocksetta
* Helium Video Playlist at
* https://www.youtube.com/watch?v=wt_WZ1sBDeM&list=PL57Dnr1H_egu0PkIBAbbdfQ21O_NJqJg8&index=4
*
* Helium Arduino Documnetation at
* https://docs.helium.com/use-the-network/devices/development/arduino/lora-vision-shield/arduino/
*
* Note: Helium and Adafruit.io must be setup for what it does with the data
*/

#include <Arduino.h> // Only needed by https://platformio.org/
#include <MKRWAN.h>


LoRaModem modem;
bool connected = false;
bool myWaitForDownlink = false;
bool myDownLink = false;

unsigned long myStoredMillisA;
unsigned long myStoredMillisB;
const unsigned long myTimerDurationA = 30000;  // delay between sending data
const unsigned long myTimerDurationB = 5000;   // delay to wait for a downlink


// Please enter your sensitive data in the arduino_secrets.h tab
// Note: Best to have the App_Device hard coded. Run the program once to see the value.
#include "arduino_secrets.h"


String appEui = SECRET_APP_EUI;   // set these in your arduino_secrets.h tab
String appKey = SECRET_APP_KEY;


void setup() 
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(LEDR,OUTPUT);
  pinMode(LEDG,OUTPUT);
  pinMode(LEDB,OUTPUT);
  digitalWrite(LEDR, HIGH); // new boards HIGH = off
  digitalWrite(LEDG, LOW);
  digitalWrite(LEDB, HIGH);
  
  //while (!Serial);       // don't wait for serial
  
  Serial.println("Wait 4");
  delay(3000);             // delay instead, so it works when disconnected
  digitalWrite(LEDG, HIGH);// allows time to connect serial monitor

  Serial.println("Wait 3");
  delay(3000);       
  digitalWrite(LEDG, LOW);
 
  Serial.println("Wait 2");
  delay(3000);       
  digitalWrite(LEDG, HIGH);

  Serial.println("Wait 1");
  delay(3000);       
  digitalWrite(LEDG, LOW);
  delay(3000);  
       
  // change this to your regional band (eg. US915, AS923, EU868, AU915, AS923, KR920, IN865 ...)
  if (!modem.begin(US915)) {
    Serial.println("Failed to start module");
    while (1) {}  // since error keep program here
  };
 
  Serial.print("Your module version is: ");
  Serial.println(modem.version());
  Serial.print("Your device EUI is: ");
  Serial.println(modem.deviceEUI());

  Serial.println("Now Disabling all channels and enable channel 1 only for Helium ");

  modem.disableChannel(0);
  modem.enableChannel(1);    // only one enabled for Helium
  modem.disableChannel(2);
  modem.disableChannel(3);
  modem.disableChannel(4);
  modem.disableChannel(5);
  modem.disableChannel(6);
  delay(5000);
  Serial.println("Now Joining the Helium Network ");

}

void loop() 
{
  while (!connected) {
    Serial.println("trying to reconnect");
    digitalWrite(LEDR, HIGH); // new boards HIGH = off
    digitalWrite(LEDG, LOW);
    digitalWrite(LEDB, LOW);
    connected = modem.joinOTAA(appEui, appKey);
    delay(5000);    
    digitalWrite(LEDR, HIGH); // new boards HIGH = off
    digitalWrite(LEDG, LOW);
    digitalWrite(LEDB, HIGH);
    delay(1000);
  }

  if (myWaitForDownlink){
    
    char rcv[64];
    int i = 0;
    while (modem.available()) {
      rcv[i++] = (char)modem.read();
      myDownLink = true;
    }

    if (millis() - myStoredMillisB >= myTimerDurationB){  // Test whether the period has elapsed
      myStoredMillisB = millis();  
      if (!modem.available()) {
        Serial.println("No downlink message received at this time.");
        myWaitForDownlink = false;
      }
    }
      

    if (myDownLink) {
      myWaitForDownlink = false;
      myDownLink = false;
      Serial.print("Received: ");
      Serial.write(rcv, i);
      // for (unsigned int j = 0; j < i; j++) {
      //   Serial.print(rcv[j] >> 4, HEX);
      //   Serial.print(rcv[j] & 0xF, HEX);
      //   Serial.print(" ");
      // }
      Serial.println();
      digitalWrite(LEDR, LOW); // new boards HIGH = off
      digitalWrite(LEDG, HIGH);
      digitalWrite(LEDB, LOW);
    }        
  }

  if (millis() - myStoredMillisA >= myTimerDurationA){  // Test whether the period has elapsed
    myStoredMillisA = millis();                         // IMPORTANT to save the next stored time 
 
    String msg = String(analogRead(A0));   // program expecting numbers!

    Serial.println();
    Serial.print("Sending: " + msg + " - ");
    for (unsigned int i = 0; i < msg.length(); i++) {
      Serial.print(msg[i] >> 4, HEX);
      Serial.print(msg[i] & 0xF, HEX);
      Serial.print(" ");
    }
    Serial.println();

    int err;
    modem.beginPacket();
    modem.print(msg);
    err = modem.endPacket(true);  
    if (err > 0) {
      Serial.println("Message sent correctly!");
      digitalWrite(LEDR, HIGH); // new boards HIGH = off
      digitalWrite(LEDG, HIGH);
      digitalWrite(LEDB, LOW);
    } else {
      Serial.println("Error sending message :(");
      Serial.println("(you may send a limited amount of messages per minute, depending on the signal strength");
      Serial.println("it may vary from 1 message every couple of seconds to 1 message every minute)");
      digitalWrite(LEDR, LOW); // new boards HIGH = off
      digitalWrite(LEDG, LOW);
      digitalWrite(LEDB, HIGH);
    }
    myStoredMillisB = millis();
    myWaitForDownlink = true;
    // delay(30000);  // delay 30 seconds for testing
     
  }  // end timerA

}

