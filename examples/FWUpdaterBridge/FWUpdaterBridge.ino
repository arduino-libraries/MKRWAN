/*
  FW Updater Bridge
  This sketch demonstrates how to update the FW on the MKR WAN 1300/1310 LoRa module.
  Once flashed it should be used in conjunction with stm32flash utility (https://github.com/facchinm/stm32flash)

  This example code is in the public domain.
*/

#define Serial1 Serial

void setup() {
  Serial1.begin(115200, SERIAL_8E1);
  Serial2.begin(115200, SERIAL_8E1);

  delay(1000);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LORA_BOOT0, OUTPUT);
  digitalWrite(LORA_BOOT0, HIGH);
  pinMode(LORA_RESET, OUTPUT);
  digitalWrite(LORA_RESET, HIGH);
  delay(200);
  digitalWrite(LORA_RESET, LOW);
  delay(200);
  digitalWrite(LORA_RESET, HIGH);
}

void resetModule() {
  Serial.println("resetting module");
  digitalWrite(LORA_RESET, HIGH);
  delay(100);
  digitalWrite(LORA_RESET, LOW);
  delay(100);
  digitalWrite(LORA_RESET, HIGH);

  while (!Serial);
}

char rx_buf[512];
char tx_buf[512];

int rx = 0;
int tx = 0;

void loop() {
  while (Serial1.available()) {      // If anything comes in Serial (USB),
    tx_buf[tx++] = Serial1.read();   // read it and send it out Serial1 (pins 0 & 1)
  }

  if (tx > 0) {
    Serial2.write(tx_buf, tx);
    tx = 0;
  }

  while (Serial2.available()) {      // If anything comes in Serial (USB),
    rx_buf[rx++] = Serial2.read();   // read it and send it out Serial1 (pins 0 & 1)
  }

  if (rx > 0) {
    Serial1.write(rx_buf, rx);
    rx = 0;
  }
}
