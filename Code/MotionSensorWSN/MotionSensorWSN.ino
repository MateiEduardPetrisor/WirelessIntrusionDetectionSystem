// ~~~ INCLUDE LIBRARIES ~~~
#include <SPI.h>
#include <RH_NRF24.h>
#include <RHEncryptedDriver.h>
#include <RHReliableDatagram.h>
#include <Speck.h>
#include <Battery.h>
// ~~~ END OF INCLUDE LIBRARIES ~~~

// ~~~ CONSTANTS ~~~
#define BASE_STATION_ADDRESS 1    // base station address
#define MOTION_SENSOR_ADDRESS 5   // motion node address
byte rwcl0516Pin = 6;
const int batteryPin = A0;
const int pirPin = A1;
const char sType[4] = "PIR";
// ~~~ END OF CONSTANTS ~~~

// ~~~ BUFFERS DATA ~~~
char DataBuffer[RH_NRF24_MAX_MESSAGE_LEN];
char* DataBufferPtr = NULL;
uint8_t sAddress;
// ~~~ END OF BUFFERS DATA ~~~

RH_NRF24 nrf24(8, 10);
Speck myCipher;
RHEncryptedDriver driver(nrf24, myCipher);
unsigned char encryptKey[17] = "1349496701951491";
RHReliableDatagram manager(driver, MOTION_SENSOR_ADDRESS);
Battery battery(3400, 4200, batteryPin);

void setup() {
  Serial.begin(115200);
  pinMode(rwcl0516Pin, INPUT);
  pinMode(pirPin, INPUT);
  if (!manager.init()) {
    Serial.println("intit fail");
  }
  if (!nrf24.setChannel(0))
    Serial.println("setChannel failed");
  if (!nrf24.setRF(RH_NRF24::DataRate2Mbps, RH_NRF24::TransmitPower0dBm))
    Serial.println("setRF failed");
  manager.setRetries(10);
  myCipher.setKey(encryptKey, sizeof(encryptKey) - 1);
  battery.begin(5000, 1.0);
}

float checkBattery() {
  return battery.voltage();
}

// 1 - Motion Detected
// 0 - No Motion Detected
int readRWCL0516() {
  return digitalRead(rwcl0516Pin);
}

// < 200 - Motion Detected
// > 200 - No Motion Detected
float readPIR() {
  return analogRead(pirPin);
}

float getSensorValue() {
  if (readPIR() < 200 && readRWCL0516() == HIGH) {
    return 1;
  } else {
    return 0;
  }
}

void prepareString(float batteryVoltage, char* sensorType, float sensorValue) {
  char buf[10];
  char* ptr = DataBuffer;
  ltoa(batteryVoltage, buf, 10);
  memcpy(DataBuffer, buf, strlen(buf));
  ptr += strlen(buf);
  *ptr = ',';
  ptr++;
  memcpy(ptr, sensorType, strlen(sensorType));
  ptr += strlen(sensorType);
  *ptr = ',';
  ptr++;
  ltoa(sensorValue, buf, 10);
  memcpy(ptr, buf, strlen(buf));
  ptr += strlen(buf);
  *ptr = '\0';
}

int computeCommandDataSize(char* command) {
  return (strlen(command) + 1);
}

bool SEND(char* data, uint8_t sensorAddress) {
  if (manager.sendtoWait(data, computeCommandDataSize(data), sensorAddress)) {
    return true;
  }
  return false;
}

char* RECV(int receiveTimeout) {
  uint8_t len = sizeof(DataBuffer);
  if (manager.recvfromAckTimeout(DataBuffer, &len, receiveTimeout, &sAddress)) {
    return *&DataBuffer;
  } else {
    return NULL;
  }
}

void loop() {
  DataBufferPtr = RECV(10);
  if (DataBufferPtr) {
    if (strcmp(DataBuffer, "GET") == 0 && sAddress == BASE_STATION_ADDRESS) {
      prepareString(checkBattery(), sType, getSensorValue());
      //      Serial.println(DataBuffer);
      if (!SEND(DataBuffer, BASE_STATION_ADDRESS)) {
        Serial.println("Send Fail to BS");
      }
    }
    DataBufferPtr = NULL;
  }
}