// ~~~ INCLUDE LIBRARIES ~~~
#include <SPI.h>
#include <RH_NRF24.h>
#include <RHEncryptedDriver.h>
#include <RHReliableDatagram.h>
#include <Speck.h>
#include <Keypad.h>
#include <EEPROM.h>
#include <Password.h>
#include <DS3231.h>
#include <GSM.h>
#include <FancyDelay.h>
// ~~~ END OF INCLUDE LIBRARIES ~~~

// ~~~ CONSTANTS ~~~
const byte encryptionKeyLength = 17;
char encryptKey[encryptionKeyLength];
const byte telephoneNumberLength = 11;
char telephoneNumber[telephoneNumberLength];
const byte codeLength = 5;
const byte keypadRows = 4;
const byte keypadColumns = 4;
const byte resetPin = 4;
const byte buzzerPin = 9;
// ~~~ END OF CONSTATNTS ~~~

// ~~~ EEPROM ADDRESSES ~~~
byte serviceCodeAddress;
byte extCodeAddress;
byte allCodeAddress;
byte telephoneAddress;
byte encryptionKeyAddress;
// ~~~ END OF EEPROM ADDRESSES ~~~

// ~~~ SENSOR NODES ADDRESSES ~~~
#define BASE_STATION_ADDRESS 1 //base station address
#define DOOR_NODE_ADDRESS 2    //door node address
#define WINDOW_NODE_ADDRESS 3  //window node address
#define GAS_NODE_ADDRESS 4     //gas node address
#define PIR_NODE_ADDRESS 5     //motion node address
// ~~~ END OF SENSOR NODES ADDRESSES ~~~

// ~~~ RF MODULE INIT ~~~
RH_NRF24 nrf24(8, 53);
Speck myCipher;
RHEncryptedDriver driver(nrf24, myCipher);
RHReliableDatagram manager(driver, BASE_STATION_ADDRESS);
// ~~~ END OF RF MODULE INIT ~~~

// ~~~ COMMANDS FOR NODES ~~~
const char COMMAND_GET[4] = "GET";
char dataBuffer[RH_NRF24_MAX_MESSAGE_LEN];
char* dataBufferPtr = NULL;
uint8_t sAddress;
// ~~~ END OF COMMANDS FOR NODES ~~~

// ~~~ KEYPAD INIT ~~~
char hexaKeys[keypadColumns][keypadRows] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};
const byte rowPins[keypadRows] = {29, 27, 25, 23};
const byte colPins[keypadColumns] = {28, 26, 24, 22};
Keypad customKeypad = Keypad(makeKeymap(hexaKeys), colPins, rowPins, keypadRows, keypadColumns);
// ~~~ END OF KEYPAD INIT ~~~

// ~~~ CODES INIT ~~~
Password serviceCode = NULL;
Password extCode = NULL;
Password allCode = NULL;
// ~~~ END OF CODES INIT ~~~

// ~~~ ALARM STATE INIT ~~~
byte alarmWasTriggered = 0;
byte alarmActive = 0;
// ~~~ END OF ALARM STATE INIT ~~~

// ~~~ VARS FOR READING EEPROM DATA ~~~
byte* serviceCodeEeprom = NULL;
byte* extCodeEeprom = NULL;
byte* allCodeEeprom = NULL;
byte* encryptionKeyEeprom = NULL;
byte* telephoneEeprom = NULL;
// ~~~ END OF VARS FOR READING EEPROM DATA ~~~

// ~~~ RTC INIT ~~~
byte SDA_PIN = 20;
byte SCL_PIN = 21;
DS3231 rtc(SDA_PIN, SCL_PIN);
// ~~~ END OF RTC INIT ~~~

// ~~~ GSM INIT ~~~
#define PINNUMBER ""
GSM gsmObject;
GSM_SMS smsObject;
bool isConnected = false;
char smsText[128];
// ~~~ END OF GSM INIT ~~~

// ~~~ LOW BATTERY WARNING VARIABLES ~~~
byte isDoorBatteryLow = 0;
byte isWindowBatteryLow = 0;
byte isGasBatteryLow = 0;
byte isMotionBatteryLow = 0;
// ~~~ END OF LOW BATTERY WARNING VARIABLES ~~~

// ~~~ TIMEOUT CONSTATNTS ~~~
unsigned long HOUSE_LEAVE_TIMEOUT = 15000;
unsigned long GSM_CONNECT_TIMEOUT = 10000;
unsigned long FANCY_DELAY_TIMEOUT = 500;

// ~~~ DELAY WITHOT BLOCKING INIT ~~~
FancyDelay querryTime(FANCY_DELAY_TIMEOUT);
// ~~~ END OF DELAY WITHOT BLOCKING INIT ~~~

void setup() {
  digitalWrite(resetPin, HIGH);
  pinMode(resetPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, HIGH); //STOP BUZZER
  Serial.begin(115200);
  //  CLEAR_EEPROM(0, EEPROM.length());
  if (IS_EEPROM_EMPTY(EEPROM.length())) {
    WRITE_DEFAULT_CFG_TO_EEPROM();
  }
  Serial.println();
  READ_CFG_FROM_EEPROM();
  if (!manager.init()) {
    Serial.println("intit fail");
  }
  if (!nrf24.setChannel(0))
    Serial.println("setChannel failed");
  if (!nrf24.setRF(RH_NRF24::DataRate2Mbps, RH_NRF24::TransmitPower0dBm))
    Serial.println("setRF failed");
  manager.setRetries(10);
  myCipher.setKey(encryptKey, sizeof(encryptKey) - 1);
  rtc.begin();
}

// ~~~ EEPROM FUNCTIONS ~~~
void WRITE_DEFAULT_CFG_TO_EEPROM() {
  const char serviceCode[5] = "6193";
  const char extCode[5] = "1234";
  const char allCode[5] = "4321";
  const char encryptionKey[17] = "1349496701951491";
  const char telephone[11] = "1122334455";

  if (IS_EEPROM_EMPTY(EEPROM.length())) {
    Serial.println("EEPROM EMPTY");
    int nextAddress = 0;
    Serial.print("START ADDRESS = ");
    Serial.println(nextAddress);

    nextAddress = WRITE_TO_EEPROM(nextAddress, serviceCode);
    Serial.print("NEXT WRITABLE ADDRESS = ");
    Serial.println(nextAddress);

    nextAddress = WRITE_TO_EEPROM(nextAddress, extCode);
    Serial.print("NEXT WRITABLE ADDRESS = ");
    Serial.println(nextAddress);

    nextAddress = WRITE_TO_EEPROM(nextAddress, allCode);
    Serial.print("NEXT WRITABLE ADDRESS = ");
    Serial.println(nextAddress);

    nextAddress = WRITE_TO_EEPROM(nextAddress, telephone);
    Serial.print("NEXT WRITABLE ADDRESS = ");
    Serial.println(nextAddress);

    nextAddress = WRITE_TO_EEPROM(nextAddress, encryptionKey);
    Serial.print("NEXT WRITABLE ADDRESS = ");
    Serial.println(nextAddress);
  }
}

void CLEAR_EEPROM(int startAddress, int endAddress) {
  int currentAddress = startAddress;
  int numberOfClearedBlocks = 0;
  while (currentAddress < endAddress) {
    if (EEPROM.read(currentAddress) != 0xFF) {
      EEPROM.write(currentAddress, 0xFF);
      numberOfClearedBlocks++;
    }
    currentAddress++;
  }
  Serial.print("CLEARED BLOCKS = ");
  Serial.println(numberOfClearedBlocks);
}

void READ_CFG_FROM_EEPROM() {
  int readAddress = 0;
  serviceCodeAddress  = readAddress;

  serviceCodeEeprom = READ_FROM_EEPROM(readAddress, codeLength);
  serviceCode = Password((char*)serviceCodeEeprom);
  readAddress += strlen(serviceCodeEeprom) + 1;
  Serial.print("serviceCodeEeprom = ");
  Serial.println((char*&)serviceCode);

  extCodeAddress = readAddress;
  extCodeEeprom = READ_FROM_EEPROM(readAddress, codeLength);
  extCode = Password((char*)extCodeEeprom);
  readAddress +=  strlen(extCodeEeprom) + 1;
  Serial.print("extCodeEeprom = ");
  Serial.println((char*&)extCode);

  allCodeAddress = readAddress;
  allCodeEeprom = READ_FROM_EEPROM(readAddress, codeLength);
  allCode = Password((char*)allCodeEeprom);
  readAddress += strlen(allCodeEeprom) + 1;
  Serial.print("allCodeEeprom = ");
  Serial.println((char*&)allCode);

  telephoneAddress = readAddress;
  telephoneEeprom = READ_FROM_EEPROM(readAddress, telephoneNumberLength);
  memcpy((void*)telephoneNumber, (void*)telephoneEeprom, telephoneNumberLength - 1);
  readAddress += strlen(telephoneEeprom) + 1;
  Serial.print("telephoneEeprom = ");
  Serial.println((char*)telephoneEeprom);

  encryptionKeyAddress = readAddress;
  encryptionKeyEeprom = READ_FROM_EEPROM(readAddress, encryptionKeyLength);
  memcpy((void*)encryptKey, (void*)encryptionKeyEeprom, encryptionKeyLength - 1);
  readAddress += strlen(encryptionKeyEeprom) + 1;
  Serial.print("encryptionKeyEeprom = ");
  Serial.println((char*)encryptKey);

  Serial.print("NEXT READABLE ADDRESS = ");
  Serial.println(readAddress);
}

int WRITE_TO_EEPROM(int startAddress, char bytesToWrite[]) {
  int nextUsableAddress = startAddress;
  for (int i = 0 ; i < strlen(bytesToWrite) + 1; i++) {
    EEPROM.write(nextUsableAddress, bytesToWrite[i]);
    nextUsableAddress++;
  }
  EEPROM.write(nextUsableAddress, '\0');
  return nextUsableAddress;
}

bool IS_EEPROM_EMPTY(int eepromLength) {
  int currentAddress = 0;
  int unusedBlocks = 0;
  while (currentAddress < eepromLength) {
    byte value = EEPROM.read(currentAddress);
    if (value == 0xFF) {
      unusedBlocks++;
    }
    currentAddress++;
  }
  if (eepromLength == unusedBlocks) {
    return true;
  }
  return false;
}

byte* READ_FROM_EEPROM(int startAddress, int numberOfBytesToRead) {
  int currentAddress = startAddress;
  byte* dataBlock = new byte[numberOfBytesToRead + 1];
  Serial.println(numberOfBytesToRead);
  for (int i = 0 ; i < numberOfBytesToRead; i++) {
    *(dataBlock + i) = EEPROM.read(currentAddress);
    currentAddress++;
  }
  dataBlock[numberOfBytesToRead] = '\0';
  return dataBlock;
}

void SAVE_TO_EEPROM(int startAddress, byte* bytesToWrite, int numberOfBytesToWrite) {
  int nextUsableAddress = startAddress;
  Serial.print("Length = ");
  Serial.println(strlen(bytesToWrite));
  for (int i = 0 ; i < numberOfBytesToWrite; i++) {
    EEPROM.update(nextUsableAddress, bytesToWrite[i]);
    nextUsableAddress++;
  }
}
// ~~~ END OF EEPROM FUNCTIONS ~~~

// ~~~ ENABLE / DISABLE LOGIC ~~~
void clearCodes() {
  serviceCode.reset();
  allCode.reset();
  extCode.reset();
}

void activateAllSensors() {
  clearCodes();
  alarmActive = 2;
  long timeStamp = millis();
  while (millis() - timeStamp < HOUSE_LEAVE_TIMEOUT) {
    readKey();
  }
  Serial.println("SYS ON ALL");
}

void activateDoorsWindowsSensors() {
  clearCodes();
  alarmActive = 1;
  long timeStamp = millis();
  while (millis() - timeStamp < HOUSE_LEAVE_TIMEOUT) {
    readKey();
  }
  Serial.println("SYS ON EXT");
}

void checkPassword() {
  if (serviceCode.evaluate()) {
    if (alarmActive == 0) {
      Serial.println("Enter Service Mode");
      alarmActive = 3;
    } else {
      Serial.println("System Must Be Off");
    }
  }
  if (extCode.evaluate()) {
    if (alarmActive == 0 && alarmWasTriggered == 0) {
      activateDoorsWindowsSensors();
    } else if (alarmActive == 1 && alarmWasTriggered == 1) {
      deactivate();
    } else if (alarmWasTriggered == 0 && alarmActive == 1) {
      deactivate();
    }
  }
  if (allCode.evaluate()) {
    if (alarmActive == 0 && alarmWasTriggered == 0) {
      activateAllSensors();
    } else if (alarmActive == 2 && alarmWasTriggered == 1) {
      deactivate();
    } else if (alarmWasTriggered == 0 && alarmActive == 2) {
      deactivate();
    }
  }
}

void resetFunction() {
  //Clear the codes used
  delete[] serviceCodeEeprom;
  serviceCodeEeprom = NULL;
  delete[] allCodeEeprom;
  allCodeEeprom = NULL;
  delete[] extCodeEeprom;
  extCodeEeprom = NULL;
  delete[] encryptionKeyEeprom;
  encryptionKeyEeprom = NULL;
  delete[] telephoneEeprom;
  telephoneEeprom = NULL;
  //Clear battery status
  isDoorBatteryLow = 0;
  isWindowBatteryLow = 0;
  isGasBatteryLow = 0;
  isMotionBatteryLow = 0;
  isConnected = false;
  //Resets the base station
  digitalWrite(resetPin, LOW);
}

void deactivate() {
  Serial.println("SYS OFF");
  digitalWrite(buzzerPin, HIGH); //STOP BUZZER
  alarmWasTriggered = 0;
  alarmActive = 0;
  clearCodes();
  resetFunction();
}
// ~~~ END OF ENABLE / DISABLE LOGIC ~~~

// ~~~ SEND / RECEIVE DATA ~~~
int computeCommandDataSize(char* command) {
  return (strlen(command) + 1);
}

bool SEND(char* command, uint8_t toAddress) {
  if (manager.sendtoWait(command, computeCommandDataSize(command), toAddress)) {
    return true;
  }
  return false;
}

char* RECV(int receiveTimeout) {
  uint8_t len = sizeof(dataBuffer);
  if (manager.recvfromAckTimeout(dataBuffer, &len, receiveTimeout, &sAddress)) {
    return *&dataBuffer;
  } else {
    return NULL;
  }
}

void CHECK_VALUES(char* recvData) {
  if (recvData) {
    //Serial.println(recvData);
    float bVoltage;
    char sType[4];
    float sValue;
    char* ptr = strtok(recvData, ",");
    bVoltage = atof(ptr);
    ptr = strtok(NULL, ",");
    strcpy(sType, ptr);
    ptr = strtok(NULL, ",");
    sValue = atof(ptr);
    if (strcmp(sType, "HES") == 0 && sAddress == 2 && isDoorBatteryLow == 0) {
      // 1 -> Door Opened
      // 0 -> Door Closed
      if (sValue == 1) {
        alarmWasTriggered = 1;
        createMessage(sType, "Door Opened");
        digitalWrite(buzzerPin, LOW); //START BUZZER
        Serial.println("Alarm Triggered By Door Sensor");
        Serial.println(smsText);
        Serial.println("Waiting 15 Sec For Access Code");
        long timeStamp = millis();
        while (millis() - timeStamp < HOUSE_LEAVE_TIMEOUT) {
          readKey();
        }
        GSM_CONNECT(GSM_CONNECT_TIMEOUT);
        sendSMS(telephoneNumber, smsText);
        Serial.println("SMS Sent");
        GSM_DISCONNECT();
        ALARM_TRIGGERED();
      }
      if (bVoltage < 3300) {
        createMessage("Battery Warning", "Recharge Door Sensor Battery");
        Serial.println("Low Battery Door Sensor");
        Serial.println(smsText);
        GSM_CONNECT(GSM_CONNECT_TIMEOUT);
        sendSMS(telephoneNumber, smsText);
        Serial.println("SMS Sent");
        GSM_DISCONNECT();
        isDoorBatteryLow = 1;
      }
      sAddress = 0;
    }
    if (strcmp(sType, "HES") == 0 && sAddress == 3 && isWindowBatteryLow == 0) {
      // 1 -> Window Opened
      // 0 -> Window Closed
      if (sValue == 1) {
        alarmWasTriggered = 1;
        createMessage(sType, "Window Opened");
        digitalWrite(buzzerPin, LOW); //START BUZZER
        Serial.println("Alarm Triggered By Window Sensor");
        Serial.println(smsText);
        GSM_CONNECT(GSM_CONNECT_TIMEOUT);
        sendSMS(telephoneNumber, smsText);
        Serial.println("SMS Sent");
        GSM_DISCONNECT();
        ALARM_TRIGGERED();
      }
      if (bVoltage < 3300) {
        createMessage("Battery Warning", "Recharge Window Sensor Battery");
        Serial.println("Low Battery Window Sensor");
        Serial.println(smsText);
        GSM_CONNECT(GSM_CONNECT_TIMEOUT);
        sendSMS(telephoneNumber, smsText);
        Serial.println("SMS Sent");
        GSM_DISCONNECT();
        isWindowBatteryLow = 1;
      }
      sAddress = 0;
    }
    if (strcmp(sType, "GAS") == 0 && sAddress == 4 && isGasBatteryLow == 0) {
      // 0 -> Gas Detected
      // 1 -> No Gas Detected
      if (sValue == 0) {
        alarmWasTriggered = 1;
        createMessage(sType, "Gas Leakage Detected");
        digitalWrite(buzzerPin, LOW); //START BUZZER
        Serial.println("Alarm Triggered By Gas Sensor");
        Serial.println(smsText);
        GSM_CONNECT(GSM_CONNECT_TIMEOUT);
        sendSMS(telephoneNumber, smsText);
        Serial.println("SMS Sent");
        GSM_DISCONNECT();
        ALARM_TRIGGERED();
      }
      if (bVoltage < 3300) {
        createMessage("Battery Warning", "Recharge Gas Sensor Battery");
        Serial.println("Low Battery Gas Sensor");
        Serial.println(smsText);
        GSM_CONNECT(GSM_CONNECT_TIMEOUT);
        sendSMS(telephoneNumber, smsText);
        Serial.println("SMS Sent");
        GSM_DISCONNECT();
        isGasBatteryLow = 1;
      }
      sAddress = 0;
    }
    if (strcmp(sType, "PIR") == 0 && sAddress == 5 && isMotionBatteryLow == 0) {
      // 1 -> Motion Detected
      // 0 -> No Motion Detected
      if (sValue == 1) {
        alarmWasTriggered = 1;
        createMessage(sType, "Motion Was Detected");
        digitalWrite(buzzerPin, LOW); //START BUZZER
        Serial.println("Alarm Triggered By Motion Sensor");
        Serial.println(smsText);
        GSM_CONNECT(GSM_CONNECT_TIMEOUT);
        sendSMS(telephoneNumber, smsText);
        Serial.println("SMS Sent");
        GSM_DISCONNECT();
        ALARM_TRIGGERED();
      }
      if (bVoltage < 3300) {
        createMessage("Battery Warning", "Recharge Motion Sensor Battery");
        Serial.println("Low Battery Motion Sensor");
        Serial.println(smsText);
        GSM_CONNECT(GSM_CONNECT_TIMEOUT);
        sendSMS(telephoneNumber, smsText);
        Serial.println("SMS Sent");
        GSM_DISCONNECT();
        isMotionBatteryLow = 1;
      }
      sAddress = 0;
    }
  }
}

void ALARM_TRIGGERED() {
  while (alarmWasTriggered == 1 && alarmActive == 1 || alarmActive == 2) {
    readKey();
  }
}

void readKey() {
  char key = customKeypad.getKey();
  if (key) {
    switch (key) {
      case 'A':
        checkPassword();
        break;
      case 'D':
        Serial.println("Code Reset");
        clearCodes();
        Serial.print("serviceCode = ");
        Serial.println((char*&)serviceCode);
        Serial.print("extCode = ");
        Serial.println((char*&)extCode);
        Serial.print("allCode = ");
        Serial.println((char*&)allCode);
        Serial.print("telephoneNumberEeprom = ");
        Serial.println((char*)telephoneNumber);
        Serial.print("encryptionKey = ");
        Serial.println((char*)encryptKey);
        Serial.print("Day = ");
        Serial.println(rtc.getDOWStr());
        Serial.print("Date = ");
        Serial.println(rtc.getDateStr());
        Serial.print("Time = ");
        Serial.println(rtc.getTimeStr());
        break;
      default:
        Serial.println(key);
        extCode.append(key);
        allCode.append(key);
        serviceCode.append(key);
        break;
    }
  }
}

// ~~~ CONFIGURATION FUNCTIONS ~~~
bool KB_FILTER(char key) {
  if (key && key != 'A' && key != 'B' && key != 'C' && key != 'D' && key != '*' && key != '#') {
    return true;
  }
  return false;
}

void CHG_CODE(Password code, int codeStartAddress) {
  int pressedKey = 0;
  char codeBuf[codeLength];
  Serial.println("Input new code");
  while (pressedKey < codeLength - 1) {
    char key = customKeypad.getKey();
    if (KB_FILTER(key)) {
      Serial.println(key);
      codeBuf[pressedKey] = key;
      pressedKey++;
    }
  }
  memcpy((void*&)code, codeBuf, codeLength - 1);
  SAVE_TO_EEPROM(codeStartAddress, codeBuf, codeLength - 1);
  Serial.print("Code Changed = ");
  Serial.println((char*&)code);
  code.reset();
}

void CHG_TELEPHONE(int telephoneStartAddress) {
  int pressedKey = 0;
  char telephoneBuf[telephoneNumberLength];
  Serial.println("Input new telephone number");
  while (pressedKey < telephoneNumberLength - 1) {
    char key = customKeypad.getKey();
    if (KB_FILTER(key)) {
      Serial.println(key);
      telephoneBuf[pressedKey] = key;
      pressedKey++;
    }
  }
  memcpy((void*)telephoneNumber, telephoneBuf, telephoneNumberLength - 1);
  SAVE_TO_EEPROM(telephoneStartAddress, telephoneBuf, telephoneNumberLength - 1);
  Serial.print("Telephone Changed = ");
  Serial.println((char*)telephoneNumber);
}

int KEY_TO_NUMBER(char* key) {
  int result = -1;
  if (KB_FILTER(key)) {
    result = atoi(key);
  }
  return result;
}

void SET_DAY_OF_WEEK() {
  bool isDaySet = false;
  while (!isDaySet) {
    int day = READ_N_DIGITS("Input Day Of Week", 1);
    if (day != 0 && day <= 7) {
      rtc.setDOW(day);
      isDaySet = true;
    }
  }
  Serial.print("Day Set To = ");
  Serial.println(rtc.getDOWStr());
}

int READ_N_DIGITS(char* messageToPrint, int nrDigitsToRead) {
  Serial.println(messageToPrint);
  int pressedKey = 0;
  char* readBuf = new char[nrDigitsToRead + 1];
  char key;
  while (pressedKey < nrDigitsToRead) {
    key = customKeypad.getKey();
    if (KB_FILTER(key)) {
      int result = KEY_TO_NUMBER((char*)&key);
      if (result != -1) {
        readBuf[pressedKey] = key;
        pressedKey++;
      }
    }
  }
  readBuf[pressedKey] = '\0';
  int result = KEY_TO_NUMBER(readBuf);
  delete[] readBuf;
  readBuf = NULL;
  return result;
}

void SET_CLOCK() {
  int hh = READ_N_DIGITS("Input Hour", 2);
  int mm = READ_N_DIGITS("Input Minutes", 2);
  int ss = READ_N_DIGITS("Input Seconds", 2);
  rtc.setTime(hh, mm, ss);
  Serial.print("Time Set To = ");
  Serial.println(rtc.getTimeStr());
}

void SET_DATE() {
  int dd = READ_N_DIGITS("Input Day", 2);
  int mm = READ_N_DIGITS("Input Month", 2);
  int yyyy = READ_N_DIGITS("Input Year", 4);
  rtc.setDate(dd, mm, yyyy);
  Serial.print("Date Set To = ");
  Serial.println(rtc.getDateStr());
}

void CHG_ENCRYPTION_KEY(int encryptionKeyStartAddress) {
  int pressedKey = 0;
  char encKeyBuf[encryptionKeyLength];
  Serial.println("Input new encryption key");
  while (pressedKey < encryptionKeyLength - 1) {
    char key = customKeypad.getKey();
    if (KB_FILTER(key)) {
      Serial.println(key);
      encKeyBuf[pressedKey] = key;
      pressedKey++;
    }
  }
  Serial.print("Encryption Key Changed = ");
  Serial.println((char*)encryptKey);
}
// ~~~ END OF CONFIGURATION FUNCTIONS ~~~

// ~~~ GSM FUNCTIONS ~~~
void GSM_CONNECT(unsigned long connectTimeout) {
  gsmObject.begin(PINNUMBER, true, false);
  unsigned long elapsedTimeToConnect = millis();
  isConnected = false;
  while (!isConnected && (millis() - elapsedTimeToConnect < connectTimeout)) {
    int ok = 0;
    gsmObject.ready();
    ok = gsmObject.getStatus();
    if (ok != GSM_READY) {
      continue;
    } else {
      isConnected = true;
    }
  }
}

void GSM_DISCONNECT() {
  gsmObject.shutdown();
  isConnected = false;
  long timeStamp = millis();
  while (millis() - timeStamp < GSM_CONNECT_TIMEOUT) {
    readKey();
  }
}

void createMessage(char* sensorType, char* msg) {
  char* ptr = smsText;
  memcpy(ptr, msg, sizeof(char) * strlen(msg));
  ptr += strlen(msg);
  *ptr = ' ';
  ptr++;
  memcpy(ptr, sensorType, sizeof(char) * strlen(sensorType));
  ptr += strlen(sensorType);
  *ptr = ' ' ;
  ptr++;
  memcpy(ptr, rtc.getDOWStr(), sizeof(char) * strlen(rtc.getDOWStr()));
  ptr += strlen(rtc.getDOWStr());
  *ptr = ' ';
  ptr++;
  memcpy(ptr, rtc.getDateStr(), sizeof(char) * strlen(rtc.getDateStr()));
  ptr += strlen(rtc.getDateStr());
  *ptr = ' ';
  ptr++;
  memcpy(ptr, rtc.getTimeStr(), sizeof(char) * strlen(rtc.getTimeStr()));
  ptr += strlen(rtc.getTimeStr());
  *ptr = ' ' ;
  ptr++;
  *ptr = '\0';
}

void sendSMS(char* destinationNumber, char* messageToSend) {
  smsObject.beginSMS(destinationNumber);
  smsObject.println(messageToSend);
  smsObject.endSMS();
}
// ~~~ END OF GSM FUNCTIONS ~~~

unsigned long readKeyTimeout = 250;
void loop() {
  readKey();
  // monitor all except motion (EXT CODE)
  while (alarmActive == 1) {
    readKey();
    if (querryTime.ready()) {
      // check door state
      if (isDoorBatteryLow == 0) {
        if (SEND(COMMAND_GET, DOOR_NODE_ADDRESS)) {
          dataBufferPtr = RECV(10);
          if (dataBufferPtr) {
            CHECK_VALUES(dataBufferPtr);
            dataBufferPtr = NULL;
          }
        } else {
          Serial.println("Recv From Door Fail");
        }
      }

      long timeStamp = millis();
      while (millis() - timeStamp < readKeyTimeout ) {
        readKey();
      }
      // check window state
      if (isWindowBatteryLow == 0) {
        if (SEND(COMMAND_GET, WINDOW_NODE_ADDRESS)) {
          dataBufferPtr = RECV(10);
          if (dataBufferPtr) {
            CHECK_VALUES(dataBufferPtr);
            dataBufferPtr = NULL;
          }
        } else {
          Serial.println("Recv From Widow Fail");
        }
      }
      timeStamp = millis();
      while (millis() - timeStamp < readKeyTimeout ) {
        readKey();
      }
      // check for gas leak
      if (isGasBatteryLow == 0) {
        if (SEND(COMMAND_GET, GAS_NODE_ADDRESS)) {
          dataBufferPtr = RECV(10);
          if (dataBufferPtr) {
            CHECK_VALUES(dataBufferPtr);
            dataBufferPtr = NULL;
          }
        } else {
          Serial.println("Recv From Gas Fail");
        }
      }
    }
    readKey();
  }

  // monitor all sensors(ALL CODE)
  while (alarmActive == 2) {
    if (querryTime.ready()) {
      // check the door state
      if (isDoorBatteryLow == 0) {
        if (SEND(COMMAND_GET, DOOR_NODE_ADDRESS)) {
          dataBufferPtr = RECV(10);
          if (dataBufferPtr) {
            CHECK_VALUES(dataBufferPtr);
            dataBufferPtr = NULL;
          }
        } else {
          Serial.println("Recv From Door Fail");
        }
      }
      long timeStamp = millis();
      while (millis() - timeStamp < readKeyTimeout ) {
        readKey();
      }
      // check window state
      if (isWindowBatteryLow == 0) {
        if (SEND(COMMAND_GET, WINDOW_NODE_ADDRESS)) {
          dataBufferPtr = RECV(10);
          if (dataBufferPtr) {
            CHECK_VALUES(dataBufferPtr);
            dataBufferPtr = NULL;
          }
        } else {
          Serial.println("Recv From Widow Fail");
        }
      }
      timeStamp = millis();
      while (millis() - timeStamp < readKeyTimeout ) {
        readKey();
      }
      // check for gas leak
      if (isGasBatteryLow == 0) {
        if (SEND(COMMAND_GET, GAS_NODE_ADDRESS)) {
          dataBufferPtr = RECV(10);
          if (dataBufferPtr) {
            CHECK_VALUES(dataBufferPtr);
            dataBufferPtr = NULL;
          }
        } else {
          Serial.println("Recv From Gas Fail");
        }
      }
      timeStamp = millis();
      while (millis() - timeStamp < readKeyTimeout ) {
        readKey();
      }
      // check for motion
      if (isMotionBatteryLow == 0) {
        if (SEND(COMMAND_GET, PIR_NODE_ADDRESS)) {
          dataBufferPtr = RECV(10);
          if (dataBufferPtr) {
            CHECK_VALUES(dataBufferPtr);
            dataBufferPtr = NULL;
          }
        } else {
          Serial.println("Recv From Pir Fail");
        }
      }
    }
    readKey();
  }

  // config mode
  while (alarmActive == 3) {
    char key = customKeypad.getKey();
    if (key) {
      switch (key) {
        case '1': //extCode
          Serial.println("Enter Change extCode");
          CHG_CODE(extCode, extCodeAddress);
          break;
        case '2': //allCode
          Serial.println("Enter Change allCode");
          CHG_CODE(allCode, allCodeAddress);
          break;
        case '3': //serviceCode
          Serial.println("Enter Change serviceCode");
          CHG_CODE(serviceCode, serviceCodeAddress);
          break;
        case '4': //telephone number
          Serial.println("Enter Change Telephone Number");
          CHG_TELEPHONE(telephoneAddress);
          break;
        case '5': // date & time
          Serial.println("Enter Date/Time Settings");
          SET_DAY_OF_WEEK();
          SET_CLOCK();
          SET_DATE();
          break;
        case 'C': // exit
          alarmActive = 0;
          Serial.println("Exit Service Mode");
          break;
      }
    }
  }
}
