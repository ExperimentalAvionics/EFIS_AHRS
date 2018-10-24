#include "Wire.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>
#include <mcp_can.h>

/*********************   CAN   **************/
const int CAN_CS_PIN = 10;   //CS pin for CAN board

//CAN Message details
unsigned char canMsg[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

//AHRS data
const unsigned int CAN_Euler_Msg_ID = 72; // CAN Msg ID in DEC 
const unsigned int CAN_Euler_Period = 45; // How often message sent in milliseconds
unsigned long CAN_Euler_Timestamp = 0; // when was the last message sent
int MAG = 0;
int gRoll = 0;
int gPitch = 0;
int gTurnRate = 0;

const unsigned int CAN_Acc_Msg_ID = 73; // CAN Msg ID in DEC 
const unsigned int CAN_Acc_Period = 45; // How often message sent in milliseconds
unsigned long CAN_Acc_Timestamp = 0; // when was the last message sent
int AccX = 0;
int AccY = 0;
int AccZ = 0;

/**********************   BNO 055   ********************************/

#define BNO055_ADDRESS 0x29
#define BNO_Mode Adafruit_BNO055::OPERATION_MODE_NDOF


byte CalReg = 0;

Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO055_ADDRESS);

uint8_t calstat; // calibration status
String CalDataOldString =""; // Calibration data from EEPROM

MCP_CAN CAN(CAN_CS_PIN);   // Set CS pin for CAN board

void setup() {

    Wire.begin();

    Serial.begin(115200);

   while (CAN_OK != CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ))              // init can bus : baudrate = 500k
    {
        Serial.println("CAN BUS Shield init fail");
        Serial.println(" Init CAN BUS Shield again");
        delay(200);
    }
    CAN.setMode(MCP_NORMAL);
    Serial.println("CAN BUS Shield init ok!");
    
  pinMode(4, OUTPUT); 
  pinMode(5, OUTPUT); 
  pinMode(7, INPUT);

      


    bno.begin();

    delay(50);
     bno.setMode(Adafruit_BNO055::OPERATION_MODE_CONFIG);
     delay(50);

  // Axis remap
  // swap Z and Y
  // change sign for all vectors
 // bno.write8(Adafruit_BNO055::BNO055_AXIS_MAP_CONFIG_ADDR, 0x00);
 // Adafruit_BNO055::write8(Adafruit_BNO055::BNO055_AXIS_MAP_SIGN_ADDR, 0x07);

  writeByte(BNO055_ADDRESS, Adafruit_BNO055::BNO055_AXIS_MAP_CONFIG_ADDR, 0x21);
  delay(50);
  writeByte(BNO055_ADDRESS, Adafruit_BNO055::BNO055_AXIS_MAP_SIGN_ADDR, 0x02);
  delay(50);

     CalDataOldString ="";
/*
     EEPROM.get(BNO_MemOffset, CalReg);
//     CalDataOldString += String(CalReg,HEX);
     bno.setCalvalMRL(CalReg);                    

     EEPROM.get(BNO_MemOffset + 1, CalReg);
//     CalDataOldString += String(CalReg,HEX);
     bno.setCalvalMRM(CalReg);

     EEPROM.get(BNO_MemOffset + 2, CalReg);
     CalDataOldString += String(CalReg,HEX);
     CalDataOldString += ":";
     bno.setCalvalMOXL(CalReg);

     EEPROM.get(BNO_MemOffset + 3, CalReg);
     CalDataOldString += String(CalReg,HEX);
     CalDataOldString += ":";
     bno.setCalvalMOXM(CalReg);
     
     EEPROM.get(BNO_MemOffset + 4, CalReg);
     CalDataOldString += String(CalReg,HEX);
     CalDataOldString += ":";
     bno.setCalvalMOYL(CalReg);

     EEPROM.get(BNO_MemOffset + 5, CalReg);
     CalDataOldString += String(CalReg,HEX);
     CalDataOldString += ":";
     bno.setCalvalMOYM(CalReg);

     EEPROM.get(BNO_MemOffset + 6, CalReg);
     CalDataOldString += String(CalReg,HEX);
     CalDataOldString += ":";
     bno.setCalvalMOZL(CalReg);

     EEPROM.get(BNO_MemOffset + 7, CalReg);
     CalDataOldString += String(CalReg,HEX);
     bno.setCalvalMOZM(CalReg);
     
    
//      Serial.println(CalDataOldString);
*/
      bno.setMode(Adafruit_BNO055::OPERATION_MODE_NDOF);
//      bno.setMode(Adafruit_BNO055::OPERATION_MODE_NDOF_FMC_OFF);
      delay(50);

       bno.setExtCrystalUse(true);
       delay(50);



  //     digitalWrite(4, HIGH);
  //     digitalWrite(5, HIGH);

}

void loop() {
 // digitalWrite(4, digitalRead(7));

 Gyro();

// Send Euler data
if (millis() > CAN_Euler_Timestamp + CAN_Euler_Period + random(0, 10)) {

  canMsg[0] = MAG;
  canMsg[1] = MAG >> 8;

  canMsg[2] = gRoll;
  canMsg[3] = gRoll >> 8;

  canMsg[4] = gPitch;
  canMsg[5] = gPitch >> 8;

  canMsg[6] = gTurnRate;
  canMsg[7] = gTurnRate >> 8;
  

  CAN.sendMsgBuf(CAN_Euler_Msg_ID, 0, 8, canMsg); 
  
  CAN_Euler_Timestamp = millis();
}

// Send Acceleration data
if (millis() > CAN_Acc_Timestamp + CAN_Acc_Period + random(0, 10)) {

  canMsg[0] = AccX;
  canMsg[1] = AccX >> 8;

  canMsg[2] = AccY;
  canMsg[3] = AccY >> 8;

  canMsg[4] = AccZ;
  canMsg[5] = AccZ >> 8;

  CAN.sendMsgBuf(CAN_Acc_Msg_ID, 0, 6, canMsg); 
  
  CAN_Acc_Timestamp = millis();
}

 

}


 uint8_t readByte(uint8_t address, uint8_t subAddress)
{
 uint8_t data; // `data` will store the register data   
  Wire.beginTransmission(address);         // Initialize the Tx buffer
  Wire.write(subAddress);                  // Put slave register address in Tx buffer
  Wire.endTransmission(false);        // Send the Tx buffer, but send a restart to keep connection alive
//  Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
//  Wire.requestFrom(address, 1);  // Read one byte from slave register address 
  Wire.requestFrom(address, (size_t) 1);   // Read one byte from slave register address 
  data = Wire.read();                      // Fill Rx buffer with result
  return data;                             // Return data read from slave register
}

 void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
  Wire.beginTransmission(address);  // Initialize the Tx buffer
  Wire.write(subAddress);           // Put slave register address in Tx buffer
  Wire.write(data);                 // Put data in Tx buffer
  Wire.endTransmission();           // Send the Tx buffer
}

void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{  
  Wire.beginTransmission(address);   // Initialize the Tx buffer
  Wire.write(subAddress);            // Put slave register address in Tx buffer
  Wire.endTransmission(false);  // Send the Tx buffer, but send a restart to keep connection alive
  uint8_t i = 0;
//        Wire.requestFrom(address, count);  // Read bytes from slave register address 
        Wire.requestFrom(address, (size_t) count);  // Read bytes from slave register address 
  while (Wire.available()) {
        dest[i++] = Wire.read(); }         // Put read results in the Rx buffer
}

