#include "Wire.h"
#include <EEPROM.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>
#include <mcp_can.h>

/*********************   CAN   **************/
const int CAN_CS_PIN = 10;   //CS pin for CAN board
unsigned long canId;
unsigned char len = 0;
unsigned char ext = 0;

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
//#define BNO_Mode Adafruit_BNO055::OPERATION_MODE_NDOF
#define BNO_Mode Adafruit_BNO055::OPERATION_MODE_NDOF_FMC_OFF

/*    OPERATION_MODE_CONFIG                                   = 0X00, 
      OPERATION_MODE_ACCONLY                                  = 0X01,
      OPERATION_MODE_MAGONLY                                  = 0X02,
      OPERATION_MODE_GYRONLY                                  = 0X03,
      OPERATION_MODE_ACCMAG                                   = 0X04,
      OPERATION_MODE_ACCGYRO                                  = 0X05,
      OPERATION_MODE_MAGGYRO                                  = 0X06,
      OPERATION_MODE_AMG                                      = 0X07,
      OPERATION_MODE_IMUPLUS                                  = 0X08,
      OPERATION_MODE_COMPASS                                  = 0X09,
      OPERATION_MODE_M4G                                      = 0X0A,
      OPERATION_MODE_NDOF_FMC_OFF                             = 0X0B,
      OPERATION_MODE_NDOF                                     = 0X0C
 */


byte Calibration_Mode = 0;
byte Old_Calib_Mode = 0;

// BNO Calibration Variables
byte CalMRL = 0;
byte CalMRM = 0;
byte CalMOXL = 0;
byte CalMOXM = 0;
byte CalMOYL = 0;
byte CalMOYM = 0;
byte CalMOZL = 0;
byte CalMOZM = 0;

int CalibrationStatus = 0;

unsigned long int CalUpdateTimer = 0;
unsigned long int CalUpdatePeriod = 5000;


Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO055_ADDRESS);

byte calstat; // calibration status

/**********************  EEPROM  ***************************************/
int BNO_MemOffset = 0;  // starting point of calibration data stored in EEPROM


MCP_CAN CAN(CAN_CS_PIN);   // Set CS pin for CAN board

#define GreenLED 5
#define RedLED 4
#define ButtonPin 7

long unsigned int ButtonTimer = 0;

void setup() {

// Read Gyro calibration data from EEPROM (if available)
    ReadGyroCalibration();

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

//Set modes for calibration button pin and status LED pins     
 
    pinMode(GreenLED, OUTPUT); 
    pinMode(RedLED, OUTPUT); 
    pinMode(ButtonPin, INPUT);

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
  bno.setExtCrystalUse(true);
  delay(50);
  bno.setMode(BNO_Mode);
  delay(50);
  
  ReadGyroCalibration(); // Read calibration data from EEPROM

  // Just in case check if the it is actually some meaningfull data, not just empty bytes (0xFF)
  if (CalMRL == 0xFF and CalMRM == 0xFF) {
    Serial.println("###### EEPROM is empty. Caibrate the Gyro ######");
  } else {
    UpdateGyroCalibration();
  }

}

void loop() {
// Read messages from CAN network
if(CAN_MSGAVAIL == CAN.checkReceive())            // check if data coming
  {
    CAN.readMsgBuf(&canId, &ext, &len, canMsg);    // read data,  len: data length, buf: data buf
    switch (canId) {
      case 35:
        {
        // First byte [0] is calibration command
        Serial.println("Message 35 arrived" + String(canMsg[0], HEX));
        // FIRST bit in the [0] byte is a signal to save the data into EEPROM
        // make sure the save command is not executed too often. EEPROM has limited number of write cycles (100000)
          if (canMsg[0] == 1) {
            Serial.println("writing calibration");
            WriteGyroCalibration();
            // flash the Red LED for a few moments (if installed)
            for (int i = 0; i <= 10; i++) {
              digitalWrite(RedLED, !digitalRead(RedLED));
              delay(200);
            }
          }
          
        // SECOND bit in the [0] byte is a signal to temporary disable the calibration updates 
        // to ensure they dont messup calibration efforts
          if (canMsg[0] == 0x02) {
            Calibration_Mode = 1;
            Serial.println("pause calibration refresh ");
            digitalWrite(RedLED, 1);
          } else {
            Calibration_Mode = 0;
            Serial.println("resume calibration refresh");
            digitalWrite(RedLED, 0);
          }
        }
        break;
      default: 
        // if nothing else matches, do the default
        // default is optional
      break;
    }
  }
  
// BEGIN of the Button and LED handling block *********************************************************
// the following block if for debugging only.
// it controlls the onboard button and the LED's
// calibration can be saved into EEPROM by activating the "calibration" mode by pressing the button once
// Red LED should be steady ON
// once satisfied with the calibration status press and hold the button untill the Red LED start slashing
// at that point the calibration will be written to EEPROM and temporary variables.
 
  if (digitalRead(ButtonPin) == 0) {
    digitalWrite(GreenLED,1);
    if (ButtonTimer == 0) {
      ButtonTimer = millis();
    } else {
      if (millis() - ButtonTimer > 2000 and digitalRead(RedLED) == 1) {
        digitalWrite(RedLED,1);
        // write calibration data into EEPROM
        for (int i = 0; i <= 10; i++) {
           digitalWrite(RedLED, !digitalRead(RedLED));
           delay(200);
        }
        WriteGyroCalibration();
        Calibration_Mode = 0;
        digitalWrite(RedLED,0);
        ButtonTimer = 0;
      }
    }
  } else {
    digitalWrite(GreenLED,0);
    
    //toggle Red Led and calibration status
    if (ButtonTimer > 0) { //button released
      digitalWrite(RedLED, !digitalRead(RedLED));
      if (Calibration_Mode == 0) {
        Calibration_Mode = 1;
      } else {
        Calibration_Mode = 0;
      }
    }
    
    ButtonTimer = 0;
    
  }
  
  if (Calibration_Mode == 1) {
    Gyro_Calibration();
  } 
  
// END of the Button and LED handling block *********************************************************

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

      canMsg[6] = calstat;
    
      CAN.sendMsgBuf(CAN_Acc_Msg_ID, 0, 7, canMsg); 
      
      CAN_Acc_Timestamp = millis();
    }

 
// Update calibration
// If calibration mode is activated this update will not run to avoid messing up the calibration effors
  if (millis()-CalUpdateTimer > CalUpdatePeriod and Calibration_Mode == 0) {
    UpdateGyroCalibration();
    CalUpdateTimer = millis();
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
