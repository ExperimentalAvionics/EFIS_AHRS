void Gyro() {
/*
VECTOR_MAGNETOMETER (values in uT, micro Teslas)
VECTOR_GYROSCOPE (values in rps, radians per second)
VECTOR_EULER (values in Euler angles or 'degrees', from 0..359)
VECTOR_ACCELEROMETER (values in m/s^2)
VECTOR_LINEARACCEL (values in m/s^2)
VECTOR_GRAVITY (values in m/s^2)
*/ 

// Get EULER data

  sensors_event_t event; 
  bno.getEvent(&event);

  MAG = event.orientation.x;
  gRoll = (-1)*event.orientation.y;
  gPitch = (-1)*event.orientation.z;


//Get acceleration data

  imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

  AccX = acc.x()*100; //slip ball
  AccY = acc.y()*100; //forward acceleration
  AccZ = acc.z()*100; //wing loading

  calstat = bno.getCalib();
  
}

void ReadGyroCalibration() {
     EEPROM.get(BNO_MemOffset, CalMRL);
     EEPROM.get(BNO_MemOffset + 1, CalMRM);
     EEPROM.get(BNO_MemOffset + 2, CalMOXL);
     EEPROM.get(BNO_MemOffset + 3, CalMOXM);
     EEPROM.get(BNO_MemOffset + 4, CalMOYL);
     EEPROM.get(BNO_MemOffset + 5, CalMOYM);
     EEPROM.get(BNO_MemOffset + 6, CalMOZL);
     EEPROM.get(BNO_MemOffset + 7, CalMOZM);
}

void WriteGyroCalibration() {
// if satisfied with calibration status, save the calibration data into EEPROM so it could be written later into BNO0055 on startup 
// or when BNO's self-calibration messes up the data

    Serial.println("###### Writing calibration to EEPROM ######");
    bno.setMode(Adafruit_BNO055::OPERATION_MODE_CONFIG);
    delay(50);
     
     // Read calibration data from the sensor
    CalMRL = bno.getCalvalMRL(); 
    CalMRM = bno.getCalvalMRM();
    CalMOXL = bno.getCalvalMOXL();
    CalMOXM = bno.getCalvalMOXM();
    CalMOYL = bno.getCalvalMOYL();
    CalMOYM = bno.getCalvalMOYM();
    CalMOZL = bno.getCalvalMOZL();
    CalMOZM = bno.getCalvalMOZM();

    EEPROM.put(BNO_MemOffset, CalMRL);
    EEPROM.put(BNO_MemOffset + 1, CalMRM);
    EEPROM.put(BNO_MemOffset + 2, CalMOXL);
    EEPROM.put(BNO_MemOffset + 3, CalMOXM);
    EEPROM.put(BNO_MemOffset + 4, CalMOYL);
    EEPROM.put(BNO_MemOffset + 5, CalMOYM);
    EEPROM.put(BNO_MemOffset + 6, CalMOZL);
    EEPROM.put(BNO_MemOffset + 7, CalMOZM);

    bno.setMode(BNO_Mode);
    delay(50);
}

void Gyro_Calibration() {
  // this procedure is just for bebugging only
  // it is controlled via on-board button
  // It shows calibration status in the serial output. 
  // There are 4 of them - System, Gyro, Accelerometer and Magnetometer
  // Best calibration if the values for all four equal to 3.

  //  uint8_t calstat = readByte(0x29, Adafruit_BNO055::BNO055_CALIB_STAT_ADDR);

  calstat = bno.getCalib();

  byte calSys = (0xC0 & calstat) >> 6;
  byte calGyro = (0x30 & calstat) >> 4;
  byte calAccel = (0x0C & calstat) >> 2;
  byte calMag = (0x03 & calstat) >> 0;

  Serial.println("==============================");
  Serial.println("Sys: " + String(calSys) + " G:" + String(calGyro) + " A:" + String(calAccel) + " M:" + String(calMag));
  
  Serial.print("Pitch = ");
  Serial.println(gPitch);

  Serial.print("Roll = ");
  Serial.println(gRoll);

  Serial.print("Heading = ");
  Serial.println(MAG);

  Serial.println("Acceleration");
  Serial.print("X: ");
  Serial.print(AccX);
  Serial.print(" Y: ");
  Serial.print(AccY);
  Serial.print(" Z: ");
  Serial.print(AccZ);
  Serial.println("");

  delay(100);
}

void UpdateGyroCalibration() {
  // write calibration data into BNO055 chip
  Serial.println("=****= Updating Calibration =****=");
  bno.setMode(Adafruit_BNO055::OPERATION_MODE_CONFIG);
  delay(50);
  bno.setCalvalMRL(CalMRL);                    
  bno.setCalvalMRM(CalMRM);
  bno.setCalvalMOXL(CalMOXL);
  bno.setCalvalMOXM(CalMOXM);
  bno.setCalvalMOYL(CalMOYL);
  bno.setCalvalMOYM(CalMOYM);
  bno.setCalvalMOZL(CalMOZL);
  bno.setCalvalMOZM(CalMOZM);
  
  bno.setMode(BNO_Mode);
  delay(50);

}
