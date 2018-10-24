void Gyro() {


  // Check calibration status of the sensors

  uint8_t calstat = readByte(0x29, Adafruit_BNO055::BNO055_CALIB_STAT_ADDR);
//  Serial.println("Not calibrated = 0, fully calibrated = 3");
//  Serial.print("System calibration status "); Serial.println( (0xC0 & calstat) >> 6);
//  Serial.print("Gyro   calibration status "); Serial.println( (0x30 & calstat) >> 4);
//  Serial.print("Accel  calibration status "); Serial.println( (0x0C & calstat) >> 2);
//  Serial.print("Mag    calibration status "); Serial.println( (0x03 & calstat) >> 0);

  /*
  bno.setMode(Adafruit_BNO055::OPERATION_MODE_CONFIG);
  Serial.println(bno.getCalvalMOXL());
  Serial.println(bno.getCalvalMOXM());
  Serial.println(bno.getCalvalMOYL());
  Serial.println(bno.getCalvalMOYM());
  Serial.println(bno.getCalvalMOZL());
  Serial.println(bno.getCalvalMOZM());
  bno.setMode(Adafruit_BNO055::OPERATION_MODE_NDOF);
  */
  calstat = bno.getCalib();
  Serial.println(0x03 & calstat);
  
  sensors_event_t event; 
  bno.getEvent(&event);

  MAG = event.orientation.x;
  gRoll = (-1)*event.orientation.y;
  gPitch = (-1)*event.orientation.z;

 Serial.print("Pitch = ");
Serial.println(gPitch);

Serial.print("Roll = ");
Serial.println(gRoll);

Serial.print("Heading = ");
Serial.println(MAG);

/*  
  if (abs(gPitch)>90) {
    if (gRoll > 0 ) {
      gRoll = 180-gRoll;
    } else {
      gRoll = -180-gRoll;
    }
  }

  if (gPitch>90) {
    gPitch = 180 - gPitch;
  }

  if (gPitch<-90) {
    gPitch = -180 - gPitch;
  }
*/






//************************ Slip ball **************************
/*
VECTOR_MAGNETOMETER (values in uT, micro Teslas)
VECTOR_GYROSCOPE (values in rps, radians per second)
VECTOR_EULER (values in Euler angles or 'degrees', from 0..359)
VECTOR_ACCELEROMETER (values in m/s^2)
VECTOR_LINEARACCEL (values in m/s^2)
VECTOR_GRAVITY (values in m/s^2)
*/
  imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

Serial.print("X: ");
Serial.print(acc.x());
Serial.print(" Y: ");
Serial.print(acc.y());
Serial.print(" Z: ");
Serial.print(acc.z());
Serial.println("");

  AccX = acc.x()*100; //slip ball
  AccY = acc.y()*100; //forward acceleration
  AccZ = acc.z()*100; //wing loading


}


