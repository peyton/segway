/*

Sensors Module

*/

int getGyroRate() {

  gyro.readGyro(&dx,&dy,&dz); 
  sensorValue[4] = dy;
  
  return int(sensorValue[4] / 2.844444664897526);  // in quid/sec
}

int getAccAngle() {  

  int accelx,accely,accelz;  
  adxl.readAccel(&accelx, &accely, &accelz); //read the accelerometer values and store them
  
  accelx = accelx - balancezero;
  
  sensorValue[0] = accelx;
  sensorValue[2] = accelz;
  
  return sensorValue[0];
}

 
