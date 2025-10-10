///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup MPU6050 registers
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void gyro_setup(void) {
  Wire.beginTransmission(gyro_address);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(gyro_address);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();

  Wire.beginTransmission(gyro_address);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();

  Wire.beginTransmission(gyro_address);
  Wire.write(0x1A);
  Wire.write(0x03);
  Wire.endTransmission();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Gyro calibration
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void calibrate_gyro(void) {
  gyro_roll_cal = gyro_pitch_cal = gyro_yaw_cal = 0;
  for (cal_int = 0; cal_int < 2000; cal_int++) {
    gyro_signalen();
    gyro_roll_cal += gyro_roll;
    gyro_pitch_cal += gyro_pitch;
    gyro_yaw_cal += gyro_yaw;
    delay(4);
  }
  gyro_roll_cal /= 2000;
  gyro_pitch_cal /= 2000;
  gyro_yaw_cal /= 2000;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Read raw gyro and accelerometer data
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void gyro_signalen(void) {
  Wire.beginTransmission(gyro_address);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(gyro_address, 14);

  acc_y = Wire.read() << 8 | Wire.read();
  acc_x = Wire.read() << 8 | Wire.read();
  acc_z = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read(); // temperature, ignored
  gyro_roll = Wire.read() << 8 | Wire.read();
  gyro_pitch = Wire.read() << 8 | Wire.read();
  gyro_yaw = Wire.read() << 8 | Wire.read();

  gyro_pitch *= -1;
  gyro_yaw *= -1;

  if (cal_int >= 2000) {
    gyro_roll -= gyro_roll_cal;
    gyro_pitch -= gyro_pitch_cal;
    gyro_yaw -= gyro_yaw_cal;
  }
}