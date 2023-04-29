#include "Wire.h" // This library allows you to communicate with I2C devices.
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <EEPROM.h>

#define SEALEVELPRESSURE_HPA (1013.25)

const int MPU_ADDR = 0x68; // I2C address of the MPU-6050. If AD0 pin is set to HIGH, the I2C address will be 0x69.

int16_t accelerometer_x, accelerometer_y, accelerometer_z; // variables for accelerometer raw data
int16_t gyro_x, gyro_y, gyro_z; // variables for gyro raw data
int16_t mpu_temp; // variables for temperature data

double bmp_temp;
double bmp_pressure;
double bmp_altitude;

uint8_t temp_norm, pres_norm, alt_norm;
int8_t ay_norm, gy_norm;

bool isArmed = false;
int c_addr = 0;

char tmp_str[7]; // temporary variable used in convert function

Adafruit_BMP3XX bmp;

char* convert_int16_to_str(int16_t i) { // converts int16 to string. Moreover, resulting strings will have the same length in the debug monitor.
  sprintf(tmp_str, "%6d", i);
  return tmp_str;
}

void setup() {
  pinMode(2, INPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  
  Serial.begin(115200);
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave (GY-521 board)
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  bmp.begin_I2C();
  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_2X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
}
void loop() {
  isArmed = digitalRead(2);
  
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
  Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
  Wire.requestFrom(MPU_ADDR, 7*2, true); // request a total of 7*2=14 registers
  
  // "Wire.read()<<8 | Wire.read();" means two registers are read and stored in the same variable
  accelerometer_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
  accelerometer_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
  accelerometer_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
  mpu_temp = Wire.read()<<8 | Wire.read(); // reading registers: 0x41 (TEMP_OUT_H) and 0x42 (TEMP_OUT_L)
  gyro_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
  gyro_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
  gyro_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)
  
  bmp.performReading();
  bmp_temp = bmp.temperature;
  bmp_pressure = bmp.pressure;
  bmp_altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  
/*
  // print out raw data
  Serial.print("aX = "); Serial.print(convert_int16_to_str(accelerometer_x));
  Serial.print(" | aY = "); Serial.print(convert_int16_to_str(accelerometer_y - 16000));
  Serial.print(" | aZ = "); Serial.print(convert_int16_to_str(accelerometer_z));
  // the following equation was taken from the documentation [MPU-6000/MPU-6050 Register Map and Description, p.30]
  Serial.print(" | tmp = "); Serial.print(mpu_temp/340.00+36.53);
  Serial.print(" | gX = "); Serial.print(convert_int16_to_str(gyro_x));
  Serial.print(" | gY = "); Serial.print(convert_int16_to_str(gyro_y));
  Serial.print(" | gZ = "); Serial.print(convert_int16_to_str(gyro_z));
  Serial.println();
  
  Serial.print("bmp_temp = "); Serial.print(bmp_temp);
  Serial.print(" | bmp_pressure = "); Serial.print(bmp_pressure);
  Serial.print(" | bmp_altitude = "); Serial.print(bmp_altitude);
  Serial.println();
  */

  // Normalize sensor readings to the size of a byte
  ay_norm = (accelerometer_y - 16000) / 62.5;
  gy_norm = (gyro_y * 256) / 32000;
  temp_norm = (bmp_temp - 4.4)* 10;
  pres_norm = (bmp_pressure - 96725) / 17.969;
  alt_norm = (bmp_altitude / 5);

  /*
  Serial.print("ay = "); Serial.print(ay_norm);
  Serial.print(" | gy = "); Serial.print(gy_norm);
  Serial.print(" | temp = "); Serial.print(temp_norm);
  Serial.print(" | pressure = "); Serial.print(pres_norm);
  Serial.print(" | altitude = "); Serial.print(alt_norm);
  Serial.println();
  */

  // Write to EEPROM
  if(isArmed && c_addr <= 1016){
    digitalWrite(3, HIGH);
    digitalWrite(4, LOW);
    
    EEPROM.update(c_addr, ay_norm);
    EEPROM.update(c_addr + 1, gy_norm);
    EEPROM.update(c_addr + 2, temp_norm);
    EEPROM.update(c_addr + 3, pres_norm);
    EEPROM.update(c_addr + 4, alt_norm);
    
    c_addr += 4;

    Serial.print("ay = "); Serial.print(EEPROM.read(c_addr));
    Serial.print(" | gy = "); Serial.print(EEPROM.read(c_addr + 1));
    Serial.print(" | temp = "); Serial.print(EEPROM.read(c_addr + 2));
    Serial.print(" | pressure = "); Serial.print(EEPROM.read(c_addr + 3));
    Serial.print(" | altitude = "); Serial.print(EEPROM.read(c_addr + 4));
    Serial.print(" | c_addr = "); Serial.print(c_addr);
    Serial.println();
  }
  else{
    digitalWrite(3, LOW);
    digitalWrite(4, HIGH);
  }
  
  // delay
  delay(100);
}
