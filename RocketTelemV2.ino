#include "Wire.h"
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <EEPROM.h>

#define SEALEVELPRESSURE_HPA (1013.25)

const int MPU_ADDR = 0x68;

// MPU variables
int16_t accelerometer_x, accelerometer_y, accelerometer_z;
int16_t gyro_x, gyro_y, gyro_z;
int16_t mpu_temp;

// BMP variables
double bmp_temp, bmp_pressure, bmp_altitude;

// Normalization variables
int8_t ax_norm, gx_norm;
uint8_t temp_norm, pres_norm, alt_norm;

bool isArmed = false;
int c_addr = 0;

Adafruit_BMP3XX bmp;

void setup() {
  pinMode(3, INPUT);
  pinMode(7, OUTPUT);
  pinMode(12, OUTPUT);
  
  Serial.begin(115200);
  
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  bmp.begin_I2C();
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_2X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
}
void loop() {
  isArmed = digitalRead(3);

  // Setup MPU data request
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
  Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
  Wire.requestFrom(MPU_ADDR, 7*2, true); // request a total of 7*2=14 registers

  // Read raw MPU sensor data
  accelerometer_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
  accelerometer_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
  accelerometer_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
  mpu_temp = Wire.read()<<8 | Wire.read(); // reading registers: 0x41 (TEMP_OUT_H) and 0x42 (TEMP_OUT_L)
  gyro_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
  gyro_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
  gyro_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)

  // Read raw BMP sensor data
  bmp.performReading();
  bmp_temp = bmp.temperature;
  bmp_pressure = bmp.pressure;
  bmp_altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);

  // Normalize sensor readings to the range of a byte
  ax_norm = constrain(((accelerometer_x - 17500) / 62.5), -128, 127); // signed
  gx_norm = constrain((((gyro_x + 300)* 256) / 32000), -128, 127); // signed
  temp_norm = constrain(((bmp_temp - 4.4) * 10), 0, 255);
  pres_norm = constrain(((bmp_pressure - 96725) / 17.969), 0, 255);
  alt_norm = constrain((bmp_altitude / 5), 0, 255);

  // Write normalized readings to EEPROM when armed. Will iterate through the entire 1024 byte EEPROM only once
  if(isArmed && c_addr <= 1015){
    // Update status LEDS to "armed" mode
    digitalWrite(7, HIGH);
    digitalWrite(12, LOW);
    
    EEPROM.update(c_addr, ax_norm + 128); // Add 128 to adjust signed to unsigned
    EEPROM.update(c_addr + 1, gx_norm + 128);
    EEPROM.update(c_addr + 2, temp_norm);
    EEPROM.update(c_addr + 3, pres_norm);
    EEPROM.update(c_addr + 4, alt_norm);

    c_addr += 5;
  }
  else{
    // Update status LEDS to "idle" mode
    digitalWrite(7, LOW);
    digitalWrite(12, HIGH);
  }
  
  delay(100);
}
