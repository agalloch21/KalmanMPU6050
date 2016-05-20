/*
 * Calibration method made by Moderators at
 * http://www.i2cdevlib.com/forums/topic/96-arduino-sketch-to-automatically-calculate-mpu6050-offsets/
 * 
 * 
 * Kalman Library made by Lauszus at
 * https://github.com/TKJElectronics/KalmanFilter
 * 
 * 
 * I2C and MPU6050 made by jrowberg at
 * https://github.com/jrowberg/i2cdevlib
 */



#ifndef _MPU6050_H_
#define _KALMANMPU6050_

#include <Arduino.h>
#include <MPU6050.h>
#include <I2Cdev.h>
#include <Wire.h>
#include <Kalman.h>

class KalmanMPU6050
{
	public:
		KalmanMPU6050();

		void setup();
		void update();
    void reset();
    
		void calibration(int times = 1000);
		double getRoll();
		double getPitch();
    
	private:
		void initializeMPU6050();
		void initializeKalmanAngle();

		void readRawData();
		void calculate();

		void storeCalibrationData();
		void readCalibrationData();

	private:
    MPU6050 mpu;

    int16_t calibData[6];

		double accX, accY, accZ;
		double gyroX, gyroY, gyroZ;
		

		double gyroXangle, gyroYangle; // Angle calculate using the gyro only
		double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

		Kalman kalmanX; // Create the Kalman instances
		Kalman kalmanY;

    uint32_t timer;

};
#endif
