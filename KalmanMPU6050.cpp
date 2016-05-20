#include "KalmanMPU6050.h"

KalmanMPU6050::KalmanMPU6050()
{
}

void KalmanMPU6050::setup()
{
	Wire.begin();

	#if ARDUINO >= 157
	Wire.setClock(400000UL); // Set I2C frequency to 400kHz
	#else
	TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
	#endif
  
  initializeMPU6050(); 

  initializeKalmanAngle();

  timer = micros();
}

void KalmanMPU6050::update() {
  readRawData();
  
  calculate();
}

double KalmanMPU6050::getRoll() {
  return kalAngleX;
}

double KalmanMPU6050::getPitch() {
  return kalAngleY;
}

void KalmanMPU6050::initializeMPU6050()
{
	mpu.initialize();

	delay(100); // Wait for sensor to stabilize
}

void KalmanMPU6050::initializeKalmanAngle()
{
	readRawData();

	double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  kalmanX.setAngle(roll); // Set starting angle
	kalmanY.setAngle(pitch);
	gyroXangle = roll;
	gyroYangle = pitch;
}

void KalmanMPU6050::readRawData() {
  int16_t ax,ay,az,gx,gy,gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); 
  accX = ax; accY = ay; accZ = az;
  gyroX = gx; gyroY = gy; gyroZ = gz;  
}

void KalmanMPU6050::calculate(){
  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

	double roll  = atan2(accY, accZ) * RAD_TO_DEG;
	double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;

	double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s
  
	if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) 
	{
	    kalmanX.setAngle(roll);
	    kalAngleX = roll;
	    gyroXangle = roll;
	} 
	else
	    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

    if (abs(kalAngleX) > 90)
    	gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  	kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
 
 	gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;

	// Reset the gyro angle when it has drifted too much
	if (gyroXangle < -180 || gyroXangle > 180)
	gyroXangle = kalAngleX;
	if (gyroYangle < -180 || gyroYangle > 180)
	gyroYangle = kalAngleY;
}

void KalmanMPU6050::calibration(int times) {  
  //First 100 measures are discarded
  for(int i=0; i<times;i++)
    readRawData();
    
  float valSums[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
	for (int i = 0; i < times; i++) {
		readRawData();
		valSums[0] += accX;
		valSums[1] += accY;
		valSums[2] += accZ;
		valSums[3] += gyroX;
		valSums[4] += gyroY;
		valSums[5] += gyroZ;
	}

	for (int i = 0; i < 6; i++) {
		calibData[i] = valSums[i] / times;
	}
 
  calibData[0] = -calibData[0]/8;
  calibData[1] = -calibData[1]/8;
  calibData[2] = (16384-calibData[2])/8;

  calibData[3] = -calibData[3]/4;
  calibData[4] = -calibData[4]/4;
  calibData[5] = -calibData[5]/4;
 
	storeCalibrationData();
}


void KalmanMPU6050::storeCalibrationData()
{  
  mpu.setXAccelOffset(calibData[0]);
  mpu.setYAccelOffset(calibData[1]);
  mpu.setZAccelOffset(calibData[2]);
  mpu.setXGyroOffset(calibData[3]);
  mpu.setYGyroOffset(calibData[4]);
  mpu.setZGyroOffset(calibData[5]);
}

void KalmanMPU6050::readCalibrationData()
{
  calibData[0] = mpu.getXAccelOffset();
  calibData[1] = mpu.getYAccelOffset();
  calibData[2] = mpu.getZAccelOffset();
  calibData[3] = mpu.getXGyroOffsetUser();
  calibData[4] = mpu.getYGyroOffsetUser();
  calibData[5] = mpu.getZGyroOffsetUser();
}

void KalmanMPU6050::reset()
{
	mpu.reset();
}
