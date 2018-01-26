#ifndef IMU_H
#define IMU_H


#include <SoftWire.h>
#include <Arduino.h>
#include "config.h"

class IMU
{
public:
	IMU();
	void Init();
	void reset();
	void Compute();
	void Calibrate_MPU6050();
	void setAL_Mul(float mul);
	void SetOverSampling(uint8_t osr);
	float Roll_Level_Error();
	float Pitch_Level_Error();
	float Get_Altitude();
	float Get_Heading();
	float Get_GyroX_Angle();
	float Get_GyroY_Angle();
	float Get_Velocity();
	float Get_offset(uint8_t xy);
	int16_t Get_AccX();
	int16_t Get_AccY();
	int16_t Get_AccZ();
	int16_t Get_LP_AccZ();
	int16_t Get_GyroX();
	int16_t Get_GyroY();
	int16_t Get_GyroZ();


private:
	void InitHMC58331();
	void Init_MS5611();
	void InitMPU6050();
	void reset_MS5611();
	void readMPU6050();
	void read_HMC5883L();
	float Calculate_Estimated_Altitude(bool compensation = false);
	int32_t ms5611_readPressure(bool compensation = false);
	uint16_t readRegister16(uint8_t reg);
	uint32_t readRegister24(uint8_t reg);

	//MS5611 variables
	float altitude;
	float est_alt;
	float prev_alt;
	float referencePressure;
	float baro_vel;
	uint32_t D1,D2;
	int32_t TEMP2;
	uint16_t fc[6];
	uint8_t ct = CT;
	uint8_t skip_case = 0;
	int64_t OFF2, SENS2;

	//HMC5883L
	float mag_angle;
	float heading;
	int16_t mag_x,mag_y,mag_z;

	//mpu6050 variables
	bool calibrated = false;
	bool auto_level = true;
	float gyro_data[3];
	float acc_angle[2];
	float gyro_angle[2];
	float gyro_time = 0.0000610687056905589997768402099609375;  // (  250Hz/ gyro scale )
	float gyro_time_radians = 0.0000010658499149940325878560543060302734375; //converted to radians
	float gyro_offset[3];
	float z_velocity;
	int16_t gyro_raw[3];
	int16_t acc_raw[3];
	int16_t mpu6050_temperature;
	int32_t accz_offset;
	int16_t acc_z;
	int16_t prev_acc;

	 //General
	 float roll_level_ajust;
	 float pitch_level_ajust;
	 float al_mul = 5.0;
	 uint32_t prev_dt;
	 uint16_t dt;

};
#endif
