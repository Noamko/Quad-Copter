#ifndef IMU_H
#define IMU_H

#include <Wire.h>
#include "config.h"

class IMU
{
public:
	IMU();
	void Init();
	void reset();
	void Compute();
	void Calibrate_gyro();
	void Calibrate_acc();
	void Calibrate_compass();
	void setAL_Mul(float mul);
	float Roll_Level_Error();
	float Pitch_Level_Error();
	float Get_Heading();
	void Calculate_Heading();
	float Get_GyroX_Angle();
	float Get_GyroY_Angle();
	float Get_pressure();
	float Pressure_from_altitude(float ref_p,float alt);
	float Get_refPerssure();
	float Get_altitude();
	float Get_offset(uint8_t xy);
	int16_t Get_AccX();
	int16_t Get_AccY();
	int16_t Get_AccZ();
	int16_t Get_LP_AccZ();
	int16_t Get_GyroX();
	int16_t Get_GyroY();
	int16_t Get_GyroZ();
	void Calculate_pressure(uint8_t OSR);
	float roll_angle_offset = 0.0f, pitch_angle_offset = 0.0f;
	bool compass_calibrated = false;

private:
	void InitHMC58331();
	void Init_MS5611();
	void InitMPU6050();
	void readMPU6050();
	void read_HMC5883L();

	//MS5611 variables
	uint8_t temperature_read_counter = 0;
	uint8_t pressure_read_counter = 0;
	uint16_t C[7];
	uint32_t D1, D2;
	int32_t dT,dT_C5, actual_temperature, actual_pressure,ref_pressure;
	int64_t OFF,OFF_C2,SENS,SENS_C1;
	int64_t temp_C6;
	int32_t avarage_pressure_sample[20];
	int32_t avarage_temperature_sample[5];
	uint8_t avarage_pressure_index = 0;
	uint8_t avarage_temperature_index = 0;
	float pressure_avarage_total, temperature_avarage_total;
	float avarage_pressure,avarage_temperature;
	float LPF_pressure,pressure_diff;
	float actual_altitude, reletive_altitude;

	//HMC5883L
	float mag_angle;
	float heading;
	float compass_x_horizontal, compass_y_horizontal,actual_compass_heading;
	int16_t mag_x,mag_y,mag_z;
	int16_t compass_cal_values[6];
	int16_t compass_offset_x, compass_offset_y, compass_offset_z;
	float course_a, course_b, course_c, base_course_mirrored, actual_course_mirrored;
	float course_lock_heading, heading_lock_course_deviation;
	float compass_scale_y, compass_scale_z;
	

	//MPU6050 variables
	bool calibrated = false;
	bool auto_level = true;
	float gyro_data[3];
	float acc_angle[2];
	float gyro_angle[2];
	float gyro_offset[3];
	float z_velocity;
	double gyro_time = 0.0000610687056905589997768402099609375;  // (  250Hz/ gyro scale )
	double gyro_time_radians = 0.0000010658499149940325878560543060302734375; //converted to radians
	int16_t gyro_raw[3];
	int16_t acc_raw[3];
	int16_t mpu6050_temperature;
	int16_t accz_offset;
	int16_t acc_z;
	int16_t prev_acc;

	 //General
	 float roll_level_ajust;
	 float pitch_level_ajust;
	 float al_mul = 3;
	 uint32_t prev_dt;
	 uint32_t dt;
};
#endif
