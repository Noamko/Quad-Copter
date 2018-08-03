#include "IMU.h"

IMU::IMU(){};
TwoWire TWire(2,I2C_FAST_MODE);

float IMU::Pitch_Level_Error() 	{ return pitch_level_ajust; }
float IMU::Roll_Level_Error() 	{ return roll_level_ajust; }
float IMU::Get_GyroX_Angle() 	{ return gyro_angle[0]; }
float IMU::Get_GyroY_Angle() 	{ return gyro_angle[1]; }
float IMU::Get_pressure() 		{ return LPF_pressure; }
float IMU::Get_Heading() 		{ return heading; }
float IMU::Get_refPerssure() 	{ return ref_pressure; }
float IMU::Get_altitude()		{ return reletive_altitude; }


void IMU::setAL_Mul(float mul) { al_mul = mul; }

float IMU::Get_offset(uint8_t xy) { return gyro_offset[xy]; }
int16_t IMU::Get_AccX(){return acc_raw[0];}
int16_t IMU::Get_AccY(){return acc_raw[1];}
int16_t IMU::Get_AccZ(){return acc_raw[2];}
int16_t IMU::Get_LP_AccZ(){return acc_z;}
int16_t IMU::Get_GyroX() {return gyro_data[0];}
int16_t IMU::Get_GyroY() {return gyro_data[1];}
int16_t IMU::Get_GyroZ() {return gyro_data[2];}

void IMU::Init()
{
	TWire.begin();
	delay(250);
#ifdef MPU6050
	InitMPU6050();
	delay(250);
#endif
#ifdef HMC5883L
	InitHMC58331();
	delay(250);
#endif
#ifdef MS5611
	Init_MS5611();
	delay(250);
#endif
}

void IMU::InitHMC58331()
{
	TWire.beginTransmission(MPU6050_ADDRESS);
	TWire.write(0x37);
	TWire.write(0x02);
	TWire.endTransmission();
	TWire.beginTransmission(MPU6050_ADDRESS);
	TWire.write(0x6A);
	TWire.write(0x00);
	TWire.endTransmission();
	TWire.beginTransmission(MPU6050_ADDRESS);
	TWire.write(0x6B);
	TWire.write(0x00);
	TWire.endTransmission();

	//Initilize hmc5883l
	TWire.beginTransmission(HMC5883L_ADDRESS); //open communication with HMC5883
	TWire.write(0x02); //select mode register
	TWire.write(0x00); //continuous measurement mode
	TWire.endTransmission();
}
void IMU::InitMPU6050()
{
	//Initilize mpu6050
	TWire.beginTransmission(MPU6050_ADDRESS);//Start communication with the address found during search.
	TWire.write(0x6B);//We want to write to the PWR_MGMT_1 register (6B hex)
	TWire.write(0x00);//Set the register bits as 00000000 to activate the gyro
	TWire.endTransmission();//End the transmission with the gyro.

	TWire.beginTransmission(MPU6050_ADDRESS);//Start communication with the address found during search.
	TWire.write(0x1B);//We want to write to the GYRO_CONFIG register (1B hex)
	TWire.write(0x08);//Set the register bits as 00001000 (500dps full scale)
	TWire.endTransmission();//End the transmission with the gyro

	TWire.beginTransmission(MPU6050_ADDRESS);//Start communication with the address found during search.
	TWire.write(0x1C);//We want to write to the ACCEL_CONFIG register (1A hex)
	TWire.write(0x10);//Set the register bits as 00010000 (+/- 8g full scale range)
	TWire.endTransmission();
}

void IMU::read_HMC5883L()
{
	//Tell the HMC5883L where to begin reading data
	TWire.beginTransmission(HMC5883L_ADDRESS);
	TWire.write(0x03); //select register 3, X MSB register
	TWire.endTransmission();

	//Read data from each axis, 2 registers per axis
	TWire.requestFrom(HMC5883L_ADDRESS, 6);
	mag_x = TWire.read()<<8; //X msb
	mag_x |=TWire.read(); //X lsb
	mag_z = TWire.read()<<8; //Z msb
	mag_z |= TWire.read(); //Z lsb
	mag_y = TWire.read()<<8; //Y msb
	mag_y |= TWire.read(); //Y lsb
}

void IMU::readMPU6050()
{
	TWire.beginTransmission(MPU6050_ADDRESS);//Start communication with the gyro.
	TWire.write(0x3B);//Start reading @ register 43h and auto increment with every read.
	TWire.endTransmission();//End the transmission.
	TWire.requestFrom(MPU6050_ADDRESS,14);
	acc_raw[1] = TWire.read()<<8| TWire.read();//Add the low and high byte to the acc_x variable.
	acc_raw[0] = TWire.read()<<8| TWire.read();//Add the low and high byte to the acc_y variable.
	acc_raw[2] = TWire.read()<<8| TWire.read();//Add the low and high byte to the acc_z variable.
	mpu6050_temperature = TWire.read()<<8|TWire.read();//Add the low and high byte to the temperature variable.
	gyro_raw[0] = TWire.read()<<8| TWire.read();//Read high and low part of the angular data.
	gyro_raw[1] = TWire.read()<<8| TWire.read();//Read high and low part of the angular data.
	gyro_raw[2] = TWire.read()<<8| TWire.read();//Read high and low part of the angular data.
}

void IMU::Compute()
{
	dt = micros() - prev_dt;
	prev_dt = micros();
	//Gyro
	readMPU6050();
	for(int axis = 0; axis < 3; axis++){
		//apply gyro offsets.
		gyro_raw[axis] -= gyro_offset[axis];
		//lowpass filter
		gyro_data[axis] = (gyro_data[axis] * 0.85f) +((gyro_raw[axis]/GYRO_SCALE) * 0.15f);
	}
	if(auto_level){
		for(int i = 0; i < 2; i++)
		{
			gyro_angle[i] += gyro_raw[i] * gyro_time;
			acc_angle[i] = atan2(acc_raw[i] ,acc_raw[2]) * RAD_TO_DEG;
		}
		//TODO: test if switching +- will fix flight offsets for roll only
		gyro_angle[0] += gyro_angle[1] * sin(gyro_raw[2] * gyro_time_radians);
		gyro_angle[1]  -= gyro_angle[0] * sin(gyro_raw[2] * gyro_time_radians);

		//Trim & currections for accelerometer
		acc_angle[0] -= roll_angle_offset; 		//Roll
		acc_angle[1] -= pitch_angle_offset; 	//Pitch

		gyro_angle[0] = gyro_angle[0] * 0.9994 + acc_angle[0] * 0.0006;
		gyro_angle[1] = gyro_angle[1] * 0.9994 + invert(acc_angle[1]) * 0.0006;

		roll_level_ajust = gyro_angle[0] * al_mul;
		pitch_level_ajust = gyro_angle[1] * invert(al_mul);
	}
	else{
		roll_level_ajust = 0.0;
		pitch_level_ajust = 0.0;
	}

	Calculate_pressure(0x08);


#ifdef HMC5883L
	read_HMC5883L();
	mag_angle = atan2(mag_y,mag_x);
	if(mag_angle < 0) mag_angle += 2*PI;
	else if(mag_angle > 2*PI) mag_angle -= 2*PI;
	heading = mag_angle * 180/PI;
#endif

}

void IMU::Calibrate()
{
	gyro_offset[0] = 0;
	gyro_offset[1] = 0;
	gyro_offset[2] = 0;
	accz_offset = 0;
	for(int i = 0; i < CALIBRATION_REP; i++)
	{
		readMPU6050();
		for(int axis = 0; axis < 3; axis++)
		{
			gyro_offset[axis] += gyro_raw[axis];
		}
		accz_offset += acc_raw[2];
		delay(3);
		TIMER4_BASE->CCR1 = 1000;
		TIMER4_BASE->CCR2 = 1000;
		TIMER4_BASE->CCR3 = 1000;
		TIMER4_BASE->CCR4 = 1000;
		Calculate_pressure(0x08);
		if((i&15) == 0) digitalWrite(PC13,!digitalRead(PC13));

	}
	for(int axis = 0; axis < 3; axis++){
		gyro_offset[axis] /= CALIBRATION_REP;
	}
	ref_pressure = LPF_pressure;
	accz_offset /= CALIBRATION_REP;
	if(gyro_offset[0] != 0) calibrated = true;
	digitalWrite(PC13,1);
}

void IMU::reset()
{
	readMPU6050();
	gyro_angle[0]= acc_angle[0];
	gyro_angle[1] = acc_angle[1];
}


void IMU::Init_MS5611()
{
	for (uint8_t offset = 0; offset < 7; offset++){
		TWire.beginTransmission(MS5611_ADDRESS);
		TWire.write(0xA0 + offset*2);
		TWire.endTransmission();
		TWire.requestFrom(MS5611_ADDRESS,2);
		C[offset] = TWire.read() << 8 | TWire.read();
	}

    OFF_C2 = C[2] * pow(2,16);
    SENS_C1 = C[1] * pow(2,15);
    dT_C5 = C[5] * pow(2,8);
    temp_C6 = C[6] / pow(2,23);
}

void IMU::Calculate_pressure(uint8_t OSR)
{
	pressure_read_counter++;
	switch(pressure_read_counter)
	{
		case 1:
		if(temperature_read_counter == 0)
		{
			TWire.beginTransmission(MS5611_ADDRESS);
			TWire.write(0x00); 							//Read ADC
			TWire.endTransmission();
			TWire.requestFrom(MS5611_ADDRESS,3);

			D2 = TWire.read() << 16 | TWire.read() << 8 | TWire.read();

			temperature_avarage_total -= avarage_temperature_sample[avarage_temperature_index];
			avarage_temperature_sample[avarage_temperature_index] = D2;
			temperature_avarage_total += avarage_temperature_sample[avarage_temperature_index];
			avarage_temperature_index++;
			if(avarage_temperature_index == 5) avarage_temperature_index = 0;
			avarage_temperature = temperature_avarage_total / 5;
		}
		else
		{
			TWire.beginTransmission(MS5611_ADDRESS);
			TWire.write(0x00); 							//Read ADC
			TWire.endTransmission();
			TWire.requestFrom(MS5611_ADDRESS,3);

			D1 = TWire.read() << 16 | TWire.read() << 8 | TWire.read();
		}
		temperature_read_counter++;
		if(temperature_read_counter == 20)
		{
			TWire.beginTransmission(MS5611_ADDRESS);
			TWire.write(0x50 + OSR); 					//Convert D2 (Digital temperature value)
			TWire.endTransmission();
			temperature_read_counter = 0;
		}
		else
		{
			TWire.beginTransmission(MS5611_ADDRESS);
			TWire.write(0x40 + OSR); 					//Convert D2 (Digital temperature value)
			TWire.endTransmission();
		}

		break;

		case 2:
		dT = avarage_temperature - dT_C5;
		OFF = OFF_C2 + ((int64_t)C[4] * (int64_t)dT) / pow(2,7);
		SENS = SENS_C1 + ((int64_t)C[3] * (int64_t)dT) / pow(2,8);

		actual_pressure = ((D1 * SENS) / pow(2,21) - OFF) / pow(2,15);

		pressure_avarage_total -= avarage_pressure_sample[avarage_pressure_index];
		avarage_pressure_sample[avarage_pressure_index] = actual_pressure;
		pressure_avarage_total += avarage_pressure_sample[avarage_pressure_index];
		avarage_pressure_index++;
		if(avarage_pressure_index == 20) avarage_pressure_index = 0;
		avarage_pressure = pressure_avarage_total / 20;
		if(millis() < 5000) avarage_pressure = LPF_pressure;
		LPF_pressure = LPF_pressure * 0.985f + avarage_pressure * 0.015f;
		pressure_diff = LPF_pressure - avarage_pressure;
		pressure_diff = constrain(pressure_diff,-8,8);
		if(pressure_diff > 1 || pressure_diff < -1) LPF_pressure -= pressure_diff /6.0f;

		actual_temperature = 2000 + dT * temp_C6;

		if(calibrated) reletive_altitude = 44330.0f * (1.0f - pow(LPF_pressure / ref_pressure, 1.0f/5.255f));
		break;

		case 4:
		pressure_read_counter = 0;
		break;
	}
}

float IMU::Pressure_from_altitude(float ref_pr,float alt) 
{ return pow(pow(ref_pr,1.0f/5.255f) - (alt*pow(ref_pr,1.0f/5.255f)) / 44330.0f,5.255f); }