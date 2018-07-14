#include "IMU.h"

IMU::IMU(){};
SoftWire SWire(PB10, PB11, SOFT_FAST);

float IMU::Pitch_Level_Error() { return pitch_level_ajust;}
float IMU::Roll_Level_Error() { return roll_level_ajust;}
float IMU::Get_Altitude() {return est_alt;}
float IMU::Get_Heading() {return heading;}
float IMU::Get_GyroX_Angle() {return gyro_angle[0];}
float IMU::Get_GyroY_Angle() {return gyro_angle[1];}
float IMU::Get_Velocity(){return z_velocity;}
int16_t IMU::Get_AccX(){return acc_raw[0];}
int16_t IMU::Get_AccY(){return acc_raw[1];}
int16_t IMU::Get_AccZ(){return acc_raw[2];}
int16_t IMU::Get_LP_AccZ(){return acc_z;}
int16_t IMU::Get_GyroX() {return gyro_data[0];}
int16_t IMU::Get_GyroY() {return gyro_data[1];}
int16_t IMU::Get_GyroZ() {return gyro_data[2];}

void IMU::Init()
{
	SWire.setClock(400000L);
	SWire.begin();
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
	SWire.beginTransmission(MPU6050_ADDRESS);
	SWire.write(0x37);
	SWire.write(0x02);
	SWire.endTransmission();
	SWire.beginTransmission(MPU6050_ADDRESS);
	SWire.write(0x6A);
	SWire.write(0x00);
	SWire.endTransmission();
	SWire.beginTransmission(MPU6050_ADDRESS);
	SWire.write(0x6B);
	SWire.write(0x00);
	SWire.endTransmission();

	//Initilize hmc5883l
	SWire.beginTransmission(HMC5883L_ADDRESS); //open communication with HMC5883
	SWire.write(0x02); //select mode register
	SWire.write(0x00); //continuous measurement mode
	SWire.endTransmission();
}
void IMU::InitMPU6050()
{
	//Initilize mpu6050
	SWire.beginTransmission(MPU6050_ADDRESS);//Start communication with the address found during search.
	SWire.write(0x6B);//We want to write to the PWR_MGMT_1 register (6B hex)
	SWire.write(0x00);//Set the register bits as 00000000 to activate the gyro
	SWire.endTransmission();//End the transmission with the gyro.

	SWire.beginTransmission(MPU6050_ADDRESS);//Start communication with the address found during search.
	SWire.write(0x1B);//We want to write to the GYRO_CONFIG register (1B hex)
	SWire.write(0x08);//Set the register bits as 00001000 (500dps full scale)
	SWire.endTransmission();//End the transmission with the gyro

	SWire.beginTransmission(MPU6050_ADDRESS);//Start communication with the address found during search.
	SWire.write(0x1C);//We want to write to the ACCEL_CONFIG register (1A hex)
	SWire.write(0x10);//Set the register bits as 00010000 (+/- 8g full scale range)
	SWire.endTransmission();
}

void IMU::read_HMC5883L()
{
	//Tell the HMC5883L where to begin reading data
	SWire.beginTransmission(HMC5883L_ADDRESS);
	SWire.write(0x03); //select register 3, X MSB register
	SWire.endTransmission();

	//Read data from each axis, 2 registers per axis
	SWire.requestFrom(HMC5883L_ADDRESS, 6);
	// if(SWire.available() >= 6){
		mag_x = SWire.read()<<8; //X msb
		mag_x |=SWire.read(); //X lsb
		mag_z = SWire.read()<<8; //Z msb
		mag_z |= SWire.read(); //Z lsb
		mag_y = SWire.read()<<8; //Y msb
		mag_y |= SWire.read(); //Y lsb
	// }
}

void IMU::readMPU6050()
{
	SWire.beginTransmission(MPU6050_ADDRESS);//Start communication with the gyro.
	SWire.write(0x3B);//Start reading @ register 43h and auto increment with every read.
	SWire.endTransmission();//End the transmission.
	SWire.requestFrom(MPU6050_ADDRESS,14);
	// while(SWire.available() < 14); //if fail to read this will loop forever..fix it
	acc_raw[1] = SWire.read()<<8| SWire.read();//Add the low and high byte to the acc_x variable.
	acc_raw[0] = SWire.read()<<8| SWire.read();//Add the low and high byte to the acc_y variable.
	acc_raw[2] = SWire.read()<<8| SWire.read();//Add the low and high byte to the acc_z variable.
	mpu6050_temperature = SWire.read()<<8|SWire.read();//Add the low and high byte to the temperature variable.
	gyro_raw[0] = SWire.read()<<8| SWire.read();//Read high and low part of the angular data.
	gyro_raw[1] = SWire.read()<<8| SWire.read();//Read high and low part of the angular data.
	gyro_raw[2] = SWire.read()<<8| SWire.read();//Read high and low part of the angular data.
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
		gyro_data[axis] = (gyro_data[axis] * 0.7) +((gyro_raw[axis]/GYRO_SCALE) * 0.3);
	}
	if(auto_level){
		for(int i = 0; i < 2; i++)
		{
			gyro_angle[i] += gyro_raw[i] * gyro_time;
			acc_angle[i] = atan2(acc_raw[i] ,acc_raw[2]) * RAD_TO_DEG;
		}
		//TODO: test if switching +- will fix flight offsets for roll only
		gyro_angle[0] += gyro_angle[1] *sin(gyro_raw[2] * gyro_time_radians);
		gyro_angle[1]  -= gyro_angle[0] *sin(gyro_raw[2] * gyro_time_radians);

		//Trim & currections for accelerometer
		acc_angle[0] -= 0.43;
		acc_angle[1] += 1.5;

		gyro_angle[0] = gyro_angle[0] * 0.9994 + acc_angle[0] * 0.0006;
		gyro_angle[1] = gyro_angle[1] * 0.9994 + invert(acc_angle[1]) * 0.0006;

		roll_level_ajust = gyro_angle[0] * al_mul;
		pitch_level_ajust = gyro_angle[1] * invert(al_mul);
	}
	else{
		roll_level_ajust = 0.0;
		pitch_level_ajust = 0.0;
	}

	//acc_raw[2] -= accz_offset;
	acc_z = (acc_z * 0.99) + (acc_raw[2] * 0.01) ; 			//Low pass filter to reduce noise
	float acc_scalar = EARTH_G / (ACC_REST * sq(dt / 1000000.0)); 	//scale=9.8/3600 *sq(dt)
	z_velocity = (acc_z * acc_scalar) * sq(dt / 1000000.0);			//Velocity calculations
	// Serial.println(z_velocity);

	/*TODO:
		find a math model to remove z acc at an angle.
		try to implement it as basic pid with baro as drift fix

	*/


#ifdef HMC5883L
	read_HMC5883L();
	mag_angle = atan2(mag_y,mag_x);
	if(mag_angle < 0) mag_angle += 2*PI;
	else if(mag_angle > 2*PI) mag_angle -= 2*PI;
	heading = mag_angle * 180/PI;
#endif

#ifdef MS5611
	est_alt = Calculate_Estimated_Altitude(false);
	baro_vel = ((baro_vel + est_alt)  * 0.85f) + (z_velocity * 0.15f);
	// Serial.println(est_alt);
#endif
}

void IMU::Calibrate_MPU6050()
{
	gyro_offset[0] = 0;
	gyro_offset[1] = 0;
	gyro_offset[2] = 0;
	accz_offset = 0;
	for(int i = 0; i < CALIBRATION_REPEATITIONS; i++){
		readMPU6050();
		for(int axis = 0; axis < 3; axis++){
			gyro_offset[axis] += gyro_raw[axis];
		}
		accz_offset += acc_raw[2];
		delay(3);
		TIMER4_BASE->CCR1 = 1000;
		TIMER4_BASE->CCR2 = 1000;
		TIMER4_BASE->CCR3 = 1000;
		TIMER4_BASE->CCR4 = 1000;
		if((i&15) == 0) digitalWrite(PC13,!digitalRead(PC13));

	}
	for(int axis = 0; axis < 3; axis++){
		gyro_offset[axis] /= CALIBRATION_REPEATITIONS;
	}
	accz_offset /= CALIBRATION_REPEATITIONS;
	if(gyro_offset[0] != 0) calibrated = true;
	digitalWrite(PC13,1);
}

void IMU::reset()
{
	reset_MS5611();
	readMPU6050();
	gyro_angle[0]= acc_angle[0];
	gyro_angle[1] = acc_angle[1];
}

void IMU::reset_MS5611()
{
	SWire.beginTransmission(MS5611_ADDRESS);
	SWire.write(MS5611_CMD_RESET);
	SWire.endTransmission();
}

void IMU::Init_MS5611()
{
	reset_MS5611();
	delay(100);
	for (uint8_t offset = 0; offset < 6; offset++){
		fc[offset] = readRegister16(MS5611_CMD_READ_PROM + (offset * 2));
	}
	referencePressure = ms5611_readPressure(true);
	// Serial.println(referencePressure);
}

int32_t IMU::ms5611_readPressure(bool compensation)
{
	SWire.beginTransmission(MS5611_ADDRESS);
	SWire.write(MS5611_CMD_CONV_D1 + OSR);
	SWire.endTransmission();
	delay(ct);
	uint32_t d1 = readRegister24(MS5611_CMD_ADC_READ);

	SWire.beginTransmission(MS5611_ADDRESS);
	SWire.write(MS5611_CMD_CONV_D2 + OSR);
	SWire.endTransmission();
	delay(ct);
	uint32_t d2 = readRegister24(MS5611_CMD_ADC_READ);

	int32_t dT = d2 - (uint32_t)fc[4] * 256;
	int64_t OFF = (int64_t)fc[1] * 65536 + (int64_t)fc[3] * dT / 128;
	int64_t SENS = (int64_t)fc[0] * 32768 + (int64_t)fc[2] * dT / 256;

	if (compensation){
		int32_t TEMP = 2000 + ((int64_t) dT * fc[5]) / 8388608;

		OFF2 = 0;
		SENS2 = 0;

		if (TEMP < 2000)
		{
			OFF2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 2;
		    	SENS2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 4;
		}

		if (TEMP < -1500)
		{
		    	OFF2 = OFF2 + 7 * ((TEMP + 1500) * (TEMP + 1500));
		    	SENS2 = SENS2 + 11 * ((TEMP + 1500) * (TEMP + 1500)) / 2;
		}

		OFF = OFF - OFF2;
		SENS = SENS - SENS2;
	}

	uint32_t P = (d1 * SENS / 2097152 - OFF) / 32768;

	return P;
}

float IMU::Calculate_Estimated_Altitude(bool compensation)
{
	//this swtich case is made for letting the ms5611 use the loop 4ms  delay for reading pressure and temp   instead of using delay(4)
	switch(skip_case){
		case 0:
		SWire.beginTransmission(MS5611_ADDRESS);
		SWire.write(MS5611_CMD_CONV_D1 + OSR);
		SWire.endTransmission();
		skip_case++;
		break;

		case 1:
		D1 = readRegister24(MS5611_CMD_ADC_READ);
		SWire.beginTransmission(MS5611_ADDRESS);
		SWire.write(MS5611_CMD_CONV_D2 + OSR);
		SWire.endTransmission();
		skip_case++;
		break;

		case 2:
		D2 = readRegister24(MS5611_CMD_ADC_READ);
		skip_case = 0;
		break;
	}

	int32_t dT = D2 - (uint32_t)fc[4] * 256;
	int64_t OFF = (int64_t)fc[1] * 65536 + (int64_t)fc[3] * dT / 128;
	int64_t SENS = (int64_t)fc[0] * 32768 + (int64_t)fc[2] * dT / 256;
	if (compensation){
		int32_t TEMP = 2000 + ((int64_t) dT * fc[5]) / 8388608;

		OFF2 = 0;
		SENS2 = 0;

		if (TEMP < 2000)
		{
			OFF2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 2;
		    	SENS2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 4;
		}

		if (TEMP < -1500)
		{
		    	OFF2 = OFF2 + 7 * ((TEMP + 1500) * (TEMP + 1500));
		    	SENS2 = SENS2 + 11 * ((TEMP + 1500) * (TEMP + 1500)) / 2;
		}
		OFF = OFF - OFF2;
		SENS = SENS - SENS2;
	}
	uint32_t realPressure = (D1 * SENS / 2097152 - OFF) / 32768;
	float relativeAltitude = 44330.0f * (1.0f - pow((float)realPressure / referencePressure, 0.1902949f));
	altitude = (altitude * 0.95) + (relativeAltitude * 0.05);
	return altitude;
}

void IMU::SetOverSampling(uint8_t osr)
{
	//Todo
}

uint16_t IMU::readRegister16(uint8_t reg)
{
	uint16_t value;
	SWire.beginTransmission(MS5611_ADDRESS);
	SWire.write(reg);
	SWire.endTransmission();
	SWire.beginTransmission(MS5611_ADDRESS);
	SWire.requestFrom(MS5611_ADDRESS, 2);
	// while(!SWire.available()) {};
	uint8_t vha =SWire.read();
	uint8_t vla = SWire.read();
	SWire.endTransmission();

	value = vha << 8 | vla;
	return value;
}

// Read 24-bit from register (oops XSB, MSB, LSB)
uint32_t IMU::readRegister24(uint8_t reg)
{
	uint32_t value;
	SWire.beginTransmission(MS5611_ADDRESS);
	SWire.write(reg);
	SWire.endTransmission();

  SWire.beginTransmission(MS5611_ADDRESS);
  SWire.requestFrom(MS5611_ADDRESS, 3);
  // while(!SWire.available()) {};
	uint8_t vxa = SWire.read();
	uint8_t vha = SWire.read();
	uint8_t vla = SWire.read();
  SWire.endTransmission();

	value = ((int32_t)vxa << 16) | ((int32_t)vha << 8) | vla;
	return value;
}

void IMU::setAL_Mul(float mul)
{
	al_mul = mul;
}

float IMU::Get_offset(uint8_t xy)
{
	return gyro_offset[xy];
}
