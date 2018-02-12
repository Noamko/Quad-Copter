#include "IMU.h"
#include "PID.h"
#include "config.h"
#include "Telemetry.h"
// #include <libmaple/scb.h>

//IMU variables
IMU imu;
//RC variables
int32_t channel[4];
int32_t channel_start[4];
uint8_t aux1,prev_aux1;
float controller_sensativity = 3.0f;

//PID variables
#define  K_P 1.85
#define  K_I 0.04
#define  K_D 6.5

PID pid_roll, pid_pitch, pid_yaw;
PID pid_mag;
PID pid_alt;

//Telemetry
Telemetry telemetry;
uint8_t transmit_mode = 1;
uint8_t transmit_counter = 0;
///////////////

//Global variables
int16_t battery_voltage = 0;
int16_t setPoint_roll,setPoint_pitch,setPoint_yaw;
uint16_t setPoint_altitude;
uint16_t setPoint_mag;
uint16_t esc_1, esc_2, esc_3, esc_4;
uint16_t throttle;
uint16_t deltaTime;
uint16_t alt_hold_val = 0;
uint16_t throttle_bias = 400;

uint32  loop_timer;
uint32  timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
uint32  prev_deltaTime;

float mag_hold_val = 0;
float voltage_compensation = 0;

bool mag_hold = false;
bool alt_hold = false;
bool engineStart = false;
bool battery_connected = false;

void setup() {
	Serial.begin(115200);
	telemetry.Init(9600);
	delay(250);

	pinMode(IND_LED,OUTPUT);

	pinMode(PB6, PWM);
	pinMode(PB7, PWM);
	pinMode(PB8, PWM);
	pinMode(PB9, PWM);

	//Initilize Timers 3 & 4
	TIMER4_BASE->CR1 = TIMER_CR1_CEN | TIMER_CR1_ARPE;  //Enable Timer 4
	TIMER4_BASE->CR2 = 0;
	TIMER4_BASE->SMCR = 0;
	TIMER4_BASE->DIER = 0;
	TIMER4_BASE->EGR = 0;
	TIMER4_BASE->CCMR1 = (0b110 << 12) | (0b110 << 4) | TIMER_CCMR1_OC1PE;  // Set channel 1 & 2 on PWM1 mode
	TIMER4_BASE->CCMR2 = (0b110 << 12) | (0b110 << 4) | TIMER_CCMR1_OC1PE;    // Set channel 3 & 4 on PWM1 Mode
	TIMER4_BASE->CCER = TIMER_CCER_CC1E | TIMER_CCER_CC2E | TIMER_CCER_CC3E | TIMER_CCER_CC4E; //Enable Channels 1,2,3,4
	TIMER4_BASE->PSC = 71;  //Set prescaler to 71 (72hz)
	TIMER4_BASE->ARR = 5000; //set Counter to auto Reload at value of 5000
	TIMER4_BASE->DCR = 0;

	//Pwm values
	TIMER4_BASE->CCR1 = 1000;
	TIMER4_BASE->CCR2 = 1000;
	TIMER4_BASE->CCR3 = 1000;
	TIMER4_BASE->CCR4 = 1000;

	Timer3.attachCompare1Interrupt(handler_channel_1);
	Timer3.attachCompare2Interrupt(handler_channel_2);
	Timer3.attachCompare3Interrupt(handler_channel_3);
	Timer3.attachCompare4Interrupt(handler_channel_4);
	TIMER3_BASE->CR1 = TIMER_CR1_CEN;
	TIMER3_BASE->CR2 = 0;
	TIMER3_BASE->SMCR = 0;
	TIMER3_BASE->DIER = TIMER_DIER_CC1IE | TIMER_DIER_CC2IE | TIMER_DIER_CC3IE | TIMER_DIER_CC4IE;
	TIMER3_BASE->EGR = 0;
	TIMER3_BASE->CCMR1 = 0b100000001; //Register is set like this due to a bug in the define table (30-09-2017)
	TIMER3_BASE->CCMR2 = 0b100000001; //Register is set like this due to a bug in the define table (30-09-2017)
	TIMER3_BASE->CCER = TIMER_CCER_CC1E | TIMER_CCER_CC2E | TIMER_CCER_CC3E | TIMER_CCER_CC4E;
	TIMER3_BASE->PSC = 71;
	TIMER3_BASE->ARR = 0xFFFF;
	TIMER3_BASE->DCR = 0;

	imu.Init();
	delay(250);
	pid_roll.Set_gains(K_P, K_I, K_D);
	pid_pitch.Set_gains(K_P, K_I, K_D);
	pid_yaw.Set_gains(2.5, 0.07, 0.03);
	pid_mag.Set_gains(1.2,0.01,3.0);
	pid_alt.Set_gains(0,0,0);

	Send_PidGains();
	GPIOC_BASE->BSRR = (0b1 << 13);  //turn off Pin PC13
}

void loop()
{
	//Calculate delta time;
	deltaTime = (micros() - prev_deltaTime);
	prev_deltaTime = micros();

	//Imu calculations.
	imu.Compute();

	//read battery battery_voltage
	battery_voltage = analogRead(0) * VD_SCALE;

	if(StartEngines())
	{
		RC_toValue(); //convert channles values to Setpoints.

		if(mag_hold) pid_mag.Compute(mag_hold_val - imu.Get_Heading());

		pid_roll.Compute(setPoint_roll - imu.Get_GyroX());
		pid_pitch.Compute(invert(setPoint_pitch) - imu.Get_GyroY());
		pid_yaw.Compute(invert(setPoint_yaw)  - imu.Get_GyroZ());

		if(alt_hold){
			//TODO:
			//make sure at current altitude the throttle is at throttle hover
			// make sure Gyro z is not affected by vibrations if yes filter it
			pid_alt.Compute_AH(setPoint_altitude - imu.Get_Altitude(),imu.Get_Velocity());
			throttle = THROTTLE_MIN_LIMIT + throttle_bias + pid_alt.output;
		}
		else throttle = channel[THROTTLE];

		throttle = constrain(throttle,THROTTLE_MIN_LIMIT,THROTTLE_MAX_LIMIT);

		//Calculate the value for each motor
		esc_1 = throttle + pid_roll.output - pid_pitch.output - pid_yaw.output;
		esc_2 = throttle - pid_roll.output - pid_pitch.output + pid_yaw.output;
		esc_3 = throttle - pid_roll.output + pid_pitch.output - pid_yaw.output;
		esc_4 = throttle + pid_roll.output + pid_pitch.output + pid_yaw.output;


		//Calcualte the battery compensation
		if(battery_voltage < 1280 && battery_voltage > 1050) {
			battery_connected = true;
			voltage_compensation = (1240 - battery_voltage) / (float)3500;
		}
		else battery_connected = false;

		//add the battery compensation to each motor.
		esc_1 += esc_1 * voltage_compensation;
		esc_2 += esc_2 * voltage_compensation;
		esc_3 += esc_3 * voltage_compensation;
		esc_4 += esc_4 * voltage_compensation;


		esc_1 = constrain(esc_1, ESC_MIN_LIMIT, ESC_MAX_LIMIT);
		esc_2 = constrain(esc_2, ESC_MIN_LIMIT, ESC_MAX_LIMIT);
		esc_3 = constrain(esc_3, ESC_MIN_LIMIT, ESC_MAX_LIMIT);
		esc_4 = constrain(esc_4, ESC_MIN_LIMIT, ESC_MAX_LIMIT);

	}
	else { //Turn engines off
		esc_1 = ESC_OFF;
		esc_2 = ESC_OFF;
		esc_3 = ESC_OFF;
		esc_4 = ESC_OFF;
	}
	Write_4Engines();
	if(Serial1.available())
	{
		//Reads data from incoming telemetry buffer.
		int8_t telemetry_info = telemetry.GetData(Serial1.available());

		//info state machine
		switch (telemetry_info)
		{
			case -1:
			break;

			case 2:
			imu.setAL_Mul(telemetry.autolevel_stregth);
			telemetry.Transmit(String("auto level: ") + telemetry.autolevel_stregth);
			break;

			case 3:
			controller_sensativity = telemetry.tlm_controller_sensativity;
			telemetry.Transmit(String("controller sensativity: ") + controller_sensativity);
			break;

			case 4:
			transmit_mode = telemetry.tlm_transmit_flight_data;
			break;

			case 5:
			if(!engineStart)
			{
				telemetry.Transmit("Calibrating gyro...\n");
				imu.Calibrate_MPU6050();
				telemetry.Transmit(String("gyro_offset_x:") + imu.Get_offset(0));
				telemetry.Transmit(String("gyro_offset_y:") + imu.Get_offset(1));
				telemetry.Transmit("Calibration complete.\n");
			}
			break;

			case 6:
			Send_PidGains();
			break;

			case 7:
			Telemetry_Update_PID(telemetry.pid_setting,telemetry._p,telemetry._i,telemetry._d);
			break;

			case 8:
			Send_PidGains();
			break;

			case 9:
			throttle_bias = telemetry.tlm_throttle_bias;
			telemetry.Transmit(String("throttle bias: ") + throttle_bias + "\n");
			break;
		}
	}

	if(transmit_counter >= 100) //Transmit data to ground at 50Hz
	{
		Transmit_Flight_Data(transmit_mode);
		transmit_counter = 0;
	}
	else transmit_counter++;

	//Serial Debuging
	// Serial.println(imu.Get_GyroX_Angle());
	// Serial.println(imu.Get_Altitude());
	// Serial.println(battery_voltage);
	// Serial.println(imu.Get_Velocity());
	// Serial.println(pid_alt.output);
	// Serial.println(throttle);
	// Serial.println(imu.Get_Heading());
	// Serial.println(pid_alt.output);
	// Serial.print(channel[ROLL]);
	// Serial.print(",");
	// Serial.print(channel[PITCH]);
	// Serial.print(", ");
	// Serial.print(channel[THROTTLE]);
	// Serial.print(",");
	// Serial.println(channel[YAW]);
	// Serial.print(setPoint_roll);
	// Serial.print(",");
	// Serial.print(setPoint_pitch);
	// Serial.print(",");
	// Serial.println(setPoint_yaw);
	// Serial.print(esc_1);
	// Serial.print(",");
	// Serial.print(esc_2);
	// Serial.print(",");
	// Serial.print(esc_3);
	// Serial.print(",");
	// Serial.println(esc_4);
}

void RC_toValue()
{
	//Roll
	setPoint_roll = 0;
	if(channel[ROLL] > CH1_CENTERED + CONTROLLER_DEADBAND){
		setPoint_roll = channel[ROLL] - CH1_CENTERED + CONTROLLER_DEADBAND;
	}
	else if(channel[ROLL] < CH1_CENTERED - CONTROLLER_DEADBAND){
		setPoint_roll = channel[ROLL] - CH1_CENTERED - CONTROLLER_DEADBAND;
	}
	setPoint_roll /= controller_sensativity;
	setPoint_roll -= imu.Roll_Level_Error();

	//Pitch
	setPoint_pitch = 0;
	if(channel[PITCH] > CH2_CENTERED + CONTROLLER_DEADBAND){
		setPoint_pitch = channel[PITCH] - CH2_CENTERED + CONTROLLER_DEADBAND;
	}
	else if(channel[PITCH] < CH2_CENTERED - CONTROLLER_DEADBAND){
		setPoint_pitch = channel[PITCH] - CH1_CENTERED - CONTROLLER_DEADBAND;
	}
	setPoint_pitch /= controller_sensativity;
	setPoint_pitch -= imu.Pitch_Level_Error();

	//Yaw
	setPoint_yaw = 0;

	//TODO: add mag correction to yaw
	if(mag_hold) setPoint_yaw = constrain(pid_mag.output,-50,50);
	else{
		if(channel[YAW] > CH4_CENTERED + CONTROLLER_DEADBAND && channel[YAW] < AUX2_VALUE){
			setPoint_yaw = channel[YAW] - CH4_CENTERED + CONTROLLER_DEADBAND;
		}
		else if(channel[YAW] < CH4_CENTERED - CONTROLLER_DEADBAND && channel[YAW] < AUX2_VALUE){
			setPoint_yaw = channel[YAW] - CH1_CENTERED - CONTROLLER_DEADBAND;
		}
		setPoint_yaw /= CONTROLLER_SENSATIVITY;
	}

	if(channel[YAW] > AUX1_VALUE && !aux1) {
		aux1 = 1;
		if(!prev_aux1){ //single Click command
			// mag_hold = !mag_hold;
			// if(mag_hold) mag_hold_val = imu.Get_Heading();

			alt_hold = !alt_hold;
			if(alt_hold){
				setPoint_altitude = imu.Get_Altitude();
				Serial1.print("$ah,1,#");
			}
			else
			{
				Serial1.print("$ah,0,#");
			}
		}
		prev_aux1 = aux1;
	}
	else if(channel[YAW] < AUX1_VALUE && aux1) prev_aux1 = 0, aux1 = 0;

	// setPoint_altitude = map(channel[THROTTLE],1000,2000,0,10);
}

void Write_4Engines()
{
	while(micros() - loop_timer < 4000);
	loop_timer = micros();

	TIMER4_BASE->CCR1 = esc_1;
	TIMER4_BASE->CCR2 = esc_2;
	TIMER4_BASE->CCR3 = esc_3;
	TIMER4_BASE->CCR4 = esc_4;
}

bool StartEngines()
{
	if
	(channel[THROTTLE] <= MIN_THROTTLE_VALUE +5
	&& channel[THROTTLE] > 900
	&& channel[YAW] >= channel_4_MAX_VALUE - 10
	&& channel[YAW] < AUX2_VALUE)
	{
		//turn off engines and reset pid and imu angles
		engineStart = false;
		pid_roll.Reset();
		pid_pitch.Reset();
		pid_yaw.Reset();
		pid_mag.Reset();
		pid_alt.Reset();
		imu.reset();
	}

	//Start engines.
	else if
	(channel[THROTTLE] <= MIN_THROTTLE_VALUE +5
	&& channel[THROTTLE] > 900
	&& channel[YAW] <= channel_4_MIN_VALUE +5
	&& channel[YAW] < AUX2_VALUE)
	 {
		engineStart = true;
	}

	//Calibrate.
	else if
	(channel[THROTTLE] >= MAX_THROTTLE_VALUE - 5
	&& channel[YAW] >= channel_4_MAX_VALUE - 5
	&& channel[YAW] < AUX2_VALUE
	&& !engineStart)
	 {
		imu.Calibrate_MPU6050();
	}
	return engineStart;
}

void handler_channel_1(void) {
  if (0b1 & GPIOA_BASE->IDR  >> 6) {
    channel_start[ROLL] = TIMER3_BASE->CCR1;
    TIMER3_BASE->CCER |= TIMER_CCER_CC1P;
  }
  else {
    channel[ROLL] = TIMER3_BASE->CCR1 - channel_start[ROLL];
    if (channel[ROLL] < 0)channel[ROLL] += 0xFFFF;
    TIMER3_BASE->CCER &= ~TIMER_CCER_CC1P;
  }
}

void handler_channel_2(void) {
  if (0b1 & GPIOA_BASE->IDR >> 7) {
    channel_start[PITCH] = TIMER3_BASE->CCR2;
    TIMER3_BASE->CCER |= TIMER_CCER_CC2P;
  }
  else {
    channel[PITCH] = TIMER3_BASE->CCR2 - channel_start[PITCH];
    if (channel[PITCH] < 0)channel[PITCH] += 0xFFFF;
    TIMER3_BASE->CCER &= ~TIMER_CCER_CC2P;
  }
}

void handler_channel_3(void) {
  if (0b1 & GPIOB_BASE->IDR >> 0) {
    channel_start[THROTTLE] = TIMER3_BASE->CCR3;
    TIMER3_BASE->CCER |= TIMER_CCER_CC3P;
  }
  else {
    channel[THROTTLE] = TIMER3_BASE->CCR3 - channel_start[THROTTLE];
    if (channel[THROTTLE] < 0)channel[THROTTLE] += 0xFFFF;
    TIMER3_BASE->CCER &= ~TIMER_CCER_CC3P;
  }
}

void handler_channel_4(void) {
  if (0b1 & GPIOB_BASE->IDR >> 1) {
    channel_start[YAW] = TIMER3_BASE->CCR4;
    TIMER3_BASE->CCER |= TIMER_CCER_CC4P;
  }
  else {
    channel[YAW] = TIMER3_BASE->CCR4 - channel_start[YAW];
    if (channel[YAW] < 0)channel[YAW] += 0xFFFF;
    TIMER3_BASE->CCER &= ~TIMER_CCER_CC4P;
  }
}

String getValue(String data, char separator, int index){
    int found = 0;
    int strIndex[] = { 0, -1 };
    int maxIndex = data.length() - 1;

    for (int i = 0; i <= maxIndex && found <= index; i++) {
        if (data.charAt(i) == separator || i == maxIndex) {
            found++;
            strIndex[0] = strIndex[1] + 1;
            strIndex[1] = (i == maxIndex) ? i+1 : i;
        }
    }
    return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

void Send_PidGains()
{
	Serial1.print("$pr,");
	Serial1.print(pid_roll.Kp, SERIAL_DEC_SIZE);
	Serial1.print(",");
	Serial1.print(pid_roll.Ki, SERIAL_DEC_SIZE);
	Serial1.print(",");
	Serial1.print(pid_roll.Kd, SERIAL_DEC_SIZE);
	Serial1.print('\n');
	delay(500);
	Serial1.print("$pp,");
	Serial1.print(pid_pitch.Kp, SERIAL_DEC_SIZE);
	Serial1.print(",");
	Serial1.print(pid_pitch.Ki, SERIAL_DEC_SIZE);
	Serial1.print(",");
	Serial1.print(pid_pitch.Kd, SERIAL_DEC_SIZE);
	Serial1.print('\n');
	delay(500);

	Serial1.print("$py,");
	Serial1.print(pid_yaw.Kp, SERIAL_DEC_SIZE);
	Serial1.print(",");
	Serial1.print(pid_yaw.Ki, SERIAL_DEC_SIZE);
	Serial1.print(",");
	Serial1.print(pid_yaw.Kd, SERIAL_DEC_SIZE);
	Serial1.print('\n');

	delay(500);
	Serial1.print("$pa,");
	Serial1.print(pid_alt.Kp, SERIAL_DEC_SIZE);
	Serial1.print(",");
	Serial1.print(pid_alt.Ki, SERIAL_DEC_SIZE);
	Serial1.print(",");
	Serial1.print(pid_alt.Kd, SERIAL_DEC_SIZE);
	Serial1.print('\n');
	delay(500);
	Serial1.print("$pm,");
	Serial1.print(pid_mag.Kp, SERIAL_DEC_SIZE);
	Serial1.print(",");
	Serial1.print(pid_mag.Ki, SERIAL_DEC_SIZE);
	Serial1.print(",");
	Serial1.print(pid_mag.Kd, SERIAL_DEC_SIZE);
	Serial1.print('\n');
	delay(500);
}

void Telemetry_Update_PID(uint8_t pid_s,float _nP,float _nI,float _nD)
{
	switch(pid_s)
	{
		case 0:
		pid_roll.Set_gains(_nP, _nI, _nD);
		Serial1.print("$pr,");
		Serial1.print(pid_roll.Kp, SERIAL_DEC_SIZE);
		Serial1.print(",");
		Serial1.print(pid_roll.Ki, SERIAL_DEC_SIZE);
		Serial1.print(",");
		Serial1.print(pid_roll.Kd, SERIAL_DEC_SIZE);
		Serial1.print('\n');
		break;

		case 1:
		pid_pitch.Set_gains(_nP, _nI, _nD);
		Serial1.print("$pp,");
		Serial1.print(pid_pitch.Kp, SERIAL_DEC_SIZE);
		Serial1.print(",");
		Serial1.print(pid_pitch.Ki, SERIAL_DEC_SIZE);
		Serial1.print(",");
		Serial1.print(pid_pitch.Kd, SERIAL_DEC_SIZE);
		Serial1.print('\n');
		break;

		case 2:
		pid_yaw.Set_gains(_nP, _nI, _nD);
		Serial1.print("$py,");
		Serial1.print(pid_yaw.Kp, SERIAL_DEC_SIZE);
		Serial1.print(",");
		Serial1.print(pid_yaw.Ki, SERIAL_DEC_SIZE);
		Serial1.print(",");
		Serial1.print(pid_yaw.Kd, SERIAL_DEC_SIZE);
		Serial1.print('\n');
		break;

		case 3:
		pid_alt.Set_gains(_nP, _nI, _nD);
		Serial1.print("$pa,");
		Serial1.print(pid_alt.Kp, SERIAL_DEC_SIZE);
		Serial1.print(",");
		Serial1.print(pid_alt.Ki, SERIAL_DEC_SIZE);
		Serial1.print(",");
		Serial1.print(pid_alt.Kd, SERIAL_DEC_SIZE);
		Serial1.print('\n');
		break;

		case 4:
		pid_mag.Set_gains(_nP, _nI, _nD);
		Serial1.print("$pm,");
		Serial1.print(pid_mag.Kp,SERIAL_DEC_SIZE);
		Serial1.print(",");
		Serial1.print(pid_mag.Ki,SERIAL_DEC_SIZE);
		Serial1.print(",");
		Serial1.print(pid_mag.Kd,SERIAL_DEC_SIZE);
		Serial1.print('\n');
		break;
	}
}


void Transmit_Flight_Data(uint8_t mode)
{
	switch(mode)
	{
		case 0:
		//none
		break;

		case 1:
		//default flight data
		telemetry.Transmit(String("$fd,1")
		 + "," + battery_voltage
		 + "," + imu.Get_Altitude());
		break;

		case 2:
		//IMU data only + battery
		telemetry.Transmit(String("$fd,2")
		+ "," + battery_voltage
		+ "," + imu.Get_GyroX()
		+ "," + imu.Get_GyroY()
		+ "," + imu.Get_Heading());
		break;

		case 3:
		//Nav data
		//TODO: navigation data
		break;

		//Extras
		case 4:
		telemetry.Transmit(String("$fd,3")
		+ "," + battery_voltage
		+ "," + deltaTime);
		break;
	}
}
