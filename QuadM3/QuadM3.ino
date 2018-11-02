#include "IMU.h"
#include "PID.h"
#include "config.h"
#include "Telemetry.h"

//IMU variables
IMU imu;
//RC variables
int32_t channel[8];
uint8_t channel_select_counter;
int32_t measured_time, measured_time_start;
uint8_t aux1,prev_aux1;
float controller_sensativity = 3.0f;

//PID variables
PID pid_roll, pid_pitch, pid_yaw;
PID pid_mag;
PID pid_alt;
int16_t setPoint_roll,setPoint_pitch,setPoint_yaw;
float setPoint_altitude;
float pressure_setPoint;
float setPoint_mag;

//Telemetry variables
Telemetry telemetry;
uint16_t telemetry_transmit_counter;
int8_t telemetry_mode = -1;

//Global variables
int16_t battery_voltage;
uint16_t esc_1, esc_2, esc_3, esc_4;
uint16_t throttle, prev_throttle,throttle_base;
uint16_t deltaTime;
uint16_t throttle_bias = 400;

uint32  loop_timer;
uint32  esc_timer, esc_loop_timer;
uint32  prev_deltaTime;

float heading_setPoint;
float voltage_compensation;

bool mag_hold = false;
bool mag_correction = true;
bool alt_hold = false;
bool engineStart = false;
bool battery_connected = false;

void setup() {
	Serial.begin(9600);
	telemetry.Init(1200);
	delay(250);

	pinMode(IND_LED,OUTPUT);

	pinMode(PB6, PWM);
	pinMode(PB7, PWM);
	pinMode(PB8, PWM);
	pinMode(PB9, PWM);

	Init_Timers();
	delay(100);
	imu.Init();
	delay(250);

	//PID still need some tweak
	pid_roll.Set_gains(1.6, 0.03, 6.5);
	pid_pitch.Set_gains(1.6, 0.03, 6.5);
	pid_yaw.Set_gains(10, 0.05, 10);
	pid_mag.Set_gains(1.2,0.01,2.0);
	pid_alt.Set_gains(3.0,0.01,3.0);

	heading_setPoint = imu.Get_Heading();

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

		pid_roll.Compute(setPoint_roll - imu.Get_GyroX());
		pid_pitch.Compute(invert(setPoint_pitch) - imu.Get_GyroY());
		
		if(mag_hold)
		{
			pid_mag.Compute(heading_setPoint - imu.Get_Heading());
			setPoint_yaw = constrain(pid_mag.output,-50,50);
		}
		pid_yaw.Compute(invert(setPoint_yaw)  - imu.Get_GyroZ());

		if(alt_hold)
		{
			pid_alt.Compute(imu.Get_pressure() - pressure_setPoint);
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
		if(battery_connected)
		{
			esc_1 += esc_1 * voltage_compensation;
			esc_2 += esc_2 * voltage_compensation;
			esc_3 += esc_3 * voltage_compensation;
			esc_4 += esc_4 * voltage_compensation;
		}

		esc_1 = constrain(esc_1, ESC_MIN_LIMIT, ESC_MAX_LIMIT);
		esc_2 = constrain(esc_2, ESC_MIN_LIMIT, ESC_MAX_LIMIT);
		esc_3 = constrain(esc_3, ESC_MIN_LIMIT, ESC_MAX_LIMIT);
		esc_4 = constrain(esc_4, ESC_MIN_LIMIT, ESC_MAX_LIMIT);
	}

	else //Turn engines off 
	{ 
		esc_1 = ESC_OFF;
		esc_2 = ESC_OFF;
		esc_3 = ESC_OFF;
		esc_4 = ESC_OFF;
	}

	while(micros() - loop_timer < LOOP_TIME); //should sync loop speed with esc's, not sure if neccecery
	loop_timer = micros();

	Write_4Engines();

	Telemetry_TR(telemetry_mode);


	//Serial Debuging
	// Serial.println(imu.Get_pressure());
	// Serial.println(imu.Get_GyroX_Angle());
	// Serial.println(battery_voltage);
	// Serial.println(imu.Get_Velocity());
	// Serial.println(pid_alt.output);
	// Serial.println(throttle);
	// Serial.println(pid_yaw.output);
	// Serial.println(imu.Get_Heading());
	// Serial.print(",");
	// Serial.print(channel[0]);
	// Serial.print(",");
	// Serial.print(channel[1]);
	// Serial.print(",");
	// Serial.print(channel[2]);
	// Serial.print(",");
	// Serial.print(channel[3]);
	// Serial.print(",");
	// Serial.print(channel[4]);
	// Serial.print(",");
	// Serial.print(channel[5]);
	// Serial.print(",");
	// Serial.print(channel[6]);
	// Serial.print(",");
	// Serial.println(channel[7]);
	// Serial.print(setPoint_roll);
	// Serial.print(",");
	// Serial.print(setPoint_pitch);
	// Serial.print(",");
	// Serial.println(setPoint_yaw);
	// Serial.print(esc_1);
	// Serial.print(",");
	// Serial.print(esc_4);
	// Serial.print(",");
	// Serial.print(esc_2);
	// Serial.print(",");
	// Serial.println(esc_3);
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

	if(channel[YAW] > CH4_CENTERED + CONTROLLER_DEADBAND ){
		setPoint_yaw = channel[YAW] - CH4_CENTERED + CONTROLLER_DEADBAND;
		setPoint_yaw /= CONTROLLER_SENSATIVITY;
		mag_hold = false;
	}
	else if(channel[YAW] < CH4_CENTERED - CONTROLLER_DEADBAND){
		setPoint_yaw = channel[YAW] - CH1_CENTERED - CONTROLLER_DEADBAND;
		setPoint_yaw /= CONTROLLER_SENSATIVITY;
		mag_hold = false;
	}

	else if(setPoint_yaw == 0 && !mag_hold)
	{
		heading_setPoint = imu.Get_Heading();
		if(mag_correction) mag_hold = true;
	}
}

void Write_4Engines()
{
	TIMER4_BASE->CCR1 = esc_1;
	TIMER4_BASE->CCR2 = esc_2;
	TIMER4_BASE->CCR3 = esc_3;
	TIMER4_BASE->CCR4 = esc_4;
}

bool StartEngines()
{
	if
	(channel[THROTTLE] <= MIN_THROTTLE_VALUE + CONTROLLER_DEADBAND
	&& channel[THROTTLE] > 900
	&& channel[YAW] >= channel_4_MAX_VALUE - CONTROLLER_DEADBAND)
	{
		//turn off engines and reset pid and imu angles
		engineStart = false;
		pid_roll.Reset();
		pid_pitch.Reset();
		pid_yaw.Reset();
		pid_mag.Reset();
		pid_alt.Reset();
		imu.reset();
		alt_hold = false;
	}

	//Start engines.
	else if
	(channel[THROTTLE] <= MIN_THROTTLE_VALUE + CONTROLLER_DEADBAND
	&& channel[THROTTLE] > 900
	&& channel[YAW] <= channel_4_MIN_VALUE + CONTROLLER_DEADBAND)
	{
		engineStart = true;
	}

	//Calibrate.
	else if
	(channel[THROTTLE] >= MAX_THROTTLE_VALUE - CONTROLLER_DEADBAND
	&& channel[YAW] >= channel_4_MAX_VALUE - CONTROLLER_DEADBAND
	&& !engineStart)
	{
		imu.Calibrate();
	}
	return engineStart;
}

void channel_handler(void) {
	measured_time = TIMER3_BASE->CCR1 - measured_time_start;
	if (measured_time < 0)measured_time += 0xFFFF;
	measured_time_start = TIMER3_BASE->CCR1;
	if (measured_time > 4000)channel_select_counter = 0;
	else channel_select_counter++;

	if (channel_select_counter == 1) channel[0] = measured_time;
	if (channel_select_counter == 2) channel[1] = measured_time;
	if (channel_select_counter == 3) channel[2] = measured_time;
	if (channel_select_counter == 4) channel[3] = measured_time;
	if (channel_select_counter == 5) channel[4] = measured_time;
	if (channel_select_counter == 6) channel[5] = measured_time;
	if (channel_select_counter == 7) channel[6] = measured_time;
	if (channel_select_counter == 8) channel[7] = measured_time;
}

void Telemetry_TR(uint8_t mode)
{
	switch(mode)
	{
		case -1:
		return;

		case 0:
		telemetry_transmit_counter++;
		switch(telemetry_transmit_counter)
		{
			case 1:
			break;

			case 10: 
			telemetry.Transmit(String("BAT,") + battery_voltage + "\n");
			break;

			case 20:
			telemetry.Transmit(String("ENG,") + engineStart + "\n");
			break;

			case 30:
			telemetry.Transmit(String("RLL,") + imu.Get_GyroX_Angle() + "\n");
			break;

			case 40:
			telemetry.Transmit(String("PTC,") + imu.Get_GyroY_Angle() + "\n");
			telemetry.flush();
			break;

			case 50:
			telemetry.Transmit(String("ALT,") + imu.Get_altitude() + "\n");
			telemetry.flush();
			break;

			case 60:
			telemetry.Transmit(String("HED,") + imu.Get_Heading() + "\n");
			telemetry_transmit_counter = 0;
			telemetry.flush();
			break;

			case 70:
			break;
		}
		break;

		case 1:
		if(telemetry.Receive())
		{
			String data_in = String(telemetry.in_buffer);
			if(data_in.startsWith("PID"))
			{
				float _p = atof(getValue(data_in,',',1));
				float _i = atof(getValue(data_in,',',2));
				float _d = atof(getValue(data_in,',',3));
				if(data_in.startsWith("PID0")) pid_roll.Set_gains(_p,_i,_d);
				else if(data_in.startsWith("PID1")) pid_pitch.Set_gains(_p,_i,_d);
				else if(data_in.startsWith("PID2")) pid_yaw.Set_gains(_p,_i,_d);
				else if(data_in.startsWith("PID3")) pid_mag.Set_gains(_p,_i,_d);
				else if(data_in.startsWith("PID4"))	pid_alt.Set_gains(_p,_i,_d);
			}

			else if(data_in.startsWith("HBS")) throttle_bias = atoi(getValue(data_in,',',1));

			else if(data_in.startsWith("TRR")) imu.roll_angle_offset = atof(getValue(data_in,',',1));

			else if(data_in.startsWith("TRP")) imu.pitch_angle_offset = atof(getValue(data_in,',',1));
			telemetry.Transmit(String("OK\n"));
		}
		break;

		case 2: //Ping pong..
		if(telemetry.Receive())
		{
			String data_in = String(telemetry.in_buffer);
			if(data_in.startsWith("PID"))
			{
				float _p = atof(getValue(data_in,',',1));
				float _i = atof(getValue(data_in,',',2));
				float _d = atof(getValue(data_in,',',3));
				if(data_in.startsWith("PID0")) pid_roll.Set_gains(_p,_i,_d);
				else if(data_in.startsWith("PID1")) pid_pitch.Set_gains(_p,_i,_d);
				else if(data_in.startsWith("PID2")) pid_yaw.Set_gains(_p,_i,_d);
				else if(data_in.startsWith("PID3")) pid_mag.Set_gains(_p,_i,_d);
				else if(data_in.startsWith("PID4"))	pid_alt.Set_gains(_p,_i,_d);
			}

			else if(data_in.startsWith("HBS")) throttle_bias = atoi(getValue(data_in,',',1));

			else if(data_in.startsWith("TRR")) imu.roll_angle_offset = atof(getValue(data_in,',',1));

			else if(data_in.startsWith("TRP")) imu.pitch_angle_offset = atof(getValue(data_in,',',1));

			else
			{
				telemetry_transmit_counter++;
				switch(telemetry_transmit_counter)
				{
					case 1: 
					telemetry.Transmit(String("BAT,") + battery_voltage + "\n");
					break;

					case 2:
					telemetry.Transmit(String("ENG,") + engineStart + "\n");
					break;

					case 3:
					telemetry.Transmit(String("RLL,") + imu.Get_GyroX_Angle() + "\n");
					break;

					case 4:
					telemetry.Transmit(String("PTC,") + imu.Get_GyroY_Angle() + "\n");
					telemetry.flush();
					break;

					case 5:
					telemetry.Transmit(String("ALT,") + imu.Get_altitude() + "\n");
					telemetry.flush();
					break;

					case 6:
					telemetry.Transmit(String("HED,") + imu.Get_Heading() + "\n");
					telemetry_transmit_counter = 0;
					telemetry.flush();
					break;
				}
			}
		}
		break;
	}
}

char* getValue(String data, char separator, uint16_t index){
    uint16_t found = 0;
    int16_t strIndex[] = { 0, -1 };
    int16_t maxIndex = data.length() - 1; 				//This is a signed int in case data length is 0

    for (uint16_t i = 0; i <= maxIndex && found <= index; i++) {

        if (data.charAt(i) == separator || i == maxIndex) {
            found++;
            strIndex[0] = strIndex[1] + 1;
            strIndex[1] = (i == maxIndex) ? i+1 : i;
        }
    }

    if(found > index)
    {
    	String res = data.substring(strIndex[0], strIndex[1]);
    	char value[sizeof(res)];
    	for(uint16_t i = 0; i < sizeof(res); i++)
    	{
    		value[i] = res[i];
    	}

    	return value;
    }
    else return "";
}

void Init_Timers()
{
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

	Timer3.attachCompare1Interrupt(channel_handler);
  	TIMER3_BASE->CR1 = TIMER_CR1_CEN;
  	TIMER3_BASE->CR2 = 0;
  	TIMER3_BASE->SMCR = 0;
  	TIMER3_BASE->DIER = TIMER_DIER_CC1IE;
  	TIMER3_BASE->EGR = 0;
  	TIMER3_BASE->CCMR1 = TIMER_CCMR1_CC1S_INPUT_TI1;
  	TIMER3_BASE->CCMR2 = 0;
  	TIMER3_BASE->CCER = TIMER_CCER_CC1E;
  	//TIMER3_BASE->CCER |= TIMER_CCER_CC1P;    //Detect falling edge.
  	TIMER3_BASE->CCER &= ~TIMER_CCER_CC1P; //Detect rising edge.
  	TIMER3_BASE->PSC = 71;
  	TIMER3_BASE->ARR = 0xFFFF;
  	TIMER3_BASE->DCR = 0;
}