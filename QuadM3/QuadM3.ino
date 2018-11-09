#include "IMU.h"
#include "PID.h"
#include "config.h"
// #include "Telemetry.h"
#include "GPS.h"

//IMU variables
IMU imu;
//RC variables
int32_t channel[8];
uint8_t channel_select_counter;
int32_t measured_time, measured_time_start;
uint8_t aux1,prev_aux1;
float controller_sensativity = 3.0f;
float heading_lock_course_deviation, roll_mag_ajt,pitch_mag_ajt;
float course_lock_heading;
float yaw_mag_correction;
bool mag_correction_set = 0;
bool head_lock_set = 0;
float low_pass_heading;

//PID variables
PID pid_roll, pid_pitch, pid_yaw;
PID pid_mag;
PID pid_alt;
PID pid_gps;
int16_t setPoint_roll,setPoint_pitch,setPoint_yaw;
float setPoint_altitude;
float pressure_setPoint;
float setPoint_mag;

//Telemetry variables
// Telemetry telemetry;
uint16_t telemetry_transmit_counter;
int8_t telemetry_mode = -1;

//Global variables
int16_t battery_voltage;
uint16_t esc[4];
int16_t throttle, prev_throttle,throttle_base;
uint16_t deltaTime;
uint16_t throttle_bias = 300;

uint32  loop_timer;
uint32  esc_timer, esc_loop_timer;
uint32  prev_deltaTime;

float heading_setPoint;
float voltage_compensation;

bool Heading_lock = false;
bool mag_correction = true;
bool alt_hold = false;
bool engineStart = false;
bool battery_connected = false;

uint8_t flight_mode = 1;
int16_t throttle_rate;
bool pressure_setPoint_set = false;
uint8_t start_Sequence = 0;
uint32_t loop_counter = 0;

//GPS
GPS gps;
bool gps_hold_set = 0;
float gps_roll_ajt,gps_pitch_ajt;

void setup() {
	Serial.begin(9600);
	// telemetry.Init(1200);
	delay(350);
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
	pid_yaw.Set_gains(5.0, 0.01, 5.0);
	pid_mag.Set_gains(0.8,0.00,2.0);
	pid_alt.Set_gains(3.0,0.01,3.0);

	heading_setPoint = imu.Get_Heading();
	gps.Init();
	digitalWrite(IND_LED,1);
}

void loop()
{
	//Calculate delta time;
	deltaTime = (micros() - prev_deltaTime);
	prev_deltaTime = micros();

	// gps.Read();
	imu.Compute();
	Rx_toValue(); 

	//read battery battery_voltage
	battery_voltage = analogRead(0) * VD_SCALE;


	if(StartEngines())
	{
		pid_roll.Compute(setPoint_roll - imu.Get_GyroX(),300,200);
		pid_pitch.Compute(invert(setPoint_pitch) - imu.Get_GyroY(),300,200);
		
		pid_yaw.Compute(invert(setPoint_yaw)  - imu.Get_GyroZ(),300);

		if(alt_hold && pressure_setPoint_set)
		{
			if(IsBetween(channel[THROTTLE],1505,2000)) pressure_setPoint -= 0.1;
			else if(IsBetween(channel[THROTTLE],1000,1490)) pressure_setPoint += 0.1;

			float alt_error = imu.Get_pressure() - pressure_setPoint;

			pid_alt.Compute(alt_error,300);
			throttle = THROTTLE_MIN_LIMIT + throttle_bias + pid_alt.output;
		}
		else
		{
			throttle = channel[THROTTLE] - 100;
		} 

		throttle = constrain(throttle,THROTTLE_MIN_LIMIT,THROTTLE_MAX_LIMIT);

		//Calculate the value for each motor
		esc[0] = throttle + pid_roll.output - pid_pitch.output - pid_yaw.output;
		esc[1] = throttle - pid_roll.output - pid_pitch.output + pid_yaw.output;
		esc[2] = throttle - pid_roll.output + pid_pitch.output - pid_yaw.output;
		esc[3] = throttle + pid_roll.output + pid_pitch.output + pid_yaw.output;

		//Calcualte the battery compensation
		if(IsBetween(battery_voltage,1050,1280)) {
			battery_connected = true;
			voltage_compensation = (1240 - battery_voltage) / (float)3500;
		}
		else battery_connected = false;

		//add the battery compensation to each motor.
		if(battery_connected)
		{
			esc[0] += esc[0] * voltage_compensation;
			esc[1] += esc[1] * voltage_compensation;
			esc[2] += esc[2] * voltage_compensation;
			esc[3] += esc[3] * voltage_compensation;
		}

		esc[0] = constrain(esc[0], ESC_MIN_LIMIT, ESC_MAX_LIMIT);
		esc[1] = constrain(esc[1], ESC_MIN_LIMIT, ESC_MAX_LIMIT);
		esc[2] = constrain(esc[2], ESC_MIN_LIMIT, ESC_MAX_LIMIT);
		esc[3] = constrain(esc[3], ESC_MIN_LIMIT, ESC_MAX_LIMIT);
	}

	else //Turn engines off 
	{ 
		esc[0] = ESC_OFF;
		esc[1] = ESC_OFF;
		esc[2] = ESC_OFF;
		esc[3] = ESC_OFF;
	}

	while(micros() - loop_timer < LOOP_TIME); //check to see if lowering loop time below 4ms give better fly results (make sure imu calculations dont depends on 4ms loop time)
	loop_timer = micros();

	Write_4Engines();
	loop_counter++;
	if(!(loop_counter%250) && gps.Fix() >=3 && gps.Active_satellites() >=8) 
	{
		digitalWrite(IND_LED,!digitalRead(IND_LED));
	}
	else digitalWrite(IND_LED,1);
	// Telemetry_TR(telemetry_mode);


	//Serial Debuging
	// for(uint8_t i = 0; i < 8; i++)
	// {
	// 	Serial.print(",");
	// 	Serial.print(channel[i]);
	// }
	// Serial.println();
	// Serial.print(esc[0]);
	// Serial.print(",");
	// Serial.print(esc[3]);
	// Serial.print(",");
	// Serial.print(esc[1]);
	// Serial.print(",");
	// Serial.println(esc[2]);
	// Serial.println(setPoint_yaw);
}
void Rx_toValue()
{
		//Roll
		setPoint_roll = 0;
		if(channel[ROLL] > CH1_CENTERED + CONTROLLER_DEADBAND){
			setPoint_roll = channel[ROLL] - CH1_CENTERED + CONTROLLER_DEADBAND;
		}
		else if(channel[ROLL] < CH1_CENTERED - CONTROLLER_DEADBAND){
			setPoint_roll = channel[ROLL] - CH1_CENTERED - CONTROLLER_DEADBAND;
		}

		//Pitch
		setPoint_pitch = 0;
		if(channel[PITCH] > CH2_CENTERED + CONTROLLER_DEADBAND){
			setPoint_pitch = channel[PITCH] - CH2_CENTERED + CONTROLLER_DEADBAND;
		}
		else if(channel[PITCH] < CH2_CENTERED - CONTROLLER_DEADBAND){
			setPoint_pitch = channel[PITCH] - CH2_CENTERED - CONTROLLER_DEADBAND;
		}

		// Calculate_GPS_hold();

		// setPoint_roll += gps_roll_ajt;
		setPoint_roll -= imu.Roll_Level_Error();
		setPoint_roll /= controller_sensativity;

		// setPoint_pitch += gps_pitch_ajt;
		setPoint_pitch -= imu.Pitch_Level_Error();
		setPoint_pitch /= controller_sensativity;

	//Yaw
	if(channel[THROTTLE] > 1100)
	{
		setPoint_yaw = 0;

		if(channel[YAW] > CH4_CENTERED + CONTROLLER_DEADBAND ){
			setPoint_yaw = channel[YAW] - CH4_CENTERED + CONTROLLER_DEADBAND;
			setPoint_yaw /= CONTROLLER_SENSATIVITY;
			mag_correction_set = 0;
		}
		else if(channel[YAW] < CH4_CENTERED - CONTROLLER_DEADBAND){
			setPoint_yaw = channel[YAW] - CH4_CENTERED - CONTROLLER_DEADBAND;
			setPoint_yaw /= CONTROLLER_SENSATIVITY;
			mag_correction_set = 0;
		}
		//centered
		else if(mag_correction)
		{
			if(imu.Get_Heading() > 2 && imu.Get_Heading() < 358) //avoid jumping between 0 and 360
			{
				if(mag_correction_set)
				{
					low_pass_heading = low_pass_heading *0.8 + imu.Get_Heading() *0.2;
					pid_mag.Compute(low_pass_heading - yaw_mag_correction);
					setPoint_yaw = pid_mag.output *-1;
					setPoint_yaw = constrain(setPoint_yaw,-20,20);
				}
				else
				{
					yaw_mag_correction = imu.Get_Heading();
					mag_correction_set = 1;
				}
			}
		}
	}

	//AUX Channels

	//Aux1
	if(IsBetween(channel[AUX_1],990,1050)) {
		flight_mode = 1;
		alt_hold = false;
		pressure_setPoint_set = 0;
		gps_hold_set = 0;
	}
	else if(IsBetween(channel[AUX_1],1490,1550))
	{
		flight_mode = 2;
		alt_hold = true;
		gps_hold_set = 0;
		if(!pressure_setPoint_set)
		{
			pressure_setPoint = imu.Get_pressure();
			pressure_setPoint_set = 1;
		}
	}
	else if(IsBetween(channel[AUX_1],1990,2010))
	{
		flight_mode = 3;
		alt_hold = true;
		if(gps.Fix() >=3) 
		{
			if(!gps_hold_set)
			{
				gps.gps_hold_flag = 1;
				gps_hold_set = 1;
			}
		}

		if(!pressure_setPoint_set)
		{
			pressure_setPoint = imu.Get_pressure();
			pressure_setPoint_set = 1;
		}
	}

	//Aux2
	if(IsBetween(channel[AUX_2],990,1050))
	{
	}
	else if(IsBetween(channel[AUX_2],1490,1550))
	{
	}
	else if(IsBetween(channel[AUX_2],1990,2005))
	{
		if(!engineStart) imu.Calibrate_IMU();
	}

	//Aux3
	if(IsBetween(channel[AUX_3],990,1050))
	{
	}
	else if(IsBetween(channel[AUX_3],1490,1550))
	{
	}
	else if(IsBetween(channel[AUX_3],1990,2005))
	{
	}

	//Aux4
	if(IsBetween(channel[AUX_4],990,1050))
	{
		head_lock_set = 0;
	}
	else if(IsBetween(channel[AUX_4],1490,1550))
	{
	}
	else if(IsBetween(channel[AUX_4],1990,2005))
	{
		if(!head_lock_set)
		{
			head_lock_set = 1;
			heading_lock_course_deviation = imu.Get_Heading();
		}
	}
}

void Write_4Engines()
{
	TIMER4_BASE->CCR1 = esc[0];
	TIMER4_BASE->CCR2 = esc[1];
	TIMER4_BASE->CCR3 = esc[2];
	TIMER4_BASE->CCR4 = esc[3];
	TIMER4_BASE->CNT = 5000;
}

bool StartEngines()
{
	if(IsBetween(channel[THROTTLE],990,1010) && IsBetween(channel[YAW],1990,2010))
	{
		//turn off engines and reset pid and imu angles
		engineStart = false;
		start_Sequence = 0;
		pid_roll.Reset();
		pid_pitch.Reset();
		pid_yaw.Reset();
		pid_mag.Reset();
		pid_alt.Reset();
		imu.reset();
	}

	//Start engines.
	else if (IsBetween(channel[THROTTLE],990,1010) && IsBetween(channel[YAW],990,1050)) start_Sequence = 1;
	else if (IsBetween(channel[YAW],1490,1505) && start_Sequence) engineStart = true;

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


void Calculate_GPS_hold()
{
	if(flight_mode >=3)
	{
		gps_roll_ajt = ((float)gps.gps_roll_adjust_north * cos(imu.Get_Heading() * 0.017453)) + ((float)gps.gps_pitch_adjust_north * cos((imu.Get_Heading() - 90) * 0.017453));
		gps_pitch_ajt = ((float)gps.gps_pitch_adjust_north * cos(imu.Get_Heading() * 0.017453)) + ((float)gps.gps_roll_adjust_north * cos((imu.Get_Heading() + 90) * 0.017453));

		gps_roll_ajt = constrain(gps_roll_ajt,-100,100);
		gps_pitch_ajt = constrain(gps_pitch_ajt,-100,100);
	}
	else 
	{
		gps_pitch_ajt = 0;
		gps_roll_ajt = 0;
	}
}
// void Telemetry_TR(uint8_t mode)
// {
// 	switch(mode)
// 	{
// 		case -1:
// 		return;

// 		case 0:
// 		telemetry_transmit_counter++;
// 		switch(telemetry_transmit_counter)
// 		{
// 			case 1:
// 			break;

// 			case 10: 
// 			telemetry.Transmit(String("BAT,") + battery_voltage + "\n");
// 			break;

// 			case 20:
// 			telemetry.Transmit(String("ENG,") + engineStart + "\n");
// 			break;

// 			case 30:
// 			telemetry.Transmit(String("RLL,") + imu.Get_GyroX_Angle() + "\n");
// 			break;

// 			case 40:
// 			telemetry.Transmit(String("PTC,") + imu.Get_GyroY_Angle() + "\n");
// 			telemetry.flush();
// 			break;

// 			case 50:
// 			telemetry.Transmit(String("ALT,") + imu.Get_altitude() + "\n");
// 			telemetry.flush();
// 			break;

// 			case 60:
// 			telemetry.Transmit(String("HED,") + imu.Get_Heading() + "\n");
// 			telemetry_transmit_counter = 0;
// 			telemetry.flush();
// 			break;

// 			case 70:
// 			break;
// 		}
// 		break;

// 		case 1:
// 		if(telemetry.Receive())
// 		{
// 			String data_in = String(telemetry.in_buffer);
// 			if(data_in.startsWith("PID"))
// 			{
// 				float _p = atof(getValue(data_in,',',1));
// 				float _i = atof(getValue(data_in,',',2));
// 				float _d = atof(getValue(data_in,',',3));
// 				if(data_in.startsWith("PID0")) pid_roll.Set_gains(_p,_i,_d);
// 				else if(data_in.startsWith("PID1")) pid_pitch.Set_gains(_p,_i,_d);
// 				else if(data_in.startsWith("PID2")) pid_yaw.Set_gains(_p,_i,_d);
// 				else if(data_in.startsWith("PID3")) pid_mag.Set_gains(_p,_i,_d);
// 				else if(data_in.startsWith("PID4"))	pid_alt.Set_gains(_p,_i,_d);
// 			}

// 			else if(data_in.startsWith("HBS")) throttle_bias = atoi(getValue(data_in,',',1));

// 			else if(data_in.startsWith("TRR")) imu.roll_angle_offset = atof(getValue(data_in,',',1));

// 			else if(data_in.startsWith("TRP")) imu.pitch_angle_offset = atof(getValue(data_in,',',1));
// 			telemetry.Transmit(String("OK\n"));
// 		}
// 		break;

// 		case 2: //Ping pong..
// 		if(telemetry.Receive())
// 		{
// 			String data_in = String(telemetry.in_buffer);
// 			if(data_in.startsWith("PID"))
// 			{
// 				float _p = atof(getValue(data_in,',',1));
// 				float _i = atof(getValue(data_in,',',2));
// 				float _d = atof(getValue(data_in,',',3));
// 				if(data_in.startsWith("PID0")) pid_roll.Set_gains(_p,_i,_d);
// 				else if(data_in.startsWith("PID1")) pid_pitch.Set_gains(_p,_i,_d);
// 				else if(data_in.startsWith("PID2")) pid_yaw.Set_gains(_p,_i,_d);
// 				else if(data_in.startsWith("PID3")) pid_mag.Set_gains(_p,_i,_d);
// 				else if(data_in.startsWith("PID4"))	pid_alt.Set_gains(_p,_i,_d);
// 			}

// 			else if(data_in.startsWith("HBS")) throttle_bias = atoi(getValue(data_in,',',1));

// 			else if(data_in.startsWith("TRR")) imu.roll_angle_offset = atof(getValue(data_in,',',1));

// 			else if(data_in.startsWith("TRP")) imu.pitch_angle_offset = atof(getValue(data_in,',',1));

// 			else
// 			{
// 				telemetry_transmit_counter++;
// 				switch(telemetry_transmit_counter)
// 				{
// 					case 1: 
// 					telemetry.Transmit(String("BAT,") + battery_voltage + "\n");
// 					break;

// 					case 2:
// 					telemetry.Transmit(String("ENG,") + engineStart + "\n");
// 					break;

// 					case 3:
// 					telemetry.Transmit(String("RLL,") + imu.Get_GyroX_Angle() + "\n");
// 					break;

// 					case 4:
// 					telemetry.Transmit(String("PTC,") + imu.Get_GyroY_Angle() + "\n");
// 					telemetry.flush();
// 					break;

// 					case 5:
// 					telemetry.Transmit(String("ALT,") + imu.Get_altitude() + "\n");
// 					telemetry.flush();
// 					break;

// 					case 6:
// 					telemetry.Transmit(String("HED,") + imu.Get_Heading() + "\n");
// 					telemetry_transmit_counter = 0;
// 					telemetry.flush();
// 					break;
// 				}
// 			}
// 		}
// 		break;
// 	}
// }

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

float course_deviation(float course_b, float course_c) {
	float base_course_mirrored,actual_course_mirrored;
	float course_a = course_b - course_c;
	
	if (course_a < -180 || course_a > 180) {
	  if (course_c > 180)base_course_mirrored = course_c - 180;
	  else base_course_mirrored = course_c + 180;
	  if (course_b > 180)actual_course_mirrored = course_b - 180;
	  else actual_course_mirrored = course_b + 180;
	  course_a = actual_course_mirrored - base_course_mirrored;
	}
  return course_a;
}

bool IsBetween(int32_t src,int32_t min,int32_t max)
{
	if(src >= min && src <= max ) return true;
	else return false;
}