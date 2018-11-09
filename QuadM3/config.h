#include <Arduino.h>
#define invert(x) (-x)
//Sensors
#define MPU6050
#define MS5611
#define HMC5883L

//MPU6050
#define MPU6050_ADDRESS     	(0x68)
#define GYRO_SCALE          	(65.5)
#define ACC_SCALE           	(170.1388F)
#define ACC_REST            	(3600)
#define CALIBRATION_REP 		  (500)


//MS5611
#define MS5611_MAX_OSR			(0x08)
#define MS5611_ADDRESS      	(0x77)

//HMC5883L
#define HMC5883L_ADDRESS 		(0x1E)

//RC
#define CH1_CENTERED        	(1500)
#define CH2_CENTERED        	(1500)
#define CH4_CENTERED        	(1500)
#define CONTROLLER_DEADBAND 	(3)
#define CONTROLLER_SENSATIVITY 	(3.0F)

//PID
#define ROLL_MAX_P              (400)
#define ROLL_MAX_I              (200)

//Global
#define ESC_MAX_LIMIT       	(2000)
#define ESC_MIN_LIMIT       	(1096)
#define ESC_OFF             	(1010)
#define THROTTLE_MAX_LIMIT 		(1800)
#define THROTTLE_MIN_LIMIT  	(1100)
#define MIN_THROTTLE_VALUE 		(1000)
#define THROTTLE            	(2)
#define ROLL                	(0)
#define PITCH               	(1)
#define YAW						(3)
#define AUX_1                   (4)
#define AUX_2                   (5)
#define AUX_3                   (6)
#define AUX_4                   (7)
#define EARTH_G			        (9.8f)
#define SERIAL_DEC_SIZE     	(4)
#define IND_LED			        (PC13)
#define MAX_SERIAL_BUFFER		(30)
#define VD_SCALE            	(0.3287671232876712)
#define LOOP_TIME           	(4000)
#define GYRO_MODE               (1)
#define ALT_MODE                (2)
#define GPS_MODE                (3)

//Telemetry
#define AT_PIN 					(PC14)
#define BUFFER_SIZE				(16)
