#define invert(x) (-x)
//Sensors
#define MPU6050
#define MS5611
#define HMC5883L

//MPU6050
#define MPU6050_ADDRESS     (0x68)
#define GYRO_SCALE          (65.5)
#define ACC_SCALE           (170.1388F)
#define ACC_REST            (3600)
#define CALIBRATION_REPEATITIONS (500)

//MS5611
#define OSR                 (0x04)
#define CT                  (3)

#define MS5611_ADDRESS      (0x77)

#define MS5611_CMD_ADC_READ (0x00)
#define MS5611_CMD_RESET		(0x1E)
#define MS5611_CMD_CONV_D1	(0x40)
#define MS5611_CMD_CONV_D2	(0x50)
#define MS5611_CMD_READ_PROM (0xA2)

//HMC5883L
#define HMC5883L_ADDRESS 		(0x1E)

//RC
#define CH1_CENTERED        (1495)
#define CH2_CENTERED        (1495)
#define CH4_CENTERED        (1499)
#define CONTROLLER_DEADBAND (12)
#define CONTROLLER_SENSATIVITY (3.0F)

//PID
#define MAX_P               (400)
#define MAX_I               (200)

//Global
#define ESC_MAX_LIMIT       (2000)
#define ESC_MIN_LIMIT       (1096)
#define ESC_OFF             (1010)
#define THROTTLE_MAX_LIMIT 	(1800)
#define THROTTLE_MIN_LIMIT  (1100)
#define MIN_THROTTLE_VALUE 	(1015)
#define MAX_THROTTLE_VALUE 	(1993)
#define AUX1_VALUE          (9000) // this happans when top right button is clicked (NON STATIC!)
#define AUX2_VALUE          (5000) //Static this happans when light control is clicked (STATIC!)
#define channel_4_MIN_VALUE (1196)
#define channel_4_MAX_VALUE (1792)
#define THROTTLE_HOVER_VALUE (1540)
#define THROTTLE            (2)
#define ROLL                (0)
#define PITCH               (1)
#define YAW				          (3)
#define EARTH_G			        (9.8F)
#define SERIAL_DEC_SIZE     (4)
#define IND_LED			        (PC13)
#define MAX_SERIAL_BUFFER		(30)
#define VD_SCALE            (0.3287671232876712)
