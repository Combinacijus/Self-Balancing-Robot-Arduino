/*
  TODO LIST

  +Timer interupt sensor readings
  ~Fixing acc angle. Use internet equation
  ~PID
  +PID multiply by time
  +Target angle correction
  +Don't drive motors_deadzone
  +PID tunning

  +acc_angle moving average
  ~acc_angle substract acceleration vector
*/

#include <Wire.h>
#include <MPU9250.h>
#include <TM1637.h>
#include <TimerOne.h>
#include <PPMReader.h>
#include "Robot.h"
#include "Button.h"
#include "pid_controller.h"
#include "rotary_encoder.h"

const int PIN_ENA = 4; // left motor
const int PIN_ENB = 5; // right motor
const int PIN_INA = 8;
const int PIN_INB = 9;
const int PIN_INC = 10;
const int PIN_IND = 11;

const int PIN_POT = A0;
const int PIN_ENCODER_A = 2;
const int PIN_ENCODER_B = 13; // Should be 3

const int PIN_BTN1 = A1;
const int PIN_BTN2 = A2;
const int PIN_BTN3 = A3;
const int PIN_BTN4 = 12;

const int PIN_DISP_CLK = 6; // Segmented display clk pin
const int PIN_DISP_DIO = 7; // Segmented display dio pin

const int PIN_PPM_INTERUPT = 3;
const int PPM_CHANNEL_AMOUNT = 7;

const float START_ANGLE = 0.05; // Angles in degrees when robot is still active
const float STOP_ANGLE = 45;

unsigned long long counter = 1;

int enable_motors = true; // TODO change to true
int imu_status = 0;
const int ENCODER_PRESCALE = 1;
int isr_frequency = 350 * ENCODER_PRESCALE; // For rotary encoder too
long isr_period = 1000000 / isr_frequency;
unsigned int isr_counter = 0;
//long reading_count = 0; // TODO delete?
bool my_interrupts = true;
int channels[8]; // Rc control channels value

float gyro;

Button btn1 = Button(PIN_BTN1);
Button btn2 = Button(PIN_BTN2);
Button btn3 = Button(PIN_BTN3, 3);
Button btn4 = Button(PIN_BTN4, 12);

TM1637 segmented_disp(PIN_DISP_CLK, PIN_DISP_DIO);
MPU9250 IMU(Wire, 0x68); // 3v3, gnd, SCL - A5, SDA - A4
Encoder encoder(PIN_ENCODER_A, PIN_ENCODER_B);
PPMReader rc(PIN_PPM_INTERUPT, PPM_CHANNEL_AMOUNT);  // RC control
Robot robot = Robot(PIN_ENA, PIN_ENB, PIN_INA, PIN_INB, PIN_INC, PIN_IND);

//PID pid = PID(45, 5, 500); // 8V
//PID pid = PID(45, 3, 130); // 7.6V  /7
//PID pid = PID(30, 2, 50); // Gyro only
//PID pid = PID(45, 4, 110); // Gyro only
PID pid = PID(45, 4, 400); // Gyro only
int motors_deadzone = 0;
float pid_val = 0;  // PWM value of how fast to turn motors (+-)

void setup()
{
	Serial.begin(115200);
	Serial.println("Start");
	randomSeed(analogRead(0));

	pinMode(PIN_POT, INPUT);
	pinMode(PIN_ENCODER_A, INPUT);
	pinMode(PIN_ENCODER_B, INPUT);

	digitalWrite(PIN_ENA, HIGH);
	digitalWrite(PIN_ENB, HIGH);

	btn4.state_count = 6;

	imu_status = IMU.begin();
	pr("IMU init: ", imu_status);
	IMU.setAccelRange(MPU9250::ACCEL_RANGE_2G);
	IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
	// Accelerotmeter calibration
	IMU.setAccelCalX(0.095, 1); //	-9.73 9.92
	IMU.setAccelCalY(-0.08, 1); //  -9.92 9.76 
	IMU.setAccelCalZ(0.235, 9.83 / 9.86); //  -9.63 10.10
	updateIMU();
	Serial.println(String(robot.angle) + " " + String(robot.angle_acc_true));
	robot.angle = robot.angle_acc_true;
	Serial.println(String(robot.angle) + " " + String(robot.angle_acc_true));

	segmented_disp.init();
	segmented_disp.set(BRIGHT_TYPICAL); //BRIGHT_TYPICAL = 2 [0 ... 7]

	delay(100);
	// Start interrupt service routine
	Timer1.initialize(isr_period);
	Timer1.attachInterrupt(myISR);
}

void myISR()
{
	static bool i_am_running = false;
	if (i_am_running)
		return; // Bail out if we interrupted ourself
	i_am_running = true;
	interrupts(); // Enable interrupts so I2C and everything else can work

	long time = micros();

	++isr_counter;
	encoder.update();

	if (isr_counter % ENCODER_PRESCALE == 0) // Normal ISR frequency
	{
		if (my_interrupts)
		{
			updateIMU();
			updatePIDControl();
		}
		runMotors();
	}

	//Serial.println("Dt:  " + String(micros() - time));
	//Serial.println(1000000.0 / (micros() - time));  // ISR max update rate

	i_am_running = false;
}

void updateIMU()
{
	// Accelerometer sensor is broken
	int a = IMU.readSensor();

	long t1, t2, t3, t4, t5;
	long time = micros();
	gyro = IMU.getGyroY_rads();
	//gyro = IMU.getGyroX_rads();
	t1 = micros() - time;

	float accX = IMU.getAccelX_mss();
	float accY = IMU.getAccelY_mss();
	//float accY = -0.2; // May cause problems on slanted surface
	float accZ = IMU.getAccelZ_mss();

	//	float acc_mag = sqrt(accX * accX + accY * accY + accZ * accZ);
		//float angle_acc2 = -acos(accX / alcc_mag); // Angle of MPU9250

	float angle_acc = atan(accX / sqrt(accY * accY + accZ * accZ)); // TODO
	//float angle_acc = atan(accY / sqrt(accX * accX + accZ * accZ));
	robot.update_angle(gyro, angle_acc);

	//Serial.println(String(accX) + " " + String(accY) + " " + String(accZ));
	//Serial.println("gyro angle: " + String(robot.get_angle(true, true)) + " " + String(robot.get_angle(false, true)));
//  + " " + String(angle_acc * RAD_TO_DEG)
//	+ " " + String(angle_acc * RAD_TO_DEG / (1 + abs(gyro) * 3)));
}

void updatePIDControl()
{
	//Serial.println(pid.error_integral);
	pid_val = pid.control(robot.get_angle());
}

void calibrateIMU()
{
	my_interrupts = false;
	robot.motors_turnoff();
	segmented_disp.displayNum(88.888, 2);
	delay(1000);
	Serial.println("Calibrating gyro");
	imu_status = IMU.calibrateGyro();
	pr("IMU gyro calib: ", imu_status);
	pr("GyroX Bias: ", IMU.getGyroBiasX_rads() * RAD_TO_DEG);
	pr("GyroY Bias: ", IMU.getGyroBiasY_rads() * RAD_TO_DEG);
	pr("GyroZ Bias: ", IMU.getGyroBiasZ_rads() * RAD_TO_DEG);

	IMU.setGyroBiasX_rads(IMU.getGyroBiasX_rads());
	IMU.setGyroBiasY_rads(IMU.getGyroBiasY_rads());
	IMU.setGyroBiasZ_rads(IMU.getGyroBiasZ_rads());
	delay(350);

	my_interrupts = true;
}

void updateButtons()
{
	if (btn2.getOnTime(100))
	{
		enable_motors = !enable_motors;

		if (btn3.getState() == HIGH)
			calibrateIMU();
	}

	if (btn1.getState() == HIGH)
	{
		robot.resetTargetAngle();
		pid.error_integral = 0;
	}

	btn3.getOnTime(100); // Update state
	switch (btn3.state_count)
	{
	case 1:
		// Display gyro angle
		segmented_disp.displayNum(robot.get_angle(), 1);
		break;

	case 2:
		// Display accelerometer angle
		segmented_disp.displayNum(robot.get_angle(false), 1);
		break;

	case 3:
		// Display seconds while balancing
		segmented_disp.displayNum(robot.get_balance_time(), 0);
		break;
	}

	if (btn4.getOnTime(100)) // Update state
	{
		float fix_angle = (btn4.state_count - (btn4.num_of_states / 2)) / 10.0;
		segmented_disp.displayNum(fix_angle, 1);
		robot.target_angle_fix = robot.target_fix_const + fix_angle * DEG_TO_RAD;
		delay(500);
	}
}

void runMotors()
{
	if (robot.is_balancing)
	{
		pid.add_i = true;

		if (abs(robot.get_angle()) > STOP_ANGLE)
		{
			robot.set_balancing(false);
		}

		if (enable_motors && abs(robot.get_angle()) <= STOP_ANGLE && abs(robot.get_angle()) >= START_ANGLE)
		{
			robot.motor(1, pid_val - channels[1], motors_deadzone);
			robot.motor(2, pid_val + channels[1], motors_deadzone);
		}
		else  // If motors disabled or robot fell over
		{
			pid.add_i = false;
			robot.motors_turnoff();
		}
	}
	else // is_balancing == false
	{
		if (robot.is_balanced(500))  // If robot is upright for 0.5s it will start balancing
		{
			robot.set_balancing(true);
			pid.error_integral = 0;
		}
	}
}

void serialVariableUpdate()
{
	String data = "";   // Line of data (format char|double eg. k10.21)
	char code = ' ';	// Single character code of the command
	double number = 0;

	if (Serial.available())
	{
		my_interrupts = false;
		enable_motors = false;
		robot.motors_turnoff();
		delay(100);

		data = Serial.readString();
		code = data[0];
		number = data.substring(1).toDouble();

		switch (code)
		{
		case 'p':
			Serial.println("PID Kp = " + String(number));
			pid.kp = number;
			break;

		case 'i':
			Serial.println("PID Ki = " + String(number));
			pid.ki = number;
			//pid.set_antiwindup(pid.ki);
			pid.set_antiwindup(255);
			break;

		case 'd':
			Serial.println("PID Kd = " + String(number));
			pid.kd = number;
			break;

		case 'o':
			Serial.println("Deadzone = " + String(number));
			motors_deadzone = number;
			break;

		case 'r':
			Serial.println("P" + String(pid.kp) + " " + "I" + String(pid.ki) + " " + "D" + String(pid.kd) + " " + "O" + String(motors_deadzone));
			delay(1500);
			break;

		case 'c':
			Serial.println("MPU9250 calibration data");
			Serial.println("Acc biasX: " + String(IMU.getAccelBiasX_mss()));
			Serial.println("Acc biasY: " + String(IMU.getAccelBiasY_mss()));
			Serial.println("Acc biasZ: " + String(IMU.getAccelBiasZ_mss()));
			Serial.println("Acc sX: " + String(IMU.getAccelScaleFactorX()));
			Serial.println("Acc sY: " + String(IMU.getAccelScaleFactorY()));
			Serial.println("Acc sZ: " + String(IMU.getAccelScaleFactorZ()));
			Serial.println("Gyro biasX: " + String(IMU.getGyroBiasX_rads()));
			Serial.println("Gyro biasY: " + String(IMU.getGyroBiasY_rads()));
			Serial.println("Gyro biasZ: " + String(IMU.getGyroBiasZ_rads()));
			delay(4000);
			break;

		case 't':
			Serial.println("target angle" + String(number));
			robot.target_angle_fix = number * RAD_TO_DEG;
			break;
		}

		delay(500);
		my_interrupts = true;
		enable_motors = true;
	}
}

bool aprox(float val1, float val2, float range = 10)
{
	// Returns true if values are similar
	// Default range = 10 for 255 rc values
	if (abs(val1 - val2) <= range)
		return true;
	return false;
}

void radioReceive()
{
	static const int ch_amount = 5;
	static const int channels_to_read[ch_amount] = { 1, 2, 3, 6, 7 };
	static const int ppm_deadzone = 5;
	//for (int i = 1; i <= PPM_CHANNEL_AMOUNT; ++i) {
	//	unsigned long value = rc.latestValidChannelValue(i, 0);
	//	Serial.print(String(value) + " ");
	//}
	//Serial.println();

	for (int i = 0; i < ch_amount; ++i)
	{
		byte ch = channels_to_read[i];
		channels[ch] = constrain(map(rc.latestValidChannelValue(ch, 0), 1000, 2000, -255, 255), -255, 255);
		if (abs(channels[ch]) <= ppm_deadzone)
			channels[ch] = 0;
//		Serial.print("Ch" + String(ch) + ": " + String(channels[ch]) + "  ");
	}
//	Serial.println("");
}

void radioControlMovement()
{
	if (aprox(channels[6], 255))
	{
		robot.target_angle = channels[2] / 40.0 * DEG_TO_RAD;
//		Serial.println(robot.target_angle);
	}
	else
	{
		robot.target_angle = 0;
	}

	// When robot is fallen over
	if (!robot.is_balanced())
	{
		if (aprox(channels[6], 255))
		{
			if (channels[2] > 0)
			{
				robot.motor(1, channels[2] - channels[1], channels[3]);
				robot.motor(2, channels[2] + channels[1], channels[3]);
			}
			else
			{
				robot.motor(1, channels[2] + channels[1], channels[3]);
				robot.motor(2, channels[2] - channels[1], channels[3]);
			}
		}
		else
		{
			robot.motors_turnoff();
		}
	}
}

void loop()
{
	// Main functionality runs in ISR()
	updateButtons();
	serialVariableUpdate();

	radioReceive();
	radioControlMovement();

  static int a = 24;
  static float test_range = 0;
  test_range = abs(channels[7]) / 10.0;
  
  if (robot.pwm_integral > test_range)
  {
    Serial.print(String(a) + " ");
    robot.target_angle_f= 0.1 * DEG_TO_RAD;
  }
  else if (robot.pwm_integral < -test_range)
  {
    Serial.print(String(-a) + " ");
//    robot.target_angle_fix += 0.1 * DEG_TO_RAD;
  }
  else
  {
    Serial.print(" 0 ");
  }
    
	Serial.print(robot.pwm_integral);
  Serial.println(" " + String(test_range) + " " + String(-test_range));

	//static float encoder_factor = 1.0 / 4.0;
	//robot.target_angle = constrain(-encoder.position * encoder_factor, -0.025, 0.025);

	if (!enable_motors)
		encoder.position = 0;

	//Serial.println("pos&corr: " + String(encoder.position));

	//robot.target_angle = constrain(-encoder.position / 100.0, -0.1, 0.1) * DEG_TO_RAD;

	//Serial.println("Target: " + String(robot.target_angle * RAD_TO_DEG));

	// If robot is moving then slow it down
	// TODO make it continuous
	//if (enable_motors)
	//{
	//	Serial.println("PWM: " + String(pid_val));

	//	if (pid_val > 100 && robot.angle > 0)
	//	{
	//		int delay_t = 100;
	//		int angle_target = -0.7;
	//		if (pid_val > 170)
	//		{
	//			delay_t += 35;
	//			//angle_target -= 0.5;
	//		}
	//		robot.target_angle = angle_target * DEG_TO_RAD;
	//		delay(delay_t);
	//		robot.target_angle = 0;
	//		delay(50);
	//	}
	//	else if (pid_val < -100 && robot.angle < 0)
	//	{
	//		int delay_t = 100;
	//		int angle_target = 0.7;
	//		if (pid_val < -170)
	//		{
	//			delay_t += 35;
	//			//angle_target += 0.5;
	//		}
	//		robot.target_angle = angle_target * DEG_TO_RAD;
	//		delay(delay_t);
	//		robot.target_angle = 0;
	//		delay(50);
	//	}
	//}
}

void pr(String s, float f)
{
	Serial.print(s);
	Serial.println(f);
}
