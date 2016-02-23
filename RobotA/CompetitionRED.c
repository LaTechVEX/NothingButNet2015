#pragma config(I2C_Usage, I2C1, i2cSensors)
#pragma config(Sensor, I2C_1,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign)
#pragma config(Sensor, I2C_2,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign)
#pragma config(Motor,  port1,           LRWheel,       tmotorVex393_HBridge, openLoop, reversed)
#pragma config(Motor,  port3,           LFFlywheel,       tmotorVex393HighSpeed_MC29, openLoop, encoderPort, I2C_1)
#pragma config(Motor,  port6,           RFFlywheel,       tmotorVex393HighSpeed_MC29, openLoop, reversed, encoderPort, I2C_2)
#pragma config(Motor,  port4,           LRFlywheel,      tmotorVex393HighSpeed_MC29, openLoop, reversed)
#pragma config(Motor,  port7,           RRFlywheel,      tmotorVex393HighSpeed_MC29, openLoop)
#pragma config(Motor,  port5,           Feeder,        tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port2,           LFWheel,       tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port8,           RRWheel,       tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port9,           RFWheel,       tmotorVex393_MC29, openLoop)
#pragma platform(VEX)

// Competition Control and Duration Settings
#pragma competitionControl(Competition)
#pragma autonomousDuration(45)
#pragma userControlDuration(115)
#include "Vex_Competition_Includes.c"

// PID Control Definitions
#define PID_SENSOR_INDEX     I2C_1
#define PID_SENSOR_SCALE     1
#define PID_MOTOR_INDEX      LFFlywheel
#define PID_MOTOR_SCALE      (-1)
#define PID_DRIVE_MAX        127
#define PID_DRIVE_MIN        (-127)
#define PID_INTEGRAL_LIMIT   50

/////////////////////////////////////////////////////////////////////////////////////////
//                       				 Function Stubs
/////////////////////////////////////////////////////////////////////////////////////////

task pidControl();
void freeze();
void move(int dist);
void rightTurn(int angle);
void leftTurn(int angle);
void fly(int speedElement);
void adjustRight();
void adjustLeft();
void intake(bool on);
void rest();

/////////////////////////////////////////////////////////////////////////////////////////
//                       			  Global Variables
/////////////////////////////////////////////////////////////////////////////////////////

float pid_Kp = 0.506;
float pid_Ki = 0.424;
float pid_Kd = 0.04;
static int pidRunning = 1;
static float pidRequestedValue;
//int Presets[4] = {0, 142, 155, 180};
int Presets[4] = {0, 125, 136, 160};

// These variables track the robots position and orientation
// Useful for any future decisions to veer off track or calculate routes
float locX;
float locY;
float rotation;

/////////////////////////////////////////////////////////////////////////////////////////
//                          Pre-Autonomous Functions
/////////////////////////////////////////////////////////////////////////////////////////

void pre_auton()
{
  // Set bStopTasksBetweenModes to false if you want to keep user created tasks running between
  // Autonomous and Tele-Op modes. You will need to manage all user created tasks if set to false.
  bStopTasksBetweenModes = true;
}

/////////////////////////////////////////////////////////////////////////////////////////
//                                 Autonomous Task
/////////////////////////////////////////////////////////////////////////////////////////

task autonomous()
{
	while(true)
	{
		// Set the motors initially
		int currentPreset = 0;
		fly(0);
		startTask(pidControl);

		if(vexRT(Btn8U)){
		intake(true);
		fly(3);
		/*
		move(25);
		leftTurn(10);
		*/
		wait10Msec(450);
		rest();
		}
	}
}

/////////////////////////////////////////////////////////////////////////////////////////
//                                 User Control Task
/////////////////////////////////////////////////////////////////////////////////////////

task usercontrol()
{
	// Set the motors initially
	pidRunning = 1;
	int currentPreset = 0;
	fly(currentPreset);
	startTask(pidControl);

	while(true)
	{
		// Set the Flywheel Preset
		if(vexRT(Btn8U))
			currentPreset = 0;
		else if (vexRT(Btn8R))
	 		currentPreset = 1;
	 	else if (vexRT(Btn8D))
			currentPreset = 2;
	 	else if (vexRT(Btn8L))
			currentPreset = 3;

		// Set the Flywheel Speed
		fly(currentPreset);

		// Math to make joystick to bot movement exponential instead of linear
		int verticalL = pow(abs(vexRT(Ch3))/128.0,2.0)*127.0 *((vexRT(Ch3)>0)?(1):(-1));
		int verticalR = pow(abs(vexRT(Ch2))/128.0,2.0)*127.0 *((vexRT(Ch2)>0)?(1):(-1));

		// Move Robot
		motor[LRWheel] = verticalL;
		motor[LFWheel] = motor[LRWheel];
		motor[RRWheel] = verticalR;
		motor[RFWheel] = motor[RRWheel];

		//Feeder control
		if(vexRT(Btn5U))
			motor[Feeder] = 127;
		else if(vexRT(Btn6U))
			motor[Feeder] = -127;
		else
			motor[Feeder] = 0;
	}
}




/////////////////////////////////////////////////////////////////////////////////////////
//                                 User Defined Functions
/////////////////////////////////////////////////////////////////////////////////////////

// Stop all wheels
void freeze()
{
	motor[LFWheel] = 0;
	motor[LRWheel] = 0;
	motor[RFWheel] = 0;
	motor[RRWheel] = 0;
}

// Stop all motors on bot
void rest()
{
	freeze();
	fly(0);
	intake(false);
}

void move(int dist)
{
	int distance = abs(dist);
	if(dist>0){
	motor[LFWheel] = 127;
	motor[LRWheel] = 127;
	motor[RFWheel] = 127;
	motor[RRWheel] = 127;
	}else{
	motor[LFWheel] = -127;
	motor[LRWheel] = -127;
	motor[RFWheel] = -127;
	motor[RRWheel] = -127;
	}
	nMotorEncoder[LFWheel] = 0;
	while(abs(nMotorEncoder[LFWheel])/627.2<distance/(4.0*PI)){
		wait1Msec(10);
	}

	locX+=cosDegrees(rotation)*distance;
	locY+=sinDegrees(rotation)*distance;
	freeze();
}

void rightTurn (int angle)
{
	motor[LFWheel] = 127;
	motor[LRWheel] = 127;
	motor[RFWheel] = -127;
	motor[RRWheel] = -127;

	nMotorEncoder[LFWheel] = 0;
	while(nMotorEncoder[LFWheel]/627.2<(3.125*(angle/360.0))){
		wait1Msec(10);
	}

	rotation = (rotation + angle) % 360;
	freeze();
}

void leftTurn (int angle)
{
	motor[LFWheel] = -127;
	motor[LRWheel] = -127;
	motor[RFWheel] = 127;
	motor[RRWheel] = 127;

	nMotorEncoder[LFWheel] = 0;
	while(abs(nMotorEncoder[LFWheel])/627.2<(3.125*(angle/360.0))){
		wait1Msec(10);
	}

	rotation = (rotation - angle) % 360;
	freeze();
}

void fly(int speedElement)
{
	if(!((speedElement < 0) || (speedElement > 4)))
	{
		pidRequestedValue = Presets[speedElement];
		
		if(speedElement != 0)
			pidRunning = 1;
		else
			pidRunning = 0;
	}
}

void adjustRight()
{
	motor[LFWheel] = 30;
	motor[LRWheel] = 30;
	motor[RFWheel] = -30;
	motor[RRWheel] = -30;
	/*while(true){
		if(SensorValue(lineTracker)<2850){
		freeze();
		break;
		}*/
}

void adjustLeft()
{
	motor[LFWheel] = -30;
	motor[LRWheel] = -30;
	motor[RFWheel] = 30;
	motor[RRWheel] = 30;
	/*while(true){
		if(SensorValue(lineTracker)<2850){
		freeze();
		break;
	}*/
}

void intake(bool on)
{
	motor[Feeder] = (on) ? (127) : (0);
}

task pidControl()
{
	float  pidSensorCurrentValue;
    float  pidError;
    float  pidLastError;
    float  pidIntegral;
    float  pidDerivative;
    float  pidDrive;

    // If we are using an encoder then clear it
    if( SensorType[ PID_SENSOR_INDEX ] == sensorQuadEncoderOnI2CPort )
        SensorValue[ PID_SENSOR_INDEX ] = 0;

    // Init the variables
    pidLastError  = 0;
    pidIntegral   = 0;

    while( true )
    {
		// Is PID control active ?
		if( pidRunning )
		{
			// Read the sensor value and scale
			pidSensorCurrentValue = SensorValue[ PID_SENSOR_INDEX ] * PID_SENSOR_SCALE;

			// calculate error
			pidError = pidSensorCurrentValue - pidRequestedValue;

			// integral - if Ki is not 0
			if( pid_Ki != 0 )
			{
				// If we are inside controllable window then integrate the error
				if( abs(pidError) < PID_INTEGRAL_LIMIT )
					pidIntegral = pidIntegral + pidError;
				else
					pidIntegral = 0;
			}
			else
				pidIntegral = 0;

			// calculate the derivative
			pidDerivative = pidError - pidLastError;
			pidLastError  = pidError;

			// calculate drive
			pidDrive = (pid_Kp * pidError) + (pid_Ki * pidIntegral) + (pid_Kd * pidDerivative);

			// limit drive
			if( pidDrive > PID_DRIVE_MAX )
				pidDrive = PID_DRIVE_MAX;
			if( pidDrive < PID_DRIVE_MIN )
				pidDrive = PID_DRIVE_MIN;

			// send to motor
			motor[ PID_MOTOR_INDEX ] = pidDrive * PID_MOTOR_SCALE;
			motor[RFFlywheel] = motor[PID_MOTOR_INDEX];
			motor[RRFlywheel] = motor[PID_MOTOR_INDEX];
			motor[LRFlywheel] = motor[PID_MOTOR_INDEX];
			writeDebugStreamLine("pidDrive = %d", pidDrive);
			writeDebugStreamLine("pidSensorCurrentValue = %d", pidSensorCurrentValue);
			EndTimeSlice();
		}
		else
		{
			// clear all
			pidError      = 0;
			pidLastError  = 0;
			pidIntegral   = 0;
			pidDerivative = 0;
			motor[ PID_MOTOR_INDEX ] = 0;
			motor[RFFlywheel] = motor[PID_MOTOR_INDEX];
			motor[RRFlywheel] = motor[PID_MOTOR_INDEX];
			motor[LRFlywheel] = motor[PID_MOTOR_INDEX];
		}
   }
}
