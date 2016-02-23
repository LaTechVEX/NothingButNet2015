#pragma config(I2C_Usage, I2C1, i2cSensors)
#pragma config(Sensor, I2C_1,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign)
#pragma config(Motor,  port1,           LFW,           tmotorVex393_HBridge, openLoop)
#pragma config(Motor,  port2,           RFW,           tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port3,           RRW,           tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port4,           LRW,           tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port5,           REF,           tmotorVex393HighSpeed_MC29, openLoop, encoderPort, I2C_1)
#pragma config(Motor,  port6,           RF,            tmotorVex393HighSpeed_MC29, openLoop, reversed)
#pragma config(Motor,  port7,           LF,            tmotorVex393HighSpeed_MC29, openLoop)
#pragma config(Motor,  port8,           I1,            tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port9,           I2,            tmotorVex393_MC29, openLoop, reversed)
#pragma platform(VEX)

// Competition Control and Duration Settings
#pragma competitionControl(Competition)
#pragma autonomousDuration(45)
#pragma userControlDuration(115)
#include "Vex_Competition_Includes.c"

// PID Control Definitions
#define PID_SENSOR_INDEX     I2C_1
#define PID_SENSOR_SCALE     1
#define PID_MOTOR_INDEX      REF
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

// **** PLEASE DO NOT TOUCH THIS SECTION ****
float pid_Kp = 0.469;
float pid_Ki = 0.382;
float pid_Kd = 0.038;
static int pidRunning = 1;
static float pidRequestedValue;
int Presets[4] = {0, 118, 148, 200};
// **** PLEASE DO NOT TOUCH THIS SECTION ****

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
	{/*
		// Set the motors initially
		int currentPreset = 0;
		fly(0);
		startTask(pidControl);

		if(vexRT(Btn5U))
		{
			leftTurn(1);
			wait10Msec(100);
		}
		if(vexRT(Btn6U))
		{
			rightTurn(1);
			wait10Msec(100);
		}
		if(vexRT(Btn8U))
		{
			move(2);
			wait10Msec(100);
		}
		if(vexRT(Btn8D))
		{
			move(-2);
			wait10Msec(100);
		}

		if(vexRT(Btn7U))
		{
			rightTurn(360);
		}

		if(vexRT(Btn8R))
		{
			rightTurn(2);
			move(20);
			leftTurn(10);
			move(30);
			intake(true);
			fly(3);
			wait10Msec(1000);
			rest();
		}*/
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
		// Flywheel Speed
		if(vexRT(Btn8U))
			currentPreset = 0;
		else if (vexRT(Btn8R))
			currentPreset = 1;
		else if (vexRT(Btn8D))
			currentPreset = 2;
		else if (vexRT(Btn8L))
			currentPreset = 3;

		// Set the flywheel speed
		fly(currentPreset);

		// Fit the input to an exponential curve
		// Squares are always positive, so a ternary operator is needed for negative numbers
		// The abs is just in case we decide to go with odd exponents or even roots (like 3/2)
		// We devide by 128 because the program can theoretically reach -128
		int verticalL = pow(abs(vexRT(Ch3))/128.0,2.7)*127.0 *((vexRT(Ch3)>0)?(1):(-1));
		int verticalR = pow(abs(vexRT(Ch2))/128.0,2.7)*127.0 *((vexRT(Ch2)>0)?(1):(-1));

		// Move Robot
		motor[LRW] = verticalL;
		motor[LFW] = motor[LRW];
		motor[RRW] = verticalR;
		motor[RFW] = motor[RRW];

		//Feeder control
		if(vexRT(Btn6U))
			motor[I1] = 127;
		else if(vexRT(Btn5U))
			motor[I1] = -127;
		else
	 	   motor[I1] = 0;
		motor[I2] = motor[I1];
	}
}




/////////////////////////////////////////////////////////////////////////////////////////
//                                 User Defined Functions
/////////////////////////////////////////////////////////////////////////////////////////

// Stop all wheels
void freeze()
{
	motor[LFW] = 0;
	motor[LRW] = 0;
	motor[RFW] = 0;
	motor[RRW] = 0;
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
	if(dist>0)
	{
		motor[LFW] = 127;
		motor[LRW] = 127;
		motor[RFW] = 127;
		motor[RRW] = 127;
	}
	else
	{
		motor[LFW] = -127;
		motor[LRW] = -127;
		motor[RFW] = -127;
		motor[RRW] = -127;
	}
	nMotorEncoder[LFW] = 0;
	while(abs(nMotorEncoder[LFW])/627.2<distance/(4.0*PI))
	{
		wait1Msec(10);
	}
	locX+=cosDegrees(rotation)*distance;
	locY+=sinDegrees(rotation)*distance;
	freeze();
}

void rightTurn (int angle)
{
	motor[LFW] = 127;
	motor[LRW] = 127;
	motor[RFW] = -127;
	motor[RRW] = -127;

	nMotorEncoder[LFW] = 0;
	while(nMotorEncoder[LFW]/627.2<(3.125*(angle/360.0)))
	{
		wait1Msec(10);
	}

	rotation = (rotation + angle) % 360;
	freeze();
}

void leftTurn (int angle)
{
	motor[LFW] = -127;
	motor[LRW] = -127;
	motor[RFW] = 127;
	motor[RRW] = 127;

	nMotorEncoder[LFW] = 0;
	while(abs(nMotorEncoder[LFW])/627.2<(3.125*(angle/360.0)))
	{
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
	motor[LFW] = 30;
	motor[LRW] = 30;
	motor[RFW] = -30;
	motor[RRW] = -30;
	/*while(true){
		if(SensorValue(lineTracker)<2850){
		freeze();
		break;
		}*/
}

void adjustLeft()
{
	motor[LFW] = -30;
	motor[LRW] = -30;
	motor[RFW] = 30;
	motor[RRW] = 30;
	/*while(true){
		if(SensorValue(lineTracker)<2850){
		freeze();
		break;
	}*/
}

void intake(bool on)
{
	motor[I1] = (on) ? (127) : (0);
	motor[I2] = motor[I1];
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

			// Send to motor
			motor[ PID_MOTOR_INDEX ] = pidDrive * PID_MOTOR_SCALE;
			motor[RF] = motor[PID_MOTOR_INDEX];
			motor[LF] = motor[PID_MOTOR_INDEX];
			writeDebugStreamLine("pidDrive = %d", pidDrive);
			writeDebugStreamLine("pidSensorCurrentValue = %d", pidSensorCurrentValue);
			EndTimeSlice();
		}
		else
		{
			// Clear All
			pidError      = 0;
			pidLastError  = 0;
			pidIntegral   = 0;
			pidDerivative = 0;
			
			// Stop Flywheel
			motor[ PID_MOTOR_INDEX ] = 0;
			motor[RF] = motor[PID_MOTOR_INDEX];
			motor[LF] = motor[PID_MOTOR_INDEX];
		}
   }
}
