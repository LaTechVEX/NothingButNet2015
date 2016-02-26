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
#pragma userControlDuration(75)
#include "Vex_Competition_Includes.c"

/////////////////////////////////////////////////////////////////////////////////////////
//                       				 Function Stubs
/////////////////////////////////////////////////////////////////////////////////////////

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

// **** VERY IMPORTANT VALUES ****
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
		fly(currentPreset);

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
	int currentPreset = 0;
	fly(currentPreset);

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

		// If Btn6D is pressed, initiate a burst of flywheel speed to recover from shooting a ball
		if(vexRT(Btn6D))
			fly(10);

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
		motor[LFFlywheel] = Presets[speedElement];
		motor[RFFlywheel] = motor[LFFlywheel];
		motor[RRFlywheel] = motor[LFFlywheel];
		motor[LRFlywheel] = motor[LFFlywheel];
	}
	// Initiate a burst of speed to catch the flywheel back up from shooting
	else if (speedElement == 10)
	{
		motor[LFFlywheel] = Presets[speedElement] + 20;
		motor[RFFlywheel] = motor[LFFlywheel];
		motor[RRFlywheel] = motor[LFFlywheel];
		motor[LRFlywheel] = motor[LFFlywheel];
	}
	else
	{
		motor[LFFlywheel] = 0;
		motor[RFFlywheel] = motor[LFFlywheel];
		motor[RRFlywheel] = motor[LFFlywheel];
		motor[LRFlywheel] = motor[LFFlywheel];
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
