#pragma config(I2C_Usage, I2C1, i2cSensors)
#pragma config(Sensor, I2C_1,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Motor,  port1,           LFWheel,       tmotorVex393_HBridge, openLoop, encoderPort, I2C_1)
#pragma config(Motor,  port2,           RFWheel,       tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port3,           RRWheel,       tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port4,           LRWheel,       tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port5,           REF,           tmotorVex393HighSpeed_MC29, openLoop, encoderPort, I2C_1)
#pragma config(Motor,  port6,           RF,            tmotorVex393HighSpeed_MC29, openLoop, reversed)
#pragma config(Motor,  port7,           LF,            tmotorVex393HighSpeed_MC29, openLoop)
#pragma config(Motor,  port8,           I1,            tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port9,           I2,            tmotorVex393_MC29, openLoop, reversed)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

int Presets[4] = {0,55, 85,127};

//These variables track the robots position and orientation
//Useful for any future decisions to veer off track or calculate routes
float locX;
float locY;
float rotation;

//Distance is in inches
//Wheels are 4 inches, so one complete rotation moves the bot 4 inches




void freeze()//for the wheels
{
	motor[LFWheel] = 0;
	motor[LRWheel] = 0;
	motor[RFWheel] = 0;
	motor[RRWheel] = 0;
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
	while(nMotorEncoder[LFWheel]/627.2<(3.0*(angle/360.0))){
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
	while(abs(nMotorEncoder[LFWheel])/627.2<(3.0*(angle/360.0))){
		wait1Msec(10);
	}

	rotation = (rotation - angle) % 360;
	freeze();
}





void fly(int speedElement)
{
	if(!((speedElement < 0) || (speedElement > 4)))
	{
		motor[RF] = Presets[speedElement];
		motor[REF] = Presets[speedElement];
		motor[LF] = Presets[speedElement];
	}
}

/*
void adjustRight(){

	motor[LFWheel] = 30;
	motor[LRWheel] = 30;
	motor[RFWheel] = -30;
	motor[RRWheel] = -30;
	while(true){
		if(SensorValue(lineTracker)<2850){
		freeze();
		break;
		}
	}

}

void adjustLeft(){

	motor[LFWheel] = -30;
	motor[LRWheel] = -30;
	motor[RFWheel] = 30;
	motor[RRWheel] = 30;
	while(true){
		if(SensorValue(lineTracker)<2850){
		freeze();
		break;
		}
	}
}
*/


void intake(bool on)
{
	motor[I2] = motor[I1] = (on) ? (127) : (0);

}


void rest()//all motors on robot stop
{
	freeze();
	fly(0);
	intake(false);
}

task main()
{
	while(true)
	{
		if(VexRT(Btn5U){
		leftTurn(1);
		wait10Msec(100);
		}
		if(VexRT(Btn6U){
		rightTurn(1);
		wait10Msec(100);
		}
		if(VexRT(Btn8U){
		move(2);
		wait10Msec(100);
		}
		if(VexRT(Btn8D){
		move(-2);
		wait10Msec(100);
		}

		if(VexRT(Btn7U)){
		rightTurn(360);
		}


		if(vexRT(Btn8R)){
		rightTurn(2);
		move(20);
		leftTurn(10);
		move(30);
		intake(true);
		fly(3);
		wait10Msec(1000);
		rest();
		}
	}
}
