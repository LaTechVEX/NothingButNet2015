#pragma config(I2C_Usage, I2C1, i2cSensors)
#pragma config(Sensor, I2C_1,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign)
#pragma config(Motor,  port1,           LFWheel,       tmotorVex393_HBridge, openLoop)
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



void move(int distance)
{
	motor[LFWheel] = 127;
	motor[LRWheel] = 127;
	motor[RFWheel] = 127;
	motor[RRWheel] = 127;

	nMotorEncoder[LFWheel] = 0;
	//4pi inches is the distance traveled per revolution
	while(nMotorEncoder[LFWheel]/627.2<distance/(4.0*PI)){
		wait1Msec(10);
	}

	//basic trig
	locX+=cosDegrees(rotation)*distance;
	locY+=sinDegrees(rotation)*distance;

	freeze();
}

void leftTurn (int angle)
{

	motor[LFWheel] = 127;
	motor[LRWheel] = 127;
	motor[RFWheel] = -127;
	motor[RRWheel] = -127;

	nMotorEncoder[LFWheel] = 0;
	//3.125 is derived from the inches per revolution (4pi)
	//divided by the circumference of the circle we need to make (~14pi)
	while(nMotorEncoder[LFWheel]/627.2<(3.125*(angle/360.0))){
		wait1Msec(10);
	}

	rotation = (rotation + angle) % 360;
	freeze();
}

void rightTurn (int angle)
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
		motor[REF] = Presets[speedElement];
		motor[RF] = Presets[speedElement];
		motor[LF] = Presets[speedElement];
	}
}




void intake(int speed)
{
	motor[I1] = speed;
	motor[I2] = speed;

}


void rest()//all motors on robot stop
{
	freeze();
	fly(0);
	intake(0);
}

task main()
{
	while(true)
	{
		if(vexRT(Btn5D))
		{
			leftTurn(45);
			move(24);
		  rightTurn(45);
			intake(127);
			fly(4);
			move(24);
		}
	}
}
