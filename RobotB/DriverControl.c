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
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

task main()
{

	// Set the motors initially
	int Presets[4] = {0, 55, 85, 127};
	int currentPreset = 0;
	motor[REF] = Presets[currentPreset];
	motor[RF] = motor[REF];
	motor[LF] = motor[RF];

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

	  motor[REF] = Presets[currentPreset];
	  motor[RF] = motor[REF];
		motor[LF] = motor[RF];

		// Fit the input to an exponential curve
		// Squares are always positive, so a ternary operator is needed for negative numbers
		// The abs is just in case we decide to go with odd exponents or even roots (like 3/2)
		int verticalL = pow(abs(vexRT(Ch3))/127.0,2.0)*127.0 *((vexRT(Ch3)>0)?(1):(-1));
		int verticalR = pow(abs(vexRT(Ch2))/127.0,2.0)*127.0 *((vexRT(Ch2)>0)?(1):(-1));

		// Move Robot
	  motor[LRW] = verticalL;
	  motor[LFW] = motor[LRW];
    motor[RRW] = verticalR;
    motor[RFW] = motor[REF];

	  if(vexRT(Btn6U))
	     motor[I1] = 127;
	  else if(vexRT(Btn5U))
			 motor[I1] = -127;
		else if (vexRT(Btn5U))
	 	   motor[I1] = 0;

	 	motor[I2] = motor[I1];
	}
}
