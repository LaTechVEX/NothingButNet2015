#pragma config(I2C_Usage, I2C1, i2cSensors)
#pragma config(Sensor, I2C_1,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign)
#pragma config(Motor,  port2,           InsideMotor,   tmotorVex393_MC29, openLoop, encoderPort, I2C_1)
#pragma config(Motor,  port3,           OutsideMotor,  tmotorVex393_MC29, openLoop, reversed)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

task main()
{
	// Variables
	long encnow = 0;
	long encprevious = 0;
	float rpmconversion = 21*((60.0)/392.0);

	// Reset the encoder value to zero
	resetMotorEncoder(InsideMotor);
	SensorValue[I2C_1] = 0;

	// Clear the debug window
	clearDebugStream();

	while(true)
	{
		// Iterate through the different power levels 0 to -128
		for(int i = -1; i >= -128; i--)
    {
    	// Set the motor powers
			motor[InsideMotor] = i;
			motor[OutsideMotor] = i;

			// Wait one second and then get the encoder value
			wait(1, seconds);
			encnow = SensorValue[I2C_1];

			// Read the encoder value and output the sensor value and rpm calculation
			writeDebugStreamLine("%d\t%d\t%f", i, encnow, (rpmconversion*encnow));

			// Reset the ticks of the sensor
			SensorValue[I2C_1] = 0;
		}

		// Iterate through the different power levels, 0 to 127
		for(int i = 0; i <= 127; i++)
    {
    	// Set the motor powers
			motor[InsideMotor] = i;
			motor[OutsideMotor] = i;

			// Wait one second and then get the encoder value
			wait(1, seconds);
			encnow = SensorValue[I2C_1];

			// Read the encoder value and output the sensor value and rpm calculation
			writeDebugStreamLine("%d\t%d\t%f", i, encnow, (rpmconversion*encnow));

			// Reset the ticks of the sensor
			SensorValue[I2C_1] = 0;
		}
	}

}
