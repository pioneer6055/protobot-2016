/*
 * Code borrowed from Andrew Lobos, alumni of team #225
 * https://github.com/4ndr3w/BallBot/blob/master/BNO055.cpp
 * modified for roboRio - Chester Marshall 10/6/2016
 */

#include "BNO055.h"
#include "Timer.h"

BNO055::BNO055()
{
	i2c = new I2C(I2C::kOnboard, BNO055_ADDRESS);
	uint8_t buf = 0;
	while (buf!=BNO55_ID)
	{
		i2c->Read(0x00, 1, &buf);
		printf("BNO055: Waiting for gyro to appear...\n");
		Wait(0.2);
	}

	  printf("BNO055: Gyro is on I2C bus, resetting\n");

	  // Reset
	  i2c->Write(BNO055_OPR_MODE_ADDR, OPERATION_MODE_CONFIG);
	  Wait(0.5);

	  i2c->Write(BNO055_SYS_TRIGGER_ADDR, 0x20);

	  while (buf!=BNO55_ID)
	  {
		i2c->Read(0x00, 1, &buf);
		printf("BNO055: Waiting for gyro to reset...\n");
		Wait(0.3);
	  }

	  while (buf!=0xF)
	  {
		i2c->Read(BNO055_SELFTEST_RESULT_ADDR, 1, &buf);
		printf("BNO055: Waiting for gyro self-test... (%i)\n", buf);
		Wait(0.5);
	  }

	  i2c->Write(BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL);
	  Wait(0.05);

	  //  i2c->Write(BNO055_SYS_TRIGGER_ADDR, 0x80);
	  //  Wait(0.05);

	  printf("BNO055: Configuring units\n");

	  i2c->Write(BNO055_PAGE_ID_ADDR, 0);
	  Wait(0.05);

	  uint8_t unitsel = (0 << 7) |   // Orientation = Android
						  (0 << 4) | // Temperature = Celsius
						  (0 << 2) | // Euler = Degrees
						  (1 << 1) | // Gyro = Rads
						  (0 << 0);  // Accelerometer = m/s^2
	  i2c->Write(BNO055_UNIT_SEL_ADDR, unitsel);
	  Wait(0.05);


	  printf("BNO055: Requesting self-test\n");
	  i2c->Write(BNO055_SYS_TRIGGER_ADDR, 0x01);
	  Wait(0.05);


	  while (buf!=0xF)
	  {
		i2c->Read(BNO055_SELFTEST_RESULT_ADDR, 1, &buf);
		printf("BNO055: Waiting for gyro self-test... (%i)\n", buf);
		Wait(0.5);
	  }

	  i2c->Write(BNO055_OPR_MODE_ADDR, OPERATION_MODE_IMUPLUS);
	  Wait(0.05);

	  printf("BNO055: Gyro configured\n");

	  bool calFinished = false;
	  while ( !calFinished ) {
		printf("BNO055: Waiting for calibration\n");
		buf = 0;
		i2c->Read(BNO055_CALIB_STAT_ADDR, 1, &buf);
		printf("BNO055: system: %i gyro: %i acc: %i mag: %i\n", (buf>>6)&0x03, (buf>>4)&0x03, (buf>>2)&0x03, buf&0x03);
		if ( ((buf>>4)&0x03) == 3 )
		  calFinished = true;


		buf = 0;
		i2c->Read(BNO055_SYS_STAT_ADDR, 1, &buf);
		printf("BNO055: State: %i\n", buf);

		buf = 0;
		i2c->Read(BNO055_SELFTEST_RESULT_ADDR, 1, &buf);
		printf("BNO055: Self-Test: %i\n", buf);

		Wait(1.0);
	  }
	  Calibrated = true;
	  HeadingOffset = 0;
	  printf("BNO055: Gyro calibration finished\n");
}

double BNO055::GetYaw()
{
	uint8_t headingmsb = 0;
	uint8_t headinglsb = 0;
	i2c->Read(BNO055_EULER_H_LSB_ADDR, 1, (uint8_t*)&headinglsb);
	i2c->Read(BNO055_EULER_H_MSB_ADDR, 1, (uint8_t*)&headingmsb);

	uint16_t zi = ((int16_t)headinglsb) | (((int16_t)headingmsb) << 8);
	return ((double) zi)/16.0;
}

double BNO055::GetHeading()
{
	double realYaw = GetYaw();
	double offsetYaw = realYaw - HeadingOffset;
	if(offsetYaw > 180) offsetYaw -= 360;
	else if(offsetYaw < -180) offsetYaw  += 360;
	if(offsetYaw < 0) offsetYaw += 360;
	return offsetYaw;
}

bool BNO055::IsCalibrated()
{
	return Calibrated;
}

void BNO055::ZeroHeading()
{
	HeadingOffset = GetYaw();
}


