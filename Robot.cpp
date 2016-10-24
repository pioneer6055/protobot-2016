#include "WPILib.h"
#include "BNO055.h"
#include "PID.h"
#include "Pixy.h"
#include "Profile.h"

/*
 *
 * encoder: 2797 cpr @ 4x, 699.25 @ 1x
 *           GRE = GND			= DIO 0-GND
 * 			 BLU = VDC			= DIO 0-V
 * 			 YEL = A channel	= DIO 0-S
 * 			 GRA = B channel	= DIO 1-S
 *
 * gyro:
 * 			VIN - I2C 3.3V
 * 			GND - I2C GND
 * 			SDA - I2C SDA
 * 			SCL - I2C SCL
 *
 */


class Robot: public IterativeRobot
{
private:
	Joystick Xbox;
	Victor MotorLeft,MotorRight;
	RobotDrive DriveTrain;
	Encoder LeftEncoder;
	BNO055 gyro;
	SendableChooser *chooser;
	const std::string autoDoNothing = "DoNothing";
	const std::string autoDriveStraight = "DriveStraight";
	const std::string autoDogleg = "Dogleg-Right";
	const std::string autoSquare = "Square-Right";
	const std::string autoPixy = "PixySeek";
	Preferences *prefs;
	Pixy PixyCam;
	int PixyX = 0;
	int PixyY = 0;
	int PixyWidth = 0;
	int PixyHeight = 0;
	Profile AutoProfile;
	double SteerKp = 0.01;
	std::string autoSelected = "";
public:
	Robot() :
		Xbox(0),
		MotorLeft(0),
		MotorRight(1),
		DriveTrain(MotorLeft,MotorRight),
		LeftEncoder(0, 1, true, CounterBase:: k1X),  //DIO 0 and 1 channels
		gyro(),
		chooser(),
		prefs(),
		PixyCam()
	{}

	void RobotInit()
	{
		DriveTrain.SetExpiration(0.25);
		DriveTrain.SetSafetyEnabled(true);
		MotorLeft.SetInverted(true);
		MotorRight.SetInverted(true);

		chooser = new SendableChooser();
		chooser->AddDefault(autoDoNothing, (void*)&autoDoNothing);
		chooser->AddObject(autoDriveStraight, (void*)&autoDriveStraight);
		chooser->AddObject(autoDogleg, (void*)&autoDogleg);
		chooser->AddObject(autoSquare, (void*)&autoSquare);
		chooser->AddObject(autoPixy, (void*)&autoPixy);
		SmartDashboard::PutData("Auto Mode", chooser);

		prefs = Preferences::GetInstance();
		if (!prefs->ContainsKey("DistancePerPulse"))
		prefs->PutDouble("DistancePerPulse",0.0184);
		LeftEncoder.SetDistancePerPulse(prefs->GetDouble("DistancePerPulse"));
		//CameraServer::GetInstance()->SetQuality(25);
		//CameraServer::GetInstance()->StartAutomaticCapture("cam0");
		printf("RobotInit Finished\n");
	}

	void AutonomousInit()
	{
		gyro.ZeroHeading(); //reset current direction to 0 degrees
		autoSelected = *((std::string*)chooser->GetSelected());
		if(autoSelected == autoDriveStraight) AutoProfile.ReadProfile("/u/drivestraight.txt");
		if(autoSelected == autoDogleg) AutoProfile.ReadProfile("/u/dogleg-right.txt");
		if(autoSelected == autoSquare) AutoProfile.ReadProfile("/u/square-right.txt");
	}

	uint16_t filter(uint16_t raw, uint16_t current, double lpf)
	{
		return (uint16_t)(lpf * raw) + ((1-lpf) * current);
	}

	void AutonomousPeriodic()
	{
		uint16_t count;
		double heading = 0;
		double distance = 0;
		double Yerror = 0;
		double Xerror = 0;

		if(autoSelected == autoPixy)
		{
			count = PixyCam.getBlocks(1);
			if(count > 0)
			{
				PixyX = filter(PixyCam.blocks[0].x,PixyX,0.7);
				PixyY = filter(PixyCam.blocks[0].y,PixyY,0.7);
				PixyWidth = filter(PixyCam.blocks[0].width,PixyWidth,0.7);
				PixyHeight = filter(PixyCam.blocks[0].height,PixyHeight,0.7);
				SmartDashboard::PutNumber("pixy.X",PixyX);
				SmartDashboard::PutNumber("pixy.Y",PixyY);
				SmartDashboard::PutNumber("pixy.Width",PixyWidth);
				SmartDashboard::PutNumber("pixy.Height",PixyHeight);
			}
			Xerror = 160 - PixyX;
			SmartDashboard::PutNumber("PixyX_Error",Xerror);
			Yerror = 100 - PixyY;
			SmartDashboard::PutNumber("pixyY_Error",Yerror);
			DriveTrain.Drive(Yerror/100,(Xerror/160)*-1);
			SmartDashboard::PutNumber("magnitude",Yerror/100);
			SmartDashboard::PutNumber("curve",(Xerror/160)*-1);
		}
		else
		{
			heading = gyro.GetHeading();
			distance = LeftEncoder.GetDistance();
			SmartDashboard::PutNumber("Heading",heading);
			SmartDashboard::PutNumber("Distance",distance);
			//execute autonomous profile read from file
			if(AutoProfile.ProfileLoaded)
			{
				AutoProfile.ExecuteProfile(heading,distance);
				DriveTrain.Drive(AutoProfile.OutputMagnitude,AutoProfile.Curve);
			}
			else
				DriveTrain.Drive(0.0,0.0);
		}
	}

	void TeleopInit()
	{
		LeftEncoder.SetDistancePerPulse(prefs->GetDouble("DistancePerPulse"));
		LeftEncoder.Reset();
		printf("Teleop Start Distance: %5.2f\n",LeftEncoder.GetDistance());
	}

	void TeleopPeriodic()
	{
		SmartDashboard::PutNumber("Heading",gyro.GetHeading());
		SmartDashboard::PutNumber("Distance",LeftEncoder.GetDistance());
		DriveTrain.TankDrive(Xbox.GetRawAxis(1),Xbox.GetRawAxis(5));
	}

	void DisabledInit()
	{
		DriveTrain.TankDrive(0.0,0.0);
	}

	void TestInit()
	{
	}

	void TestPeriodic()
	{
		SmartDashboard::PutNumber("Heading",gyro.GetHeading());
		SmartDashboard::PutNumber("Distance",LeftEncoder.GetDistance());
	}

};

START_ROBOT_CLASS(Robot)
