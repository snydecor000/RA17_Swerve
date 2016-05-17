#include "WPILib.h"
#include <iostream>
#include <string>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <ctime>
#include <cstdlib>
#include <cmath>
#include "Logger.h"
#include "Config.h"
#include "Gamepad.h"
#include "Drive.h"

using namespace std;

class Robot: public IterativeRobot
{
private:
	LiveWindow *lw = LiveWindow::GetInstance();
	CANTalon * motorFR;
	CANTalon * motorFL;
	CANTalon * motorBL;
	CANTalon * motorBR;
	CANTalon * angleFR;
	CANTalon * angleFL;
	CANTalon * angleBL;
	CANTalon * angleBR;
	AnalogInput * absEncFR;
	AnalogInput * absEncFL;
	AnalogInput * absEncBL;
	AnalogInput * absEncBR;
	AnalogGyro * gyro;
	BuiltInAccelerometer * acceler;
	Logger * logger;
	Timer * timer;
	Drive * drive;
	Gamepad * driver;
public:
	Robot()
	{
		motorFR = NULL;
		motorFL = NULL;
		motorBL = NULL;
		motorBR = NULL;
		angleFR = NULL;
		angleFL = NULL;
		angleBL = NULL;
		angleBR = NULL;
		absEncFR = NULL;
		absEncFL = NULL;
		absEncBL = NULL;
		absEncBR = NULL;
		gyro = NULL;
		acceler = NULL;
		logger = NULL;
		timer = NULL;
		drive = NULL;
		driver = NULL;
	}

	void RobotInit()
	{
		motorFR = new CANTalon(3);
		motorFL = new CANTalon(4);
		motorBL = new CANTalon(5);
		motorBR = new CANTalon(6);
		angleFR = new CANTalon(7);
		angleFL = new CANTalon(8);
		angleBL = new CANTalon(9);
		angleBR = new CANTalon(10);
		////////////////////////////////////////////////
		absEncFR = new AnalogInput(0);
		absEncFL = new AnalogInput(1);
		absEncBL = new AnalogInput(2);
		absEncBR = new AnalogInput(3);
		////////////////////////////////////////////////
		gyro = new AnalogGyro(4);
		gyro->Reset();
		gyro->Calibrate();
		acceler = new BuiltInAccelerometer();
		////////////////////////////////////////////////
		logger = new Logger();
		timer = new Timer();
		////////////////////////////////////////////////
		drive = new Drive(motorFR,motorBR,motorFL,motorBL,
						angleFR,angleBR,angleFL,angleBL,
						absEncFR,absEncBR,absEncFL,absEncBL);
		////////////////////////////////////////////////
		driver = new Gamepad(0);

		ReloadConfig();
	}

	void AutonomousInit()
	{

	}

	void AutonomousPeriodic()
	{

	}

	void TeleopInit()
	{
		ReloadConfig();
		timer->Reset();
		timer->Start();
		StartLogging("Teleop", logger);
		SetupLogging();
		gyro->Reset();
	}

	void TeleopPeriodic()
	{
		drive->Swerve(driver->GetRightX(),driver->GetRightY(),driver->GetLeftX(),gyro->GetAngle());
		Log(timer->Get());
	}

	void DisabledInit()
	{
		logger->Close();
	}

	void DisabledPeriodic()
	{

	}

	void TestPeriodic()
	{
		lw->Run();
	}

	void StartLogging(string mode, Logger * l)
	{
		string robot = !(Config::GetSetting("isPrototype", 0) == 0) ? "_proto" : "_comp";
		l->Close();
		time_t t = time(0);
		struct tm *now = localtime(&t);
		//don't touch it
		std::string dir = "/home/lvuser";
		struct stat st;
		if(stat("/media/sda",&st) == 0)
		{
		    if(st.st_mode & (S_IFDIR != 0))
		    {
		    	dir = "/media/sda";
		    }
		}
		std::string name = dir + "/log-" +
				std::to_string(now->tm_year + 1900) +
				"-\0" +
				std::to_string(now->tm_mon + 1) +
				"-\0" + std::to_string(now->tm_mday) + "_\0" +
				std::to_string(now->tm_hour) + "-\0" +
				std::to_string(now->tm_min) + "-\0" + std::to_string(now->tm_sec) + "-\0" + mode + robot + ".csv";
		cout << name << endl;
		l->Open(name);
	}

	void SetupLogging()
	{
		logger->AddAttribute("Time");
		logger->AddAttribute("AccX");
		logger->AddAttribute("AccY");
		logger->AddAttribute("AccZ");
		drive->SetupLogging(logger);
		logger->WriteAttributes();
	}

	void Log(float time)
	{
		logger->Log("Time", time);
		logger->Log("AccX", acceler->GetX());
		logger->Log("AccY", acceler->GetY());
		logger->Log("AccZ", acceler->GetZ());
		drive->Log(logger);
		logger->WriteLine();
	}

	void ReloadConfig()
	{
		Config::LoadFromFile("/home/lvuser/config.txt");
		drive->ReloadConfig();
	}
};

START_ROBOT_CLASS(Robot)
