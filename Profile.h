/*
 * Profile.h
 *
 *  Created on: Oct 13, 2016
 *      Author: Chester Marshall, mentor for 6055 and 5721
 *
 *  This library can read a set of movement commands from a file (either on the
 *  internal flash drive or a thumb drive) and execute the commands to drive the
 *  robot in a pre-set pattern.  This is most useful for autonomous mode.  Right now,
 *  only two commands are implemented: MOVE and TURN.  Feel free to add more.
 *
 */

#ifndef Profile_h
#define Profile_h

#include "PID.h"
#include "stdlib.h"
#include <string>
#include <fstream>
#include <sstream>

#define PROFILE_MAX_STEPS 20
#define PROFILE_MAX_PARAMS 5

class Profile
{
private:
	double Steps[PROFILE_MAX_STEPS][PROFILE_MAX_PARAMS];
	bool States[PROFILE_MAX_STEPS][2];
	int StepCount;
	int StepNDX;
	double LastDistance;
	double MoveStartHeading;

	double GetNormalizedError(double heading, double newHeading);
	double Clamp(double steerRate);
public:
	bool ProfileLoaded = false;
	double SteerKp = 0.01;
	double SteerKi = 0.00;
	double SteerKd = 0.00;
	double TurnKp = 0.05;
	double TurnKi = 0.00;
	double TurnKd = 0.00;
	PID TurnPID;
	PID SteerPID;
	float OutputMagnitude;
	float Curve;

    Profile();
    //call this in AutonomousInit with filename of profile
    int ReadProfile(std::string fname);
    //call this repeatedly in AutonomousPeriodic
    //then set .Drive method with Profile.OutputMagnitude,Profile.Curve
    void ExecuteProfile(double heading, double distance);

};

#endif
