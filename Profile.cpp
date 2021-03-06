/*
 * Profile.cpp
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
#include "Profile.h"
#include "PID.h"

Profile::Profile()
{
	StepCount = 0;
	StepNDX = 0;
	LastDistance = 0;
	MoveStartHeading = 0;
	OutputMagnitude = 0;
	Curve = 0;
	SteerKp = 0.01;
	SteerKi = 0.0;
	SteerKd = 0.0;
	TurnKp = 0.05;
	TurnKi = 0.00;
	TurnKd = 0.00;
	TurnPID.Initialize(&TurnKp,&TurnKi,&TurnKd);
	SteerPID.Initialize(&SteerKp,&SteerKi,&SteerKd);
}

void Profile::ExecuteProfile(double heading, double distance)
{
	double curDistance = 0;
	double curError = 0;
	if(StepCount > 0)
	{
		curDistance = distance - LastDistance;
		switch((int)Steps[StepNDX][0]) //evaluate command
		{
			//MOVE - drive straight for given distance
			// 0 = 1 = MOVE Command
			// 1 = speed (also determines forward/backward)
			// 2 = distance
			// 3 = not used
			// 4 = not used
			case 1:
			{
				if(!States[StepNDX][0]) //Start Flag
				{
					States[StepNDX][0] = true;
					LastDistance = distance;
					curDistance = distance - LastDistance;
					MoveStartHeading = heading;
					printf("MOVE %i Start -  Tgt: %5.2f\n",StepNDX,Steps[StepNDX][2]);
					printf("MOVE %i Start - Dist: %5.2f\n",StepNDX,curDistance);
					printf("MOVE %i StartHeading: %5.2f\n",StepNDX,MoveStartHeading);
				}
				if(curDistance < Steps[StepNDX][2])
				{
					curError = GetNormalizedError(heading,MoveStartHeading);
					Curve = Clamp(SteerPID.Update(0.0,curError)) * -1;//GetSteerRate(curError,1.0);
					OutputMagnitude = Steps[StepNDX][1];
				}
				else
				{
					States[StepNDX][1] = true; //Done Flag
					printf("MOVE %i Done - Dist: %5.2f\n",StepNDX,curDistance);
					printf("MOVE %i EndHeading: %5.2f\n",StepNDX,heading);
					Curve = 0.0;
					OutputMagnitude = 0.0;
					StepNDX++;
				}
				break;
			}
			//TURN - turn to new heading
			// 0 = 2 = TURN Command
			// 1 = speed
			// 2 = new heading
			// 3 = not used
			// 4 = not used
			case 2:
			{
				if(!States[StepNDX][0]) //Start Flag
				{
					States[StepNDX][0] = true;
					printf("TURN %i Start -   Tgt: %5.2f\n",StepNDX,Steps[StepNDX][2]);
					printf("TURN %i Start - Angle: %5.2f\n",StepNDX,heading);
				}
				curError = GetNormalizedError(heading,Steps[StepNDX][2]);
				Curve = Clamp(TurnPID.Update(0.0,curError)) * -1;
				OutputMagnitude = Steps[StepNDX][1];
				//quit the step when we get close to target angle
				if(abs(curError) < 1)
				{
					States[StepNDX][1] = true; //Done Flag
					printf("Step %i Done - Angle: %5.2f\n",StepNDX,heading);
					Curve = 0.0;
					OutputMagnitude = 0.0;
					StepNDX++;
				}
				break;
			}
			default:
			{
				Curve = 0.0;
				OutputMagnitude = 0.0;
			}
		}

	}
	else
	{	//do nothing
		Curve = 0.0;
		OutputMagnitude = 0.0;
	}

}

int Profile::ReadProfile(std::string fname)
{
	std::string line,cmd,p1,p2,p3,p4;
	std::ifstream inf;
	int i=0;

	ProfileLoaded = false;
	OutputMagnitude = 0;
	Curve = 0;
	//zero the Steps and States arrays
	for(i=0; i<20; i++)
	{
		Steps[i][0] = 0.0;
		Steps[i][1] = 0.0;
		Steps[i][2] = 0.0;
		Steps[i][3] = 0.0;
		Steps[i][4] = 0.0;
		States[i][0] = false;
		States[i][1] = false;
	}
	i=0;
	//  /home/lvuser  for internal drive
	//  /media/sda1 or /u or /U   for USB flash drive
	inf.open(fname);
	if(!inf)
	{
		printf("Failed to open file: %s\n",fname.c_str());
		return 0;
	}
	while (getline(inf, line))
	{
		if(i < PROFILE_MAX_STEPS)
		{
			std::stringstream linestream(line);
			getline(linestream, cmd, ',');
			getline(linestream, p1, ',');
			getline(linestream, p2, ',');
			getline(linestream, p3, ',');
			getline(linestream, p4, '\n');
			Steps[i][0] = strtod(cmd.c_str(),NULL);
			Steps[i][1] = strtod(p1.c_str(),NULL);
			Steps[i][2] = strtod(p2.c_str(),NULL);
			Steps[i][3] = strtod(p3.c_str(),NULL);
			Steps[i][4] = strtod(p4.c_str(),NULL);
			printf("PROFILE Step%i: %5.2f,%5.2f,%5.2f,%5.2f\n",(int)Steps[i][0],Steps[i][1],Steps[i][2],Steps[i][3],Steps[i][4]);
			i++;
		}
	}
	inf.close();
	ProfileLoaded = true;
	StepCount = i-1;
	return StepCount;
}

double Profile::GetNormalizedError(double heading, double newHeading)
{
	double rawError = newHeading - heading;

	if(rawError > 180) return rawError -= 360;
	else if(rawError < -180) return rawError += 360;
	else return rawError;
}

double Profile::Clamp(double steerRate)
{
	double steerClamp = steerRate;
	if(steerClamp < -1.0) steerClamp = -1.0;
	if(steerClamp > 1.0) steerClamp = 1.0;
	return steerClamp;
}




