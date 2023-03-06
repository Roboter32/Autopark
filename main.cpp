#include <hFramework.h>
//#include "hCloudClient.h"
#include <stddef.h>
#include <stdio.h>
#include <math.h>
#include "/usr/lib/gcc/modules-master/include/DistanceSensor.h"

using namespace hFramework;
using namespace hModules;

bool btn;
bool buff_btn;
bool running = false;
int dir = 1;
int Power = 400;
float drivePos = 0;
float spaceLength = 0;
int steerOffset = 0;
int steerPos = 0;

bool steer(int pos)
{
	int i = 0;
	if (pos > hMot2.getEncoderCnt() - steerOffset)
	{
		while (hMot2.getEncoderCnt() < pos + steerOffset)
		{
			// printf("%d\n", hMot2.getEncoderCnt());
			hMot2.setPower(300);
			i++;
			if (i > 200)
				break;
			sys.delay(5);
		}
	}
	if (pos < hMot2.getEncoderCnt() - steerOffset)
	{
		while (hMot2.getEncoderCnt() > pos + steerOffset)
		{
			// printf("%d\n", hMot2.getEncoderCnt());
			hMot2.setPower(-300);
			i++;
			if (i > 200)
				break;
			sys.delay(5);
		}
	}

	hMot2.setPower(0);
	if (i > 200)
		return false;
	else
		return true;
}

float GetDrivePos()
{
	return float(hMot1.getEncoderCnt() * M_PI * 6.88 / 720);
}

void hMain()
{
	
	
	sys.setLogDev(&Serial);
	// platform.begin(&RPi);

	DistanceSensor sonar(hSens1);
	DistanceSensor lSonar(hSens2);
	DistanceSensor rSonar(hSens3);

	hMot1.setMotorPolarity(Polarity::Reversed);
	hMot1.setEncoderPolarity(Polarity::Normal);
	hMot2.setEncoderPolarity(Polarity::Normal);
	hMot2.setMotorPolarity(Polarity::Reversed);
	// hMot2.setActiveBraking();

	int lDist = 0;
	int rDist = 0;
	int cDist = 0;
	int bufflDist = 0;
	int buffrDist = 0;
	int nMeasures = 0;

	int wallDist = 15;
	int parkDir = 0;
	float parkAngle = -1;
	float dDist = 0;

	steer(-360);
	int minSteer = hMot2.getEncoderCnt();
	steer(360);
	int maxSteer = hMot2.getEncoderCnt();

	Serial.printf("MinSteer: %d, MaxSteer: %d\n", minSteer, maxSteer);

	steerOffset = (minSteer + maxSteer) / 2;
	minSteer += steerOffset;
	maxSteer += steerOffset;
	Serial.printf("MinSteer: %d, MaxSteer: %d, SteerOffset: %d\n", minSteer, maxSteer, steerOffset);
	bool go = true;

	if (!steer(0))
	{
		Serial.printf("ERROR: Steering calibration failed, program terminated\n");
		go = false;
	}

	while (go)
	{
		buff_btn = btn;
		btn = hBtn1.isPressed();

		if (btn && !buff_btn)
			running = !running;

		if (running)
		{

			steerPos = hMot2.getEncoderCnt();
			int buffDrivePos = drivePos;
			drivePos = GetDrivePos();

			// Serial.printf("%d\n", speed);

			bufflDist = lDist;
			buffrDist = rDist;

			lDist = lSonar.getDistance();
			rDist = rSonar.getDistance();
			cDist = sonar.getDistance();

			// if (cDist < 20)break;

			if (parkDir == 0)
			{
				hMot1.setPower(Power);
				if (lDist <= wallDist)
					parkDir = -1;
				else if (rDist <= wallDist)
					parkDir = 1;
			}
			else if (spaceLength == 0)
			{
				
				dDist = 0;
				int spaceStartPos;
				if (parkDir == -1)
				{
					while (lDist <= wallDist)
					{
						lDist = lSonar.getDistance();
					}
					spaceStartPos = GetDrivePos();
					Serial.printf("Left!\n");

					while (lDist > wallDist)
					{
						float buffDrivePos = drivePos;
						drivePos = GetDrivePos();

						
						bufflDist = lDist;
						lDist = lSonar.getDistance();
						Serial.printf("%f %d\n", drivePos - buffDrivePos, lDist);

						dDist += (lDist - bufflDist) / (drivePos - buffDrivePos);
						nMeasures++;
					}
				}
				else
				{
					while (rDist <= wallDist)
					{
						rDist = rSonar.getDistance();
					}
					spaceStartPos = GetDrivePos();
					Serial.printf("Right!\n");

					while (rDist > wallDist)
					{
						float buffDrivePos = drivePos;
						drivePos = GetDrivePos();

						
						buffrDist = rDist;
						rDist = rSonar.getDistance();
						Serial.printf("%f %d\n", drivePos - buffDrivePos, rDist);

						dDist += (rDist - buffrDist) / (drivePos - buffDrivePos);
						nMeasures++;
					}
				}

				dDist /= nMeasures;

				spaceLength = drivePos - spaceStartPos;

				hMot1.setPower(0);
			}
			Serial.printf("Space length: %f, dDist: %f\n", spaceLength, dDist);

			float parkStartPos;
			
			if (spaceLength > 40)
			{
				//Parallel

				if (parkDir == -1)steer(minSteer);
				else steer(maxSteer);
				parkStartPos = GetDrivePos();
				while (GetDrivePos() > parkStartPos - 27)hMot1.setPower(-Power);
				hMot1.stop();

				if (parkDir == -1)steer(maxSteer);
				else steer(minSteer);
				parkStartPos = GetDrivePos();
				while (GetDrivePos() > parkStartPos - 27)hMot1.setPower(-Power);
				hMot1.stop();

				steer(0);
				running = false;


			}
			else if (spaceLength > 16)
			{

			}
			else if (spaceLength > 24)
			{
				// Perpendicular

				if (parkDir == -1) steer(minSteer);
				else steer(maxSteer);

				parkStartPos = GetDrivePos();

				while (GetDrivePos() > parkStartPos - 10)hMot1.setPower(-Power);
				hMot1.stop();

				if (parkDir == -1) steer(maxSteer);
				else steer(minSteer);
				parkStartPos = GetDrivePos();
				while (GetDrivePos() < parkStartPos + 10)hMot1.setPower(Power);
				hMot1.stop();

				if (parkDir == -1) steer(minSteer);
				else steer(maxSteer);
				parkStartPos = GetDrivePos();
				while (GetDrivePos() > parkStartPos - 20)hMot1.setPower(-Power);
				hMot1.stop();
				steer(0);

				while (lSonar.getDistance() < 20)hMot1.setPower(-Power);

				parkStartPos = GetDrivePos();
				while (GetDrivePos() > parkStartPos - 25)hMot1.setPower(-Power);
				hMot1.stop();
				running = false;
			}

			sys.delay(10);
		}

		sys.delay(25);
	}
}
