#ifndef SCURVEPROFILE_H
#define SCURVEPROFILE_H

using namespace std;
#include <cmath>
#include "TrajectoryProfile.h"
#include "CommandInfo.h"

class ScurveProfile : public TrajectoryProfile
{
public:
    bool isCreated;

    double  initTime[7];
	double  diffTime[7];
	double	initVel[7];
	double  diffVel[7];
	double  initPos[7];
	double  diffPos[7];

	double	curSecTime;
	double	distance;
	double	targetVel;
	int areaNo;
    double accDec;
    double jerk;
    double curDis;

    ScurveProfile();
    double getCurDis(void);
    bool isDone(void);
    bool isDecelerating(void);
    bool makeProf(double acc, double vel, double dis);
    bool calDis(double cycleTime);

private:
    void fillAllProfile(); 

};

#endif //SCURVEPROFILE_H