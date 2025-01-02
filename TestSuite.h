#ifndef TESTSUITE_H
#define TESTSUITE_H

using namespace std;
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <cmath>
#include <memory>

#include "MotionController.h"
#include "CommandInfo.h"
#include "TrajectoryProfile.h"
#include "TrapezoidalProfile.h"
#include "ScurveProfile.h"

bool testCase_TrapezoidalProfile_001();
bool testCase_TrapezoidalProfile_002();
bool testCase_TrapezoidalProfile_003();
bool testCase_TrapezoidalProfile_004();
bool testCase_TrapezoidalProfile_005();
bool testCase_TrapezoidalProfile_006();
bool testCase_TrapezoidalProfile_007();

bool testCase_TrapezoidalProfile_101();
bool testCase_TrapezoidalProfile_102();
bool testCase_TrapezoidalProfile_103();
bool testCase_TrapezoidalProfile_104();

bool testCase_ScurveProfile_001();
bool testCase_ScurveProfile_002();
bool testCase_ScurveProfile_003();
bool testCase_ScurveProfile_004();
bool testCase_ScurveProfile_005();
bool testCase_ScurveProfile_006();
bool testCase_ScurveProfile_007();
bool testCase_ScurveProfile_008();

bool testCase_ScurveProfile_101();
bool testCase_ScurveProfile_102();

#endif // TESTSUITE_H