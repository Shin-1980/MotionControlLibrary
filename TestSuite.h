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

bool testCase01();
bool testCase02();
bool testCase101();
bool testCase102();
bool testCase109();
bool testCase_201();
bool testCase_202();
bool testCase_203();
bool testCase301();
bool testCaseXXX();

#endif // TESTSUITE_H