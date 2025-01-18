#ifndef TESTSUITE_H
#define TESTSUITE_H

using namespace std;
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <cmath>
#include <memory>
#include <vector>
#include <random>
#include <cassert>

#include "MotionController.h"
#include "CommandInfo.h"
#include "TrajectoryProfile.h"
#include "TrapezoidalProfile.h"
#include "ScurveProfile.h"

#define epsilon 0.0001

void testCase_TrapezoidalProfile_001();
void testCase_TrapezoidalProfile_002();
void testCase_TrapezoidalProfile_003();
void testCase_TrapezoidalProfile_004();
void testCase_TrapezoidalProfile_005();
void testCase_TrapezoidalProfile_006();
void testCase_TrapezoidalProfile_007();

void testCase_TrapezoidalProfile_101();
void testCase_TrapezoidalProfile_102();
void testCase_TrapezoidalProfile_103();
void testCase_TrapezoidalProfile_104();

void testCase_TrapezoidalProfile_201();
void testCase_TrapezoidalProfile_rand();


void testCase_ScurveProfile_001();
void testCase_ScurveProfile_002();
void testCase_ScurveProfile_003();
void testCase_ScurveProfile_004();
void testCase_ScurveProfile_005();
void testCase_ScurveProfile_006();
void testCase_ScurveProfile_007();
void testCase_ScurveProfile_008();

void testCase_ScurveProfile_101();
void testCase_ScurveProfile_102();

void testCase_ScurveProfile_201();
void testCase_ScurveProfile_rand();
#endif // TESTSUITE_H