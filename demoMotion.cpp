using namespace std;
#include <iostream>
#include "TestSuite.h"

int main(){

    int count = 0;

    count++;
    if (testCase_TrapezoidalProfile_001()) cout << "PATH:" << count << endl;
    else cout << "NG:" << count << endl;

    count++;
    if (testCase_TrapezoidalProfile_002()) cout << "PATH:" << count << endl;
    else cout << "NG:" << count << endl;

    count++;
    if (testCase_TrapezoidalProfile_003()) cout << "PATH:" << count << endl;
    else cout << "NG:" << count << endl;

    count++;
    if (testCase_TrapezoidalProfile_004()) cout << "PATH:" << count << endl;
    else cout << "NG:" << count << endl;

    count++;
    if (testCase_TrapezoidalProfile_005()) cout << "PATH:" << count << endl;
    else cout << "NG:" << count << endl;

    count++;
    if (testCase_TrapezoidalProfile_006()) cout << "PATH:" << count << endl;
    else cout << "NG:" << count << endl;

    count++;
    if (testCase_TrapezoidalProfile_007()) cout << "PATH:" << count << endl;
    else cout << "NG:" << count << endl;


    count = 101;
    if (testCase_TrapezoidalProfile_101()) cout << "PATH:" << count << endl;
    else cout << "NG:" << count << endl;

    count++;
    if (testCase_TrapezoidalProfile_102()) cout << "PATH:" << count << endl;
    else cout << "NG:" << count << endl;

    count++;
    if (testCase_TrapezoidalProfile_103()) cout << "PATH:" << count << endl;
    else cout << "NG:" << count << endl;

    count++;
    if (testCase_TrapezoidalProfile_104()) cout << "PATH:" << count << endl;
    else cout << "NG:" << count << endl;

    count = 1;
    if (testCase_ScurveProfile_001()) cout << "PATH:" << count << endl;
    else cout << "NG:" << count << endl;

    count++;
    if (testCase_ScurveProfile_002()) cout << "PATH:" << count << endl;
    else cout << "NG:" << count << endl;

    count++;
    if (testCase_ScurveProfile_003()) cout << "PATH:" << count << endl;
    else cout << "NG:" << count << endl;

    count++;
    if (testCase_ScurveProfile_004()) cout << "PATH:" << count << endl;
    else cout << "NG:" << count << endl;

    count++;
    if (testCase_ScurveProfile_005()) cout << "PATH:" << count << endl;
    else cout << "NG:" << count << endl;

    count++;
    if (testCase_ScurveProfile_006()) cout << "PATH:" << count << endl;
    else cout << "NG:" << count << endl;

    count++;
    if (testCase_ScurveProfile_007()) cout << "PATH:" << count << endl;
    else cout << "NG:" << count << endl;

    count++;
    if (testCase_ScurveProfile_008()) cout << "PATH:" << count << endl;
    else cout << "NG:" << count << endl;

    count = 101;
    if (testCase_ScurveProfile_101()) cout << "PATH:" << count << endl;
    else cout << "NG:" << count << endl;

    count++;
    if (testCase_ScurveProfile_102()) cout << "PATH:" << count << endl;
    else cout << "NG:" << count << endl;

}
