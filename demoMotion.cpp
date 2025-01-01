using namespace std;
#include <iostream>
#include "TestSuite.h"

int main(){

    int count = 0;

    count++;
    if (testCase01()) cout << "PATH:" << count << endl;
    else cout << "NG:" << count << endl;

    count++;
    if (testCase02()) cout << "PATH:" << count << endl;
    else cout << "NG:" << count << endl;

    count = 101;
    if (testCase101()) cout << "PATH:" << count << endl;
    else cout << "NG:" << count << endl;

    count++;
    if (testCase102()) cout << "PATH:" << count << endl;
    else cout << "NG:" << count << endl;

    count = 109;
    if (testCase109()) cout << "PATH:" << count << endl;
    else cout << "NG:" << count << endl;

    count = 201;
    if (testCase_201()) cout << "PATH:" << count << endl;
    else cout << "NG:" << count << endl;

    count++;
    if (testCase_202()) cout << "PATH:" << count << endl;
    else cout << "NG:" << count << endl;

    count++;
    if (testCase_203()) cout << "PATH:" << count << endl;
    else cout << "NG:" << count << endl;

    if (testCase301()) cout << "PATH:" << count << endl;
    else cout << "NG:" << count << endl;

    //testCaseXXX();

}
