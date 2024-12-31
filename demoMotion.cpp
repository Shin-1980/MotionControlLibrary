using namespace std;
#include <iostream>
#include "TestSuite.h"

int main(){

    int count = 0;

    count++;
    if (testCase01()) std::cout << "PATH:" << count << std::endl;
    else std::cout << "NG:" << count << std::endl;

    count++;
    if (testCase02()) std::cout << "PATH:" << count << std::endl;
    else std::cout << "NG:" << count << std::endl;

    count = 101;
    if (testCase101()) std::cout << "PATH:" << count << std::endl;
    else std::cout << "NG:" << count << std::endl;

    count++;
    if (testCase102()) std::cout << "PATH:" << count << std::endl;
    else std::cout << "NG:" << count << std::endl;

    count = 109;
    if (testCase109()) std::cout << "PATH:" << count << std::endl;
    else std::cout << "NG:" << count << std::endl;

//    testCaseXXX()

    count = 201;
    if (testCase_201()) std::cout << "PATH:" << count << std::endl;
    else std::cout << "NG:" << count << std::endl;

    count++;
    if (testCase_202()) std::cout << "PATH:" << count << std::endl;
    else std::cout << "NG:" << count << std::endl;

    count++;
    if (testCase_203()) std::cout << "PATH:" << count << std::endl;
    else std::cout << "NG:" << count << std::endl;

    if (testCase301()) std::cout << "PATH:" << count << std::endl;
    else std::cout << "NG:" << count << std::endl;

}
