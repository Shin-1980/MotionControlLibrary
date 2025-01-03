#include "TestSuite.h"

bool testCase_TrapezoidalProfile_001() 
{
    TrapezoidalProfile prof;

    double startPos = 0.0;
    double tarPos = 50;
    double acc = 100;
    double vel = 100;
    double cycleTime = 0.1;
    double prePos = startPos;
    double curPos = startPos;
    double preVel = 0;
    double curVel = 0;

    string filename = "./testCase/case_Trape_001.csv";
    ifstream file(filename);

    if (!file.is_open()) 
    {
        cerr << "Error: Could not open file " << filename << endl;
        return false;
    }

    if (!prof.makeVelProf(tarPos - startPos, vel, acc, 0.0)) return false;

    string line;
    while (getline(file, line) && prof.calDis(cycleTime)) 
    { // Read the file line by line
        stringstream ss(line);   // Use stringstream to parse the line
        string value;
        vector<string> row; // Store the values of the current row

        while (getline(ss, value, ',')) 
        { // Split by comma
            row.push_back(value);
        }

        double dis = prof.getCurDis();
        if (abs(dis - stof(row[0]) > 0.0001)) 
        {
            return false;
        }
    }
    
    if (abs(prof.getCurDis() - tarPos > 0.0001)) 
    {
        return false;
    }

    file.close();

    return true;
}

bool testCase_TrapezoidalProfile_002() 
{
    TrapezoidalProfile prof;

    double startPos = 0.0;
    double tarPos = 25;
    double acc = 100;
    double vel = 100;
    double cycleTime = 0.1;
    double prePos = startPos;
    double curPos = startPos;
    double preVel = 0;
    double curVel = 0;

    string filename = "./testCase/case_Trape_002.csv";
    ifstream file(filename);

    if (!file.is_open()) 
    {
        cerr << "Error: Could not open file " << filename << endl;
        return 1;
    }

    if (!prof.makeVelProf(tarPos - startPos, vel, acc, 0.0)) return false;

    string line;
    while (getline(file, line) && prof.calDis(cycleTime)) 
    { // Read the file line by line
        stringstream ss(line);   // Use stringstream to parse the line
        string value;
        vector<string> row; // Store the values of the current row

        while (getline(ss, value, ',')) 
        { // Split by comma
            row.push_back(value);
        }

        double dis = prof.getCurDis();
        
        if (abs(dis - stof(row[0]) > 0.0001)) 
        {
            return false;
        }
    }
    
    if (abs(prof.getCurDis() - tarPos > 0.0001)) 
    {
        return false;
    }

    file.close();

    return true;

}

bool testCase_TrapezoidalProfile_003() 
{
    TrapezoidalProfile prof;

    double cycleTime = 0.01;

    if(prof.calDis(cycleTime) == false) return true;

    return false;
}

bool testCase_TrapezoidalProfile_004() 
{
    TrapezoidalProfile prof;

    if (prof.getCurDis() == 0.0) return true;

    return false;
}

bool testCase_TrapezoidalProfile_005() 
{
    TrapezoidalProfile prof;

    double startPos = 0.0;
    double tarPos = -25;
    double acc = 100;
    double vel = 100;
    double cycleTime = 0.1;
    double prePos = startPos;
    double curPos = startPos;
    double preVel = 0;
    double curVel = 0;

    if (!prof.makeVelProf(tarPos - startPos, vel, acc, 0.0)) return true;

    return false;
}

bool testCase_TrapezoidalProfile_006() 
{
    TrapezoidalProfile prof;

    double startPos = 0.0;
    double tarPos = 25;
    double acc = -100;
    double vel = 100;
    double cycleTime = 0.1;
    double prePos = startPos;
    double curPos = startPos;
    double preVel = 0;
    double curVel = 0;

    if (!prof.makeVelProf(tarPos - startPos, vel, acc, 0.0)) return true;

    return false;
}

bool testCase_TrapezoidalProfile_007() 
{
    TrapezoidalProfile prof;

    double startPos = 0.0;
    double tarPos = 25;
    double acc = 100;
    double vel = -100;
    double cycleTime = 0.1;
    double prePos = startPos;
    double curPos = startPos;
    double preVel = 0;
    double curVel = 0;

    if (!prof.makeVelProf(tarPos - startPos, vel, acc, 0.0)) return true;

    return false;
}

bool testCase_TrapezoidalProfile_101(void)
{
    int dof = 6;
    MotionController mc(dof);

    shared_ptr<TrapezoidalProfile> execProf1 = make_shared<TrapezoidalProfile>();

    mc.setTrapezoidalProfile(execProf1);

    vector<double> curPose;
    curPose.push_back(1.5708f);
    curPose.push_back(-3.14159f);
    curPose.push_back(1.5708f);
    curPose.push_back(0.0f);
    curPose.push_back(0.0f);
    curPose.push_back(0.0f);

    mc.setCurrentPose(curPose);

    vector<double> targetPose;
    targetPose.push_back(0.693782f);
    targetPose.push_back(-2.36364f);
    targetPose.push_back(2.38406f);
    targetPose.push_back(2.1103f);
    targetPose.push_back(-1.07417f);
    targetPose.push_back(-1.14081f);

    vector<double> targetVelsRad;
    targetVelsRad.push_back(5.23598776f);
    targetVelsRad.push_back(5.23598776f);
    targetVelsRad.push_back(6.54498469f);
    targetVelsRad.push_back(6.54498469f);
    targetVelsRad.push_back(6.54498469f);
    targetVelsRad.push_back(10.47197551f);

    vector<double> targetAccsRad;
    targetAccsRad.push_back(17.45329252f);
    targetAccsRad.push_back(17.45329252f);
    targetAccsRad.push_back(26.17993878f);
    targetAccsRad.push_back(26.17993878f);
    targetAccsRad.push_back(26.17993878f);
    targetAccsRad.push_back(34.90658504f);

    vector<double> targetJerks;
    targetJerks.push_back(0.0f);
    mc.setVelocityProfParam(targetPose, targetVelsRad, targetAccsRad, targetJerks);

    double cycleTime = 0.01;

    vector<double> cmdPose = curPose;

    string filename = "./testCase/case_Trape_101.csv";
    ifstream file(filename);

    if (!file.is_open()) 
    {
        cerr << "Error: Could not open file " << filename << endl;
        return 1;
    }
    string line;
    while (getline(file, line) && mc.execCmd(cycleTime))
    { // Read the file line by line
        stringstream ss(line);   // Use stringstream to parse the line
        string value;
        vector<string> row; // Store the values of the current row

        while (getline(ss, value, ',')) { // Split by comma
            row.push_back(value);
        }

        cmdPose = mc.getCmdPose();

        for (size_t i=0;i<dof;i++) {
            if (abs(cmdPose[i] - stof(row[i])) > 0.0001) return false;
        }
    }

    cmdPose = mc.getCmdPose();

    for (size_t i=0;i<dof;i++) 
    {
        if (abs(cmdPose[i] - targetPose[i]) > 0.0001) return false;
    }

    execProf1.reset();
    file.close();

    return true;
}

bool testCase_TrapezoidalProfile_102(void)
{
        
    int dof = 6;
    MotionController mc(dof);

    shared_ptr<TrapezoidalProfile> execProf = make_shared<TrapezoidalProfile>();
    mc.setTrapezoidalProfile(execProf);

    vector<double> curPose;
    curPose.push_back(1.5708f);
    curPose.push_back(-3.14159f);
    curPose.push_back(1.5708f);
    curPose.push_back(0.0f);
    curPose.push_back(0.0f);
    curPose.push_back(0.0f);

    mc.setCurrentPose(curPose);

    vector<double> targetPose;
    targetPose.push_back(0.693782f);
    targetPose.push_back(-2.36364f);
    targetPose.push_back(2.38406f);
    targetPose.push_back(2.1103f);
    targetPose.push_back(-1.07417f);
    targetPose.push_back(-1.14081f);

    vector<double> targetVelsRad;

    targetVelsRad.push_back(0.52359878f);
    targetVelsRad.push_back(0.52359878f);
    targetVelsRad.push_back(0.65449847f);
    targetVelsRad.push_back(0.65449847f);
    targetVelsRad.push_back(0.65449847f);
    targetVelsRad.push_back(1.04719755f);

    vector<double> targetAccsRad;
    targetAccsRad.push_back(17.45329252f);
    targetAccsRad.push_back(17.45329252f);
    targetAccsRad.push_back(26.17993878f);
    targetAccsRad.push_back(26.17993878f);
    targetAccsRad.push_back(26.17993878f);
    targetAccsRad.push_back(34.90658504f);

    vector<double> targetJerks;
    targetJerks.push_back(0.0f);
    mc.setVelocityProfParam(targetPose, targetVelsRad, targetAccsRad, targetJerks);

    double cycleTime = 0.01;

    vector<double> cmdPose = curPose;

    string filename = "./testCase/case_Trape_102.csv";
    ifstream file(filename);

    if (!file.is_open()) 
    {
        cerr << "Error: Could not open file " << filename << endl;
        return 1;
    }

    string line;
    while (getline(file, line) && mc.execCmd(cycleTime)) 
    { // Read the file line by line
        stringstream ss(line);   // Use stringstream to parse the line
        string value;
        vector<string> row; // Store the values of the current row

        while (getline(ss, value, ',')) 
        { // Split by comma
            row.push_back(value);
        }

        cmdPose = mc.getCmdPose();

        for (size_t i=0;i<dof;i++) 
        {
            if (abs(cmdPose[i] - stof(row[i])) > 0.0001) {
                return false;
            }
        }
    }

    if (mc.execCmd(cycleTime)) return false;

    cmdPose = mc.getCmdPose();
    for (size_t i=0;i<dof;i++) 
    {
        if (abs(cmdPose[i] - targetPose[i]) > 0.0001) return false;
    }

    execProf.reset();
    file.close();

    return true;
}

bool testCase_TrapezoidalProfile_103()
{
    int dof = 6;
    MotionController mc(dof);

    double cycleTime = 0.01;
    
    if (mc.execCmd(cycleTime) == false) return true;
    return false;
}

bool testCase_TrapezoidalProfile_104()
{
    int dof = 6;
    MotionController mc(dof);

    shared_ptr<TrapezoidalProfile> execProf1 = make_shared<TrapezoidalProfile>();
    shared_ptr<TrapezoidalProfile> execProf2 = make_shared<TrapezoidalProfile>();
    shared_ptr<TrapezoidalProfile> execProf3 = make_shared<TrapezoidalProfile>();

    mc.setTrapezoidalProfile(execProf1);
    mc.setTrapezoidalProfile(execProf2);
    mc.setTrapezoidalProfile(execProf3);

    vector<double> curPose;
    curPose.push_back(1.5708f);
    curPose.push_back(-3.14159f);
    curPose.push_back(1.5708f);
    curPose.push_back(0.0f);
    curPose.push_back(0.0f);
    curPose.push_back(0.0f);
    mc.setCurrentPose(curPose);

    vector<double> targetPose;
    targetPose.push_back(0.693782f);
    targetPose.push_back(-2.36364f);
    targetPose.push_back(2.38406f);
    targetPose.push_back(2.1103f);
    targetPose.push_back(-1.07417f);
    targetPose.push_back(-1.14081f);

    vector<double> targetVelsRad;
    targetVelsRad.push_back(5.23598776f);
    targetVelsRad.push_back(5.23598776f);
    targetVelsRad.push_back(6.54498469f);
    targetVelsRad.push_back(6.54498469f);
    targetVelsRad.push_back(6.54498469f);
    targetVelsRad.push_back(10.47197551f);

    vector<double> targetAccsRad;
    targetAccsRad.push_back(17.45329252f);
    targetAccsRad.push_back(17.45329252f);
    targetAccsRad.push_back(26.17993878f);
    targetAccsRad.push_back(26.17993878f);
    targetAccsRad.push_back(26.17993878f);
    targetAccsRad.push_back(34.90658504f);
        
    vector<double> targetJerks;
    targetJerks.push_back(0.0f);
    mc.setVelocityProfParam(targetPose, targetVelsRad, targetAccsRad, targetJerks);

    targetPose[0] = 0.456344;
    targetPose[1] = -2.54862;
    targetPose[2] = 2.16375;
    targetPose[3] = 1.78586;
    targetPose[4] = -0.350709;
    targetPose[5] = -1.29741;
    mc.setVelocityProfParam(targetPose, targetVelsRad, targetAccsRad, targetJerks);

    targetPose[0] = -0.449;
    targetPose[1] = -0.890494;
    targetPose[2] = 2.60354;
    targetPose[3] = 2.06081;
    targetPose[4] = -0.0605401;
    targetPose[5] = -0.624552;
    mc.setVelocityProfParam(targetPose, targetVelsRad, targetAccsRad, targetJerks);

    double cycleTime = 0.01;
    vector<double> cmdPose = curPose;
    string filename = "./testCase/case_Trape_104.csv";

    ifstream file(filename);

    if (!file.is_open())
    {
        cerr << "Error: Could not open file " << filename << endl;
        return 1;
    }

/*
    ofstream foutp;
    foutp.open(filename);
*/

    string line;
    while (getline(file, line) && mc.execCmd(cycleTime)) 
    { // Read the file line by line
        stringstream ss(line);
        string value;
        vector<string> row;

        while (getline(ss, value, ',')) 
        {
            row.push_back(value);
        }

//    while(mc.execCmd(cycleTime)) {
        //cout << "1097" << endl;
        cmdPose = mc.getCmdPose();

        for (size_t i=0;i<dof;i++) 
        {
            if (abs(cmdPose[i] - stof(row[i])) > 0.0001) {
                return false;
            }
        }
    }

/*
        for (size_t i=0;i<dof;i++) {
            foutp << cmdPose[i] << ",";
        }    
        //cout << endl;
        foutp << endl;
    }
*/       
    if (mc.execCmd(cycleTime)) return false;

    cmdPose = mc.getCmdPose();
    for (size_t i=0;i<dof;i++) 
    {
        if (abs(cmdPose[i] - targetPose[i]) > 0.0001) return false;
    }

    execProf1.reset();
    execProf2.reset();
    execProf3.reset();

    file.close();

    return true;

}


bool testCase_ScurveProfile_001()
{

    ScurveProfile sc;
    // acc, dec, vel, pos
    if (!sc.makeVelProf(100,5,1,1.0)) return false;

    double cycleTime = 0.2;
    double curPos = 0.0;
    double prePos = 0.0;
    double curVel = 0.0;
    double preVel = 0.0;
    double curAcc = 0.0;

    string filename = "./testCase/case_Scurve_001.csv";
    ifstream file(filename);

    if (!file.is_open()) {
        cerr << "Error: Could not open file " << filename << endl;
        return 1;
    }

    string line;
    while (getline(file, line) && sc.calDis(cycleTime)) { // Read the file line by line
        stringstream ss(line);
        string value;
        vector<string> row;

        while (getline(ss, value, ',')) 
        {
            row.push_back(value);
        }

        curPos = sc.getCurDis();
        curVel = (curPos - prePos) / cycleTime;
        curAcc = (curVel - preVel) / cycleTime;

        if (abs(curPos - stof(row[0])) > 0.0001) 
        {
                return false;
        }
        if (abs(curVel - stof(row[1])) > 0.0001) 
        {
                return false;
        }
        if (abs(curAcc - stof(row[2])) > 0.0001) 
        {
                return false;
        }

        prePos = curPos;
        preVel = curVel;
    }

    file.close();

    return true;

}

bool testCase_ScurveProfile_002()
{

    ScurveProfile sc;
    // acc, dec, vel, pos
    if(!sc.makeVelProf(0.25, 5, 1, 1.0)) return false;

    double cycleTime = 0.2;
    double curPos = 0.0;
    double prePos = 0.0;
    double curVel = 0.0;
    double preVel = 0.0;
    double curAcc = 0.0;

    string filename = "./testCase/case_Scurve_002.csv";
    ifstream file(filename);

    if (!file.is_open()) 
    {
        cerr << "Error: Could not open file " << filename << endl;
        return 1;
    }

    string line;
    while (getline(file, line) && sc.calDis(cycleTime)) 
    { // Read the file line by line
        stringstream ss(line);
        string value;
        vector<string> row;

        while (getline(ss, value, ',')) 
        {
            row.push_back(value);
        }

        curPos = sc.getCurDis();
        curVel = (curPos - prePos) / cycleTime;
        curAcc = (curVel - preVel) / cycleTime;

        if (abs(curPos - stof(row[0])) > 0.0001) 
        {
                return false;
        }
        if (abs(curVel - stof(row[1])) > 0.0001) 
        {
                return false;
        }
        if (abs(curAcc - stof(row[2])) > 0.0001) 
        {
                return false;
        }

        prePos = curPos;
        preVel = curVel;
    }

    file.close();
    return true;

}

bool testCase_ScurveProfile_003()
{

    ScurveProfile sc;
    // acc, dec, vel, pos
    if (!sc.makeVelProf(10, 100, 1, 1.0)) return false;

    double cycleTime = 0.2;
    double curPos = 0.0;
    double prePos = 0.0;
    double curVel = 0.0;
    double preVel = 0.0;
    double curAcc = 0.0;

    string filename = "./testCase/case_Scurve_003.csv";
    ifstream file(filename);

    if (!file.is_open()) 
    {
        cerr << "Error: Could not open file " << filename << endl;
        return 1;
    }

    string line;
    while (getline(file, line) && sc.calDis(cycleTime)) 
    { // Read the file line by line
        stringstream ss(line);
        string value;
        vector<string> row;

        while (getline(ss, value, ',')) 
        {
            row.push_back(value);
        }

        curPos = sc.getCurDis();
        curVel = (curPos - prePos) / cycleTime;
        curAcc = (curVel - preVel) / cycleTime;

        if (abs(curPos - stof(row[0])) > 0.0001) 
        {
                return false;
        }
        if (abs(curVel - stof(row[1])) > 0.0001) 
        {
                return false;
        }
        if (abs(curAcc - stof(row[2])) > 0.0001) 
        {
                return false;
        }

        prePos = curPos;
        preVel = curVel;
    }

    file.close();
    return true;

}

bool testCase_ScurveProfile_004() 
{
    TrapezoidalProfile prof;

    if (prof.getCurDis() == 0.0) return true;

    return false;
}

bool testCase_ScurveProfile_005() 
{
    TrapezoidalProfile prof;

    double startPos = 0.0;
    double tarPos = -25;
    double acc = 100;
    double vel = 100;
    double jerk = 100;
    double cycleTime = 0.1;
    double prePos = startPos;
    double curPos = startPos;
    double preVel = 0;
    double curVel = 0;

    if (!prof.makeVelProf(tarPos - startPos, vel, acc, jerk)) return true;

    return false;
}

bool testCase_ScurveProfile_006() 
{
    TrapezoidalProfile prof;

    double startPos = 0.0;
    double tarPos = 25;
    double acc = -100;
    double vel = 100;
    double jerk = 100;
    double cycleTime = 0.1;
    double prePos = startPos;
    double curPos = startPos;
    double preVel = 0;
    double curVel = 0;

    if (!prof.makeVelProf(tarPos - startPos, vel, acc, jerk)) return true;

    return false;
}

bool testCase_ScurveProfile_007() 
{
    ScurveProfile prof;

    double startPos = 0.0;
    double tarPos = 25;
    double acc = 100;
    double vel = -100;
    double jerk = 100;
    double cycleTime = 0.1;
    double prePos = startPos;
    double curPos = startPos;
    double preVel = 0;
    double curVel = 0;

    if (!prof.makeVelProf(tarPos - startPos, vel, acc, jerk)) return true;

    return false;
}

bool testCase_ScurveProfile_008() 
{
    ScurveProfile prof;

    double startPos = 0.0;
    double tarPos = 25;
    double acc = 100;
    double vel = 100;
    double jerk = -100;
    double cycleTime = 0.1;
    double prePos = startPos;
    double curPos = startPos;
    double preVel = 0;
    double curVel = 0;

    if (!prof.makeVelProf(tarPos - startPos, vel, acc, jerk)) return true;

    return false;
}



bool testCase_ScurveProfile_101(void)
{
        
    int dof = 6;
    MotionController mc(dof);

    shared_ptr<ScurveProfile> execProf1 = make_shared<ScurveProfile>();
    shared_ptr<ScurveProfile> execProf2 = make_shared<ScurveProfile>();

    mc.setScurveProfile(execProf1);
    mc.setScurveProfile(execProf2);

    vector<double> curPose;
    curPose.push_back(1.5708f);
    curPose.push_back(-3.14159f);
    curPose.push_back(1.5708f);
    curPose.push_back(0.0f);
    curPose.push_back(0.0f);
    curPose.push_back(0.0f);

    mc.setCurrentPose(curPose);

    vector<double> targetPose;

    targetPose.push_back(0.0f);
    targetPose.push_back(0.0f);
    targetPose.push_back(1.5708f);
    targetPose.push_back(0.0f);
    targetPose.push_back(0.0f);
    targetPose.push_back(0.0f);

    vector<double> targetVelsRad;
    targetVelsRad.push_back(5.23598776f);
    targetVelsRad.push_back(5.23598776f);
    targetVelsRad.push_back(6.54498469f);
    targetVelsRad.push_back(6.54498469f);
    targetVelsRad.push_back(6.54498469f);
    targetVelsRad.push_back(10.47197551f);

    vector<double> targetAccsRad;
    targetAccsRad.push_back(17.45329252f);
    targetAccsRad.push_back(17.45329252f);
    targetAccsRad.push_back(26.17993878f);
    targetAccsRad.push_back(26.17993878f);
    targetAccsRad.push_back(26.17993878f);
    targetAccsRad.push_back(34.90658504f);

    vector<double> targetJerksRad;
    targetJerksRad.push_back(1.0f);
    targetJerksRad.push_back(1.0f);
    targetJerksRad.push_back(1.0f);
    targetJerksRad.push_back(1.0f);
    targetJerksRad.push_back(1.0f);
    targetJerksRad.push_back(1.0f);
    mc.setVelocityProfParam(targetPose, targetVelsRad, targetAccsRad, targetJerksRad);

    targetPose[0] = 0.0;
    targetPose[1] = 0.0;
    targetPose[2] = 0.0;
    targetPose[3] = 0.0;
    targetPose[4] = 0.0;
    targetPose[5] = 0.0;
    mc.setVelocityProfParam(targetPose, targetVelsRad, targetAccsRad, targetJerksRad);

    double cycleTime = 0.01;

    vector<double> cmdPose = curPose;
    vector<double> prePose = curPose;
    vector<double> cmdVels;
    cmdVels.push_back(0.0f);
    cmdVels.push_back(0.0f);
    cmdVels.push_back(0.0f);
    cmdVels.push_back(0.0f);
    cmdVels.push_back(0.0f);
    cmdVels.push_back(0.0f);

    vector<double> preVels = cmdVels;
    vector<double> cmdAccs = cmdVels;

    string filenamep = "./testCase/case_Scurve_101_pose.csv";
    //string filenamev = "./testCase/case_Scurve_101_vels.csv";
    //string filenamea = "./testCase/case_Scurve_101_accs.csv";

    ifstream filep(filenamep);
    //ifstream filev(filenamev);
    //ifstream filea(filenamea);

    if (!filep.is_open()) 
    {
        cerr << "Error: Could not open file " << endl;
        return false;
    }

/*
    ofstream foutp;
    ofstream foutv;
    ofstream fouta;

    foutp.open(filenamep);
    foutv.open(filenamev);
    fouta.open(filenamea);
*/
    string line;
    while (getline(filep, line) && mc.execCmd(cycleTime)) 
    { // Read the file line by line
        stringstream ss(line);
        string value;
        vector<string> row;

        while (getline(ss, value, ',')) 
        {
            row.push_back(value);
        }

        cmdPose = mc.getCmdPose();

        for (size_t i=0;i<dof;i++) 
        {
            if (abs(cmdPose[i] - stof(row[i])) > 0.0001) return false;
        }

        for (size_t i=0;i<dof;i++) 
        {
            cmdVels[i] = (cmdPose[i] - prePose[i]) / cycleTime;
            cmdAccs[i] = (cmdVels[i] - preVels[i]) / cycleTime;

            //cout << cmdPose[i] << ",";
            //foutp << cmdPose[i] << ",";
            //foutv << cmdVels[i] << ",";
            //fouta << cmdAccs[i] << ",";

            prePose[i] = cmdPose[i];
            preVels[i] = cmdVels[i];
        }
        //cout << endl;
        //foutp << endl;
        //foutv << endl;
        //fouta << endl;

    }

    cmdPose = mc.getCmdPose();

    for (size_t i=0;i<dof;i++) 
    {
        if (abs(cmdPose[i] - targetPose[i]) > 0.0001) return false;
    }

    execProf1.reset();
    execProf2.reset();

    filep.close();

    return true;
}


bool testCase_ScurveProfile_102(void)
{
        
    int dof = 6;
    MotionController mc(dof);

    shared_ptr<ScurveProfile> execProf1 = make_shared<ScurveProfile>();
    shared_ptr<ScurveProfile> execProf2 = make_shared<ScurveProfile>();

    mc.setScurveProfile(execProf1);
    mc.setScurveProfile(execProf2);

    vector<double> curPose;
    curPose.push_back(1.5708f);
    curPose.push_back(-3.14159f);
    curPose.push_back(1.5708f);
    curPose.push_back(0.0f);
    curPose.push_back(0.0f);
    curPose.push_back(0.0f);

    mc.setCurrentPose(curPose);

    vector<double> targetPose;

    targetPose.push_back(0.0f);
    targetPose.push_back(0.0f);
    targetPose.push_back(1.5708f);
    targetPose.push_back(0.0f);
    targetPose.push_back(0.0f);
    targetPose.push_back(0.0f);

    vector<double> targetVelsRad;
    targetVelsRad.push_back(5.23598776f);
    targetVelsRad.push_back(5.23598776f);
    targetVelsRad.push_back(6.54498469f);
    targetVelsRad.push_back(6.54498469f);
    targetVelsRad.push_back(6.54498469f);
    targetVelsRad.push_back(10.47197551f);

    vector<double> targetAccsRad;
    targetAccsRad.push_back(17.45329252f);
    targetAccsRad.push_back(17.45329252f);
    targetAccsRad.push_back(26.17993878f);
    targetAccsRad.push_back(26.17993878f);
    targetAccsRad.push_back(26.17993878f);
    targetAccsRad.push_back(34.90658504f);

    vector<double> targetJerksRad;
    targetJerksRad.push_back(50.0f);
    targetJerksRad.push_back(50.0f);

    targetJerksRad.push_back(50.0f);
    targetJerksRad.push_back(50.0f);
    targetJerksRad.push_back(50.0f);
    targetJerksRad.push_back(50.0f);
    mc.setVelocityProfParam(targetPose, targetVelsRad, targetAccsRad, targetJerksRad);

    targetPose[0] = 0.0;
    targetPose[1] = 0.0;
    targetPose[2] = 0.0;
    targetPose[3] = 0.0;
    targetPose[4] = 0.0;
    targetPose[5] = 0.0;
    mc.setVelocityProfParam(targetPose, targetVelsRad, targetAccsRad, targetJerksRad);

    double cycleTime = 0.01;

    vector<double> cmdPose = curPose;
    vector<double> prePose = curPose;
    vector<double> cmdVels;
    cmdVels.push_back(0.0f);
    cmdVels.push_back(0.0f);
    cmdVels.push_back(0.0f);
    cmdVels.push_back(0.0f);
    cmdVels.push_back(0.0f);
    cmdVels.push_back(0.0f);

    vector<double> preVels = cmdVels;
    vector<double> cmdAccs = cmdVels;

    string filenamep = "./testCase/case_Scurve102_pose.csv";
    //string filenamev = "./testCase/case_Scurve102_vels.csv";
    //string filenamea = "./testCase/case_Scurve102_accs.csv";

    ifstream filep(filenamep);
    //ifstream filev(filenamev);
    //ifstream filea(filenamea);

    if (!filep.is_open()) 
    {
        cerr << "Error: Could not open file " << endl;
        return false;
    }

/*
    ofstream foutp;
    ofstream foutv;
    ofstream fouta;

    foutp.open(filenamep);
    foutv.open(filenamev);
    fouta.open(filenamea);
*/
    string line;
    while (getline(filep, line) && mc.execCmd(cycleTime)) 
    { // Read the file line by line
        stringstream ss(line);
        string value;
        vector<string> row;

        while (getline(ss, value, ',')) 
        {
            row.push_back(value);
        }

        cmdPose = mc.getCmdPose();

        for (size_t i=0;i<dof;i++) 
        {
            if (abs(cmdPose[i] - stof(row[i])) > 0.0001) return false;
        }
        
        for (size_t i=0;i<dof;i++) 
        {
            cmdVels[i] = (cmdPose[i] - prePose[i]) / cycleTime;
            cmdAccs[i] = (cmdVels[i] - preVels[i]) / cycleTime;

            //cout << cmdPose[i] << ",";
            //foutp << cmdPose[i] << ",";
            //foutv << cmdVels[i] << ",";
            //fouta << cmdAccs[i] << ",";

            prePose[i] = cmdPose[i];
            preVels[i] = cmdVels[i];
        }
        //cout << endl;
        //foutp << endl;
        //foutv << endl;
        //fouta << endl;
    }

    cmdPose = mc.getCmdPose();

    for (size_t i=0;i<dof;i++) 
    {
        if (abs(cmdPose[i] - targetPose[i]) > 0.0001) return false;
        //cout << cmdPose[i] << ",";
    }
    //cout << endl;

    execProf1.reset();
    execProf2.reset();

    filep.close();

    return true;
}

