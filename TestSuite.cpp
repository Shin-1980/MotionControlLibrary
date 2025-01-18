#include "TestSuite.h"

void testCase_TrapezoidalProfile_001() 
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
        return;
    }

    if (!prof.makeVelProf(tarPos - startPos, vel, acc, 0.0)) return;

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
        assert(abs(dis - stof(row[0]) < epsilon));

    }
    
    assert(abs(prof.getCurDis() - tarPos) < 0.0001);
    file.close();

    cout << "testCase_TrapezoidalProfile_001 passed!" << endl;
}

void testCase_TrapezoidalProfile_002() 
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
        return;
    }

    if (!prof.makeVelProf(tarPos - startPos, vel, acc, 0.0)) return;

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

        assert(abs(dis - stof(row[0]) < epsilon));
    }

    assert(abs(prof.getCurDis() - tarPos) < 0.0001);

    file.close();

    cout << "testCase_TrapezoidalProfile_002 passed!" << endl;
}

void testCase_TrapezoidalProfile_003() 
{
    TrapezoidalProfile prof;

    double cycleTime = 0.01;

    assert(prof.calDis(cycleTime) == false);

    cout << "testCase_TrapezoidalProfile_003 passed!" << endl;
}

void testCase_TrapezoidalProfile_004() 
{
    TrapezoidalProfile prof;

    assert(prof.getCurDis() == 0.0);

    cout << "testCase_TrapezoidalProfile_004 passed!" << endl;
}

void testCase_TrapezoidalProfile_005() 
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

    assert(!prof.makeVelProf(tarPos - startPos, vel, acc, 0.0) == true);

    cout << "testCase_TrapezoidalProfile_005 passed!" << endl;
}

void testCase_TrapezoidalProfile_006() 
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

    assert(!prof.makeVelProf(tarPos - startPos, vel, acc, 0.0) == true);

    cout << "testCase_TrapezoidalProfile_006 passed!" << endl;
}

void testCase_TrapezoidalProfile_007() 
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

    assert(!prof.makeVelProf(tarPos - startPos, vel, acc, 0.0) == true);

    cout << "testCase_TrapezoidalProfile_007 passed!" << endl;
}

void testCase_TrapezoidalProfile_101(void)
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
    targetVelsRad.push_back(4.23598776f);
    targetVelsRad.push_back(4.23598776f);
    targetVelsRad.push_back(5.23598776f);
    targetVelsRad.push_back(5.23598776f);
    targetVelsRad.push_back(5.23598776f);
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
        return;
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
            assert(abs(cmdPose[i] - stof(row[i])) < epsilon);
        }
    }

    mc.execCmd(cycleTime);
    cmdPose = mc.getCmdPose();

    for (size_t i=0;i<dof;i++) 
    {
        assert(abs(cmdPose[i] - targetPose[i]) < 0.0001);
    }

    execProf1.reset();
    file.close();

    cout << "testCase_TrapezoidalProfile_101 passed!" << endl;
}

void testCase_TrapezoidalProfile_102(void)
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

    targetVelsRad.push_back(0.52359877f);
    targetVelsRad.push_back(0.52359877f);
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
        return;
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
            assert(abs(cmdPose[i] - stof(row[i])) < epsilon);
        }
    }

    if (mc.execCmd(cycleTime)) return;

    cmdPose = mc.getCmdPose();
    for (size_t i=0;i<dof;i++) 
    {
        assert(abs(cmdPose[i] - targetPose[i]) < 0.0001);
    }

    execProf.reset();
    file.close();

    cout << "testCase_TrapezoidalProfile_102 passed!" << endl;
}

void testCase_TrapezoidalProfile_103()
{
    int dof = 6;
    MotionController mc(dof);

    double cycleTime = 0.01;
    
    assert(mc.execCmd(cycleTime) == false);

    cout << "testCase_TrapezoidalProfile_103 passed!" << endl;
}

void testCase_TrapezoidalProfile_104()
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
    targetVelsRad.push_back(6.54498469f);
    targetVelsRad.push_back(6.54498469f);
    targetVelsRad.push_back(6.54498469f);
    targetVelsRad.push_back(7.54498469f);
    targetVelsRad.push_back(7.54498469f);
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
        return;
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
            assert(abs(cmdPose[i] - stof(row[i])) < epsilon);
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
    if (mc.execCmd(cycleTime)) return;

    cmdPose = mc.getCmdPose();
    for (size_t i=0;i<dof;i++) 
    {
        assert(abs(cmdPose[i] - targetPose[i]) < 0.0001);
    }

    execProf1.reset();
    execProf2.reset();
    execProf3.reset();

    file.close();

    cout << "testCase_TrapezoidalProfile_104 passed!" << endl;
}

void testCase_TrapezoidalProfile_201(void)
{
    
    for (int i=0;i<100;i++) 
    {
        testCase_TrapezoidalProfile_rand();
    }

    cout << "testCase_TrapezoidalProfile_201 passed!" << endl;
}

void testCase_TrapezoidalProfile_rand(void)
{
    int dof = 6;
    MotionController mc(dof);

    vector<double> curPose(dof, 0.0f);
    mc.setCurrentPose(curPose);

    vector<double> targetPose(dof, 0.0f);
    vector<double> targetVelsRad(dof, 0.0f);
    vector<double> targetAccsRad(dof, 0.0f);
    vector<double> targetJerksRad(dof, 0.0f);

    double ratio = 0.0f;    // [0.0, 1.0]

    for (int i=0;i<dof;i++) {
        ratio = (double)((int)rand() % 100) / 100.0;
        if (ratio != 0.0) targetVelsRad[i] = 10.47197551 * ratio;
        else targetVelsRad[i] = 1,0;
        //cout << targetVelsRad[i] << ",";
    }
    //cout << endl;

    for (int i=0;i<dof;i++) {
        ratio = (double)((int)rand() % 100) / 100.0;
        if (ratio != 0.0) targetAccsRad[i] = 31.4159 * ratio;
        else targetAccsRad[i] = 10.0;
        //cout << targetAccsRad[i] << ",";
    }
    //cout << endl;


    for (int tern=0;tern<10;tern++) 
    {
        //cout << tern << endl;

        for (int i=0;i<dof;i++) {
            ratio = (double)((int)rand() % 100) / 100.0;
            targetPose[i] = 3.14159 * ratio - 1.5708;
            //cout << targetPose[i] << ",";
        }
        //cout << endl;

        shared_ptr<TrapezoidalProfile> execProf = make_shared<TrapezoidalProfile>();
        mc.setTrapezoidalProfile(execProf);
        mc.setVelocityProfParam(targetPose, targetVelsRad, targetAccsRad, targetJerksRad);
    }

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

    while (mc.execCmd(cycleTime)) 
    { 
        cmdPose = mc.getCmdPose();
        
        for (size_t i=0;i<dof;i++) 
        {
            cmdVels[i] = (cmdPose[i] - prePose[i]) / cycleTime;
            cmdAccs[i] = (cmdVels[i] - preVels[i]) / cycleTime;
            //cout << cmdPose[i] << ",";
            assert(abs(cmdVels[i] < targetVelsRad[i] * 2.0));

            prePose[i] = cmdPose[i];
            preVels[i] = cmdVels[i];
        }
        //cout << endl;
    }

    cmdPose = mc.getCmdPose();
    /*
    for (size_t i=0;i<dof;i++) 
    {
        cout << cmdPose[i] << ",";
    }
    cout << endl;
    */

}

void testCase_ScurveProfile_001()
{

    ScurveProfile sc;
    // acc, dec, vel, pos
    if (!sc.makeVelProf(100,5,1,1.0)) return;

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
        return;
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

        assert(abs(curPos - stof(row[0])) < epsilon);
        assert(abs(curVel - stof(row[1])) < epsilon);
        assert(abs(curAcc - stof(row[2])) < epsilon);

        prePos = curPos;
        preVel = curVel;
    }

    file.close();

    cout << "testCase_ScurveProfile_001 passed!" << endl;
}

void testCase_ScurveProfile_002()
{

    ScurveProfile sc;
    // acc, dec, vel, pos
    if(!sc.makeVelProf(0.25, 5, 1, 1.0)) return;

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
        return;
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

        assert(abs(curPos - stof(row[0])) < epsilon);
        assert(abs(curVel - stof(row[1])) < epsilon);
        assert(abs(curAcc - stof(row[2])) < epsilon);

        prePos = curPos;
        preVel = curVel;
    }

    file.close();

    cout << "testCase_ScurveProfile_002 passed!" << endl;
}

void testCase_ScurveProfile_003()
{

    ScurveProfile sc;
    // acc, dec, vel, pos
    if (!sc.makeVelProf(10, 100, 1, 1.0)) return;

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
        return;
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

        assert(abs(curPos - stof(row[0])) < epsilon);
        assert(abs(curVel - stof(row[1])) < epsilon);
        assert(abs(curAcc - stof(row[2])) < epsilon);

        prePos = curPos;
        preVel = curVel;
    }

    file.close();

    cout << "testCase_ScurveProfile_003 passed!" << endl;
}

void testCase_ScurveProfile_004() 
{
    TrapezoidalProfile prof;

    assert(abs(prof.getCurDis() == 0.0));
}

void testCase_ScurveProfile_005() 
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

    assert(abs(prof.makeVelProf(tarPos - startPos, vel, acc, jerk)) == false);

    cout << "testCase_ScurveProfile_005 passed!" << endl;
}

void testCase_ScurveProfile_006() 
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

    assert(abs(prof.makeVelProf(tarPos - startPos, vel, acc, jerk)) == false);

    cout << "testCase_ScurveProfile_006 passed!" << endl;    
}

void testCase_ScurveProfile_007() 
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

    assert(abs(prof.makeVelProf(tarPos - startPos, vel, acc, jerk)) == false);

    cout << "testCase_ScurveProfile_007 passed!" << endl;    
}

void testCase_ScurveProfile_008() 
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

    assert(abs(prof.makeVelProf(tarPos - startPos, vel, acc, jerk)) == false);

    cout << "testCase_ScurveProfile_008 passed!" << endl;    
}


void testCase_ScurveProfile_101(void)
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
    targetJerksRad.push_back(25.0f);
    targetJerksRad.push_back(25.0f);
    targetJerksRad.push_back(25.0f);
    targetJerksRad.push_back(25.0f);
    targetJerksRad.push_back(25.0f);
    targetJerksRad.push_back(25.0f);

    for (int i=0;i<dof;i++) 
    {
        targetVelsRad[i] *= 0.2;
        targetAccsRad[i] = 1000000.0;
        targetJerksRad[i] = 1000000000.0;
    }

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

    string filenamep = "./testCase/case_Scurve_101.csv";
    ifstream filep(filenamep);

    if (!filep.is_open()) 
    {
        cerr << "Error: Could not open file " << endl;
        return;
    }
/*
    string line;
    while (getline(filep, line) && mc.execCmd(cycleTime)) 
    { 
        stringstream ss(line);
        string value;
        vector<string> row;

        while (getline(ss, value, ',')) 
        {
            row.push_back(value);
        }
*/

    while (mc.execCmd(cycleTime))
    {
        cmdPose = mc.getCmdPose();
/*
        for (size_t i=0;i<dof;i++) 
        {
            if (abs(cmdPose[i] - stof(row[i])) > 0.0001) return;
        }
*/
        for (size_t i=0;i<dof;i++) 
        {
            cmdVels[i] = (cmdPose[i] - prePose[i]) / cycleTime;
            cmdAccs[i] = (cmdVels[i] - preVels[i]) / cycleTime;
            // cout << cmdPose[i] << ",";

            prePose[i] = cmdPose[i];
            preVels[i] = cmdVels[i];
        }
        // cout << endl;

    }

    cmdPose = mc.getCmdPose();

    for (size_t i=0;i<dof;i++) 
    {
        assert(abs(cmdPose[i] - targetPose[i]) < epsilon);
    }

    execProf1.reset();
    execProf2.reset();

    filep.close();

    cout << "testCase_ScurveProfile_101 passed!" << endl;    
}


void testCase_ScurveProfile_102(void)
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

    string filenamep = "./testCase/case_Scurve102.csv";
    ifstream filep(filenamep);

    if (!filep.is_open()) 
    {
        cerr << "Error: Could not open file " << endl;
        return;
    }

    string line;
    while (getline(filep, line) && mc.execCmd(cycleTime)) 
    {
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
            assert(abs(cmdPose[i] - stof(row[i])) < epsilon);
        }
        
        for (size_t i=0;i<dof;i++) 
        {
            cmdVels[i] = (cmdPose[i] - prePose[i]) / cycleTime;
            cmdAccs[i] = (cmdVels[i] - preVels[i]) / cycleTime;

            prePose[i] = cmdPose[i];
            preVels[i] = cmdVels[i];
        }
    }

    cmdPose = mc.getCmdPose();

    for (size_t i=0;i<dof;i++) 
    {
        assert(abs(cmdPose[i] - targetPose[i]) < epsilon);
    }

    execProf1.reset();
    execProf2.reset();

    filep.close();

    cout << "testCase_ScurveProfile_102 passed!" << endl;    

    return;
}

void testCase_ScurveProfile_201(void)
{
    
    for (int i=0;i<2;i++) 
    {
        testCase_ScurveProfile_rand();
    }

    cout << "testCase_ScurveProfile_201 passed!" << endl;    
    
}

void testCase_ScurveProfile_rand(void)
{
    int dof = 6;
    MotionController mc(dof);

    vector<double> curPose(dof, 0.0f);
    mc.setCurrentPose(curPose);

    vector<double> targetPose(dof, 0.0f);
    vector<double> targetVelsRad(dof, 0.0f);
    vector<double> targetAccsRad(dof, 0.0f);
    vector<double> targetJerksRad(dof, 0.0f);

    double ratio = 0.0f;    // [0.0, 1.0]

    for (int i=0;i<dof;i++) {
        ratio = (double)((int)rand() % 100) / 100.0;
        if (ratio != 0.0) targetVelsRad[i] = 10.47197551 * ratio;
        else targetVelsRad[i] = 1,0;
        //cout << targetVelsRad[i] << ",";
    }
    //cout << endl;

    for (int i=0;i<dof;i++) {
        ratio = (double)((int)rand() % 100) / 100.0;
        if (ratio != 0.0) targetAccsRad[i] = 31.4159 * ratio;
        else targetAccsRad[i] = 10.0;
        //cout << targetAccsRad[i] << ",";
    }
    //cout << endl;

    for (int i=0;i<dof;i++) {
        ratio = (double)((int)rand() % 100) / 100.0;
        if (ratio != 0.0) targetJerksRad[i] = 1000 * ratio;
        else targetJerksRad[i] = 100.0;
        //cout << targetJerksRad[i] << ",";
    }
    //cout << endl;

    for (int tern=0;tern<100;tern++) 
    {
        //cout << tern << endl;

        for (int i=0;i<dof;i++) {
            ratio = (double)((int)rand() % 100) / 100.0;
            targetPose[i] = 3.14159 * ratio - 1.5708;
            //cout << targetPose[i] << ",";
        }
        //cout << endl;

        shared_ptr<ScurveProfile> execProf = make_shared<ScurveProfile>();
        mc.setScurveProfile(execProf);
        mc.setVelocityProfParam(targetPose, targetVelsRad, targetAccsRad, targetJerksRad);
    }

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

    while (mc.execCmd(cycleTime)) 
    { 
        cmdPose = mc.getCmdPose();
        
        for (size_t i=0;i<dof;i++) 
        {
            cmdVels[i] = (cmdPose[i] - prePose[i]) / cycleTime;
            cmdAccs[i] = (cmdVels[i] - preVels[i]) / cycleTime;
            //cout << cmdPose[i] << ",";

            assert(abs(cmdVels[i]) < targetVelsRad[i] * 2);

            prePose[i] = cmdPose[i];
            preVels[i] = cmdVels[i];
        }
        //cout << endl;
    }

    cmdPose = mc.getCmdPose();
    /*
    for (size_t i=0;i<dof;i++) 
    {
        cout << cmdPose[i] << ",";
    }
    cout << endl;
    */
}
