#include "TestSuite.h"

bool testCase01() {
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

    std::string filename = "./testCase/case1.csv";
    std::ifstream file(filename);

    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        return false;
    }

    if (!prof.makeProf(acc,vel,tarPos - startPos)) return false;

    std::string line;
    while (std::getline(file, line) && prof.calDis(cycleTime)) { // Read the file line by line
        std::stringstream ss(line);   // Use stringstream to parse the line
        std::string value;
        std::vector<std::string> row; // Store the values of the current row

        while (std::getline(ss, value, ',')) { // Split by comma
            row.push_back(value);
        }

        double dis = prof.getCurDis();
        if (abs(dis - std::stof(row[0]) > 0.0001)) {
            return false;
        }
    }
    
    if (abs(prof.getCurDis() - tarPos > 0.0001)) {
        return false;
    }

    file.close();

    return true;
}

bool testCase02() {
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

    std::string filename = "./testCase/case2.csv";
    std::ifstream file(filename);

    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        return 1;
    }

    if (!prof.makeProf(acc,vel,tarPos - startPos)) return false;

    std::string line;
    while (std::getline(file, line) && prof.calDis(cycleTime)) { // Read the file line by line
        std::stringstream ss(line);   // Use stringstream to parse the line
        std::string value;
        std::vector<std::string> row; // Store the values of the current row

        while (std::getline(ss, value, ',')) { // Split by comma
            row.push_back(value);
        }

        double dis = prof.getCurDis();
        
        if (abs(dis - std::stof(row[0]) > 0.0001)) {
            return false;
        }
    }
    
    if (abs(prof.getCurDis() - tarPos > 0.0001)) {
        return false;
    }

    file.close();

    return true;

}

bool testCase101(void){
    int dof = 6;
    MotionController mc(dof);

    TrapezoidalProfile* execProf1 = new TrapezoidalProfile();
    
    mc.setTrapezoidalProfile(execProf1);

    std::vector<double> curPose;
    curPose.push_back(1.5708f);
    curPose.push_back(-3.14159f);
    curPose.push_back(1.5708f);
    curPose.push_back(0.0f);
    curPose.push_back(0.0f);
    curPose.push_back(0.0f);

    mc.setCurrentPose(curPose);

    std::vector<double> targetPose;
    targetPose.push_back(0.693782f);
    targetPose.push_back(-2.36364f);
    targetPose.push_back(2.38406f);
    targetPose.push_back(2.1103f);
    targetPose.push_back(-1.07417f);
    targetPose.push_back(-1.14081f);

    std::vector<double> targetVelsRad;
    targetVelsRad.push_back(5.23598776f);
    targetVelsRad.push_back(5.23598776f);
    targetVelsRad.push_back(6.54498469f);
    targetVelsRad.push_back(6.54498469f);
    targetVelsRad.push_back(6.54498469f);
    targetVelsRad.push_back(10.47197551f);

    std::vector<double> targetAccsRad;
    targetAccsRad.push_back(17.45329252f);
    targetAccsRad.push_back(17.45329252f);
    targetAccsRad.push_back(26.17993878f);
    targetAccsRad.push_back(26.17993878f);
    targetAccsRad.push_back(26.17993878f);
    targetAccsRad.push_back(34.90658504f);
    mc.setCmd(targetPose, targetVelsRad, targetAccsRad);

    double cycleTime = 0.01;

    std::vector<double> cmdPose = curPose;

    std::string filename = "./testCase/case101.csv";
    std::ifstream file(filename);

    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        return 1;
    }
    std::string line;
    while (std::getline(file, line) && mc.execCmd(cycleTime)) { // Read the file line by line
        std::stringstream ss(line);   // Use stringstream to parse the line
        std::string value;
        std::vector<std::string> row; // Store the values of the current row

        while (std::getline(ss, value, ',')) { // Split by comma
            row.push_back(value);
        }

        cmdPose = mc.getCmdPose();

        for (size_t i=0;i<dof;i++) {
            if (abs(cmdPose[i] - std::stof(row[i])) > 0.0001) return false;
        }
    }

    cmdPose = mc.getCmdPose();

    for (size_t i=0;i<dof;i++) {
        if (abs(cmdPose[i] - targetPose[i]) > 0.0001) return false;
    }

    return true;
}

bool testCase102(void){
        
    int dof = 6;
    MotionController mc(dof);

    TrapezoidalProfile* execPorf = new TrapezoidalProfile();
    mc.setTrapezoidalProfile(execPorf);

    std::vector<double> curPose;
    curPose.push_back(1.5708f);
    curPose.push_back(-3.14159f);
    curPose.push_back(1.5708f);
    curPose.push_back(0.0f);
    curPose.push_back(0.0f);
    curPose.push_back(0.0f);

    mc.setCurrentPose(curPose);

    std::vector<double> targetPose;
    targetPose.push_back(0.693782f);
    targetPose.push_back(-2.36364f);
    targetPose.push_back(2.38406f);
    targetPose.push_back(2.1103f);
    targetPose.push_back(-1.07417f);
    targetPose.push_back(-1.14081f);

    std::vector<double> targetVelsRad;

    targetVelsRad.push_back(0.52359878f);
    targetVelsRad.push_back(0.52359878f);
    targetVelsRad.push_back(0.65449847f);
    targetVelsRad.push_back(0.65449847f);
    targetVelsRad.push_back(0.65449847f);
    targetVelsRad.push_back(1.04719755f);

    std::vector<double> targetAccsRad;
    targetAccsRad.push_back(17.45329252f);
    targetAccsRad.push_back(17.45329252f);
    targetAccsRad.push_back(26.17993878f);
    targetAccsRad.push_back(26.17993878f);
    targetAccsRad.push_back(26.17993878f);
    targetAccsRad.push_back(34.90658504f);

    mc.setCmd(targetPose, targetVelsRad, targetAccsRad);

    double cycleTime = 0.01;

    std::vector<double> cmdPose = curPose;

    std::string filename = "./testCase/case102.csv";
    std::ifstream file(filename);

    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        return 1;
    }

    std::string line;
    while (std::getline(file, line) && mc.execCmd(cycleTime)) { // Read the file line by line
        std::stringstream ss(line);   // Use stringstream to parse the line
        std::string value;
        std::vector<std::string> row; // Store the values of the current row

        while (std::getline(ss, value, ',')) { // Split by comma
            row.push_back(value);
        }

        cmdPose = mc.getCmdPose();

        for (size_t i=0;i<dof;i++) {
            if (abs(cmdPose[i] - std::stof(row[i])) > 0.0001) {
                return false;
            }
        }
    }

    if (mc.execCmd(cycleTime)) return false;

    cmdPose = mc.getCmdPose();
    for (size_t i=0;i<dof;i++) {
        if (abs(cmdPose[i] - targetPose[i]) > 0.0001) return false;
    }

    return true;
}

bool testCase109(){
    int dof = 6;
    MotionController mc(dof);

    TrapezoidalProfile* execPorf1 = new TrapezoidalProfile();
    TrapezoidalProfile* execPorf2 = new TrapezoidalProfile();
    TrapezoidalProfile* execPorf3 = new TrapezoidalProfile();

    mc.setTrapezoidalProfile(execPorf1);
    mc.setTrapezoidalProfile(execPorf2);
    mc.setTrapezoidalProfile(execPorf3);

    std::vector<double> curPose;
    curPose.push_back(1.5708f);
    curPose.push_back(-3.14159f);
    curPose.push_back(1.5708f);
    curPose.push_back(0.0f);
    curPose.push_back(0.0f);
    curPose.push_back(0.0f);
    mc.setCurrentPose(curPose);

    std::vector<double> targetPose;
    targetPose.push_back(0.693782f);
    targetPose.push_back(-2.36364f);
    targetPose.push_back(2.38406f);
    targetPose.push_back(2.1103f);
    targetPose.push_back(-1.07417f);
    targetPose.push_back(-1.14081f);

    std::vector<double> targetVelsRad;
    targetVelsRad.push_back(5.23598776f);
    targetVelsRad.push_back(5.23598776f);
    targetVelsRad.push_back(6.54498469f);
    targetVelsRad.push_back(6.54498469f);
    targetVelsRad.push_back(6.54498469f);
    targetVelsRad.push_back(10.47197551f);

    std::vector<double> targetAccsRad;
    targetAccsRad.push_back(17.45329252f);
    targetAccsRad.push_back(17.45329252f);
    targetAccsRad.push_back(26.17993878f);
    targetAccsRad.push_back(26.17993878f);
    targetAccsRad.push_back(26.17993878f);
    targetAccsRad.push_back(34.90658504f);
    mc.setCmd(targetPose, targetVelsRad, targetAccsRad);

    targetPose[0] = 0.456344;
    targetPose[1] = -2.54862;
    targetPose[2] = 2.16375;
    targetPose[3] = 1.78586;
    targetPose[4] = -0.350709;
    targetPose[5] = -1.29741;
    mc.setCmd(targetPose, targetVelsRad, targetAccsRad);

    targetPose[0] = -0.449;
    targetPose[1] = -0.890494;
    targetPose[2] = 2.60354;
    targetPose[3] = 2.06081;
    targetPose[4] = -0.0605401;
    targetPose[5] = -0.624552;
    mc.setCmd(targetPose, targetVelsRad, targetAccsRad);

    double cycleTime = 0.01;
    std::vector<double> cmdPose = curPose;
    std::string filename = "./testCase/case109.csv";

    std::ifstream file(filename);

    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        return 1;
    }

/*
    std::ofstream foutp;
    foutp.open(filename);
*/

    std::string line;
    while (std::getline(file, line) && mc.execCmd(cycleTime)) { // Read the file line by line
        std::stringstream ss(line);
        std::string value;
        std::vector<std::string> row;

        while (std::getline(ss, value, ',')) {
            row.push_back(value);
        }

//    while(mc.execCmd(cycleTime)) {
        //std::cout << "1097" << std::endl;
        cmdPose = mc.getCmdPose();

        for (size_t i=0;i<dof;i++) {
            if (abs(cmdPose[i] - std::stof(row[i])) > 0.0001) {
                return false;
            }
        }
    }

/*
        for (size_t i=0;i<dof;i++) {
            foutp << cmdPose[i] << ",";
        }    
        //std::cout << std::endl;
        foutp << std::endl;
    }
*/       
    if (mc.execCmd(cycleTime)) return false;

    cmdPose = mc.getCmdPose();
    for (size_t i=0;i<dof;i++) {
        if (abs(cmdPose[i] - targetPose[i]) > 0.0001) return false;
    }

    return true;

}

bool testCaseXXX(){
    int dof = 6;
    MotionController mc(dof);

    std::vector<double> curPose;
    curPose.push_back(1.5708f);
    curPose.push_back(-3.14159f);
    curPose.push_back(1.5708f);
    curPose.push_back(0.0f);
    curPose.push_back(0.0f);
    curPose.push_back(0.0f);

    mc.setCurrentPose(curPose);

    std::vector<double> targetPose;
    targetPose.push_back(1.68019f);
    targetPose.push_back(-2.35679f);
    targetPose.push_back(2.45324f);
    targetPose.push_back(2.56348f);
    targetPose.push_back(-0.904948f);
    targetPose.push_back(3.63343f);

    std::vector<double> targetVelsRad;
    targetVelsRad.push_back(5.23598776f);
    targetVelsRad.push_back(5.23598776f);
    targetVelsRad.push_back(6.54498469f);
    targetVelsRad.push_back(6.54498469f);
    targetVelsRad.push_back(6.54498469f);
    targetVelsRad.push_back(10.47197551f);

    std::vector<double> targetAccsRad;
    targetAccsRad.push_back(17.45329252f);
    targetAccsRad.push_back(17.45329252f);
    targetAccsRad.push_back(26.17993878f);
    targetAccsRad.push_back(26.17993878f);
    targetAccsRad.push_back(26.17993878f);
    targetAccsRad.push_back(34.90658504f);

    mc.setCmd(targetPose, targetVelsRad, targetAccsRad);

    targetPose[0] = 0.0;
    targetPose[1] = 0.0;
    targetPose[2] = 1.5708;
    targetPose[3] = 0.0;
    targetPose[4] = 0.0;
    targetPose[5] = 0.0;
    mc.setCmd(targetPose, targetVelsRad, targetAccsRad);

    double cycleTime = 0.1;

    std::vector<double> cmdPose = curPose;

    for (size_t i=0;i<dof;i++) {
        std::cout << "vertex[" << i << "] = " << curPose[i] << ';' << std::endl;
    }   
    std::cout << std::endl;

    while (mc.execCmd(cycleTime)) { 
        cmdPose = mc.getCmdPose();
        for (size_t i=0;i<dof;i++) {
            std::cout << "vertex[" << i << "] = " << cmdPose[i] << ';' << std::endl;
        }   
        std::cout << std::endl;
    }

    if (mc.execCmd(cycleTime)) return false;

    cmdPose = mc.getCmdPose();

    for (size_t i=0;i<dof;i++) {
        std::cout << "vertex[" << i << "] = " << cmdPose[i] << ';' << std::endl;
    }   
    std::cout << std::endl;

    return true;

}

bool testCase_201(){

    ScurveProfile sc;
    // acc, dec, vel, pos
    if (!sc.makeProf(1, 5, 100)) return false;

    double cycleTime = 0.2;
    double curPos = 0.0;
    double prePos = 0.0;
    double curVel = 0.0;
    double preVel = 0.0;
    double curAcc = 0.0;

    std::string filename = "./testCase/case201.csv";
    std::ifstream file(filename);

    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        return 1;
    }

    std::string line;
    while (std::getline(file, line) && sc.calDis(cycleTime)) { // Read the file line by line
        std::stringstream ss(line);
        std::string value;
        std::vector<std::string> row;

        while (std::getline(ss, value, ',')) {
            row.push_back(value);
        }

        curPos = sc.getCurDis();
        curVel = (curPos - prePos) / cycleTime;
        curAcc = (curVel - preVel) / cycleTime;

        if (abs(curPos - std::stof(row[0])) > 0.0001) {
                return false;
        }
        if (abs(curVel - std::stof(row[1])) > 0.0001) {
                return false;
        }
        if (abs(curAcc - std::stof(row[2])) > 0.0001) {
                return false;
        }

        prePos = curPos;
        preVel = curVel;
    }

    return true;

}

bool testCase_202(){

    ScurveProfile sc;
    // acc, dec, vel, pos
    if(!sc.makeProf(1, 5, 0.25)) return false;

    double cycleTime = 0.2;
    double curPos = 0.0;
    double prePos = 0.0;
    double curVel = 0.0;
    double preVel = 0.0;
    double curAcc = 0.0;

    std::string filename = "./testCase/case202.csv";
    std::ifstream file(filename);

    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        return 1;
    }

    std::string line;
    while (std::getline(file, line) && sc.calDis(cycleTime)) { // Read the file line by line
        std::stringstream ss(line);
        std::string value;
        std::vector<std::string> row;

        while (std::getline(ss, value, ',')) {
            row.push_back(value);
        }

        curPos = sc.getCurDis();
        curVel = (curPos - prePos) / cycleTime;
        curAcc = (curVel - preVel) / cycleTime;

        if (abs(curPos - std::stof(row[0])) > 0.0001) {
                return false;
        }
        if (abs(curVel - std::stof(row[1])) > 0.0001) {
                return false;
        }
        if (abs(curAcc - std::stof(row[2])) > 0.0001) {
                return false;
        }

        prePos = curPos;
        preVel = curVel;
    }

    return true;

}

bool testCase_203(){

    ScurveProfile sc;
    // acc, dec, vel, pos
    if (!sc.makeProf(1, 100, 10)) return false;

    double cycleTime = 0.2;
    double curPos = 0.0;
    double prePos = 0.0;
    double curVel = 0.0;
    double preVel = 0.0;
    double curAcc = 0.0;

    std::string filename = "./testCase/case203.csv";
    std::ifstream file(filename);

    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        return 1;
    }

    std::string line;
    while (std::getline(file, line) && sc.calDis(cycleTime)) { // Read the file line by line
        std::stringstream ss(line);
        std::string value;
        std::vector<std::string> row;

        while (std::getline(ss, value, ',')) {
            row.push_back(value);
        }

        curPos = sc.getCurDis();
        curVel = (curPos - prePos) / cycleTime;
        curAcc = (curVel - preVel) / cycleTime;

        if (abs(curPos - std::stof(row[0])) > 0.0001) {
                return false;
        }
        if (abs(curVel - std::stof(row[1])) > 0.0001) {
                return false;
        }
        if (abs(curAcc - std::stof(row[2])) > 0.0001) {
                return false;
        }

        prePos = curPos;
        preVel = curVel;
    }

    return true;

}

bool testCase301(void){
        
    int dof = 6;
    MotionController mc(dof);

    ScurveProfile* execPorf1 = new ScurveProfile();
    mc.setScurveProfile(execPorf1);
    ScurveProfile* execPorf2 = new ScurveProfile();
    mc.setScurveProfile(execPorf2);

    std::vector<double> curPose;
    curPose.push_back(1.5708f);
    curPose.push_back(-3.14159f);
    curPose.push_back(1.5708f);
    curPose.push_back(0.0f);
    curPose.push_back(0.0f);
    curPose.push_back(0.0f);

    mc.setCurrentPose(curPose);

    std::vector<double> targetPose;

    targetPose.push_back(0.0f);
    targetPose.push_back(0.0f);
    targetPose.push_back(1.5708f);
    targetPose.push_back(0.0f);
    targetPose.push_back(0.0f);
    targetPose.push_back(0.0f);

    std::vector<double> targetVelsRad;
    targetVelsRad.push_back(5.23598776f);
    targetVelsRad.push_back(5.23598776f);
    targetVelsRad.push_back(6.54498469f);
    targetVelsRad.push_back(6.54498469f);
    targetVelsRad.push_back(6.54498469f);
    targetVelsRad.push_back(10.47197551f);

    std::vector<double> targetAccsRad;
    targetAccsRad.push_back(17.45329252f);
    targetAccsRad.push_back(17.45329252f);
    targetAccsRad.push_back(26.17993878f);
    targetAccsRad.push_back(26.17993878f);
    targetAccsRad.push_back(26.17993878f);
    targetAccsRad.push_back(34.90658504f);

    mc.setCmd(targetPose, targetVelsRad, targetAccsRad);

    targetPose[0] = 0.0;
    targetPose[1] = 0.0;
    targetPose[2] = 0.0;
    targetPose[3] = 0.0;
    targetPose[4] = 0.0;
    targetPose[5] = 0.0;
    
    mc.setCmd(targetPose, targetVelsRad, targetAccsRad);

    double cycleTime = 0.01;

    std::vector<double> cmdPose = curPose;
    std::vector<double> prePose = curPose;
    std::vector<double> cmdVels;
    cmdVels.push_back(0.0f);
    cmdVels.push_back(0.0f);
    cmdVels.push_back(0.0f);
    cmdVels.push_back(0.0f);
    cmdVels.push_back(0.0f);
    cmdVels.push_back(0.0f);

    std::vector<double> preVels = cmdVels;
    std::vector<double> cmdAccs = cmdVels;

    std::string filenamep = "./testCase/case301_pose.csv";
    //std::string filenamev = "./testCase/case301_vels.csv";
    //std::string filenamea = "./testCase/case301_accs.csv";

    std::ifstream filep(filenamep);
    //std::ifstream filev(filenamev);
    //std::ifstream filea(filenamea);

    if (!filep.is_open()) {
        std::cerr << "Error: Could not open file " << std::endl;
        return false;
    }

/*
    std::ofstream foutp;
    std::ofstream foutv;
    std::ofstream fouta;

    foutp.open(filenamep);
    foutv.open(filenamev);
    fouta.open(filenamea);
*/
    std::string line;
    while (std::getline(filep, line) && mc.execCmd(cycleTime)) { // Read the file line by line
        std::stringstream ss(line);
        std::string value;
        std::vector<std::string> row;

        while (std::getline(ss, value, ',')) {
            row.push_back(value);
        }

        cmdPose = mc.getCmdPose();

        for (size_t i=0;i<dof;i++) {
            if (abs(cmdPose[i] - std::stof(row[i])) > 0.0001) return false;
        }

        for (size_t i=0;i<dof;i++) {
            cmdVels[i] = (cmdPose[i] - prePose[i]) / cycleTime;
            cmdAccs[i] = (cmdVels[i] - preVels[i]) / cycleTime;

            //std::cout << cmdPose[i] << ",";
            //foutp << cmdPose[i] << ",";
            //foutv << cmdVels[i] << ",";
            //fouta << cmdAccs[i] << ",";

            prePose[i] = cmdPose[i];
            preVels[i] = cmdVels[i];
        }
        //std::cout << std::endl;
        //foutp << std::endl;
        //foutv << std::endl;
        //fouta << std::endl;

    }

    cmdPose = mc.getCmdPose();

    for (size_t i=0;i<dof;i++) {
        if (abs(cmdPose[i] - targetPose[i]) > 0.0001) return false;
    }

    return true;
}

