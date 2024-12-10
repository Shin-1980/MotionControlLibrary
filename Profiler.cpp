#include <iostream>
#include <math.h>
#include <vector>
#include <queue>

#include <fstream>
#include <sstream>
#include <string>

class ProfileInfo{
public:
    std::vector<double> targetPose;
    std::vector<double> targetVels;
    std::vector<double> targetAccs;
    std::vector<double> targetDecs;

    std::vector<double> startPose;
    
    int baseCoordinate;
    int dof;

    ProfileInfo(int degreesOfFreedom) :
        dof(degreesOfFreedom),
        targetPose(dof, 0.0f),
        targetVels(dof, 0.0f),
        targetAccs(dof, 0.0f),
        targetDecs(dof, 0.0f),
        startPose(dof, 0.0f),
        baseCoordinate(0)
    {
        // initialization
    }

    void setParam(std::vector<double> targetPose, std::vector<double> targetVels, std::vector<double> targetAccs, std::vector<double> targetDecs){
        this->targetPose = targetPose;
        this->targetVels = targetVels;
        this->targetAccs = targetAccs;
        this->targetDecs = targetDecs;
    }

    void setStartPose(std::vector<double> startPose){
        this->startPose = startPose;
    }

    void setBaseCoordinate(int base){
        this->baseCoordinate = base;
    }

    std::vector<double> getTargetPose(void) {
        return this->targetPose;
    }

    std::vector<double> getTargetVels(void) {
        return this->targetVels;
    }

    std::vector<double> getTargetAccs(void) {
        return this->targetAccs;    
    }

    std::vector<double> getTargetDecs(void) {
        return this->targetDecs;  
    }

    std::vector<double> getStartPose(void) {
        return this->startPose;
    }

    int getBaseCoordinate(void) {
        return this->baseCoordinate;
    }

    std::vector<double> getUnsignedTotalDistance(void) {
        std::vector<double> unsignedTotalDis(this->dof);
        for (size_t i = 0; i < this->dof; ++i) {
            unsignedTotalDis[i] = std::abs(this->targetPose[i] - this->startPose[i]);
        }
        return unsignedTotalDis;
    }

    std::vector<double> getSignedTotalDistance(void) {
        std::vector<double> signedTotalDis(this->dof);
        for (size_t i = 0; i < this->dof; ++i) {
            signedTotalDis[i] = this->targetPose[i] - this->startPose[i];
        }
        return signedTotalDis;
    }
};

class TrapezoidalProfile{
public:
    bool isCreated;
    double accTime = 0;
    double constTime = 0;
    double decTime = 0;
    double accConstTime = 0;
    double totalTime = 0;

    double acc = 0;
    double dec = 0;
    
    double startVel = 0;
    double targetVel = 0;
    double endVel = 0;
    
    double accDis = 0;
    double constDis = 0;
    double decDis = 0;
    double totalDis = 0;
    double curDis = 0;
    double elapsedTime = 0;
    ProfileInfo profInfo;

    TrapezoidalProfile():
        isCreated(false),
        accTime(0.0f),
        constTime(0.0f),
        decTime(0.0f),
        accConstTime(0.0f),
        totalTime(0.0f),
        acc(0.0f),
        dec(0.0f),
        startVel(0.0f),
        targetVel(0.0f),
        endVel(0.0f),
        accDis(0.0f),
        constDis(0.0f),
        decDis(0.0f),
        totalDis(0.0f),
        curDis(0.0f),
        elapsedTime(0.0f),
        profInfo(6)
    {
        // initilization
        
    }

    double getCurDis(void){
        return this->curDis;
    }

    bool isDone(void){
        if (this->elapsedTime >= this->totalTime) 
            return true;
        else
            return false;
    }

    bool isDecelerating(void) {
        if ((this->accTime + this->constTime) < this->elapsedTime && this->elapsedTime < this->totalTime) 
            return true;
        else
            return false;
    }

    bool makeProf(double acc, double dec, double vel, double dis) {

        if (acc <= 0 or dec <= 0 or vel <= 0 or dis <= 0)
            return false;

        this->acc = std::abs(acc);
        this->dec = std::abs(dec);
        this->targetVel = std::abs(vel);

        this->totalDis = abs(dis); 
        this->elapsedTime = 0;
        this->accTime = this->targetVel / this->acc;
        this->decTime = this->targetVel / this->dec;        
        this->accDis = 0.5 * this->targetVel * this->accTime;
        this->decDis = 0.5 * this->targetVel * this->decTime;  
        this->constDis = this->totalDis - (this->accDis + this->decDis);
        
        if (this->constDis < 0) {
            double tmp = 2. * (this->acc * this->dec) * this->totalDis / (this->acc + this->dec);
            if (tmp > 0)
                this->targetVel = sqrt(tmp);
            else
                return false;
            this->accTime = this->targetVel / this->acc;
            this->decTime = this->targetVel / this->dec;        
            this->accDis = 0.5 * this->targetVel * this->accTime;
            this->decDis = 0.5 * this->targetVel * this->decTime; 
            this->constDis = 0;
        }
            
        this->constTime = this->constDis / this->targetVel;
        this->accConstTime = this->accTime + this->constTime;
        this->totalTime = this->accConstTime + this->decTime;

        this->isCreated = true;

        return this->isCreated; 
    }

    bool calDis(double cycleTime) {

        if (!this->isCreated)
            return false;

        this->elapsedTime += cycleTime;
        this->curDis = 0;

        if (this->elapsedTime < this->accTime)
            this->curDis = 0.5 * this->elapsedTime * this->acc * this->elapsedTime;

        else if (this->elapsedTime < this->accConstTime)
            this->curDis = this->accDis + this->targetVel * (this->elapsedTime - this->accTime);
        else {
            double complementaryTime = this->totalTime - this->elapsedTime;
            this->curDis = this->totalDis - 0.5 * complementaryTime * this->dec * complementaryTime;
        }

        if (this->totalTime < this->elapsedTime) {
            this->curDis = this->totalDis;
            return false;
        }

        return true;
    }
};

class Profiler{
public:
    int dof;
    std::vector<double> curPose;
    std::vector<double> curVels;
    std::vector<double> prePose;
    std::vector<double> preVels;
    std::vector<double> curAccs;
    int execProfId;
    int imposedProfId;
    int maxProfNum;
    int storedProfNum;

    TrapezoidalProfile profContainer[8];

    Profiler(int degreesOfFreedom):
        dof(degreesOfFreedom),        
        curPose(dof, 0.0f),
        curVels(dof, 0.0f),
        prePose(dof, 0.0f),
        preVels(dof, 0.0f),
        curAccs(dof, 0.0f),
        execProfId(-1),
        imposedProfId(-1),
        maxProfNum(8),
        storedProfNum(0)
    {
        // initialization
    }

    void setCurrentPose(std::vector<double> curPose){
        this->curPose = curPose;
        this->prePose = curPose;
    }

    void setCmd(std::vector<double> targetPose, std::vector<double> targetVels, std::vector<double> targetAccs, std::vector<double> targetDecs){
        
        if (this->storedProfNum > this->maxProfNum) {
            std::cout << "The number of profiler exceeds the limit" << std::endl;
        }

        this->profContainer[this->storedProfNum].profInfo.setParam(targetPose, targetVels, targetAccs, targetDecs);
        this->storedProfNum++;

    }

    std::vector<double> getCmdPose(void){
        return this->curPose;
    }

    std::vector<double> getCmdVels(void){
        return this->curVels;
    }

    std::vector<double> getCmdAccs(void){
        return this->curAccs;
    }

    bool makeLinearProf(TrapezoidalProfile* prof, std::vector<double> startPose) {
                    
        if (prof == nullptr)
            return false;

        prof->profInfo.setStartPose(startPose);

        std::vector<double> times(this->dof, 0.0f);
        std::vector<double> unsignedTotalDis = prof->profInfo.getUnsignedTotalDistance();
        std::vector<double> targetVel = prof->profInfo.getTargetVels();

        for(size_t i=0; i<this->dof; i++) {
            times[i] = unsignedTotalDis[i] / targetVel[i];
        }
        
        double maxTime = 0.0;

        for (size_t i=0;i<this->dof;i++){
            if (maxTime < times[i]){
                maxTime = times[i];
                prof->profInfo.setBaseCoordinate(i);
            }
        }

        std::vector<double> accs = prof->profInfo.getTargetAccs();
        std::vector<double> decs = prof->profInfo.getTargetDecs();
        std::vector<double> vels = prof->profInfo.getTargetVels();
        std::vector<double> diss = prof->profInfo.getUnsignedTotalDistance();

        for (size_t i=0;i<this->dof;i++){
            if (accs[i] < 0 || decs[i] < 0 || vels[i] < 0)
                return false;
        }

        double acc = accs[prof->profInfo.getBaseCoordinate()];
        double dec = decs[prof->profInfo.getBaseCoordinate()];
        double vel = vels[prof->profInfo.getBaseCoordinate()];
        double dis = diss[prof->profInfo.getBaseCoordinate()];

        return prof->makeProf(acc, dec, vel, dis);
    }

    bool isEnableToExecImposedProf(TrapezoidalProfile* execProf, TrapezoidalProfile* imposedProf) {

        if (imposedProf != nullptr && execProf->isDecelerating()){
            if (execProf->decTime < imposedProf->totalTime)
                return true;
            else
                if (execProf->elapsedTime > (execProf->totalTime - imposedProf->accTime)) return true;
        }

        return false;
    }


    bool execCmd(double cycleTime) {

        if (this->execProfId == -1)
            if (this->storedProfNum == 0) {
                return false;
            }
            else {
                this->execProfId = 0;
                if (!this->makeLinearProf(&this->profContainer[this->execProfId], this->curPose)) 
                    return false;
            }

        if (this->imposedProfId == -1 && this->execProfId < this->storedProfNum - 1) {
            this->imposedProfId = this->execProfId + 1;
            
            if (!this->makeLinearProf(&this->profContainer[this->imposedProfId], 
                                      this->profContainer[this->execProfId].profInfo.getTargetPose()))
                return false;
        
        }

        if (this->profContainer[this->execProfId].calDis(cycleTime)){
            double curDis = this->profContainer[this->execProfId].getCurDis();
            int baseCoordinate = this->profContainer[this->execProfId].profInfo.getBaseCoordinate();
            std::vector<double> totalDis = this->profContainer[this->execProfId].profInfo.getUnsignedTotalDistance();

            double rate = curDis / totalDis[baseCoordinate];

            std::vector<double> startPose = this->profContainer[this->execProfId].profInfo.getStartPose();
            std::vector<double> signedTotalDis = this->profContainer[this->execProfId].profInfo.getSignedTotalDistance();

            for (size_t i=0;i<this->dof;i++){
                this->curPose[i] = startPose[i] + signedTotalDis[i] * rate;
            }

            if (this->isEnableToExecImposedProf(&this->profContainer[this->execProfId], &this->profContainer[this->imposedProfId])) {
                if (this->profContainer[this->imposedProfId].calDis(cycleTime)){
                    double ipDis = this->profContainer[this->imposedProfId].getCurDis();
                    double ipBaseCoordinate = this->profContainer[this->imposedProfId].profInfo.getBaseCoordinate();
                    std::vector<double> ipTotalDis = this->profContainer[this->imposedProfId].profInfo.getUnsignedTotalDistance();
                    double ipRate = ipDis / ipTotalDis[ipBaseCoordinate];  

                    std::vector<double> signedTotalDis = this->profContainer[this->imposedProfId].profInfo.getSignedTotalDistance();
                    for (size_t i=0;i<this->dof;i++){
                        this->curPose[i] += signedTotalDis[i] * ipRate;
                    }
                }
            }

            for (size_t i=0;i<this->dof;i++) {
                this->curVels[i] = (this->curPose[i] - this->prePose[i]) / cycleTime;
                this->curAccs[i] = (this->curVels[i] - this->preVels[i]) / cycleTime;

                this->prePose[i] = this->curPose[i];
                this->preVels[i] = this->curVels[i];
            }
        }

        if (this->execProfId != -1 && this->profContainer[this->execProfId].isDone()){
            if (this->imposedProfId == -1) {
                std::vector<double> endPos = this->profContainer[this->execProfId].profInfo.getTargetPose();
                for (size_t i=0;i<this->dof;i++) {
                    this->curVels[i] = 0.0;
                    this->preVels[i] = 0.0;
                    this->curPose[i] = endPos[i];
                    this->execProfId = -1;
                }
                return false;
            }
            else {
                this->execProfId = this->imposedProfId;
                this->imposedProfId = -1;
            }
        }

        return true;

    }

};


bool testCase01() {
    TrapezoidalProfile prof;

    double startPos = 0.0;
    double tarPos = 50;
    double acc = 100;
    double dec = 100;
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
        return 1;
    }

    if (!prof.makeProf(acc,dec,vel,tarPos - startPos)) return false;

    std::string line;
    while (std::getline(file, line) && prof.calDis(cycleTime)) { // Read the file line by line
        std::stringstream ss(line);   // Use stringstream to parse the line
        std::string value;
        std::vector<std::string> row; // Store the values of the current row

        while (std::getline(ss, value, ',')) { // Split by comma
            row.push_back(value);
        }

        double dis = prof.getCurDis();
        /*
        std::cout << dis << std::endl;
        std::cout << row[0] << std::endl;
        */
        if (abs(dis - std::stof(row[0]) > 0.0001)) {
            return false;
        }
    }
    
    if (abs(prof.getCurDis() - tarPos > 0.0001)) {
        return false;
    }

    return true;
/*
    if (prof.makeProf(acc,dec,vel,tarPos - startPos)) {

        while (prof.calDis(cycleTime)) {
            std::cout << prof.getCurDis() << std::endl;
        }
    }
*/
}

bool testCase02() {
    TrapezoidalProfile prof;

    double startPos = 0.0;
    double tarPos = 25;
    double acc = 100;
    double dec = 100;
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

    if (!prof.makeProf(acc,dec,vel,tarPos - startPos)) return false;

    std::string line;
    while (std::getline(file, line) && prof.calDis(cycleTime)) { // Read the file line by line
        std::stringstream ss(line);   // Use stringstream to parse the line
        std::string value;
        std::vector<std::string> row; // Store the values of the current row

        while (std::getline(ss, value, ',')) { // Split by comma
            row.push_back(value);
        }

        double dis = prof.getCurDis();
        
        /*
        std::cout << dis << std::endl;
        std::cout << row[0] << std::endl;
        */

        if (abs(dis - std::stof(row[0]) > 0.0001)) {
            return false;
        }
    }
    
    if (abs(prof.getCurDis() - tarPos > 0.0001)) {
        return false;
    }

    return true;

}

bool testCase101(void){
        
    int dof = 6;
    Profiler profiler(dof);

    std::vector<double> curPose;
    curPose.push_back(1.5708f);
    curPose.push_back(-3.14159f);
    curPose.push_back(1.5708f);
    curPose.push_back(0.0f);
    curPose.push_back(0.0f);
    curPose.push_back(0.0f);

    profiler.setCurrentPose(curPose);

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

    profiler.setCmd(targetPose, targetVelsRad, targetAccsRad, targetAccsRad);

    double cycleTime = 0.01;

    std::vector<double> cmdPose = curPose;

    std::string filename = "./testCase/case101.csv";
    std::ifstream file(filename);

    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        return 1;
    }

    std::string line;
    while (std::getline(file, line) && profiler.execCmd(cycleTime)) { // Read the file line by line
        std::stringstream ss(line);   // Use stringstream to parse the line
        std::string value;
        std::vector<std::string> row; // Store the values of the current row

        while (std::getline(ss, value, ',')) { // Split by comma
            row.push_back(value);
        }

        cmdPose = profiler.getCmdPose();

        for (size_t i=0;i<dof;i++) {
            if (abs(cmdPose[i] - std::stof(row[i])) > 0.0001) return false;
        }
    }

    cmdPose = profiler.getCmdPose();

    for (size_t i=0;i<dof;i++) {
        if (abs(cmdPose[i] - targetPose[i]) > 0.0001) return false;
    }

    return true;
}

bool testCase102(void){
        
    int dof = 6;
    Profiler profiler(dof);

    std::vector<double> curPose;
    curPose.push_back(1.5708f);
    curPose.push_back(-3.14159f);
    curPose.push_back(1.5708f);
    curPose.push_back(0.0f);
    curPose.push_back(0.0f);
    curPose.push_back(0.0f);

    profiler.setCurrentPose(curPose);

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

    profiler.setCmd(targetPose, targetVelsRad, targetAccsRad, targetAccsRad);

    double cycleTime = 0.01;

    std::vector<double> cmdPose = curPose;

    std::string filename = "./testCase/case102.csv";
    std::ifstream file(filename);

    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        return 1;
    }

    std::string line;
    while (std::getline(file, line) && profiler.execCmd(cycleTime)) { // Read the file line by line
        std::stringstream ss(line);   // Use stringstream to parse the line
        std::string value;
        std::vector<std::string> row; // Store the values of the current row

        while (std::getline(ss, value, ',')) { // Split by comma
            row.push_back(value);
        }

        cmdPose = profiler.getCmdPose();

        for (size_t i=0;i<dof;i++) {
            if (abs(cmdPose[i] - std::stof(row[i])) > 0.0001) {
                return false;
            }
        }
    }

    if (profiler.execCmd(cycleTime)) return false;

    cmdPose = profiler.getCmdPose();
    for (size_t i=0;i<dof;i++) {
        if (abs(cmdPose[i] - targetPose[i]) > 0.0001) return false;
    }

    return true;
}

bool testCase109(){
    int dof = 6;
    Profiler profiler(dof);

    std::vector<double> curPose;
    curPose.push_back(1.5708f);
    curPose.push_back(-3.14159f);
    curPose.push_back(1.5708f);
    curPose.push_back(0.0f);
    curPose.push_back(0.0f);
    curPose.push_back(0.0f);

    profiler.setCurrentPose(curPose);

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

    profiler.setCmd(targetPose, targetVelsRad, targetAccsRad, targetAccsRad);

    targetPose[0] = 0.456344;
    targetPose[1] = -2.54862;
    targetPose[2] = 2.16375;
    targetPose[3] = 1.78586;
    targetPose[4] = -0.350709;
    targetPose[5] = -1.29741;
    profiler.setCmd(targetPose, targetVelsRad, targetAccsRad, targetAccsRad);

    targetPose[0] = -0.449;
    targetPose[1] = -0.890494;
    targetPose[2] = 2.60354;
    targetPose[3] = 2.06081;
    targetPose[4] = -0.0605401;
    targetPose[5] = -0.624552;
    profiler.setCmd(targetPose, targetVelsRad, targetAccsRad, targetAccsRad);

    double cycleTime = 0.01;

    std::vector<double> cmdPose = curPose;

    std::string filename = "./testCase/case109.csv";
    std::ifstream file(filename);

    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        return 1;
    }

    std::string line;
    while (std::getline(file, line) && profiler.execCmd(cycleTime)) { // Read the file line by line
        std::stringstream ss(line);   // Use stringstream to parse the line
        std::string value;
        std::vector<std::string> row; // Store the values of the current row

        while (std::getline(ss, value, ',')) { // Split by comma
            row.push_back(value);
        }

        cmdPose = profiler.getCmdPose();

        for (size_t i=0;i<dof;i++) {
            if (abs(cmdPose[i] - std::stof(row[i])) > 0.0001) {
                return false;
            }
        }
    }

    if (profiler.execCmd(cycleTime)) return false;

    cmdPose = profiler.getCmdPose();
    for (size_t i=0;i<dof;i++) {
        if (abs(cmdPose[i] - targetPose[i]) > 0.0001) return false;
    }

    return true;

}



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


}