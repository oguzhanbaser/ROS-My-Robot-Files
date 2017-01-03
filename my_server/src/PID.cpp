#include <iostream>

#include <../include/my_server/PID.hpp>
#include <opencv2/opencv.hpp>
#include <QString>

namespace my_server {

PID::~PID(){}

PID::PID()
{
    kp = 1.0, kd = 1.0, ki = 1.0;
    lastTime = 0, timeNow = 0;
    controlSignal = 0;
    lastControlSignal = 0;
    limitControlSignal = 200;
    noiseLevel = 5;

    lastError = 0;
    in = 0;
    limitIn = 100;
    limitPID = 255;
}

int PID::getPIDVal()
{
    return PIDval;
}

void PID::resetValues()
{
    PIDval = 0;
    in = 0;
    lastError = 0;
    lastControlSignal = 0;
    controlSignal = 0;
}

int PID::compute(double val)
{
    //std::cout << val << "\t";

    //controlSignal = lastControlSignal + val;

    if(controlSignal > limitControlSignal) controlSignal = limitControlSignal;

    if(controlSignal < -limitControlSignal) controlSignal = -limitControlSignal;

    if(controlSignal < noiseLevel && controlSignal > -noiseLevel) controlSignal = 0;


    error = val;
    lastControlSignal = controlSignal;

    pr = error * kp;

    timeNow = cv::getTickCount();
    time = (double)(timeNow - lastTime) / (double)cv::getTickFrequency();
    //der = (error - lastError) * kd / time;
    der = (error - lastError) * kd;

    //in = in + (error * ki) * time;
    in = in + (error * ki);
    //std::cout << controlSignal << "\t" <<in << std::endl;

    if(in > limitIn) in = limitIn;
    if(in < -limitIn) in = -limitIn;

    PIDval = pr + der + in;
    //std::cout << time << std::endl;
    //std::cout << "Ctrl: " << controlSignal << " Pr: " << pr << " Der: " << der << " In: " << in << " PID: " << PIDval << std::endl;
    if(PIDval > limitPID) PIDval = limitPID;
    if(PIDval < -limitPID) PIDval = -limitPID;

    if(PIDval < noiseLevel && PIDval > noiseLevel) PIDval = 0;

    debugData[0] = controlSignal;
    debugData[1] = pr;
    debugData[2] = der;
    debugData[3] = in;
    debugData[4] = PIDval;

    setLastTime();

    lastError = error;

    return PIDval;
}

double *PID::getDebugVals()
{
    return debugData;
}

void PID::setConstants(double _kp, double _kd, double _ki)
{
    kp = _kp;
    kd = _kd;
    ki = _ki;
}

void PID::setConstants(std::string name, double val)
{
    if(name == "kp")
    {
        kp = val;
    }else if(name == "kd")
    {
        kd = val;
    }else if(name == "ki")
    {
        ki = val;
    }
}

void PID::setLastTime()
{
    lastTime = cv::getTickCount();
}

int PID::getControlSignal()
{
    return controlSignal;
}

}
