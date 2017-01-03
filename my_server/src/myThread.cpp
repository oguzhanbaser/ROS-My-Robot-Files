#include <iostream>
#include <QThread>
#include <QDebug>
#include "../include/my_server/myThread.hpp"


namespace my_server {

myThread::myThread(QObject *parent, bool b) : QThread(parent), Stop(b){
    _value = 0;
}

myThread::~myThread()
{
    Stop = true;
}

void myThread::run()
{
    while(true)
    {
        if(this->Stop) break;
        //_value++;
        Q_EMIT valueChanged();
        //if(_value == 1000) _value = 0;
        this->msleep(50);
    }
}



}
