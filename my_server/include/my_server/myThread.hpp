#ifndef my_server_MYTHREAD_HPP_
#define my_server_MYTHREAD_HPP_

#include <iostream>

#include <QThread>
#include <QDebug>

namespace my_server {

class myThread : public QThread
{
    Q_OBJECT
public:
    explicit myThread(QObject *parent = 0, bool b = false);
    ~myThread();
    void run();

    bool Stop;

Q_SIGNALS:
    void valueChanged();

public Q_SLOTS:

private:
    int _value;


};

}

#endif
