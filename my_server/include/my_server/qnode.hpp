/**
 * @file /include/my_server/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef my_server_QNODE_HPP_
#define my_server_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include "cudaOpticalFlow.hpp"
#include <my_server/servo_control.h>
#include <my_server/robot_control.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace my_server {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
    bool init();
    bool init(const std::string &master_url, const std::string &host_url);
    void run();
    //void publishConstMsg(QString s, double val);
    void image_callback(const sensor_msgs::ImageConstPtr& msg);
    cv::Mat imageProccess(cv::Mat);
    bool getOpticalFlowStatus(){ return opticalFlowStatus;}
    void setOpticalFlowStatus(bool _stat) { opticalFlowStatus = _stat; }
    void changeMaxSpeed(int newVal);
    void setDataPublishVal(bool val);
    void frontLedCtrl(bool val);

    cudaOpticalFlow cudaFLow;

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
    cv::Mat getFrame() { return _frame;}
    void log( const LogLevel &level, const std::string &msg);

Q_SIGNALS:
	void loggingUpdated();
    void rosShutdown();

private:
    int init_argc;
    Mutex mtx;
	char** init_argv;
    my_server::robot_control controlMsg;
    ros::Publisher chatter_publisher;
    QStringListModel logging_model;
    std_msgs::Float32MultiArray debugArray;
    std_msgs::Bool ledStatusVal;
    //std_msgs::Int16 _speed;

    image_transport::Subscriber sub;
    image_transport::Publisher imageDebug;

    ros::Publisher servo_publisher;
    ros::Publisher debugDataPub;
    ros::Publisher controlPub;
    ros::Publisher frontLedPub;

    cv::Mat imgA, imgB;
    cv::Mat _frame, _frame2;
    sensor_msgs::ImagePtr imagePtrMsg;
    double kp, kd, ki;
    bool opticalFlowStatus, sendData;
};

}  // namespace my_server

#endif /* my_server_QNODE_HPP_ */
