/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/my_server/qnode.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"
#include <ctime>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include "../include/my_server/cudaOpticalFlow.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/
using namespace cv;
using namespace std;
namespace my_server {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
{
    kp = 1.0; kd = 1.0; ki = 1.0, sendData = true;

}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

void QNode::image_callback(const sensor_msgs::ImageConstPtr &msg)
{
    try{
        //clock_t begin = clock();

        mtx.lock();
        _frame2 = cv_bridge::toCvCopy(msg, "bgr8")->image;

        if(opticalFlowStatus)
        {
            _frame = cudaFLow.imageproccess(_frame2);
            controlMsg.speed = cudaFLow.pid.getPIDVal();
            if(sendData)
                controlPub.publish(controlMsg);
        }
        else
            _frame = _frame2.clone();

        debugArray.data.resize(5);
        debugArray.data[0] = cudaFLow.pid.getDebugVals()[0];
        debugArray.data[1] = cudaFLow.pid.getDebugVals()[1];
        debugArray.data[2] = cudaFLow.pid.getDebugVals()[2];
        debugArray.data[3] = cudaFLow.pid.getDebugVals()[3];
        debugArray.data[4] = cudaFLow.pid.getDebugVals()[4];

        debugDataPub.publish(debugArray);

        imagePtrMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", _frame).toImageMsg();
        imageDebug.publish(imagePtrMsg);
        //segmantation fualt verirse yerini toCopy in altına al
        mtx.unlock();

    }catch(cv_bridge::Exception &e){
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void QNode::setDataPublishVal(bool val)
{
    sendData = val;
}

void QNode::changeMaxSpeed(int newVal)
{
    controlMsg.maxSpeed = newVal;
    controlPub.publish(controlMsg);
}

void QNode::frontLedCtrl(bool val)
{
    ledStatusVal.data = val;
    frontLedPub.publish(ledStatusVal);
}

bool QNode::init() {
    ros::init(init_argc,init_argv,"my_server");

    opticalFlowStatus = false;

    if ( ! ros::master::check() ) {
        return false;
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);
    imageDebug = it.advertise("debugImage", 1);
    // Add your ros communications here.
    sub = it.subscribe("/image_raw", 1, &QNode::image_callback,this);
    //chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
    debugDataPub = n.advertise<std_msgs::Float32MultiArray>("debugDataPublisher", 1000);
    servo_publisher = n.advertise<my_server::servo_control>("servo_control", 1000);
    controlPub = n.advertise<my_server::robot_control>("robotControl", 1000);
    frontLedPub = n.advertise<std_msgs::Bool>("frontLedControl", 1000);
    start();
    return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
    ros::init(remappings,"my_server");

	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
    image_transport::ImageTransport it(n);
    // Add your ros communications here.
    //sub = it.subscribe("image_raw", 1, &QNode::image_callback,this);
    sub = it.subscribe("/image_raw/compressed", 1, &QNode::image_callback,this);
    servo_publisher = n.advertise<my_server::servo_control>("servo_control", 1000);
    controlPub = n.advertise<my_server::robot_control>("robotControl", 1000);
    frontLedPub = n.advertise<std_msgs::Bool>("frontLedControl", 1000);
    //chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
	start();
	return true;
}

void QNode::run() {
    //ros::Rate loop_rate(100);

    int count = 0;

    while ( ros::ok() ) {


        ros::spinOnce();
        //loop_rate.sleep();
        //++count;
    }

    //ros::spin();

	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

}  // namespace my_server
