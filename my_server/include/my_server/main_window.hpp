/**
 * @file /include/my_server/main_window.hpp
 *
 * @brief Qt based gui for my_server.
 *
 * @date November 2010
 **/
#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include <opencv2/opencv.hpp>
#include "myThread.hpp"
#include "cudaOpticalFlow.hpp"
#include "PID.hpp"

#include <QTimer>
#include <QImage>
#include <QPixmap>
#include <QLabel>
#include <QSlider>

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace my_server{

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow{
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
    MainWindow();
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
    void showNoMasterMessage();
public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_actionAbout_triggered();
	void on_button_connect_clicked(bool check );
	void on_checkbox_use_environment_stateChanged(int state);
    void on_pushButton_clicked();
    void on_servo_x_valueChanged();
    void on_servo_y_valueChanged();
    void on_kp_valueChanged();
    void on_kd_valueChanged();
    void on_ki_valueChanged();
    void on_gain_valueChanged();
    void on_pid_settings_entered();
    void on_ctrl_settings_entered();
    void on_in_settings_entered();
    void on_maxSpeed_dial_changed();
    void on_reset_button_clicked();
    void on_data_check_changed();
    void on_front_led_button_clicked();

    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically
    void processFrameAndUpdateGUI();

private:
    bool ledStatus;
    Ui::MainWindowDesign ui;
    QTimer *tmrTimer;
    cv::Mat frame;
    QNode qnode;
    myThread *my_thread;
    QSlider *servo_x_slider, *servo_y_slider, *kp_slider;
};

class QPictureLabel : public QLabel
{
private:
    QPixmap _qpSource; //preserve the original, so multiple resize events won't break the quality
    QPixmap _qpCurrent;
    void _displayImage();

public:
    QPictureLabel(QWidget *aParent) : QLabel(aParent) { }
    void setPixmap(QPixmap aPicture);
    void paintEvent(QPaintEvent *aEvent);
};

}  // namespace my_server

#endif // my_server_MAIN_WINDOW_H
