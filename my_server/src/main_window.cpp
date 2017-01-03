/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QProcess>
#include <QThread>
#include <QDebug>
#include <QString>
#include <QImage>
#include <QPixmap>
#include <QLabel>
#include <QSlider>
#include <iostream>
#include "../include/my_server/main_window.hpp"
#include "../include/my_server/myThread.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace my_server {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    ReadSettings();
	setWindowIcon(QIcon(":/images/icon.png"));
	ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

	/*********************
	** Logging
	**********************/
    ui.view_logging->setModel(qnode.loggingModel());
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

    /*********************
    ** Auto Start
    **********************/
    if ( ui.checkbox_remember_settings->isChecked() ) {
        on_button_connect_clicked(true);
    }

    /*frame update thread */
    my_thread = new myThread(this);
    connect(my_thread, SIGNAL(valueChanged()), this, SLOT(processFrameAndUpdateGUI()));
    my_thread->start();

    ledStatus = false;

    /*ui items*/
    servo_x_slider = ui.horizontalSlider;
    servo_y_slider = ui.verticalSlider_2;

    connect(servo_x_slider, SIGNAL(valueChanged(int)), this, SLOT(on_servo_x_valueChanged()));
    connect(servo_y_slider, SIGNAL(valueChanged(int)), this, SLOT(on_servo_y_valueChanged()));
    connect(ui.kp_slider, SIGNAL(valueChanged(int)), this, SLOT(on_kp_valueChanged()));
    connect(ui.kd_slider, SIGNAL(valueChanged(int)), this, SLOT(on_kd_valueChanged()));
    connect(ui.ki_slider, SIGNAL(valueChanged(int)), this, SLOT(on_ki_valueChanged()));
    connect(ui.pidEdit, SIGNAL(returnPressed()), this, SLOT(on_pid_settings_entered()));
    connect(ui.inEdit, SIGNAL(returnPressed()), this, SLOT(on_in_settings_entered()));
    connect(ui.ctrlEdit, SIGNAL(returnPressed()), this, SLOT(on_ctrl_settings_entered()));
    connect(ui.maxSpeedDial, SIGNAL(valueChanged(int)), this, SLOT(on_maxSpeed_dial_changed()));
    connect(ui.button_zero, SIGNAL(clicked()), this, SLOT(on_reset_button_clicked()));
    connect(ui.dataOnCheck, SIGNAL(stateChanged(int)), this, SLOT(on_data_check_changed()));
    connect(ui.frontLedButton, SIGNAL(clicked()), this, SLOT(on_front_led_button_clicked()));
    connect(ui.gain_slider, SIGNAL(valueChanged(int)), this, SLOT(on_gain_valueChanged()));


    ui.imageOn->setChecked(true);
    ui.dataOnCheck->setChecked(true);
    ui.label_5->setText("Stopped");
    ui.label_6->setText("Stopped");
    ui.label_7->setText("Stopped");

    servo_x_slider->setValue(90);
    servo_y_slider->setValue(90);
}

void MainWindow::on_front_led_button_clicked()
{
    ledStatus = !ledStatus;
    qnode.frontLedCtrl(ledStatus);
}

/*ui items slot*/
void MainWindow::on_data_check_changed()
{
    qnode.setDataPublishVal(ui.dataOnCheck->checkState());
}

void MainWindow::on_reset_button_clicked()
{
    qnode.cudaFLow.pid.resetValues();
    ui.pidText->setText(QString::number(qnode.cudaFLow.getSteerVal()));

    ui.lcdPIDVal->setSegmentStyle(QLCDNumber::Filled);
    ui.lcdPIDVal->display(QString::number(qnode.cudaFLow.pid.getPIDVal()));
}

void MainWindow::on_maxSpeed_dial_changed()
{
    int val = ui.maxSpeedDial->value();
    ui.maxSpeedLabel->setText(QString::number(val));
    qnode.changeMaxSpeed(val);
}

void MainWindow::on_pid_settings_entered()
{
    qnode.cudaFLow.pid.limitPID = ui.pidEdit->text().toInt();
    cout << "enterd: " << qnode.cudaFLow.pid.limitPID << endl;
}

void MainWindow::on_in_settings_entered()
{
    qnode.cudaFLow.pid.limitIn = ui.inEdit->text().toDouble();
    cout << "enterd: " << qnode.cudaFLow.pid.limitIn << endl;
}

void MainWindow::on_ctrl_settings_entered()
{
    qnode.cudaFLow.pid.limitControlSignal = ui.ctrlEdit->text().toInt();
    cout << "enterd: " << qnode.cudaFLow.pid.limitControlSignal << endl;
}

void MainWindow::on_kp_valueChanged()
{
    double val = ui.kp_slider->value() / 250.0;
    qnode.cudaFLow.pid.setConstants("kp", val);
    ui.kp_text->setText(QString::number(val));
}

void MainWindow::on_kd_valueChanged()
{
    double val = ui.kd_slider->value() / 250.0;
    qnode.cudaFLow.pid.setConstants("kd", val);
    ui.kd_text->setText(QString::number(val));
}

void MainWindow::on_ki_valueChanged()
{
    double val = ui.ki_slider->value() / 250.0;
    qnode.cudaFLow.pid.setConstants("ki", val);
    ui.ki_text->setText(QString::number(val));
}

void MainWindow::on_gain_valueChanged()
{
    double val = ui.gain_slider->value() / 250.0;
    qnode.cudaFLow.setGain(val);
    ui.gain_text->setText(QString::number(val));
}

void MainWindow::on_servo_x_valueChanged()
{
    //std::cout << "here: " << servo_x_slider->value() << std::endl;
}

void MainWindow::on_servo_y_valueChanged()
{
    //std::cout << "here: " << servo_y_slider->value() << std::endl;
}

void MainWindow::on_pushButton_clicked()
{

    if(qnode.getOpticalFlowStatus())
    {
        qnode.setOpticalFlowStatus(false);
        ui.label_5->setText("Stoped");
    }else{
        qnode.setOpticalFlowStatus(true);
        ui.label_5->setText("Started");
    }
}

QImage Mat2QImage(const cv::Mat3b &src) {
        QImage dest(src.cols, src.rows, QImage::Format_ARGB32);
        for (int y = 0; y < src.rows; ++y) {
                const cv::Vec3b *srcrow = src[y];
                QRgb *destrow = (QRgb*)dest.scanLine(y);
                for (int x = 0; x < src.cols; ++x) {
                        destrow[x] = qRgba(srcrow[x][2], srcrow[x][1], srcrow[x][0], 255);
                }
        }
        return dest;
}

void MainWindow::processFrameAndUpdateGUI()
{
    if(ui.imageOn->isChecked())
    {
        frame = qnode.getFrame();
        //QImage qimgOriginal((uchar*) frame->data,frame->cols,frame->rows,frame->step,QImage::Format_RGB888); // for color images

        if(!frame.empty())
            ui.label_4->setPixmap(QPixmap::fromImage(Mat2QImage(frame)
                                                     .scaled(ui.label_4->width(), ui.label_4->height(), Qt::IgnoreAspectRatio)));

    }
    if(qnode.getOpticalFlowStatus())
    {
        ui.label_5->setText(QString::number(qnode.cudaFLow.getSteerVal()));
        //ui.pidText->setText(QString::number(qnode.cudaFLow.pid.getPIDVal()));
        ui.lcdPIDVal->setSegmentStyle(QLCDNumber::Filled);
        ui.lcdPIDVal->display(QString::number(qnode.cudaFLow.pid.getPIDVal()));
    }
}

MainWindow::~MainWindow() {
    my_thread->Stop = true;
}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
    close();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void MainWindow::on_button_connect_clicked(bool check ) {
	if ( ui.checkbox_use_environment->isChecked() ) {
		if ( !qnode.init() ) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
		}
	} else {
		if ( ! qnode.init(ui.line_edit_master->text().toStdString(),
				   ui.line_edit_host->text().toStdString()) ) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
			ui.line_edit_master->setReadOnly(true);
			ui.line_edit_host->setReadOnly(true);
			ui.line_edit_topic->setReadOnly(true);
		}
	}
}

void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
	bool enabled;
	if ( state == 0 ) {
		enabled = true;
	} else {
		enabled = false;
	}
	ui.line_edit_master->setEnabled(enabled);
	ui.line_edit_host->setEnabled(enabled);
	//ui.line_edit_topic->setEnabled(enabled);
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
        ui.view_logging->scrollToBottom();
}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "my_server");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
    ui.kp_slider->setValue(settings.value("kp", 1).toDouble());
    ui.kd_slider->setValue(settings.value("kd", 1).toDouble());
    ui.ki_slider->setValue(settings.value("ki", 1).toDouble());
    ui.gain_slider->setValue(settings.value("gain", 1).toDouble());

    on_kp_valueChanged();
    on_kd_valueChanged();
    on_ki_valueChanged();
    on_gain_valueChanged();

    ui.maxSpeedDial->setValue(settings.value("maxSpeed", 75).toInt());
    //on_maxSpeed_dial_changed();

    qnode.cudaFLow.pid.limitPID = settings.value("limitPID", 255).toInt();
    qnode.cudaFLow.pid.limitIn = settings.value("limitIn", 100).toInt();
    qnode.cudaFLow.pid.limitControlSignal = settings.value("limitCtrl", 100).toInt();
    ui.pidEdit->setText(QString::number(qnode.cudaFLow.pid.limitPID));
    ui.inEdit->setText(QString::number(qnode.cudaFLow.pid.limitIn));
    ui.ctrlEdit->setText(QString::number(qnode.cudaFLow.pid.limitControlSignal));

    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    ui.line_edit_master->setText(master_url);
    ui.line_edit_host->setText(host_url);
    //ui.line_edit_topic->setText(topic_name);
    bool remember = settings.value("remember_settings", false).toBool();
    ui.checkbox_remember_settings->setChecked(remember);
    bool checked = settings.value("use_environment_variables", false).toBool();
    ui.checkbox_use_environment->setChecked(checked);
    if ( checked ) {
    	ui.line_edit_master->setEnabled(false);
    	ui.line_edit_host->setEnabled(false);
    	//ui.line_edit_topic->setEnabled(false);
    }
}

void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "my_server");
    settings.setValue("kp", QString::number(ui.kp_slider->value()));
    settings.setValue("kd", QString::number(ui.kd_slider->value()));
    settings.setValue("ki", QString::number(ui.ki_slider->value()));
    settings.setValue("gain", QString::number(ui.gain_slider->value()));
    settings.setValue("maxSpeed", QString::number(ui.maxSpeedDial->value()));
    settings.setValue("master_url",ui.line_edit_master->text());
    settings.setValue("host_url",ui.line_edit_host->text());
    //settings.setValue("topic_name",ui.line_edit_topic->text());
    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));

    settings.setValue("limitPID", qnode.cudaFLow.pid.limitPID);
    settings.setValue("limitIn", qnode.cudaFLow.pid.limitIn);
    settings.setValue("limitCtrl", qnode.cudaFLow.pid.limitControlSignal);
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    my_thread->Stop = true;
	WriteSettings();
	QMainWindow::closeEvent(event);
}


/*
void QPictureLabel::paintEvent(QPaintEvent *aEvent)
{
    QLabel::paintEvent(aEvent);
    _displayImage();
}

void QPictureLabel::setPixmap(QPixmap aPicture)
{
    _qpSource = _qpCurrent = aPicture;
    repaint();
}

void QPictureLabel::_displayImage()
{
    if (_qpSource.isNull()) //no image was set, don't draw anything
        return;

    float cw = width(), ch = height();
    float pw = _qpCurrent.width(), ph = _qpCurrent.height();

    if (pw > cw && ph > ch && pw/cw > ph/ch || //both width and high are bigger, ratio at high is bigger or
        pw > cw && ph <= ch || //only the width is bigger or
        pw < cw && ph < ch && cw/pw < ch/ph //both width and height is smaller, ratio at width is smaller
        )
        _qpCurrent = _qpSource.scaledToWidth(cw, Qt::TransformationMode::FastTransformation);
    else if (pw > cw && ph > ch && pw/cw <= ph/ch || //both width and high are bigger, ratio at width is bigger or
        ph > ch && pw <= cw || //only the height is bigger or
        pw < cw && ph < ch && cw/pw > ch/ph //both width and height is smaller, ratio at height is smaller
        )
        _qpCurrent = _qpSource.scaledToHeight(ch, Qt::TransformationMode::FastTransformation);

    int x = (cw - _qpCurrent.width())/2, y = (ch - _qpCurrent.height())/2;

    QPainter paint(this);
    paint.drawPixmap(x, y, _qpCurrent);
}*/

}  // namespace my_server

