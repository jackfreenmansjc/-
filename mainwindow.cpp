#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "led.h"
#include <iostream>
#include <QQmlContext>
#include <QQmlEngine>
#include <QQuickItem>
#include <QGeoCoordinate>
#include <QTimer>
#include <QtWidgets/QGesture>          //手势头文件（鼠标动作）
#include <QtWidgets/QGraphicsScene>    //绘图屏幕参数头文件
#include <QtCharts>
#include <cmath>
#include <vector>
#include <algorithm>
// WGS-84 ellipsiod parameters
const double WGS84_A = 6378137.0; // major axis
const double WGS84_B = 6356752.3; // minor axis
const double WGS84_F = 1 / 298.257223563; // flattening

double haversineDistance(double lat1, double lon1, double lat2, double lon2) {
    double dLat = qDegreesToRadians(lat2 - lat1);
    double dLon = qDegreesToRadians(lon2 - lon1);

    lat1 = qDegreesToRadians(lat1);
    lat2 = qDegreesToRadians(lat2);

    double a = sin(dLat / 2) * sin(dLat / 2) +
               sin(dLon / 2) * sin(dLon / 2) * cos(lat1) * cos(lat2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));

    return WGS84_A * c; // Return distance in meters
}

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{

    db = (DB1 *)malloc(sizeof(DB1));
    db_ipc = (Boatcontrol *)malloc(sizeof(Boatcontrol));
    memset(db, 0, sizeof(DB1));
    memset(db_ipc, 0, sizeof(Boatcontrol));
    db_ipc->HA=HAND;
    ui->setupUi(this);
    setLED(ui->label_5,0,16);
    //初始化TcpSocket
    socket = new QTcpSocket();
    //取消原有连接
    socket->abort();
    //控件初始化状态
    ui->lcdNumber->display(ui->dial_direction->value());
    ui->lcdNumber_2->display(ui->dial_speed->value());
    ui->lcdNumber_19->display(ui->dial_3->value());
    ui->lcdNumber_24->display(ui->horizontalSlider_15->value()/10);
    ui->lcdNumber_12->display(ui->horizontalSlider->value()/10);
    ui->lcdNumber_13->display(ui->horizontalSlider_2->value()/10);
    ui->lcdNumber_22->display(ui->horizontalSlider_13->value());
    ui->lcdNumber_23->display(ui->horizontalSlider_14->value()/10);
    // 创建QQuickWidget并加载QML文件
    quickWidget = new QQuickWidget(this);
    quickWidget->setSource(QUrl(QStringLiteral("qrc:/MapView.qml")));
    quickWidget->setResizeMode(QQuickWidget::SizeRootObjectToView);
    quickWidget->setWindowFlags(Qt::Window); // 或者使用默认的窗口标志
    quickWidget->setStyleSheet("QQuickWidget { z-index: 1; }");
    // 将QQuickWidget添加到TabWidget的第一个标签页
    ui->tabWidget_2->addTab(quickWidget, "Map");

    // 获取QML上下文
    QQmlContext *context = quickWidget->rootContext();
    context->setContextProperty("mapView", this);
    // 连接按钮点击信号到槽函数
    //connect(ui->pushButton, &QPushButton::clicked, this, &MainWindow::onUpdateCarPositionClicked);
    QTimer *timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &MainWindow::updateCarPosition);
    timer->start(1000);  // 每秒更新一次小车位置

    //connect(ui->dial_direction,SIGNAL(onmoved(int)), this, SLOT(onwrapped(int)));
    QObject::connect(ui->dial_direction, &QDial::valueChanged, ui->lcdNumber, QOverload<int>::of(&QLCDNumber::display));
    QObject::connect(ui->dial_speed, &QDial::valueChanged, ui->lcdNumber_2, QOverload<int>::of(&QLCDNumber::display));
    //QObject::connect(ui->dial_direction, &QDial::valueChanged, this, &MainWindow::manual_control_direction);
    //QObject::connect(ui->dial_speed, &QDial::valueChanged, this, &MainWindow::manual_control_direction);
    // 连接slider的valueChanged信号到lcdNumber的display槽
   // QObject::connect(ui->horizontalSlider, &QSlider::valueChanged, ui->lcdNumber_12, static_cast<void(QLCDNumber::*)(int)>(&QLCDNumber::display));
    QObject::connect(ui->horizontalSlider, &QSlider::valueChanged,this, &MainWindow::show_lcd);
    QObject::connect(ui->horizontalSlider_2, &QSlider::valueChanged, this, &MainWindow::show_lcd_1);
    QObject::connect(ui->horizontalSlider_4, &QSlider::valueChanged,this, &MainWindow::show_lcd_20);
    QObject::connect(ui->horizontalSlider_11, &QSlider::valueChanged, this, &MainWindow::show_lcd_21);
    QObject::connect(ui->horizontalSlider_15, &QSlider::valueChanged, this, &MainWindow::show_lcd_24);
    //QObject::connect(ui->horizontalSlider, &QSlider::valueChanged, this, &MainWindow::manual_control_direction);
    //QObject::connect(ui->horizontalSlider_2, &QSlider::valueChanged, this, &MainWindow::manual_control_direction);
    //QObject::connect(ui->checkBox, &QCheckBox::stateChanged, this, &MainWindow::manual_control_direction);
    QObject::connect(ui->horizontalSlider_6, &QSlider::valueChanged, this, &MainWindow::updateLineEdit_1);
    QObject::connect(ui->horizontalSlider_7, &QSlider::valueChanged, this, &MainWindow::updateLineEdit_2);
    QObject::connect(ui->horizontalSlider_3, &QSlider::valueChanged, this, &MainWindow::updateLineEdit_3);
    QObject::connect(ui->horizontalSlider_5, &QSlider::valueChanged, this, &MainWindow::updateLineEdit_4);
    QObject::connect(ui->horizontalSlider_8, &QSlider::valueChanged, this, &MainWindow::updateLineEdit_5);
    QObject::connect(ui->horizontalSlider_9, &QSlider::valueChanged, this, &MainWindow::updateLineEdit_6);
    QObject::connect(ui->horizontalSlider_10, &QSlider::valueChanged, this, &MainWindow::updateLineEdit_7);
    QObject::connect(ui->horizontalSlider_13, &QSlider::valueChanged, this, &MainWindow::show_lcd_22);
    QObject::connect(ui->horizontalSlider_14, &QSlider::valueChanged, this, &MainWindow::show_lcd_23);
    //QObject::connect(ui->checkBox_2, &QCheckBox::stateChanged, this, &MainWindow::manual_control_direction);
    QObject::connect(ui->pushButton_8, &QPushButton::clicked, this, &MainWindow::manual_control_direction);
    //QObject::connect(ui->pushButton_8, &QPushButton::clicked, this, &MainWindow::send_ipc);

    QObject::connect(ui->pushButton_5, &QPushButton::clicked, this, &MainWindow::pushButton_5_on_clicked);
    QObject::connect(ui->pushButton_7, &QPushButton::clicked, this, &MainWindow::pushButton_7_on_clicked);
    QObject::connect(ui->pushButton_15, &QPushButton::clicked, this, &MainWindow::pushButton_15_on_clicked);
    QObject::connect(ui->pushButton_16, &QPushButton::clicked, this, &MainWindow::pushButton_16_on_clicked);
    QObject::connect(ui->pushButton_27, &QPushButton::clicked, this, &MainWindow::pushButton_27_on_clicked);
    QObject::connect(ui->pushButton_33, &QPushButton::clicked, this, &MainWindow::pushButton_27_on_clicked);
    QObject::connect(ui->pushButton_34, &QPushButton::clicked, this, &MainWindow::pushButton_34_on_clicked);
    QObject::connect(ui->dial, &QDial::valueChanged, ui->lcdNumber_14, static_cast<void(QLCDNumber::*)(int)>(&QLCDNumber::display));
    QObject::connect(ui->dial_2, &QDial::valueChanged, ui->lcdNumber_15, static_cast<void(QLCDNumber::*)(int)>(&QLCDNumber::display));
    QObject::connect(ui->dial_3, &QDial::valueChanged, ui->lcdNumber_19, static_cast<void(QLCDNumber::*)(int)>(&QLCDNumber::display));
    QObject::connect(ui->pushButton_6, &QPushButton::clicked, this, &MainWindow::target_pid_send);
    QObject::connect(ui->pushButton_28, &QPushButton::clicked, this, &MainWindow::target_pid_send_rpm);
    QObject::connect(ui->pushButton_17, &QPushButton::clicked, this, &MainWindow::stop_boat);
    QObject::connect(ui->tabWidget, &QTabWidget::currentChanged, this, &MainWindow::change_index);

    QObject::connect(ui->pushButton_9, &QPushButton::clicked, this, &MainWindow::pushButton_9_on_clicked);
    QObject::connect(ui->pushButton_10, &QPushButton::clicked, this, &MainWindow::target_z_send);
    QObject::connect(ui->pushButton_29, &QPushButton::clicked, this, &MainWindow::target_z_send_rpm);
    QObject::connect(ui->pushButton_11, &QPushButton::clicked, this, &MainWindow::pushButton_16_on_clicked);
    QObject::connect(ui->pushButton_12, &QPushButton::clicked, this, &MainWindow::pushButton_12_on_clicked);
    QObject::connect(ui->pushButton_13, &QPushButton::clicked, this, &MainWindow::target_t_send);
    QObject::connect(ui->pushButton_32, &QPushButton::clicked, this, &MainWindow::back_rudder_send);
    //LOS 轨迹跟踪
    QObject::connect(ui->pushButton_26, &QPushButton::clicked, this, &MainWindow::pushButton_26_on_clicked);
    QObject::connect(ui->pushButton_24, &QPushButton::clicked, this, &MainWindow::pushButton_7_on_clicked);
    // 连接按钮点击事件到槽函数
    QObject::connect(ui->pushButton_25, &QPushButton::clicked, this, &MainWindow::clearComboBox);
    // Connect the button click event to the slot
    QObject::connect(ui->pushButton_25, &QPushButton::clicked, this, &MainWindow::clearPathsAndMarkers);

    //期望航速
    QObject::connect(ui->horizontalSlider_12, &QSlider::valueChanged, this, &MainWindow::updateLineEdit);

    //记录小船轨迹
    connect(ui->pushButton, &QPushButton::clicked, this, &MainWindow::onRecordButtonClicked);
    connect(ui->pushButton_13, &QPushButton::clicked, this, &MainWindow::onRecordButtonClicked);//回转试验中记录小船轨迹
    connect(ui->pushButton_2, &QPushButton::clicked, this, &MainWindow::onClearTrackButtonClicked); // 连接清除轨迹按钮的信号
    connect(ui->pushButton_3, &QPushButton::clicked, this, &MainWindow::onClearTrackButtonClicked); // 连接清除轨迹按钮的信号
    connect(ui->pushButton_14, &QPushButton::clicked, this, &MainWindow::onCalculateCurvatureButtonClicked); // 连接清除轨迹按钮的信号
    // Create a chart
    initChart();


    connect(ui->pushButton_30, &QPushButton::clicked, this, &MainWindow::recordData); // 连接记录数据按钮的信号
    connect(ui->pushButton_31, &QPushButton::clicked, this, &MainWindow::saveDataToFile); // 连接处理数据按钮的信号

    connect(ui->radioButton_13, &QRadioButton::toggled, this, &MainWindow::onRadioButtonToggled);
    connect(ui->radioButton_14, &QRadioButton::toggled, this, &MainWindow::onRadioButtonToggled);


}

MainWindow::~MainWindow()
{
    delete this->socket;
    delete ui;
}

void MainWindow::on_Btn_Connect_clicked()
{
    if(ui->Btn_Connect->text() == tr("连接") && socket->state() != QTcpSocket::ConnectedState )
    {
        //获取IP地址
        QString IP = ui->lineEdit_IP->text();
        //获取端口号
        int port = ui->lineEdit_Port->text().toInt();

        connect(socket, &QTcpSocket::readyRead, this, &MainWindow::Read_Data);
        connect(socket, &QTcpSocket::stateChanged, this, &MainWindow::onStateChanged);
        //connect(socket, SIGNAL(error(QAbstractSocket::SocketError)), this, SLOT(onErrorOccurred()));
        connect(socket, &QTcpSocket::errorOccurred, this, &MainWindow::onErrorOccurred);
        //connect(ui->dial_direction,SIGNAL(valueChanged(int)), this, SLOT(onwrapped(int)));
        //取消原有连接
        socket->abort();
        //连接服务器
        socket->connectToHost(IP, port);

        //等待连接成功
        if(!socket->waitForConnected(3000))
        {
            return;
        }
        else {
            ui->Btn_Connect->setText("断开\n连接");

            QMessageBox::information(this, "提示", "连接成功", QMessageBox::Yes);
            setLED(ui->label_5,2,20);
        }
    }
    else
    {
        //断开连接
        socket->disconnectFromHost();
        //修改按键文字
        ui->Btn_Connect->setText("连接");
        return;
    }
}

void MainWindow::onStateChanged(int state)
{
    if (state == QTcpSocket::UnconnectedState) {
        ui->Btn_send->setEnabled(false);
        ui->Btn_Connect->setText("连接");
        setLED(ui->label_5,1,16);

    }
    else if (state == QTcpSocket::ConnectedState) {
        ui->Btn_send->setEnabled(true);
        ui->Btn_Connect->setText("断开连接");
        setLED(ui->label_5,2,16);
    }
}

void MainWindow::onErrorOccurred()
{
    QMessageBox::information(this, "错误", socket->errorString(), QMessageBox::Yes);
}

void MainWindow::Read_Data()
{
    QByteArray buffer;
    //读取缓冲区数据
    buffer = socket->readAll();
    //qDebug() << buffer;

    if(!buffer.isEmpty())
    {
        //QMessageBox::information(this, "收到消息", buffer, QMessageBox::Yes);
        // 假设receivedData为接收到的字节流
        QDataStream inStream(&buffer, QIODevice::ReadOnly);
        // 从数据流中读取数据并填充到receivedStruct中
        inStream.readRawData(reinterpret_cast<char*>(db), sizeof(DB1));
        ui->lcdNumber_3->display(db->yaw);
        ui->lcdNumber_6->display(db->current_speed);
        ui->lcdNumber_10->display(db->Tar_yaw);
        ui->lcdNumber_11->display(db->Tar_speed);
        ui->lcdNumber_4->display(db->current_rpm);
        //ui->label_15->setText(QString::number(db->mode));
        ui->lcdNumber_5->display(db->out_front_board);
        ui->lcdNumber_8->display(db->out_tail_board);
        ui->lcdNumber_7->display(db->out_rudder_left);
        ui->lcdNumber_9->display(db->out_rudder_right);
        if(db->out_engine == ON)
        {
            ui->label_54->setText("运行正常");
        }
        else
        {
            ui->label_54->setText("关闭");
        }
        if(db->out_gate_left==0)
        {
           ui->label_17->setText("打开");
        }
        else
        {
            ui->label_17->setText("关闭");
        }

        if(db->out_gate_right==0)
        {
            ui->label_21->setText("打开");
        }
        else
        {
            ui->label_21->setText("关闭");
        }
        //ui->label_21->setText(QString::number(db->out_gate_right));
        switch (db->mode) {
        case STANDBY_MODE:
            ui->label_15->setText("standby");
            break;
        case FAILURE_MODE:
            ui->label_15->setText("failure mode");
            break;
        case MANUAL_MODE:
            ui->label_15->setText("manual mode");
            break;
        case PID_MODE:
            ui->label_15->setText("pid mode");
            break;
        case LOS_MODE:
            ui->label_15->setText("los mode");
            break;
        case Z_MODE:
            ui->label_15->setText("z mode");
            break;
        case TURN_MODE:
            ui->label_15->setText("turn mode");
            break;
        default:
            break;
        }
        //std::cout << "Latitude1: " << db->current_latitude << ", Longitude1: " << db->current_longitude << std::endl;
        if(record==ON)
        {
            //记录数据
            Data newData;
            // Assuming gps and imuData are available from socket
            newData.timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss");
            newData.x_coord = db->current_latitude;
            newData.y_coord = db->current_longitude;
            newData.heading_angle = db->current_yaw;
            newData.expected_heading_angle = db->Tar_yaw;  // You might calculate this
            // Store this data in a list
            collectedData.append(newData);

        }
        else
        {
            collectedData.clear();
        }

    }
}



//****************************
/*
void MainWindow::Read_Data()
{
    QByteArray buffer;
    //读取缓冲区数据
    buffer = socket->readAll();
    //qDebug() << buffer;

    if(!buffer.isEmpty())
    {
        //QMessageBox::information(this, "收到消息", buffer, QMessageBox::Yes);
        // 假设receivedData为接收到的字节流
        QDataStream inStream(&buffer, QIODevice::ReadOnly);
        // 从数据流中读取数据并填充到receivedStruct中
        inStream.readRawData(reinterpret_cast<char*>(db), sizeof(DB1));
        qDebug() << "debug:" << db->current_yaw;
        ui->lcdNumber_3->display(db->current_yaw);
        ui->lcdNumber_6->display(db->gps_speed);
        ui->lcdNumber_10->display(db->Tar_yaw);
        ui->lcdNumber_11->display(db->Tar_speed);
        ui->lcdNumber_4->display(db->current_rpm);
        //ui->label_15->setText(QString::number(db->mode));
        ui->lcdNumber_7->display(db->out_rudder_left);
        ui->lcdNumber_9->display(db->out_rudder_right);
        if(1)//db->out_engine == ON)
        {
            ui->label_54->setText("运行正常");
        }
        else
        {
            ui->label_54->setText("关闭");
        }
        //ui->label_21->setText(QString::number(db->out_gate_right));
        switch (db->mode) {
        case STANDBY_MODE:
            ui->label_15->setText("standby");
            break;
        case FAILURE_MODE:
            ui->label_15->setText("failure mode");
            break;
        case MANUAL_MODE:
            ui->label_15->setText("manual mode");
            break;
        case PID_MODE:
            ui->label_15->setText("pid mode");
            break;
        case LOS_MODE:
            ui->label_15->setText("los mode");
            break;
        case Z_MODE:
            ui->label_15->setText("z mode");
            break;
        case TURN_MODE:
            ui->label_15->setText("turn mode");
            break;
        default:
            break;
        }
        std::cout << "Latitude1: " << db->current_latitude << ", Longitude1: " << db->current_longitude << std::endl;
        if(record==ON)
        {
            //记录数据
            Data newData;
            // Assuming gps and imuData are available from socket
            newData.timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss");
            newData.x_coord = db->latitude;
            newData.y_coord = db->longitude;
            newData.heading_angle = db->current_yaw;
            newData.expected_heading_angle = db->Tar_yaw;  // You might calculate this
            // Store this data in a list
            collectedData.append(newData);

        }
        else
        {
            collectedData.clear();
        }

    }
}
*/
void MainWindow::on_Btn_exit_clicked()
{
    this->close();
}

void MainWindow::on_Btn_send_clicked()
{
    QString data = ui->lineEdit_Send->text();
    socket->write(data.toLatin1());
    socket->flush();
}

void  MainWindow::keyPressEvent(QKeyEvent *event)
{
    switch (event->key()) {
    case Qt::Key_W:
        qDebug() << "W key pressed";
        break;
    case Qt::Key_A:
        qDebug() << "A key pressed";
        break;
    case Qt::Key_S:
        qDebug() << "S key pressed";
        break;
    case Qt::Key_D:
        qDebug() << "D key pressed";
        break;
    default:
        QMainWindow::keyPressEvent(event);
    }
}
void MainWindow::debug_print()
{
    qDebug() << "mode:" << db_ipc->mode;
    qDebug() << "engine:" << db_ipc->engine;
    qDebug() << "PID self test:" <<db_ipc->PID_self_test;
    qDebug() << "Target yaw:" <<db_ipc->Tar_yaw;
    qDebug() << "Target speed:" <<db_ipc->Tar_speed;
    qDebug() << "rudder KP:" <<db_ipc->kp_r;
    qDebug() << "rudder KI:" <<db_ipc->ki_r;
    qDebug() << "rudder KD:" <<db_ipc->kd_r;
    qDebug() << "rudder output min :" <<db_ipc->output_min_r;
    qDebug() << "engine KP" <<db_ipc->kp_e;
    qDebug() << "engine KI:" <<db_ipc->ki_e;
    qDebug() << "engine Kd:" <<db_ipc->kd_e;
    qDebug() << "engine output min:" <<db_ipc->output_min_e;
    qDebug() << "manual rpm:" <<db_ipc->rpm;
    qDebug() << "rudder left:" <<db_ipc->rudder_left;
    qDebug() << "rudder right:" <<db_ipc->rudder_right;
    qDebug() << "front board:" <<db_ipc->front_board;
    qDebug() << "tail board:" <<db_ipc->tail_board;
    qDebug() << "gate left:" <<db_ipc->gate_left;
    qDebug() << "gate right:" <<db_ipc->gate_right;

}
void MainWindow::manual_control_direction()
{
    if(1)//ui->radioButton->isChecked()==true)
    {
        db_ipc->manualOK = GOOD;
        db_ipc->PID_self_test = NOTGOOD;
        db_ipc->start_Z=OFF;
        db_ipc->start_T=OFF;
        if(ui->dial_direction->value()>=0)
        {
            db_ipc->rudder_left= 0.0f;
            //db_ipc->rudder_right = ui->dial_direction->value();
            float intValue = ui->dial_direction->value();
            db_ipc->rudder_right= static_cast<float>(intValue);
        }
        else
        {
            //db_ipc->rudder_left= -ui->dial_direction->value();
            float intValue = -ui->dial_direction->value();
            db_ipc->rudder_left= static_cast<float>(intValue);
            db_ipc->rudder_right = 0.0f;
        }
        db_ipc->mode = MANUAL_MODE;
        db_ipc->engine = ON;
        db_ipc->rpm = ui->dial_speed->value();
        db_ipc->front_board = ui->horizontalSlider->value();
        db_ipc->tail_board = ui->horizontalSlider_2->value();
        db_ipc->acc_point=ui->horizontalSlider_4->value();
        db_ipc->dec_point=ui->horizontalSlider_11->value();
        if(ui->checkBox->isChecked())
        {
            db_ipc->gate_left = OFF;
        }else
        {
            db_ipc->gate_left = ON;
        }
        if(ui->checkBox_2->isChecked())
        {
            db_ipc->gate_right = OFF;
        }else
        {
            db_ipc->gate_right = ON;
        }
        if( ui->radioButton_11->isChecked()==true)
        {
            db_ipc->engine = ON;
        }
        if(ui->radioButton_12->isChecked()==true)
        {
            db_ipc->engine = OFF;
        }
        MainWindow::debug_print();
        //db_ipc->gate_left = ui->checkBox->isChecked();
        // 将结构体数据转换为QByteArray
        QByteArray block;
        QDataStream out(&block, QIODevice::WriteOnly);
        out.setVersion(QDataStream::Qt_6_0); // 设置数据流的版本
        out.writeRawData(reinterpret_cast<const char*>(db_ipc), sizeof(Boatcontrol));
        socket->write(block);
        socket->flush();
        qDebug() << "Data sent";
    }
}

void MainWindow::pushButton_5_on_clicked()
{
    if(1)//ui->radioButton_2->isChecked()==true)
    {
        db_ipc->mode = PID_MODE;
        db_ipc->engine = ON;
        //rudder pid
        db_ipc->PID_self_test = NOTGOOD;
        db_ipc->start_Z=OFF;
        db_ipc->start_T=OFF;

        db_ipc->kp_r = ui->lineEdit->text().toFloat();

        db_ipc->ki_r = ui->lineEdit_2->text().toFloat();

        db_ipc->kd_r = ui->lineEdit_5->text().toFloat();

        db_ipc->umin_r = ui->lineEdit_6->text().toFloat();

        db_ipc->umax_r = ui->lineEdit_7->text().toFloat();

        db_ipc->error_max_r = ui->lineEdit_8->text().toFloat();

        db_ipc->output_min_r = ui->lineEdit_13->text().toFloat();

        db_ipc->output_max_r = ui->lineEdit_14->text().toFloat();


        //engine pid

        db_ipc->kp_e = ui->lineEdit_3->text().toFloat();

        db_ipc->ki_e = ui->lineEdit_4->text().toFloat();

        db_ipc->kd_e = ui->lineEdit_9->text().toFloat();

        db_ipc->umin_e = ui->lineEdit_10->text().toFloat();

        db_ipc->umax_e = ui->lineEdit_11->text().toFloat();

        db_ipc->error_max_e = ui->lineEdit_12->text().toFloat();

        db_ipc->output_min_e = ui->lineEdit_15->text().toFloat();

        db_ipc->output_max_e = ui->lineEdit_16->text().toFloat();

        db_ipc->Tar_rpm=ui->dial_3->value();
        db_ipc->Tar_speed=ui->dial_2->value();
        //db_ipc->front_board = ui->horizontalSlider->value();

        db_ipc->acc_point=ui->horizontalSlider_4->value();
        db_ipc->dec_point=ui->horizontalSlider_11->value();
        MainWindow::debug_print();
        QByteArray byteArray(reinterpret_cast<const char*>(db_ipc), sizeof(Boatcontrol));
        socket->write(byteArray);
        socket->flush();
    }

}

void MainWindow::pushButton_9_on_clicked()
{
    if(ui->lineEdit_18->text().isEmpty() && ui->lineEdit_19->text().isEmpty() && ui->lineEdit_20->text().isEmpty() && ui->lineEdit_22->text().isEmpty())//ui->radioButton_2->isChecked()==true)
    {
        qDebug() << "please input some value";
        QMessageBox::warning(this, "输入错误", "请输入内容");
    }
    else{
        db_ipc->mode = STANDBY_MODE;
        db_ipc->engine = ON;
        //rudder pid
        db_ipc->rudder_turn=ui->lineEdit_20->text().toFloat();//舵角
        db_ipc->rudder_turn_speed=ui->lineEdit_20->text().toInt();//转舵速度
        db_ipc->yaw_delta=ui->lineEdit_22->text().toFloat();//超越角度
        if(ui->radioButton_7->isChecked())
        {
            db_ipc->rudder_left_first=ON;//先左转还是右转   ON是先左转
        }
        else
        {
            db_ipc->rudder_left_first=OFF;//先左转还是右转
        }
        db_ipc->start_Z=OFF;
        db_ipc->start_T=OFF;
        db_ipc->PID_self_test = NOTGOOD;

        db_ipc->kp_r = ui->lineEdit->text().toFloat();

        db_ipc->ki_r = ui->lineEdit_2->text().toFloat();

        db_ipc->kd_r = ui->lineEdit_5->text().toFloat();

        db_ipc->umin_r = ui->lineEdit_6->text().toFloat();

        db_ipc->umax_r = ui->lineEdit_7->text().toFloat();

        db_ipc->error_max_r = ui->lineEdit_8->text().toFloat();

        db_ipc->output_min_r = ui->lineEdit_13->text().toFloat();

        db_ipc->output_max_r = ui->lineEdit_14->text().toFloat();


        //engine pid

        db_ipc->kp_e = ui->lineEdit_3->text().toFloat();

        db_ipc->ki_e = ui->lineEdit_4->text().toFloat();

        db_ipc->kd_e = ui->lineEdit_9->text().toFloat();

        db_ipc->umin_e = ui->lineEdit_10->text().toFloat();

        db_ipc->umax_e = ui->lineEdit_11->text().toFloat();

        db_ipc->error_max_e = ui->lineEdit_12->text().toFloat();

        db_ipc->output_min_e = ui->lineEdit_15->text().toFloat();

        db_ipc->output_max_e = ui->lineEdit_16->text().toFloat();

        db_ipc->Tar_rpm=ui->dial_2->value();
        db_ipc->acc_point=ui->horizontalSlider_4->value();
        db_ipc->dec_point=ui->horizontalSlider_11->value();

        db_ipc->Tar_speed = ui->lineEdit_19->text().toFloat();
        db_ipc->Tar_yaw = ui->lineEdit_18->text().toFloat();
        db_ipc->rpmorspeed=1; //speed mode
        MainWindow::debug_print();
        QByteArray byteArray(reinterpret_cast<const char*>(db_ipc), sizeof(Boatcontrol));
        socket->write(byteArray);
        socket->flush();
    }

}

void MainWindow::target_z_send()
{
    //开始打舵
    db_ipc->mode = Z_MODE;
    db_ipc->Tar_speed = ui->lineEdit_19->text().toFloat();
    db_ipc->Tar_yaw = ui->lineEdit_18->text().toFloat();
    db_ipc->rudder_turn = ui->lineEdit_20->text().toFloat(); //操舵角
    db_ipc->yaw_delta = ui->lineEdit_22->text().toFloat(); //超越角
    if(ui->radioButton_7->isChecked())
    {
        db_ipc->rudder_left_first = ON;
    }
    else
    {
        db_ipc->rudder_left_first = OFF;
    }

    db_ipc->PID_self_test = NOTGOOD;
    db_ipc->start_Z=ON;
    db_ipc->start_T=OFF;
    db_ipc->rpmorspeed=1; //speed mode
    MainWindow::debug_print();
    QByteArray byteArray(reinterpret_cast<const char*>(db_ipc), sizeof(Boatcontrol));
    socket->write(byteArray);
    socket->flush();
}


void MainWindow::target_z_send_rpm()
{
    db_ipc->mode = Z_MODE;
    //db_ipc->Tar_speed = ui->dial_2->value();
    db_ipc->Tar_yaw = ui->lineEdit_18->text().toFloat();
    db_ipc->Tar_rpm=ui->horizontalSlider_13->value();
    db_ipc->Tar_slider= ui->horizontalSlider_14->value();
    db_ipc->PID_self_test = NOTGOOD;
    db_ipc->start_Z=OFF;
    db_ipc->start_T=OFF;
    db_ipc->rpmorspeed=1;  //rpm mode
    MainWindow::debug_print();
    QByteArray byteArray(reinterpret_cast<const char*>(db_ipc), sizeof(Boatcontrol));
    socket->write(byteArray);
    socket->flush();
}


void MainWindow::pushButton_26_on_clicked()
{
    db_ipc->mode = LOS_MODE;
    //db_ipc->Tar_speed = ui->dial_2->value();
    db_ipc->Tar_rpm=ui->lineEdit_19->text().toFloat();
    db_ipc->Tar_yaw = ui->lineEdit_18->text().toFloat();
    db_ipc->Tar_speed=ui->lineEdit_30->text().toFloat();
    db_ipc->LOS_ONOFF=ON;
    db_ipc->PID_self_test = NOTGOOD;
    db_ipc->start_Z=OFF;
    db_ipc->start_T=OFF;
    for(int m=0; m<15;m++)
    {
        std::cout << "Latitude: " << db_ipc->GPS_X[m] << ", Longitude: " << db_ipc->GPS_Y[m] << std::endl;
    }
    MainWindow::debug_print();
    QByteArray byteArray(reinterpret_cast<const char*>(db_ipc), sizeof(Boatcontrol));
    socket->write(byteArray);
    socket->flush();
}

void MainWindow::pushButton_12_on_clicked()
{
    if(ui->lineEdit_23->text().isEmpty() && ui->lineEdit_24->text().isEmpty() && ui->lineEdit_25->text().isEmpty())//ui->radioButton_2->isChecked()==true)
    {
        qDebug() << "please input some value";
        QMessageBox::warning(this, "输入错误", "请输入内容");
    }
    else{
        db_ipc->mode = TURN_MODE;
        db_ipc->engine = ON;
        //rudder pid
        db_ipc->t_delta=ui->lineEdit_25->text().toFloat();//舵角
        if(ui->radioButton_10->isChecked())
        {
            db_ipc->turn_LR=LEFT;//先左转还是右转   ON是先左转
        }
        else
        {
            db_ipc->turn_LR=RIGHT;
        }
        db_ipc->start_Z=OFF;
        db_ipc->start_T=OFF;
        db_ipc->PID_self_test = NOTGOOD;

        db_ipc->kp_r = ui->lineEdit->text().toFloat();

        db_ipc->ki_r = ui->lineEdit_2->text().toFloat();

        db_ipc->kd_r = ui->lineEdit_5->text().toFloat();

        db_ipc->umin_r = ui->lineEdit_6->text().toFloat();

        db_ipc->umax_r = ui->lineEdit_7->text().toFloat();

        db_ipc->error_max_r = ui->lineEdit_8->text().toFloat();

        db_ipc->output_min_r = ui->lineEdit_13->text().toFloat();

        db_ipc->output_max_r = ui->lineEdit_14->text().toFloat();


        //engine pid

        db_ipc->kp_e = ui->lineEdit_3->text().toFloat();

        db_ipc->ki_e = ui->lineEdit_4->text().toFloat();

        db_ipc->kd_e = ui->lineEdit_9->text().toFloat();

        db_ipc->umin_e = ui->lineEdit_10->text().toFloat();

        db_ipc->umax_e = ui->lineEdit_11->text().toFloat();

        db_ipc->error_max_e = ui->lineEdit_12->text().toFloat();

        db_ipc->output_min_e = ui->lineEdit_15->text().toFloat();

        db_ipc->output_max_e = ui->lineEdit_16->text().toFloat();

        db_ipc->Tar_rpm=ui->dial_2->value();
        db_ipc->acc_point=ui->horizontalSlider_4->value();
        db_ipc->dec_point=ui->horizontalSlider_11->value();

        db_ipc->Tar_speed = ui->lineEdit_24->text().toFloat();
        db_ipc->Tar_yaw = ui->lineEdit_23->text().toFloat();
        db_ipc->rpmorspeed=2; //speed mode
        MainWindow::debug_print();
        QByteArray byteArray(reinterpret_cast<const char*>(db_ipc), sizeof(Boatcontrol));
        socket->write(byteArray);
        socket->flush();
    }

}

void MainWindow::target_t_send()
{
    //开始打舵
    db_ipc->mode = TURN_MODE;
    db_ipc->Tar_speed = ui->lineEdit_24->text().toFloat();
    db_ipc->Tar_yaw = ui->lineEdit_23->text().toFloat();
    db_ipc->PID_self_test = NOTGOOD;
    db_ipc->start_T=ON;
    db_ipc->start_Z=OFF;
    db_ipc->rpmorspeed=2; //speed mode
    db_ipc->back_to_straight=OFF;
    MainWindow::debug_print();
    QByteArray byteArray(reinterpret_cast<const char*>(db_ipc), sizeof(Boatcontrol));
    socket->write(byteArray);
    socket->flush();
}

void MainWindow::back_rudder_send()
{
    //开始打舵
    db_ipc->mode = TURN_MODE;
    db_ipc->PID_self_test = NOTGOOD;
    db_ipc->start_T=ON;
    db_ipc->start_Z=OFF;
    db_ipc->back_to_straight=ON;
    MainWindow::debug_print();
    QByteArray byteArray(reinterpret_cast<const char*>(db_ipc), sizeof(Boatcontrol));
    socket->write(byteArray);
    socket->flush();
}
void MainWindow::pushButton_7_on_clicked()
{
    db_ipc->mode = STANDBY_MODE;
    db_ipc->standy=4;//tingche buxihuo
    db_ipc->PID_self_test = NOTGOOD;
    db_ipc->LOS_ONOFF=OFF;
    db_ipc->start_Z=OFF;
    db_ipc->start_T=OFF;
    db_ipc->back_to_straight=OFF;
    MainWindow::debug_print();
    QByteArray byteArray(reinterpret_cast<const char*>(db_ipc), sizeof(Boatcontrol));
    socket->write(byteArray);
}

void MainWindow::pushButton_15_on_clicked()
{
    db_ipc->mode = STANDBY_MODE;
    db_ipc->standy=2;//启动发动机，机构复位
    db_ipc->PID_self_test = NOTGOOD;
    db_ipc->start_Z=OFF;
    db_ipc->start_T=OFF;
    db_ipc->back_to_straight=OFF;
    MainWindow::debug_print();
    QByteArray byteArray(reinterpret_cast<const char*>(db_ipc), sizeof(Boatcontrol));
    socket->write(byteArray);
}

void MainWindow::pushButton_16_on_clicked()
{
    db_ipc->mode = STANDBY_MODE;
    db_ipc->standy=4;//tingche buxihuo
    db_ipc->PID_self_test = NOTGOOD;
    db_ipc->start_Z=OFF;
    db_ipc->start_T=OFF;
    db_ipc->back_to_straight=OFF;
    MainWindow::debug_print();
    QByteArray byteArray(reinterpret_cast<const char*>(db_ipc), sizeof(Boatcontrol));
    socket->write(byteArray);
}

void MainWindow::pushButton_27_on_clicked()
{
    db_ipc->mode = STANDBY_MODE;
    db_ipc->standy=5;//tingche xihuo
    db_ipc->PID_self_test = NOTGOOD;
    db_ipc->start_Z=OFF;
    db_ipc->start_T=OFF;
    db_ipc->back_to_straight=OFF;
    MainWindow::debug_print();
    QByteArray byteArray(reinterpret_cast<const char*>(db_ipc), sizeof(Boatcontrol));
    socket->write(byteArray);
}

void MainWindow::pushButton_34_on_clicked()
{
    db_ipc->mode = STANDBY_MODE;
    db_ipc->standy=1;//
    db_ipc->PID_self_test = NOTGOOD;
    db_ipc->start_Z=OFF;
    db_ipc->start_T=OFF;
    db_ipc->back_to_straight=OFF;
    MainWindow::debug_print();
    QByteArray byteArray(reinterpret_cast<const char*>(db_ipc), sizeof(Boatcontrol));
    socket->write(byteArray);
}


void MainWindow::target_pid_send()
{
    db_ipc->mode = PID_MODE;
    db_ipc->Tar_speed = ui->dial_2->value();
    db_ipc->Tar_yaw = ui->dial->value();
    db_ipc->PID_self_test = GOOD;
    db_ipc->start_Z=OFF;
    db_ipc->start_T=OFF;
    db_ipc->rpmorspeed=2; //speed mode
    db_ipc->back_to_straight=OFF;
    MainWindow::debug_print();
    QByteArray byteArray(reinterpret_cast<const char*>(db_ipc), sizeof(Boatcontrol));

    socket->write(byteArray);
    socket->flush();
}

void MainWindow::target_pid_send_rpm()
{
    db_ipc->mode = PID_MODE;
    //db_ipc->Tar_speed = ui->dial_2->value();
    db_ipc->Tar_yaw = ui->dial->value();
    db_ipc->Tar_rpm=ui->dial_3->value();
    db_ipc->Tar_slider= ui->horizontalSlider_15->value();
    db_ipc->PID_self_test = GOOD;
    db_ipc->start_Z=OFF;
    db_ipc->start_T=OFF;
    db_ipc->rpmorspeed=1;  //rpm mode
    db_ipc->back_to_straight=OFF;
    MainWindow::debug_print();
    QByteArray byteArray(reinterpret_cast<const char*>(db_ipc), sizeof(Boatcontrol));

    socket->write(byteArray);
    socket->flush();
}


void MainWindow::send_ipc()
{
    db_ipc->mode = PID_MODE;
    db_ipc->engine = ON;
    db_ipc->PID_self_test = NOTGOOD;
    db_ipc->Tar_yaw = 123.1f;
    db_ipc->Tar_speed = 25.5f;
    db_ipc->kp_r = 1.0f;
    db_ipc->ki_r = 0.02f;
    db_ipc->kd_r = 0.1f;
    db_ipc->umin_r =45.0f;
    db_ipc->umax_r = 60.0f;
    db_ipc->error_max_r = 100.0f;
    db_ipc->output_min_r = -60.0f;
    db_ipc->output_max_r = 60.0f;
    db_ipc->kp_e = 1.1f;
    db_ipc->ki_e = 0.01f;
    db_ipc->kd_e = 1.0f;
    db_ipc->umin_e = 1000.0f;
    db_ipc->umax_e = 6000.1f;
    db_ipc->error_max_e = 2000.0f;
    db_ipc->output_min_e = 1000.0f;
    db_ipc->output_max_e = 6000.0f;
    db_ipc->rpm = 2300;
    db_ipc->rudder_left = -15.0f;
    db_ipc->rudder_right = 0.0f;
    db_ipc->front_board = 22.0f;
    db_ipc->tail_board = -3.0f;
    db_ipc->gate_left = ON;
    db_ipc->gate_right = ON;
    db_ipc->rudder_turn = 15.0f;
    db_ipc->rudder_turn_speed = 180;
    db_ipc->yaw_delta = 25.0f;
    db_ipc->rudder_left_first = ON;
    db_ipc->t_delta = 35.0f;
    db_ipc->t_turn_speed = 180;
    db_ipc->turn_LR = LEFT;
    db_ipc->back_to_straight = OFF;

    // 将结构体数据转换为QByteArray
    QByteArray block;
    QDataStream out(&block, QIODevice::WriteOnly);
    out.setVersion(QDataStream::Qt_6_0); // 设置数据流的版本
    out.writeRawData(reinterpret_cast<const char*>(db_ipc), sizeof(Boatcontrol));
    socket->write(block);
    socket->flush();
    qDebug() << "Data sent";

}


void MainWindow::stop_boat()
{
    db_ipc->PID_self_test = NOTGOOD;
    db_ipc->start_Z=OFF;
    db_ipc->start_T=OFF;
    db_ipc->engine = ON;
    db_ipc->gate_left  = ON;
    db_ipc->gate_right = ON;
    db_ipc->rudder_left = 0.0f;
    db_ipc->rudder_right = 0.0f;
    db_ipc->back_to_straight=OFF;
    db_ipc->rpm = 900;
    // 将结构体数据转换为QByteArray
    QByteArray block;
    QDataStream out(&block, QIODevice::WriteOnly);
    out.setVersion(QDataStream::Qt_6_0); // 设置数据流的版本
    out.writeRawData(reinterpret_cast<const char*>(db_ipc), sizeof(Boatcontrol));
    socket->write(block);
    socket->flush();
    qDebug() << "Data sent";
}

void MainWindow::change_index(int index)
{
    switch (index)
    {
    case 0:
        ui->radioButton->setChecked(false);
        ui->radioButton_2->setChecked(false);
        ui->radioButton_3->setChecked(false);
        ui->radioButton_4->setChecked(true);
        ui->radioButton_5->setChecked(false);
        ui->radioButton_6->setChecked(false);
        break;
    case 1:
        ui->radioButton->setChecked(true);
        ui->radioButton_2->setChecked(false);
        ui->radioButton_3->setChecked(false);
        ui->radioButton_4->setChecked(false);
        ui->radioButton_5->setChecked(false);
        ui->radioButton_6->setChecked(false);
        break;
    case 2:
        ui->radioButton->setChecked(false);
        ui->radioButton_2->setChecked(true);
        ui->radioButton_3->setChecked(false);
        ui->radioButton_4->setChecked(false);
        ui->radioButton_5->setChecked(false);
        ui->radioButton_6->setChecked(false);
        break;
    case 3:
        ui->radioButton->setChecked(false);
        ui->radioButton_2->setChecked(false);
        ui->radioButton_3->setChecked(true);
        ui->radioButton_4->setChecked(false);
        ui->radioButton_5->setChecked(false);
        ui->radioButton_6->setChecked(false);
        break;
    case 4:
        ui->radioButton->setChecked(false);
        ui->radioButton_2->setChecked(false);
        ui->radioButton_3->setChecked(false);
        ui->radioButton_4->setChecked(false);
        ui->radioButton_5->setChecked(true);
        ui->radioButton_6->setChecked(false);
        break;
    case 5:
        ui->radioButton->setChecked(false);
        ui->radioButton_2->setChecked(false);
        ui->radioButton_3->setChecked(false);
        ui->radioButton_4->setChecked(false);
        ui->radioButton_5->setChecked(false);
        ui->radioButton_6->setChecked(true);
        break;
    default:
        break;
    }

}


void MainWindow::show_lcd(int index)
{
    ui->lcdNumber_12->setDigitCount(4);
    ui->lcdNumber_12->setSmallDecimalPoint(true);
    float realvalue = index /10.0;
    ui->lcdNumber_12->display(realvalue);
}

void MainWindow::show_lcd_1(int index)
{

    ui->lcdNumber_13->setDigitCount(4);
    ui->lcdNumber_13->setSmallDecimalPoint(true);
    float realvalue = index /10.0;
    ui->lcdNumber_13->display(realvalue);
}

void MainWindow::show_lcd_20(int index)
{

    ui->lcdNumber_20->setDigitCount(4);
    ui->lcdNumber_20->setSmallDecimalPoint(true);
    float realvalue = index /10.0;
    ui->lcdNumber_20->display(realvalue);
}


void MainWindow::show_lcd_21(int index)
{

    ui->lcdNumber_21->setDigitCount(4);
    ui->lcdNumber_21->setSmallDecimalPoint(true);
    float realvalue = index /10.0;
    ui->lcdNumber_21->display(realvalue);
}

void MainWindow::show_lcd_24(int index)
{

    ui->lcdNumber_24->setDigitCount(4);
    ui->lcdNumber_24->setSmallDecimalPoint(true);
    float realvalue = index /10.0;
    ui->lcdNumber_24->display(realvalue);
}

void MainWindow::show_lcd_22(int index)
{

    ui->lcdNumber_22->setDigitCount(4);
    ui->lcdNumber_22->setSmallDecimalPoint(true);
    float realvalue = index;
    ui->lcdNumber_22->display(realvalue);
}
void MainWindow::show_lcd_23(int index)
{

    ui->lcdNumber_23->setDigitCount(4);
    ui->lcdNumber_23->setSmallDecimalPoint(true);
    float realvalue = index /10.0;
    ui->lcdNumber_23->display(realvalue);
}


void MainWindow::updateLineEdit_1(int value) {
    ui->lineEdit_18->setText(QString::number(value));
}

void MainWindow::updateLineEdit_2(int value) {
    double newValue = value / 10.0;
    ui->lineEdit_19->setText(QString::number(newValue,'f',1));
}

void MainWindow::updateLineEdit_3(int value) {
    double newValue = value / 10.0;
    ui->lineEdit_20->setText(QString::number(newValue,'f',1));
}

void MainWindow::updateLineEdit_4(int value) {
    double newValue = value / 10.0;
    ui->lineEdit_22->setText(QString::number(newValue,'f',1));
}

void MainWindow::updateLineEdit_5(int value) {
    double newValue = value ;
    ui->lineEdit_23->setText(QString::number(newValue,'f',1));
}
void MainWindow::updateLineEdit_6(int value) {
    double newValue = value / 10.0;
    ui->lineEdit_24->setText(QString::number(newValue,'f',1));
}
void MainWindow::updateLineEdit_7(int value) {
    double newValue = value / 10.0;
    ui->lineEdit_25->setText(QString::number(newValue,'f',1));
}

void MainWindow::initChart()
{
    record=OFF;
    chart = new QChart();

    targetSeries = new QSplineSeries;
    targetSeries->setName(QStringLiteral("目标值"));
    chart->addSeries(targetSeries);
    currentSeries = new QSplineSeries;
    currentSeries->setName(QStringLiteral("当前值"));
    chart->addSeries(currentSeries);


    // Create a chart view
    chartView = new QChartView(chart);
    chartView->setRenderHint(QPainter::Antialiasing);
    chartView->chart()->setTitle(QStringLiteral("PID调参"));
    chartView->chart()->setTheme(QChart::ChartThemeDark);
    //创建坐标轴
    QValueAxis *axisX = new QValueAxis;
    axisX->setRange(0, 10);
    axisX->setTickCount(11);
    axisX->setLabelsVisible(false);//x轴坐标显示
    QValueAxis *axisY = new QValueAxis;
    axisY->setRange(-185, 185);
    axisY->setTickCount(7);
    QFont font;
    font.setFamily("Microsoft Yahei");
    font.setPixelSize(10);
    axisY->setLabelsFont(font);
    chartView->chart()->setAxisX(axisX, targetSeries);
    chartView->chart()->setAxisY(axisY, targetSeries);
    chartView->chart()->setAxisX(axisX, currentSeries);
    chartView->chart()->setAxisY(axisY, currentSeries);
    //显示曲线的数据点
    targetSeries->setPointsVisible(true);
    currentSeries->setPointsVisible(true);
    //图例
    chartView->chart()->legend()->setAlignment(Qt::AlignBottom);

    //
    ui->verticalLayout->addWidget(chartView);

    timer_update = new QTimer(this);
    connect(timer_update, &QTimer::timeout, this, &MainWindow::updateData);
    timer_update->start(50); // 每100毫秒更新一次数据
}



void MainWindow::updateData()
{
    double time = QTime::currentTime().msecsSinceStartOfDay() / 1000.0;
    float targetyaw = db->Tar_yaw;
    float currentyaw = db->current_yaw;
    float temp;

    temp = currentyaw - targetyaw;
    if(abs(temp)<=180)
    {
        currentyaw = temp;
        targetyaw = 0;
    }
    else
    {
        if(temp >= 180)
        {
            temp = -360 + temp;
        }
        else if (temp <= -180)
        {
            temp = 360 + temp;
        }
        currentyaw = temp;
        targetyaw = 0;
    }
    // 将数据添加到曲线中
    targetSeries->append(time, targetyaw);
    currentSeries->append(time, currentyaw);

    // 更新X轴范围
    QValueAxis *axisX = qobject_cast<QValueAxis *>(chart->axes(Qt::Horizontal).at(0));
    axisX->setRange(time - 10, time); // 仅显示最新的10秒数据
}




void MainWindow::updateCarPosition()
{
    // 生成新位置的经纬度（这里仅为示例，你应该从实际数据源获取）
    static double latitude = 30.1250;
    static double longitude = 120.2029;
    static double heading = 0.0;
    //latitude +=0.00051*sin(theta);  // 每次更新时改变位置
    //longitude += 0.00051*cos(theta);

    if(db->current_latitude!=0)
    {
        latitude=db->current_latitude;
    }
    if(db->current_longitude!=0)
    {
        longitude=db->current_longitude;
    }

    if(db->current_yaw!=NULL)
    {
        heading = db->current_yaw;
    }

    //std::cout << "Latitude: " << latitude << ", Longitude: " << longitude << std::endl;

    // 获取QML中的root对象
    QObject *rootObject = quickWidget->rootObject();

    if (rootObject) {
        // 调用QML中的updateCarPosition方法
        QVariant lat = latitude;
        QVariant lon = longitude;
        QVariant head = heading;
        QMetaObject::invokeMethod(rootObject, "updateCarPosition",
                                  Q_ARG(QVariant, lat),
                                  Q_ARG(QVariant, lon),
                                  Q_ARG(QVariant, head));
    }
    ui->lineEdit_27->setText(QString::number(latitude,'f',7));
    ui->lineEdit_28->setText(QString::number(longitude,'f',7));

    // 更新小车位置
    QMetaObject::invokeMethod(quickWidget->rootObject(), "updateCarPosition",
                              Q_ARG(QVariant, latitude),
                              Q_ARG(QVariant, longitude),
                              Q_ARG(QVariant, heading));
    // 传递数据到QML的updatePositionFromGPS方法

    if (isRecording) {
        // 记录轨迹点
        trackPoints.append(QGeoCoordinate(latitude, longitude));

        // 更新轨迹显示
        QVariantList trackCoordinates;
        for (const QGeoCoordinate &coord : trackPoints) {
            trackCoordinates.append(QVariant::fromValue(coord));
        }

        QMetaObject::invokeMethod(quickWidget->rootObject(), "updateTrack",
                                  Q_ARG(QVariant, QVariant::fromValue(trackCoordinates)));
    }

}

void MainWindow::handleRightClick(double lat, double lon)
{
    // 在此处处理右击坐标
    std::cout << "Latitude: " << lat << ", Longitude: " << lon << std::endl;

    // 将获取的坐标存入全局数组中
    if (db_ipc->num_points < 15) {
        db_ipc->GPS_X[db_ipc->num_points]= lat;
        db_ipc->GPS_Y[db_ipc->num_points] = lon;
        db_ipc->num_points++;  // 更新索引
    }
    else
    {

        db_ipc->num_points=0;
    }
    // 计算距离
    double distance = 0.0;
    if (!previousPoints.isEmpty()) {
        QPointF lastPoint = previousPoints.last();
        distance = haversineDistance(lastPoint.x(), lastPoint.y(), lat, lon);
    }

    // 将坐标点和距离添加到 comboBox
    /*
    QString coordinateStr = QString("Lat: %1, Lon: %2, Distance: %3 m")
                                .arg(lat, 0, 'f', 9)
                                .arg(lon, 0, 'f', 9)
                                .arg(distance, 0, 'f', 2);
*/
    QString coordinateStr = QString("Dist: %3 m")
                                .arg(distance, 0, 'f', 2);
    ui->comboBox->addItem(coordinateStr);
    // 将坐标点添加到comboBox
    //QString coordinateStr = QString("Lat: %1, Lon: %2").arg(lat, 0, 'f', 7).arg(lon, 0, 'f', 7);
    //ui->comboBox->addItem(coordinateStr);
    // 在地图上添加标记
    QMetaObject::invokeMethod(quickWidget->rootObject(), "addMarker",
                              Q_ARG(QVariant, lat),
                              Q_ARG(QVariant, lon));
    // 保存当前点以便下次计算距离
    previousPoints.append(QPointF(lat, lon));
}

void MainWindow::clearComboBox()
{
    ui->comboBox->clear();
    // 将经纬度数组数据置零
    for (int i = 0; i < 15; i++) {
        db_ipc->GPS_X[i] = 0.0;
        db_ipc->GPS_Y[i]= 0.0;

    }

    // 重置计数器
    db_ipc->num_points = 0;
}

void MainWindow::clearPathsAndMarkers()
{
    // 获取 QML 根对象
    QObject *rootObject = quickWidget->rootObject();

    // 调用 QML 中的 clearPathsAndMarkers 函数
    QMetaObject::invokeMethod(rootObject, "clearPathsAndMarkers");
}

void MainWindow::updateLineEdit(int value)
{
    // 将 QSlider 的值转换为字符串并设置到 QLineEdit 中
    ui->lineEdit_30->setText(QString::number(value));
}

void MainWindow::onRecordButtonClicked()
{
    isRecording = !isRecording;
    if (isRecording) {
        ui->pushButton->setText("停止记录");
        ui->pushButton_13->setText("停止记录");
        trackPoints.clear();
    } else {
        ui->pushButton->setText("开始记录");
        ui->pushButton_13->setText("启动试验");
    }
}

void MainWindow::onClearTrackButtonClicked()
{
    // 清空轨迹点
    trackPoints.clear();

    // 更新轨迹显示
    QMetaObject::invokeMethod(quickWidget->rootObject(), "updateTrack",
                              Q_ARG(QVariant, QVariantList())); // 传递空列表清除轨迹
}


struct Circle {
    double x, y, radius;
};

Circle fitCircle(const std::vector<QGeoCoordinate>& points) {
    double sumX = 0, sumY = 0;
    int n = points.size();

    for (const auto& point : points) {
        sumX += point.longitude();
        sumY += point.latitude();
    }

    double centerX = sumX / n;
    double centerY = sumY / n;

    double sumR = 0;
    for (const auto& point : points) {
        double dx = point.longitude() - centerX;
        double dy = point.latitude() - centerY;
        sumR += std::sqrt(dx * dx + dy * dy);
    }

    double radius = sumR / n;

    return {centerX, centerY, radius};
}

void MainWindow::onCalculateCurvatureButtonClicked() {
    // 获取 QML 中的 rootObject
    QObject *rootObject = quickWidget->rootObject();

    // 获取 QML 中的 track 变量
    QVariant trackVar;
    bool result = QMetaObject::invokeMethod(rootObject, "getTrack", Q_RETURN_ARG(QVariant, trackVar));
    if (!result) {
        qWarning() << "Failed to invoke getTrack method";
        return;
    }
    QList<QVariant> trackList = trackVar.toList();
    std::vector<QGeoCoordinate> trackCoordinates;
    qDebug() << "Track data from QML:";
    for (const QVariant &pointVar : trackList) {
        QVariantMap pointMap = pointVar.toMap();
        double latitude = pointMap["latitude"].toDouble();
        double longitude = pointMap["longitude"].toDouble();
        trackCoordinates.emplace_back(latitude, longitude);
        qDebug() << "Latitude:" << latitude << ", Longitude:" << longitude;
    }
    // 计算拟合圆之前检查点数
    qDebug() << "Number of track points:" << trackCoordinates.size();
    Circle circle = fitCircle(trackCoordinates);

    std::cout << "Fitted Circle Center: (" << circle.x << ", " << circle.y << ")" << std::endl;
    std::cout << "Fitted Circle Radius: " << circle.radius << std::endl;
    double radius=0.0;
    radius=circle.radius*100000;
    // 显示曲率半径
    ui->label_62->setText(QString::number(radius, 'f', 7));
}



void MainWindow::recordData()
{
    if(ui->pushButton_30->text() == tr("采集数据"))
    {
        ui->pushButton_30->setText("停止采集");
        record=ON;
    }
    else
    {
        //修改按键文字
        ui->pushButton_30->setText("采集数据");
        record=OFF;
    }
}

// Slot function to save data to process.txt when button clicked
void MainWindow::saveDataToFile() {
    QFile file("C:/Users/1/Desktop/qtsocket/myClientprocess.txt");
    if (file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QTextStream out(&file);
        for (const Data &data : collectedData) {
            out << data.timestamp << ","
                << data.x_coord << ","
                << data.y_coord << ","
                << data.heading_angle << ","
                << data.expected_heading_angle << "\n";
        }
        collectedData.clear();
        file.close();
        QProcess *process = new QProcess(this);
        process->start("python3 C:/Users/1/Desktop/qtsocket/processdata.py C:/Users/1/Desktop/qtsocket/myClientprocess.txt");
    }
}



// 槽函数定义
void MainWindow::onRadioButtonToggled()
{
    // 判断哪个按钮被选中
    if (ui->radioButton_14->isChecked()) {
        //qDebug() << "radioButton_11 被选中";
        db_ipc->HA=HAND;
        // 你可以在这里添加radioButton_11被选中时的操作
    } else if (ui->radioButton_13->isChecked()) {
        //qDebug() << "radioButton_12 被选中";
        db_ipc->HA=AUTO;
        // 你可以在这里添加radioButton_12被选中时的操作
    }
}

