#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTcpSocket>
#include <QLocalSocket>
#include <QMessageBox>
#include <QKeySequenceEdit>
#include <QKeyEvent>
#include <QDebug>
#include <QTimer>
#include "data.h"
#include "qgeocoordinate.h"
#include <QScatterSeries>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>         //画坐标轴头文件
#include <QtCharts/QChartView>         //画绘图区域头文件
#include <QtCore/QTimer>               //时间定时器头文件
#include <QtWidgets/QGesture>          //手势头文件（鼠标动作）
#include <QtWidgets/QGraphicsScene>    //绘图屏幕参数头文件
#include <QtCharts>

#include <QQuickWidget>

#include <vector>
#include <algorithm>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    DB1 *db;
    Boatcontrol *db_ipc;
    void debug_print();
    QSplineSeries *targetSeries;
    QSplineSeries *currentSeries;
    QChart *chart;
    QTimer *timer_update;
    QChartView *chartView;
    Q_INVOKABLE void handleRightClick(double lat, double lon);
    void initChart();
    ONOFF record;
    QList<Data> collectedData;

private slots:
    void on_Btn_Connect_clicked();

    void onStateChanged(int state);

    void onErrorOccurred();

    void Read_Data();

    void on_Btn_exit_clicked();

    void on_Btn_send_clicked();

    void manual_control_direction();

    void pushButton_5_on_clicked();

    void pushButton_7_on_clicked();

    void pushButton_15_on_clicked();

    void pushButton_16_on_clicked();

    void pushButton_27_on_clicked();

    void pushButton_9_on_clicked();
    void pushButton_12_on_clicked();

    void pushButton_26_on_clicked();

    void target_pid_send();
    void target_pid_send_rpm();

    void target_z_send();
    void target_z_send_rpm();

    void target_t_send();
    void send_ipc();

    void stop_boat();

    void change_index(int index);

    void show_lcd(int index);
    void show_lcd_1(int index);
    void show_lcd_20(int index);
    void show_lcd_21(int index);
    void show_lcd_24(int index);
    void updateLineEdit_1(int value);
    void updateLineEdit_2(int value);
    void updateLineEdit_3(int value);
    void updateLineEdit_4(int value);
    void updateLineEdit_5(int value);
    void updateLineEdit_6(int value);
    void updateLineEdit_7(int value);
    void show_lcd_22(int index);
    void show_lcd_23(int index);
    void updateData();

    void onRadioButtonToggled();
    //python后处理
    void saveDataToFile();
    void recordData();

    //更新地图上小船的位置
    void updateCarPosition();

    void clearComboBox();
    void clearPathsAndMarkers();
    //**************
    void updateLineEdit(int value);

    //记录小船的运动轨迹
    void onRecordButtonClicked();
    void onClearTrackButtonClicked();
    void onCalculateCurvatureButtonClicked();

    void back_rudder_send();

    void pushButton_34_on_clicked();

protected:
    void keyPressEvent(QKeyEvent*event) override;

private:
    Ui::MainWindow *ui;
    QTcpSocket *socket;
    QQuickWidget *quickWidget;
    /* 声明QKeySequenceEdit对象 */
    QKeySequenceEdit  *keySequenceEdit;

    QVector<QPointF> previousPoints; // 保存先前的坐标点

    bool isRecording;
    QList<QGeoCoordinate> trackPoints;

};

#endif // MAINWINDOW_H
