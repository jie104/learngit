#include "widget.h"
#include "ui_widget.h"
#include <iostream>

Widget::Widget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Widget)
{
    ui->setupUi(this);

//    ui->lineEditIp=new QLineEdit;
//    ui->lineEditPort=new QLineEdit;
//    ui->BtnDetect=new QPushButton;
    connect(ui->BtnDetect,SIGNAL(clicked()),this,SLOT(clicked_lineEdit()));

}

Widget::~Widget()
{
    delete ui;
}



void Widget::on_lineEditIp_textChanged(const QString &arg1)
{
    //等于雷达所设置的ip,待载入
    if (arg1=="192.168.71.2"){  //此处先默认雷达ip为192.168.71.2
        flag_ip=true;
    }
}

void Widget::on_lineEditPort_textChanged(const QString &arg1)
{
    //等于雷达所设置的port，待载入
    if (arg1=="2007"){
        flag_port=true;
    }

}

void Widget::clicked_lineEdit()
{
    if (flag_ip && flag_port){

        //角度误差结果，待写入
        ui->lineEditAngleError->insert("2");
        //测距误差结果，待写入
        ui->lineEditRangeError->insert("2");
        flag_ip=false;
        flag_port=false;

    }

}

