#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>

namespace Ui {
class Widget;
}

class Widget : public QWidget
{
    Q_OBJECT

public:
    explicit Widget(QWidget *parent = nullptr);
    ~Widget();

private slots:

    void on_lineEditIp_textChanged(const QString &arg1);

    void on_lineEditPort_textChanged(const QString &arg1);


    void clicked_lineEdit();
private:
    Ui::Widget *ui;
    bool flag_ip=false;
    bool flag_port=false;
};

#endif // WIDGET_H
