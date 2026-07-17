#ifndef PLCCONNECT_H
#define PLCCONNECT_H

#include <QWidget>

namespace Ui {
class PlcConnect;
}

class PlcConnect : public QWidget
{
    Q_OBJECT

public:
    explicit PlcConnect(QWidget *parent = nullptr);
    ~PlcConnect();

    void setPlcConnected(bool connected);

    void setPlcConnecting();

signals:
    void connectRequested();
    void disconnectRequested();


private:
    void initStyle();

private:
    Ui::PlcConnect *ui;
    bool m_isConnected;
};

#endif // PLCCONNECT_H
