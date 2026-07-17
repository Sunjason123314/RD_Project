#include "plc.h"
#include <QModbusTcpClient>
#include <QDebug>
#include <QTimer>
#include <QThread>

Plc::Plc(QObject *parent) : QObject(parent)
{
    m_modbus_device = new QModbusTcpClient(this);

    // 连接状态变化的槽
    connect(m_modbus_device, &QModbusDevice::stateChanged, this, &Plc::handleStateChange);

    // 错误处理槽
    connect(m_modbus_device, &QModbusDevice::errorOccurred, this, &Plc::handleError);
}

PlcDataSnapshot Plc::currentData() const
{
    // 将 PLC 子线程内维护的实时成员变量复制为值类型快照。
    // 该快照随后通过 dataUpdated 信号发送给 GUI 线程，避免界面直接跨线程读 Plc 成员。
    PlcDataSnapshot snapshot;
    snapshot.m_new_angle = m_new_angle;
    snapshot.m_new_angle_px = m_new_angle_px;
    snapshot.m_new_lizhu1_X = m_new_lizhu1_X;
    snapshot.m_new_sensor1_Y = m_new_sensor1_Y;
    snapshot.m_new_sensor1_Z = m_new_sensor1_Z;
    snapshot.m_new_sensor2_Y = m_new_sensor2_Y;
    snapshot.m_new_sensor2_Z = m_new_sensor2_Z;
    snapshot.m_tiaoqing1 = m_tiaoqing1;
    snapshot.m_tiaoqing2 = m_tiaoqing2;
    snapshot.m_tiaoqing3 = m_tiaoqing3;
    snapshot.m_tiaoqing4 = m_tiaoqing4;
    snapshot.m_tiaoxin1 = m_tiaoxin1;
    snapshot.m_tiaoxin2 = m_tiaoxin2;
    snapshot.m_small_angle = m_small_angle;
    return snapshot;
}

// 处理PLC连接状态变化
void Plc::handleStateChange(QModbusDevice::State state)
{
    switch (state) {
    case QModbusDevice::ConnectingState:
        qDebug() << "PLC connecting...";
        emit plcConnectionStateReady(false);
        emit plcConnecting();
        break;
    case QModbusDevice::ConnectedState:
        qDebug() << "PLC connected.";
        emit plcConnectionStateReady(true);
        emit plcConnected();
        break;
    case QModbusDevice::UnconnectedState:
        qDebug() << "PLC disconnected.";
        emit plcConnectionStateReady(false);
        emit plcDisconnected();
        break;
    default:
        break;
    }
}

// 处理PLC错误
void Plc::handleError(QModbusDevice::Error error)
{
    if (error != QModbusDevice::NoError) {
        QString err = m_modbus_device ? m_modbus_device->errorString() : QStringLiteral("PLC device is null");
        qWarning() << "PLC error:" << err;
        emit plcConnectionStateReady(false);
        emit plcError(err);
    }
}

// 从Modbus数据单元中读取LReal类型数据
double Plc::readLReal(const QModbusDataUnit &unit, int index, bool littleEndian)
{
    union {
        uint16_t u16[4];
        double d;
    } value;

    // 根据字节序填充数据
    if (littleEndian) {
        for (int i = 0; i < 4; ++i)
            value.u16[i] = unit.value(index + i);
    } else {
        for (int i = 0; i < 4; ++i)
            value.u16[3 - i] = unit.value(index + i);
    }

    return value.d;
}

// 将LReal类型数据写入Modbus
QList<quint16> Plc::writeLReal(double value, bool littleEndian)
{
    union {
        double d;
        quint16 u16[4];
    } data;
    data.d = value;

    QList<quint16> list;
    // 根据字节序添加数据
    if (littleEndian) {
        for (int i = 0; i < 4; ++i)
            list.append(data.u16[i]);
    } else {
        for (int i = 3; i >= 0; --i)
            list.append(data.u16[i]);
    }

    return list;
}

// 连接PLC
bool Plc::connectPlc()
{
    if (!m_modbus_device) {
        qWarning() << "modbusDevice is null, cannot connect PLC.";
        emit plcError(QStringLiteral("modbusDevice is null, cannot connect PLC."));
        return false;
    }

    if (m_modbus_device->state() == QModbusDevice::ConnectedState) {
        qDebug() << "PLC is already connected.";
        emit plcConnectionStateReady(true);
        return true;
    }

    if (m_modbus_device->state() == QModbusDevice::ConnectingState) {
        qDebug() << "PLC is already connecting.";
        emit plcConnecting();
        return false;
    }

    if (m_modbus_device->state() == QModbusDevice::ClosingState) {
        qDebug() << "PLC is closing, please wait.";
        return false;
    }

    // 设置连接参数
    m_modbus_device->setConnectionParameter(QModbusDevice::NetworkPortParameter, 502);
    m_modbus_device->setConnectionParameter(QModbusDevice::NetworkAddressParameter, "192.168.1.10");
    m_modbus_device->setTimeout(2000);
    m_modbus_device->setNumberOfRetries(3);

    emit plcConnecting();

    // 尝试连接PLC
    if (!m_modbus_device->connectDevice()) {
        qWarning() << "PLC connect request failed:" << m_modbus_device->errorString();
        emit plcConnectionStateReady(false);
        emit plcError(m_modbus_device->errorString());
        return false;
    }

    qDebug() << "PLC connect request started.";
    return true;
}

// 断开PLC连接
void Plc::disconnectPlc()
{
    if (!m_modbus_device) {
        qWarning() << "modbusDevice is null, cannot disconnect PLC.";
        emit plcConnectionStateReady(false);
        return;
    }

    QModbusDevice::State state = m_modbus_device->state();
    if (state == QModbusDevice::UnconnectedState) {
        qDebug() << "PLC is already disconnected.";
        emit plcConnectionStateReady(false);
        emit plcDisconnected();
        return;
    }

    if (state == QModbusDevice::ClosingState) {
        qDebug() << "PLC is already closing.";
        return;
    }

    qDebug() << "Disconnecting PLC...";
    m_modbus_device->disconnectDevice();
}

// 判断PLC是否已连接
bool Plc::isPlcConnected() const
{
    return m_modbus_device && m_modbus_device->state() == QModbusDevice::ConnectedState;
}

void Plc::requestConnectionState()
{
    // 跨线程状态查询入口：外部线程投递该槽函数，PLC 线程再用信号返回结果。
    emit plcConnectionStateReady(isPlcConnected());
}

// 开始采集数据
void Plc::startCollectingData()
{
    // 该函数由 GUI 定时器通过 QueuedConnection 投递到 PLC 子线程执行。
    // 在未连接时直接返回，避免在界面线程中阻塞或产生无效 Modbus 请求。
    if (!m_modbus_device)
        return;

    if (m_modbus_device->state() != QModbusDevice::ConnectedState)
        return;

    QModbusDataUnit readUnit(QModbusDataUnit::InputRegisters, 0, 60);
    if (auto *reply = m_modbus_device->sendReadRequest(readUnit, 1)) {
        if (!reply->isFinished())
            connect(reply, &QModbusReply::finished, this, &Plc::readReady);
        else
            delete reply;
    }
}

// 处理读取到的数据
void Plc::readReady()
{
    auto reply = qobject_cast<QModbusReply *>(sender());
    if (!reply)
        return;

    if (reply->error() == QModbusDevice::NoError) {
        const QModbusDataUnit unit = reply->result();
        // 在 PLC 子线程内解析 Modbus 数据并更新实时成员变量。
        m_new_angle = readLReal(unit, 0);
        m_new_lizhu1_X = readLReal(unit, 20);
        m_new_sensor1_Y = readLReal(unit, 4);
        m_new_sensor1_Z = readLReal(unit, 12);
        m_tiaoqing1 = readLReal(unit, 28);
        m_tiaoqing2 = readLReal(unit, 32);
        m_tiaoqing3 = readLReal(unit, 36);
        m_tiaoqing4 = readLReal(unit, 40);
        m_tiaoxin1 = readLReal(unit, 44);
        m_tiaoxin2 = readLReal(unit, 48);
        m_small_angle = readLReal(unit, 24);
        m_new_sensor2_Y = readLReal(unit, 8);
        m_new_sensor2_Z = readLReal(unit, 16);

        // 只把值类型快照发送给 GUI 线程，避免 GUI 线程直接访问子线程对象内部数据。
        emit dataUpdated(currentData());
    } else {
        qWarning() << "PLC read error:" << reply->errorString();
        emit plcError(reply->errorString());
    }
    reply->deleteLater();
}

void Plc::startHighFrequencyCollecting()
{
    // 高频采集入口保留给调心流程使用，目前复用普通采集逻辑，仍在 PLC 子线程执行。
    startCollectingData();
}

void Plc::readReadyHighFreq()
{
    // 高频读取完成后的处理与普通读取一致，统一生成快照并通知界面。
    readReady();
}

// 持续运动（写入位移数据）
void Plc::startContinuousMovement(double cylinder, double address)
{
    writeCoilData(cylinder, address);
}

// 移动到绝对位置（写入位移数据并复位触发信号）
void Plc::moveToAbsolutePosition(double cylinder, double address)
{
    writeCoilData(cylinder, address);

    // 100ms 后自动写 0，复位触发信号。
    // this 属于 PLC 子线程，因此 singleShot 的回调也会回到 PLC 子线程执行。
    QTimer::singleShot(100, this, [=]() {
        if (!m_modbus_device || m_modbus_device->state() != QModbusDevice::ConnectedState)
            return;

        QModbusDataUnit resetUnit(QModbusDataUnit::Coils, address, 1);
        resetUnit.setValue(0, 0);  // 写入 0
        if (auto *resetReply = m_modbus_device->sendWriteRequest(resetUnit, 1)) {
            if (!resetReply->isFinished()) {
                connect(resetReply, &QModbusReply::finished, this, [=]() {
                    if (resetReply->error() != QModbusDevice::NoError)
                        qDebug() << "Write 0 error:" << resetReply->errorString();
                    resetReply->deleteLater();
                });
            } else {
                resetReply->deleteLater();
            }
        }
    });
}

// 写入线圈数据，通用方法
void Plc::writeCoilData(double value, double address)
{
    // 所有写 PLC 操作都在 PLC 子线程内执行；断开时直接忽略，避免无效写入。
    if (!m_modbus_device || m_modbus_device->state() != QModbusDevice::ConnectedState)
        return;

    QModbusDataUnit writeUnit(QModbusDataUnit::Coils, address, 1);
    writeUnit.setValue(0, value);

    if (auto *reply = m_modbus_device->sendWriteRequest(writeUnit, 1)) {
        if (!reply->isFinished()) {
            connect(reply, &QModbusReply::finished, this, [this, reply]() {
                if (reply->error() != QModbusDevice::NoError)
                    qDebug() << "Write error:" << reply->errorString();
                reply->deleteLater();
            });
        } else {
            reply->deleteLater();
        }
    }
}

// 写寄存器数据
void Plc::writeHoldingRegisters(double registers, double address)
{
    // 该槽函数由 MainWindow 的 postPlcWriteHoldingRegisters 投递到 PLC 子线程执行。
    // 先检查连接状态，防止 PLC 断开时继续发起 Modbus 写请求。
    if (!m_modbus_device || m_modbus_device->state() != QModbusDevice::ConnectedState)
        return;

    double val = registers;
    qDebug() << "Writing" << val << "to PLC";

    // 将 double 拆成 4 个 uint16 寄存器
    QList<quint16> regs = writeLReal(val);

    // 写入 4 个寄存器，从地址 0 开始
    QModbusDataUnit writeUnit(QModbusDataUnit::HoldingRegisters, address, 4);
    for (int i = 0; i < 4; ++i) {
        writeUnit.setValue(i, regs[i]);
    }

    qDebug() << "Writing to PLC:" << regs;

    if (auto *reply = m_modbus_device->sendWriteRequest(writeUnit, 1)) {
        if (!reply->isFinished()) {
            connect(reply, &QModbusReply::finished, this, [this, reply]() {
                if (reply->error() != QModbusDevice::NoError)
                    qDebug() << "Write error:" << reply->errorString();
                reply->deleteLater();
            });
        } else {
            reply->deleteLater();
        }
    }
}
