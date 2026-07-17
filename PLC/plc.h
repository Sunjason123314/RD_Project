#ifndef PLC_H
#define PLC_H

#include <QObject>
#include <QList>
#include <QMetaType>
#include <QModbusDataUnit> // 存储接收和发送数据的类，数据类型为1bit和16bit
#include <QModbusReply>    // 客户端访问服务器后得到的回复（如客户端读服务器数据时包含数据信息）
#include <QModbusTcpClient>
#include <QTimer>
#include <QThread>
#include <QString>

class QModbusClient;
class QModbusReply;

// PLC 数据快照：用于从 PLC 子线程向界面线程安全传递最新数据。
// Plc 对象 moveToThread 后，界面线程不要直接读取 Plc 的成员变量，
// 而是接收 dataUpdated 信号中的这个值类型拷贝。
struct PlcDataSnapshot
{
    double m_new_angle = 0.0;      // 转台角度
    double m_new_angle_px = 0.0;   // 转台角度（像素）
    double m_new_lizhu1_X = 0.0;   // 立柱1位置坐标
    double m_new_sensor1_Y = 0.0;  // 传感器1位置坐标
    double m_new_sensor1_Z = 0.0;  // 传感器1位置坐标
    double m_new_sensor2_Y = 0.0;  // 传感器2位置坐标
    double m_new_sensor2_Z = 0.0;  // 传感器2位置坐标
    double m_tiaoqing1 = 0.0;      // 调倾1轴位置
    double m_tiaoqing2 = 0.0;      // 调倾2轴位置
    double m_tiaoqing3 = 0.0;      // 调倾3轴位置
    double m_tiaoqing4 = 0.0;      // 调倾4轴位置
    double m_tiaoxin1 = 0.0;       // 调心1轴位置
    double m_tiaoxin2 = 0.0;       // 调心2轴位置
    double m_small_angle = 0.0;    // 小转台角度
};

// 声明为 Qt 元类型，配合 qRegisterMetaType 后可通过 QueuedConnection 跨线程传递。
Q_DECLARE_METATYPE(PlcDataSnapshot)

class Plc : public QObject
{
    Q_OBJECT

public:
    explicit Plc(QObject *parent = nullptr);

    // PLC 通讯对象。它是 Plc 的子对象，会随着 Plc 一起在 PLC 子线程中运行。
    QModbusTcpClient *m_modbus_device = nullptr;

    // PLC 实时数据：仅在 PLC 所在线程内更新；界面线程请通过 dataUpdated 快照读取。
    double m_new_angle = 0.0;      // 转台角度
    double m_new_angle_px = 0.0;   // 转台角度（像素）
    double m_new_lizhu1_X = 0.0;   // 立柱1位置坐标
    double m_new_sensor1_Y = 0.0;  // 传感器1位置坐标
    double m_new_sensor1_Z = 0.0;  // 传感器1位置坐标
    double m_new_sensor2_Y = 0.0;  // 传感器2位置坐标
    double m_new_sensor2_Z = 0.0;  // 传感器2位置坐标
    double m_tiaoqing1 = 0.0;      // 调倾1轴位置
    double m_tiaoqing2 = 0.0;      // 调倾2轴位置
    double m_tiaoqing3 = 0.0;      // 调倾3轴位置
    double m_tiaoqing4 = 0.0;      // 调倾4轴位置
    double m_tiaoxin1 = 0.0;       // 调心1轴位置
    double m_tiaoxin2 = 0.0;       // 调心2轴位置
    double m_small_angle = 0.0;    // 小转台角度

    // PLC连接状态查询。跨线程请通过 requestConnectionState/plcConnectionStateReady。
    bool isPlcConnected() const;

    // LReal类型数据转换方法
    double readLReal(const QModbusDataUnit &unit, int index, bool littleEndian = true);
    QList<quint16> writeLReal(double value, bool littleEndian = true);

    // 将当前 PLC 实时数据复制成快照；通常只在 PLC 子线程内调用。
    PlcDataSnapshot currentData() const;

public slots:
    // 以下槽函数是 PLC 子线程的对外入口。
    // MainWindow 不直接跨线程调用成员函数，而是用 QMetaObject::invokeMethod 以队列方式投递。
    bool connectPlc();
    void disconnectPlc();
    void requestConnectionState();

    // Modbus 异步读取完成后的回调槽，在 PLC 子线程内解析数据并发送快照。
    void readReady();

    // 开始采集数据：由界面定时器投递到 PLC 子线程执行，避免阻塞 UI。
    void startCollectingData();

    // 调心时高频率采集
    void startHighFrequencyCollecting();
    void readReadyHighFreq();

    // 开始持续运动
    void startContinuousMovement(double cylinder, double address);

    // 移动到绝对位置
    void moveToAbsolutePosition(double cylinder, double address);

    // 写入寄存器数据
    void writeHoldingRegisters(double registers, double address);

    // 写入线圈数据，通用方法
    void writeCoilData(double value, double address);

private slots:
    // 连接状态变化处理
    void handleStateChange(QModbusDevice::State state);

    // 错误处理
    void handleError(QModbusDevice::Error error);

signals:
    // PLC 数据更新信号：携带值类型快照，安全发送到 GUI 线程刷新界面。
    void dataUpdated(const PlcDataSnapshot &snapshot);

    // PLC 连接状态变化信号：GUI 线程只维护状态缓存，不直接查询子线程对象。
    void plcConnectionStateReady(bool connected);
    void plcConnecting();                              // PLC连接中
    void plcConnected();                               // PLC已连接
    void plcDisconnected();                            // PLC已断开
    void plcError(const QString &errorString);         // PLC错误信号
};

#endif // PLC_H
