#ifndef THREADMANAGER_H
#define THREADMANAGER_H

#include <QObject>
#include <QByteArray>
#include <QMap>
#include <QList>
#include <QMetaType>
#include <QString>
#include <QThread>

/**
 * @brief 统一线程管理器。
 *
 * ThreadManager 用于把不同业务模块的 worker 对象移动到独立 QThread 中运行，
 * 并集中处理线程启动、关闭、超时和对象释放。这样主窗口不需要为每个模块重复
 * 编写线程管理代码。
 */
class ThreadManager : public QObject
{
    Q_OBJECT

public:
    /**
     * @brief 可被 ThreadManager 管理的业务模块枚举。
     *
     * 枚举值会作为 m_modules 的 key 使用，因此新增模块时只需要在这里补充枚举，
     * 并在 moduleName() 中增加默认线程名即可。
     */
    enum class Module
    {
        ConfocalSensor = 0, ///< 光谱共焦传感器线程。
        Plc,               ///< PLC 通信线程。
        Dlp,               ///< DLP 投影仪线程。
        Camera,            ///< 工业相机线程，可按实例编号管理多个相机窗口。
        CameraProjector,   ///< 相机/DLP 投影相关预留线程。
        PointCloud,        ///< 点云通用处理线程。
        PointCloudDisplay  ///< 点云显示相关的数据加载/准备线程。
    };
    Q_ENUM(Module)

    explicit ThreadManager(QObject *parent = nullptr);
    ~ThreadManager() override;

    /**
     * @brief 启动指定模块线程，并将 worker 移动到该线程。
     * @param module 模块类型。
     * @param worker 要放入线程执行的工作对象，启动失败时由本函数负责 deleteLater。
     * @param threadName 自定义线程名；为空时使用 moduleName() 的默认名称。
     * @param shutdownMethod 停止线程前需要在 worker 线程中调用的清理槽函数名，可为空。
     * @param priority QThread 启动优先级。
     * @return 启动成功返回 true，参数无效或模块已存在返回 false。
     */
    bool startModule(Module module,
                     QObject *worker,
                     const QString &threadName = QString(),
                     const char *shutdownMethod = nullptr,
                     QThread::Priority priority = QThread::InheritPriority,
                     int instanceId = 0);

    void stopModule(Module module, int waitMs = 3000, int instanceId = 0); ///< 停止指定模块线程。
    void stopAll(int waitMs = 3000);                   ///< 停止所有已注册模块线程。

    bool hasModule(Module module, int instanceId = 0) const; ///< 指定模块实例是否已注册线程上下文。
    bool isRunning(Module module, int instanceId = 0) const; ///< 指定模块实例线程是否正在运行。
    QObject *worker(Module module, int instanceId = 0) const; ///< 获取指定模块实例 worker 对象。
    QThread *thread(Module module, int instanceId = 0) const; ///< 获取指定模块实例线程对象。
    QString moduleName(Module module) const; ///< 获取模块默认线程名。

signals:
    void moduleStarted(ThreadManager::Module module, const QString &threadName); ///< 模块线程启动成功。
    void moduleStopped(ThreadManager::Module module, const QString &threadName); ///< 模块线程已停止。
    void moduleStartFailed(ThreadManager::Module module, const QString &reason); ///< 模块线程启动失败。
    void moduleStopTimedOut(ThreadManager::Module module, const QString &threadName); ///< 等待线程停止超时。

private:
    /**
     * @brief 单个模块的线程上下文。
     *
     * 保存线程对象、worker 对象、线程名以及退出前需要调用的 worker 清理方法。
     */
    struct ModuleContext
    {
        Module module = Module::ConfocalSensor; ///< 模块类型。
        int instanceId = 0;             ///< 同一模块的实例编号，相机窗口等多实例模块会使用。
        QThread *thread = nullptr;      ///< 模块线程对象。
        QObject *worker = nullptr;      ///< 已移动到模块线程的 worker。
        QString threadName;             ///< 线程名称，用于调试和日志。
        QByteArray shutdownMethod;      ///< 停止线程前调用的 worker 清理方法名。
    };

    int moduleKey(Module module, int instanceId = 0) const; ///< 将模块枚举和实例编号转换为 m_modules 使用的整数 key。
    void requestShutdownAndQuit(Module module, const ModuleContext &context); ///< 请求 worker 清理并退出线程事件循环。
    void deleteThreadObject(QThread *thread); ///< 根据当前线程关系安全释放 QThread 对象。

    QMap<int, ModuleContext> m_modules; ///< 已启动模块的线程上下文表。
};

Q_DECLARE_METATYPE(ThreadManager::Module)

#endif // THREADMANAGER_H
