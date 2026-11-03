#include "threadmanager.h"

#include <QMetaObject>
#include <QThread>

ThreadManager::ThreadManager(QObject *parent)
    : QObject(parent)
{
    // 该枚举会作为信号参数跨线程传递，因此在构造时注册到 Qt 元对象系统。
    qRegisterMetaType<ThreadManager::Module>("ThreadManager::Module");
}

ThreadManager::~ThreadManager()
{
    // 管理器销毁时统一关闭所有模块线程，避免程序退出时留下运行中的 QThread。
    stopAll(3000);
}

bool ThreadManager::startModule(Module module,
                                QObject *workerObject,
                                const QString &threadName,
                                const char *shutdownMethod,
                                QThread::Priority priority,
                                int instanceId)
{
    // worker 是线程中真正执行业务的对象，不能为空。
    if(workerObject == nullptr)
    {
        emit moduleStartFailed(module, tr("Worker object is null."));
        return false;
    }

    const int key = moduleKey(module, instanceId);

    // 同一个模块实例只允许启动一个线程，防止重复 moveToThread 或重复管理。
    if(hasModule(module, instanceId))
    {
        emit moduleStartFailed(module, tr("Module thread already exists."));
        workerObject->deleteLater();
        return false;
    }

    QThread *moduleThread = new QThread;
    QString actualThreadName = threadName.isEmpty() ? moduleName(module) : threadName;
    if(instanceId != 0 && threadName.isEmpty())
        actualThreadName += QStringLiteral("_%1").arg(instanceId);
    moduleThread->setObjectName(actualThreadName);

    // 将 worker 移动到新线程；线程结束后由 Qt 自动延迟删除 worker。
    workerObject->moveToThread(moduleThread);
    connect(moduleThread, &QThread::finished,
            workerObject, &QObject::deleteLater);

    // 保存模块上下文，后续停止线程或查询状态都依赖这份记录。
    ModuleContext context;
    context.module = module;
    context.instanceId = instanceId;
    context.thread = moduleThread;
    context.worker = workerObject;
    context.threadName = actualThreadName;
    if(shutdownMethod != nullptr)
        context.shutdownMethod = shutdownMethod;

    m_modules.insert(key, context);

    // 线程自然结束或 stopModule 触发结束时，从管理表中移除并通知外部。
    connect(moduleThread, &QThread::finished,
            this, [this, module, instanceId, actualThreadName]() {
                m_modules.remove(moduleKey(module, instanceId));
                emit moduleStopped(module, actualThreadName);
            });

    moduleThread->start(priority);
    emit moduleStarted(module, actualThreadName);
    return true;
}

void ThreadManager::stopModule(Module module, int waitMs, int instanceId)
{
    const int key = moduleKey(module, instanceId);
    if(!m_modules.contains(key))
        return;

    // 先从表中取出，避免关闭过程中外部误认为该模块仍可用。
    ModuleContext context = m_modules.take(key);
    requestShutdownAndQuit(module, context);

    if(context.thread)
    {
        // 等待事件循环退出；超时则恢复上下文记录，交给外部决定后续处理。
        const bool stopped = context.thread->wait(waitMs);
        if(!stopped)
        {
            emit moduleStopTimedOut(module, context.threadName);
            m_modules.insert(key, context);
            return;
        }

        deleteThreadObject(context.thread);
    }
}

void ThreadManager::stopAll(int waitMs)
{
    // 复制上下文后逐个停止，避免遍历过程中 m_modules 被修改导致迭代器失效。
    const QList<ModuleContext> contexts = m_modules.values();
    for(const ModuleContext& context : contexts)
    {
        stopModule(context.module, waitMs, context.instanceId);
    }
}

bool ThreadManager::hasModule(Module module, int instanceId) const
{
    return m_modules.contains(moduleKey(module, instanceId));
}

bool ThreadManager::isRunning(Module module, int instanceId) const
{
    const auto iter = m_modules.constFind(moduleKey(module, instanceId));
    return iter != m_modules.constEnd() && iter->thread && iter->thread->isRunning();
}

QObject *ThreadManager::worker(Module module, int instanceId) const
{
    const auto iter = m_modules.constFind(moduleKey(module, instanceId));
    return iter == m_modules.constEnd() ? nullptr : iter->worker;
}

QThread *ThreadManager::thread(Module module, int instanceId) const
{
    const auto iter = m_modules.constFind(moduleKey(module, instanceId));
    return iter == m_modules.constEnd() ? nullptr : iter->thread;
}

QString ThreadManager::moduleName(Module module) const
{
    // 默认线程名集中维护，便于调试器、日志和 Qt Creator 线程视图识别。
    switch(module)
    {
    case Module::ConfocalSensor:
        return QStringLiteral("ConfocalSensorThread");
    case Module::Dlp:
        return QStringLiteral("DlpThread");
    case Module::Camera:
        return QStringLiteral("CameraThread");
    case Module::CameraProjector:
        return QStringLiteral("CameraProjectorThread");
    case Module::PointCloud:
        return QStringLiteral("PointCloudThread");
    case Module::PointCloudDisplay:
        return QStringLiteral("PointCloudDisplayThread");
    }

    return QStringLiteral("UnknownModuleThread");
}

int ThreadManager::moduleKey(Module module, int instanceId) const
{
    // QMap 使用 int 作为 key；高位保存模块枚举，低位保存实例编号，支持相机等多实例线程。
    return static_cast<int>(module) * 10000 + instanceId;
}

void ThreadManager::requestShutdownAndQuit(Module module, const ModuleContext &context)
{
    Q_UNUSED(module)

    if(!context.thread)
        return;

    if(!context.worker || context.shutdownMethod.isEmpty())
    {
        // 没有 worker 清理函数时，直接请求线程事件循环退出。
        context.thread->quit();
        return;
    }

    QObject *worker = context.worker;
    QThread *workerThread = worker->thread();

    if(workerThread == QThread::currentThread())
    {
        // 如果当前已经在 worker 所在线程，直接调用清理方法即可。
        QMetaObject::invokeMethod(worker,
                                  context.shutdownMethod.constData(),
                                  Qt::DirectConnection);
        context.thread->quit();
        return;
    }

    if(workerThread && workerThread->isRunning())
    {
        // 跨线程关闭时同步调用 worker 的清理函数，确保 closeDevice() 真正执行完
        // 再退出线程事件循环，避免程序退出后控制器仍保持连接状态。
        QMetaObject::invokeMethod(worker,
                                  context.shutdownMethod.constData(),
                                  Qt::BlockingQueuedConnection);
    }
    else
    {
        // worker 线程事件循环已经不在运行时，阻塞队列调用无法送达，只能在当前线程兜底清理。
        QMetaObject::invokeMethod(worker,
                                  context.shutdownMethod.constData(),
                                  Qt::DirectConnection);
    }

    context.thread->quit();
}

void ThreadManager::deleteThreadObject(QThread *thread)
{
    if(!thread)
        return;

    // 如果当前就在 QThread 对象所属线程，直接 delete；否则使用 deleteLater 交给其归属线程处理。
    if(thread->thread() == QThread::currentThread())
        delete thread;
    else
        thread->deleteLater();
}
