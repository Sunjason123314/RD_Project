#ifndef POINTCLOUDLISTVIEW_H
#define POINTCLOUDLISTVIEW_H

#include <QListView>
#include <QItemSelection>
#include <QPoint>

class QRubberBand;
class QMouseEvent;
class QDragMoveEvent;
class QDropEvent;

/**
 * @brief 点云文件列表视图（扁平模型 + Word 式交互 + 分组标题点击 + 拖入分组）
 *
 * 基于 QListView，模型保持扁平（所有项都在顶层，不使用父子结构），
 * 分组通过“分组标题行 + 成员 GroupName 标记 + 行隐藏”实现，避免移动数据项。
 *
 * 在标准 QListView 基础上补齐几点交互：
 *  1. 左键框选可从空白区域「或任意未选中项」上开始，不再受限于狭小的空白空间；
 *  2. 框选过程中绘制可见的橡皮筋矩形（QRubberBand），更直观；
 *  3. 框选时滚动滚轮，已选中但滚出视野的项不会丢失（起点用内容坐标）；
 *  4. 内部拖拽移动条目后保持滚动位置，不再自动弹回顶部；
 *  5. 单击「分组标题行」时发出 groupHeaderClicked 信号，供外部切换折叠/展开；
 *  6. 把项拖到「分组标题行 / 分组成员行」上时发出 filesDroppedIntoGroup，
 *     拖到无分组的空白/普通项上时发出 filesDroppedOutOfGroup。
 *
 * 在已选中项上按住左键拖拽仍交给基类处理（InternalMove 移动位置）。
 */
class PointCloudListView : public QListView
{
    Q_OBJECT
public:
    explicit PointCloudListView(QWidget* parent = nullptr);
    ~PointCloudListView() override;

signals:
    // 单击分组标题行时发出（index 为该标题行）
    void groupHeaderClicked(const QModelIndex& index);
    // 把拖拽项放入某分组（groupName 为目标分组名）
    void filesDroppedIntoGroup(const QString& groupName);
    // 把拖拽项放到无分组区域（移出分组）
    void filesDroppedOutOfGroup();

protected:
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void dragMoveEvent(QDragMoveEvent* event) override;
    void dropEvent(QDropEvent* event) override;
    void scrollContentsBy(int dx, int dy) override;

private:
    // 判断某 index 是否为分组标题行（RoleEntryType == Group）。
    bool isGroupHeader(const QModelIndex& index) const;
    // 返回某 index 所属的分组名：标题行返回其组名；成员行返回 RoleGroupName；否则空。
    QString groupNameAt(const QModelIndex& index) const;
    // viewport 坐标 <-> 内容坐标（含滚动偏移）互转。
    QPoint toContent(const QPoint& viewportPos) const;
    QPoint toViewport(const QPoint& contentPos) const;
    // 依据当前鼠标（内容坐标）刷新橡皮筋与选择。
    void updateBandFromContent(const QPoint& curContent, Qt::KeyboardModifiers modifiers);
    // 微小移动视作单击的处理逻辑。
    void handleClickLikeRelease(Qt::KeyboardModifiers modifiers);
    // 恢复滚动位置，避免拖拽移动后跳到顶部。
    void restoreScroll(int value);

    QRubberBand* m_rubberBand = nullptr;
    QPoint m_bandOriginContent;          // 按下时的内容坐标（viewport + 滚动偏移）
    QPoint m_lastMoveContent;            // 最近一次鼠标内容坐标（供滚动时重算）
    bool m_banding = false;              // 是否处于自定义框选状态
    Qt::KeyboardModifiers m_bandModifiers = Qt::NoModifier;
    QItemSelection m_selectionBeforeBand; // 框选开始前的选择（供 Ctrl 叠加）
};

#endif // POINTCLOUDLISTVIEW_H
