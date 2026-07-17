#include "pointcloudlistview.h"

#include <QAbstractItemModel>
#include <QApplication>
#include <QDragMoveEvent>
#include <QDropEvent>
#include <QMouseEvent>
#include <QRubberBand>
#include <QScrollBar>
#include <QTimer>

namespace {
// 与 MainWindow 中的定义保持一致（mainwindow.h）：
//   RoleEntryType  = Qt::UserRole + 1
//   RoleGroupName  = Qt::UserRole + 6
//   PointCloudEntryType::Group = 4
constexpr int kRoleEntryType = Qt::UserRole + 1;
constexpr int kRoleGroupName = Qt::UserRole + 6;
constexpr int kEntryTypeGroup = 4;
}

PointCloudListView::PointCloudListView(QWidget* parent)
    : QListView(parent)
{
    // 像素级滚动，配合内容坐标做框选，滚动时不会跳过整行。
    setVerticalScrollMode(QAbstractItemView::ScrollPerPixel);
    // 拖放期间 Qt 不派发滚轮事件，靠“拖到边缘自动滚动”来滚动到远处再放下。
    setAutoScroll(true);
    setAutoScrollMargin(28);
}

PointCloudListView::~PointCloudListView() = default;

bool PointCloudListView::isGroupHeader(const QModelIndex& index) const
{
    if(!index.isValid())
        return false;
    return index.data(kRoleEntryType).toInt() == kEntryTypeGroup;
}

QString PointCloudListView::groupNameAt(const QModelIndex& index) const
{
    if(!index.isValid())
        return QString();
    if(index.data(kRoleEntryType).toInt() == kEntryTypeGroup)
        return index.data(kRoleGroupName).toString();
    return index.data(kRoleGroupName).toString();
}

QPoint PointCloudListView::toContent(const QPoint& viewportPos) const
{
    const int sx = horizontalScrollBar() ? horizontalScrollBar()->value() : 0;
    const int sy = verticalScrollBar() ? verticalScrollBar()->value() : 0;
    return QPoint(viewportPos.x() + sx, viewportPos.y() + sy);
}

QPoint PointCloudListView::toViewport(const QPoint& contentPos) const
{
    const int sx = horizontalScrollBar() ? horizontalScrollBar()->value() : 0;
    const int sy = verticalScrollBar() ? verticalScrollBar()->value() : 0;
    return QPoint(contentPos.x() - sx, contentPos.y() - sy);
}

void PointCloudListView::mousePressEvent(QMouseEvent* event)
{
    if(event->button() == Qt::LeftButton)
    {
        const QModelIndex index = indexAt(event->pos());

        // 分组标题行：既不进入框选，也不做默认选择，交给 release 时判定单击。
        if(isGroupHeader(index))
        {
            m_bandOriginContent = toContent(event->pos());
            m_banding = false;
            event->accept();
            return;
        }

        const bool onSelected = index.isValid() &&
                                selectionModel() &&
                                selectionModel()->isSelected(index);

        // 在已选中项上按下：交给基类处理（可能发起 InternalMove 拖拽移动）。
        // 在空白处或未选中项上按下：进入自定义框选。
        if(!onSelected)
        {
            m_banding = true;
            m_bandOriginContent = toContent(event->pos());
            m_lastMoveContent = m_bandOriginContent;
            m_bandModifiers = event->modifiers();
            if(selectionModel())
                m_selectionBeforeBand = selectionModel()->selection();
            else
                m_selectionBeforeBand.clear();

            if(!m_rubberBand)
                m_rubberBand = new QRubberBand(QRubberBand::Rectangle, viewport());
            m_rubberBand->setGeometry(QRect(event->pos(), QSize()));
            m_rubberBand->show();
            event->accept();
            return;
        }
    }

    QListView::mousePressEvent(event);
}

void PointCloudListView::mouseMoveEvent(QMouseEvent* event)
{
    if(m_banding && (event->buttons() & Qt::LeftButton))
    {
        m_bandModifiers = event->modifiers();
        const QPoint curContent = toContent(event->pos());
        m_lastMoveContent = curContent;
        updateBandFromContent(curContent, event->modifiers());
        event->accept();
        return;
    }

    QListView::mouseMoveEvent(event);
}

void PointCloudListView::mouseReleaseEvent(QMouseEvent* event)
{
    if(event->button() == Qt::LeftButton)
    {
        const QModelIndex releaseIndex = indexAt(event->pos());
        const QPoint originViewport = toViewport(m_bandOriginContent);
        const bool isClick =
            (event->pos() - originViewport).manhattanLength() < QApplication::startDragDistance();

        // 单击分组标题行：发出信号交由外部切换折叠/展开。
        if(isClick && isGroupHeader(releaseIndex) && isGroupHeader(indexAt(originViewport)))
        {
            emit groupHeaderClicked(releaseIndex);
            event->accept();
            return;
        }

        if(m_banding)
        {
            if(m_rubberBand)
                m_rubberBand->hide();
            m_banding = false;

            // 未产生实际拖动，按普通单击处理选择。
            if(isClick)
                handleClickLikeRelease(event->modifiers());

            m_selectionBeforeBand.clear();
            event->accept();
            return;
        }
    }

    QListView::mouseReleaseEvent(event);
}

void PointCloudListView::scrollContentsBy(int dx, int dy)
{
    QListView::scrollContentsBy(dx, dy);

    // 框选进行中滚动：用最近鼠标内容坐标立即重算，保证滚出视野的项仍保持选中。
    if(m_banding)
        updateBandFromContent(m_lastMoveContent, m_bandModifiers);
}

void PointCloudListView::dragMoveEvent(QDragMoveEvent* event)
{
    // 允许拖到分组标题行/分组成员行上（作为“放入分组”的目标），
    // 也允许拖到无分组区域（移出分组）。均接受，具体处理在 dropEvent。
    QListView::dragMoveEvent(event);

    // 拖放期间无滚轮事件：鼠标贴近上/下边缘时手动滚动，实现“拖到远处再放下”。
    if(QScrollBar* vbar = verticalScrollBar())
    {
        const int margin = autoScrollMargin();
        const int y = event->pos().y();
        const int h = viewport()->height();
        const int step = qMax(vbar->singleStep(), 8);
        if(y < margin)
            vbar->setValue(vbar->value() - step);
        else if(y > h - margin)
            vbar->setValue(vbar->value() + step);
    }

    event->acceptProposedAction();
}

void PointCloudListView::dropEvent(QDropEvent* event)
{
    const int scrollValue = verticalScrollBar() ? verticalScrollBar()->value() : 0;

    const QModelIndex dropIndex = indexAt(event->pos());
    const QString targetGroup = groupNameAt(dropIndex);

    if(!targetGroup.isEmpty())
    {
        // 放入某分组：不走默认 InternalMove（避免删行留下空白），交由外部打标记 + 重排。
        event->setDropAction(Qt::IgnoreAction);
        event->accept();
        emit filesDroppedIntoGroup(targetGroup);
        restoreScroll(scrollValue);
        return;
    }

    // 放到无分组区域：若拖拽项原本属于某分组，视作移出分组。
    // 先让基类完成排序移动，再发信号让外部清除 GroupName。
    QListView::dropEvent(event);
    emit filesDroppedOutOfGroup();
    restoreScroll(scrollValue);
}

void PointCloudListView::updateBandFromContent(const QPoint& curContent,
                                               Qt::KeyboardModifiers modifiers)
{
    if(!model() || !selectionModel())
        return;

    // 内容坐标系下的橡皮筋矩形
    const QRect contentRect = QRect(m_bandOriginContent, curContent).normalized();

    // 橡皮筋显示：换算回 viewport 坐标
    if(m_rubberBand)
    {
        const QRect viewportRect(toViewport(contentRect.topLeft()), contentRect.size());
        m_rubberBand->setGeometry(viewportRect);
    }

    // 超过拖动阈值才认为是框选
    const QPoint originViewport = toViewport(m_bandOriginContent);
    const QPoint curViewport = toViewport(curContent);
    if((curViewport - originViewport).manhattanLength() < QApplication::startDragDistance())
        return;

    QItemSelection bandSelection;
    const int rows = model()->rowCount(rootIndex());
    const int sy = verticalScrollBar() ? verticalScrollBar()->value() : 0;
    const int sx = horizontalScrollBar() ? horizontalScrollBar()->value() : 0;
    for(int row = 0; row < rows; ++row)
    {
        const QModelIndex idx = model()->index(row, 0, rootIndex());
        // 跳过隐藏行与分组标题行，框选只针对可见的数据项。
        if(isRowHidden(row) || isGroupHeader(idx))
            continue;
        // visualRect 是 viewport 坐标，换算成内容坐标再和内容矩形比较。
        QRect itemContentRect = visualRect(idx).translated(sx, sy);
        if(itemContentRect.intersects(contentRect))
            bandSelection.select(idx, idx);
    }

    // 按住 Ctrl 时，在框选开始前的选择基础上叠加。
    if(modifiers & Qt::ControlModifier)
        bandSelection.merge(m_selectionBeforeBand, QItemSelectionModel::Select);

    selectionModel()->select(bandSelection,
                             QItemSelectionModel::ClearAndSelect |
                             QItemSelectionModel::Rows);
}

void PointCloudListView::handleClickLikeRelease(Qt::KeyboardModifiers modifiers)
{
    if(!selectionModel())
        return;

    const QPoint originViewport = toViewport(m_bandOriginContent);
    const QModelIndex index = indexAt(originViewport);

    if(!index.isValid())
    {
        // 点击空白：无修饰键则清空选择。
        if(!(modifiers & (Qt::ControlModifier | Qt::ShiftModifier)))
            selectionModel()->clearSelection();
        return;
    }

    if(modifiers & Qt::ControlModifier)
    {
        selectionModel()->select(index,
                                 QItemSelectionModel::Toggle |
                                 QItemSelectionModel::Rows);
        selectionModel()->setCurrentIndex(index, QItemSelectionModel::NoUpdate);
    }
    else if(modifiers & Qt::ShiftModifier)
    {
        const QModelIndex anchor = currentIndex().isValid() ? currentIndex() : index;
        QItemSelection range(anchor, index);
        selectionModel()->select(range,
                                 QItemSelectionModel::ClearAndSelect |
                                 QItemSelectionModel::Rows);
        selectionModel()->setCurrentIndex(index, QItemSelectionModel::NoUpdate);
    }
    else
    {
        selectionModel()->select(index,
                                 QItemSelectionModel::ClearAndSelect |
                                 QItemSelectionModel::Rows);
        selectionModel()->setCurrentIndex(index, QItemSelectionModel::NoUpdate);
    }
}

void PointCloudListView::restoreScroll(int value)
{
    // 基类 dropEvent 内部可能触发 scrollTo，故延迟到事件循环末尾再恢复。
    QTimer::singleShot(0, this, [this, value]() {
        if(verticalScrollBar())
            verticalScrollBar()->setValue(value);
    });
}
