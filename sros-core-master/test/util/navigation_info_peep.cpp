/**
 * @file navigation_info_peep
 *
 * @author pengjiali
 * @date 20-3-4.
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#include "navigation_info_peep.h"
#include <glog/logging.h>
#include <QPainter>
#include <QPen>
#include <QVBoxLayout>
#include <QtWidgets/QGraphicsView>

using namespace std;
using namespace nav;
using namespace sros::core;
using namespace sros::map;
using namespace sros::map::net;

GraphicStationsItem::GraphicStationsItem(sros::map::StationMark start_station, sros::map::StationMark dst_station)
    : start_station_(start_station), dst_station_(dst_station) {
    m_boundingRect.setRect(-500, -500, 1000, 1000);
}

QRectF GraphicStationsItem::boundingRect() const { return m_boundingRect; }

void GraphicStationsItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) {
    Q_UNUSED(option);
    Q_UNUSED(widget);

    QPen pen(QColor("#9C27B0"));
    pen.setWidth(20);
    painter->setPen(pen);
    painter->drawPoint(start_station_.pos.x * METER_TO_CM, start_station_.pos.y * METER_TO_CM);
    painter->drawPoint(dst_station_.pos.x * METER_TO_CM, dst_station_.pos.y * METER_TO_CM);

    pen.setWidth(10);
    painter->setPen(pen);
    const int ARROW_LENGTH = 20;
    painter->drawLine(start_station_.pos.x * METER_TO_CM, start_station_.pos.y * METER_TO_CM,
                      start_station_.pos.x * METER_TO_CM + cos(start_station_.pos.yaw) * ARROW_LENGTH,
                      start_station_.pos.y * METER_TO_CM + sin(start_station_.pos.yaw) * ARROW_LENGTH);
    painter->drawLine(dst_station_.pos.x * METER_TO_CM, dst_station_.pos.y * METER_TO_CM,
                      dst_station_.pos.x * METER_TO_CM + cos(dst_station_.pos.yaw) * ARROW_LENGTH,
                      dst_station_.pos.y * METER_TO_CM + sin(dst_station_.pos.yaw) * ARROW_LENGTH);
}

GraphicPathsItem::GraphicPathsItem(const std::vector<sros::core::NavigationPath<double>> &paths) : paths_(paths) {
    m_boundingRect.setRect(-500, -500, 1000, 1000);
}

QRectF GraphicPathsItem::boundingRect() const { return m_boundingRect; }

void GraphicPathsItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) {
    Q_UNUSED(option);
    Q_UNUSED(widget);

    QPen pen(QColor("#F44336"));
    pen.setWidth(3);
    painter->setPen(pen);
    for (const auto &path : paths_) {
        switch (path.type_) {
            case PATH_LINE: {
                painter->drawLine(path.sx_ * METER_TO_CM, path.sy_ * METER_TO_CM, path.ex_ * METER_TO_CM,
                                  path.ey_ * METER_TO_CM);
                break;
            }
            case PATH_BEZIER: {
                QPainterPath p(QPointF(path.sx_ * METER_TO_CM, path.sy_ * METER_TO_CM));
                p.cubicTo(QPointF(path.cx_ * METER_TO_CM, path.cy_ * METER_TO_CM),
                          QPointF(path.dx_ * METER_TO_CM, path.dy_ * METER_TO_CM),
                          QPointF(path.ex_ * METER_TO_CM, path.ey_ * METER_TO_CM));
                painter->drawPath(p);
                break;
            }
            case PATH_ARC: {
                break;
            }
            default: {
                break;
            }
        }
    }
}

GraphicNodesItem::GraphicNodesItem(const std::vector<sros::map::net::Nodef> &nodes) : nodes_(nodes) {
    m_boundingRect.setRect(-500, -500, 1000, 1000);
}

QRectF GraphicNodesItem::boundingRect() const { return m_boundingRect; }

void GraphicNodesItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) {
    Q_UNUSED(option);
    Q_UNUSED(widget);

    QPen pen(QColor("#FF9800"));
    pen.setWidth(10);
    painter->setPen(pen);
    for (const auto &node : nodes_) {
        painter->drawPoint(QPointF(node.x, node.y));
    }
}

GraphicEdgesItem::GraphicEdgesItem(const std::vector<sros::map::net::Edgef> &edges) : edges_(edges) {
    m_boundingRect.setRect(-500, -500, 1000, 1000);
}

QRectF GraphicEdgesItem::boundingRect() const { return m_boundingRect; }

void GraphicEdgesItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) {
    Q_UNUSED(option);
    Q_UNUSED(widget);

    auto drawPathsFunc = [&](QPen pen) {
        painter->setPen(pen);
        for (const auto &edge : edges_) {
            switch (edge.type) {
                case EDGE_LINE: {
                    painter->drawLine(edge.sx * METER_TO_CM, edge.sy * METER_TO_CM, edge.ex * METER_TO_CM,
                                      edge.ey * METER_TO_CM);
                    break;
                }
                case EDGE_BEZIER: {
                    QPainterPath path(QPointF(edge.sx * METER_TO_CM, edge.sy * METER_TO_CM));
                    path.cubicTo(QPointF(edge.cx * METER_TO_CM, edge.cy * METER_TO_CM),
                                 QPointF(edge.dx * METER_TO_CM, edge.dy * METER_TO_CM),
                                 QPointF(edge.ex * METER_TO_CM, edge.ey * METER_TO_CM));
                    painter->drawPath(path);
                    break;
                }
                case EDGE_CIRCLE: {
                    break;
                }
                default: {
                    break;
                }
            }
        }
    };

    const int PATH_WIDTH = 5;

    // 画路径可吸附范围
    QColor color("#A5D6A7");
    color.setAlpha(100);
    QPen pen(color);
    pen.setWidth(PATH_WIDTH + nearest_edge_distance_threshold_ * METER_TO_CM);
    drawPathsFunc(pen);

    // 画路径
    pen.setColor(QColor("#4CAF50"));
    pen.setWidth(PATH_WIDTH);
    drawPathsFunc(pen);
}

GraphicsCoordinateAxisItem::GraphicsCoordinateAxisItem() { m_boundingRect.setRect(-500, -500, 1000, 1000); }

QRectF GraphicsCoordinateAxisItem::boundingRect() const { return m_boundingRect; }

void GraphicsCoordinateAxisItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) {
    Q_UNUSED(option);
    Q_UNUSED(widget);

    QPen pen(QColor("#EEEEEE"));
    painter->setPen(pen);
    painter->drawLine(m_boundingRect.x(), m_boundingRect.center().y(), m_boundingRect.right(),
                      m_boundingRect.center().y());
    painter->drawLine(m_boundingRect.center().x(), m_boundingRect.top(), m_boundingRect.center().x(),
                      m_boundingRect.bottom());
}

void GraphicsView::wheelEvent(QWheelEvent *e) {
    if (e->modifiers() & Qt::ControlModifier) {
        if (e->delta() > 0)
            scale(1.1, 1.1);
        else
            scale(0.9, 0.9);
        e->accept();
    } else {
        QGraphicsView::wheelEvent(e);
    }
}

void GraphicsView::mouseMoveEvent(QMouseEvent *event) {
    if (m_isMousePressed) {
        auto offset = m_lastPose - event->globalPos();
        m_lastPose = event->globalPos();

        double cx = viewport()->size().width() / 2.0;
        double cy = viewport()->size().height() / 2.0;

        centerOn(mapToScene(cx + offset.x(), cy + offset.y()));
    }

    QGraphicsView::mouseMoveEvent(event);
}

void GraphicsView::mousePressEvent(QMouseEvent *event) {
    m_isMousePressed = true;
    m_lastPose = event->globalPos();
}

void GraphicsView::mouseReleaseEvent(QMouseEvent *event) { m_isMousePressed = false; }

MainWindow::MainWindow(const std::vector<sros::map::net::Nodef> &nodes, const std::vector<sros::map::net::Edgef> &edges,
                       std::vector<sros::map::StationMark> &stations,
                       const std::vector<sros::core::NavigationPath<double>> &paths,
                       sros::map::StationMark start_station, sros::map::StationMark dst_station, QWidget *parent)
    : QMainWindow(parent), nodes_(nodes), edges_(edges), stations_(stations), paths_(paths) {
    m_pGraphicsScene = new GraphicsScene(this);
    GraphicsView *graphicsView;
    graphicsView = new GraphicsView(this);
    graphicsView->setMouseTracking(true);

    m_pStatusBar = new QStatusBar(this);
    setStatusBar(m_pStatusBar);

    setCentralWidget(graphicsView);

    graphicsView->setScene(m_pGraphicsScene);
    QTransform transform;  // 左手系转右手系
    transform.setMatrix(1, 0, 0, 0, -1, 0, 0, 0, 1);
    graphicsView->setTransform(transform);

    connect(m_pGraphicsScene, SIGNAL(cursorChanged(QPointF)), this, SLOT(onCursorChanged(QPointF)));

    m_pGraphicsScene->addItem(new GraphicsCoordinateAxisItem);
    m_pGraphicsScene->addItem(new GraphicNodesItem(nodes));
    m_pGraphicsScene->addItem(new GraphicEdgesItem(edges));
    m_pGraphicsScene->addItem(new GraphicStationsItem(start_station, dst_station));
    m_pGraphicsScene->addItem(new GraphicPathsItem(paths));
}
