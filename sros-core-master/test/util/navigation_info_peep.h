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

#ifndef SROS_NAVIGATION_INFO_PEEP_H
#define SROS_NAVIGATION_INFO_PEEP_H

#include <QApplication>
#include <QGraphicsRectItem>
#include <QGraphicsScene>
#include <QGraphicsSceneMouseEvent>
#include <QGraphicsView>
#include <QMainWindow>
#include <QMouseEvent>
#include <QStatusBar>
#include <QWheelEvent>
#include <QWidget>
#include <thread>
#include "core/map_manager.h"
#include "modules/navigation/navigation_on_net.h"

class GraphicStationsItem : public QGraphicsItem {
 public:
    GraphicStationsItem(sros::map::StationMark start_station, sros::map::StationMark dst_station);

    QRectF boundingRect() const;

    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);

 private:
    QRectF m_boundingRect;
    sros::map::StationMark start_station_;
    sros::map::StationMark dst_station_;
};

class GraphicPathsItem : public QGraphicsItem {
 public:
    GraphicPathsItem(const std::vector<sros::core::NavigationPath<double>> &paths);

    QRectF boundingRect() const;

    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);

 private:
    QRectF m_boundingRect;
    const std::vector<sros::core::NavigationPath<double>> &paths_;
};

class GraphicNodesItem : public QGraphicsItem {
 public:
    GraphicNodesItem(const std::vector<sros::map::net::Nodef> &nodes);

    QRectF boundingRect() const;

    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);

 private:
    QRectF m_boundingRect;
    const std::vector<sros::map::net::Nodef> &nodes_;
};

class GraphicEdgesItem : public QGraphicsItem {
 public:
    GraphicEdgesItem(const std::vector<sros::map::net::Edgef> &edges);

    QRectF boundingRect() const;

    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);

 private:
    QRectF m_boundingRect;
    const std::vector<sros::map::net::Edgef> &edges_;
    double nearest_edge_distance_threshold_ = 0.16;  // 路网导航判断最近边时允许距离的最大值,单位m
};

class GraphicsCoordinateAxisItem : public QGraphicsItem {
 public:
    GraphicsCoordinateAxisItem();

    QRectF boundingRect() const;

    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);

 private:
    QRectF m_boundingRect;
};

class GraphicsScene : public QGraphicsScene {
    Q_OBJECT
 public:
    explicit GraphicsScene(QObject *parent = nullptr){};

 signals:
    void cursorChanged(const QPointF &scenePos);

 protected:
    void mouseMoveEvent(QGraphicsSceneMouseEvent *mouseEvent) override {
        emit cursorChanged(mouseEvent->scenePos());

        QGraphicsScene::mouseMoveEvent(mouseEvent);
    }

 private:
};

class GraphicsView : public QGraphicsView {
    Q_OBJECT
 public:
    GraphicsView(QWidget *parent = Q_NULLPTR) : QGraphicsView(parent) {}

 protected:
    void wheelEvent(QWheelEvent *e) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;

 private:
    bool m_isMousePressed = false;
    QPointF m_lastPose;
    QPointF m_center;
};

class MainWindow : public QMainWindow {
    Q_OBJECT
 public:
    MainWindow(const std::vector<sros::map::net::Nodef> &nodes, const std::vector<sros::map::net::Edgef> &edges,
               std::vector<sros::map::StationMark> &stations,
               const std::vector<sros::core::NavigationPath<double>> &paths, sros::map::StationMark start_station,
               sros::map::StationMark dst_station, QWidget *parent = nullptr);
    ~MainWindow() {}

    GraphicsScene *m_pGraphicsScene = nullptr;

    const std::vector<sros::map::StationMark> &stations_;
    const std::vector<sros::map::net::Edgef> &edges_;
    const std::vector<sros::map::net::Nodef> &nodes_;
    const std::vector<sros::core::NavigationPath<double>> &paths_;

 private slots:
    void onCursorChanged(const QPointF &scenePos) {
        m_pStatusBar->showMessage("scenePos : (" + QString::number(scenePos.x()) + "cm, " +
                                  QString::number(scenePos.y()) + "cm)");
    }

 private:
    QStatusBar *m_pStatusBar = nullptr;
};

class NavigationInfoPeep {
 public:
    void drawNodes(const std::vector<sros::map::net::Nodef> &nodes) { nodes_ = nodes; }
    void drawEdges(const std::vector<sros::map::net::Edgef> &edges) { edges_ = edges; }
    void drawStations(sros::map::StationMark start_station, sros::map::StationMark dst_station) {
        start_station_ = start_station;
        dst_station_ = dst_station;
    }
    void drawPaths(const std::vector<sros::core::NavigationPath<double>> &paths) { paths_ = paths; }

    void show() {
        std::thread([&]() {
            char *name = "NavigationOnNetTest";
            int arg_num = 1;
            QApplication a(arg_num, &name);
            MainWindow window(nodes_, edges_, stations_, paths_, start_station_, dst_station_);
            window.showMaximized();
            return a.exec();
        }).join();
    }

 private:
    double nearest_edge_distance_threshold_ = 0.16;  // 路网导航判断最近边时允许距离的最大值,单位m
    sros::map::StationMark start_station_;
    sros::map::StationMark dst_station_;
    std::vector<sros::map::StationMark> stations_;
    std::vector<sros::map::net::Edgef> edges_;
    std::vector<sros::map::net::Nodef> nodes_;
    std::vector<sros::core::NavigationPath<double>> paths_;
};

#endif  // SROS_NAVIGATION_INFO_PEEP_H
