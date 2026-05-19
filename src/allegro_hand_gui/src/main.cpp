#include "mainwindow.h"

#include <QApplication>
#include <QTimer>
#include <QScreen>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // High DPI: 디스플레이 배율(100%/200%)에 맞춰 UI 스케일. QApplication 생성 전에 설정.
    QApplication::setAttribute(Qt::AA_EnableHighDpiScaling, true);
    QApplication::setAttribute(Qt::AA_UseHighDpiPixmaps, true);

    QApplication a(argc, argv);

    // 기준 해상도(1920×1080) 대비 현재 화면 비율을 계산해 전역으로 노출.
    // 각 다이얼로그는 이 값으로 고정 px 값을 스케일한다.
    const QSize screenSize = a.primaryScreen()->size();
    const double scaleX = screenSize.width()  / 1920.0;
    const double scaleY = screenSize.height() / 1080.0;
    const double uiScale = qMin(scaleX, scaleY);   // 비율 유지: 작은 쪽 기준

    auto node = std::make_shared<rclcpp::Node>("allegro_hand_gui_node");
    MainWindow w(nullptr, uiScale);
    w.setNode(node);
    w.show();

    // Periodically spin ROS2 so Ctrl+C (SIGINT) is processed and rclcpp::ok() becomes false
    QTimer ros_timer(nullptr);
    QObject::connect(&ros_timer, &QTimer::timeout, [&a, node]() {
        rclcpp::spin_some(node);
        if (!rclcpp::ok()) {
            a.quit();
        }
    });
    ros_timer.start(100);

    int result = a.exec();

    rclcpp::shutdown();
    return result;
}
