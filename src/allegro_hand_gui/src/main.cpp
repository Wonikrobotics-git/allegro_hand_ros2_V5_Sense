#include "mainwindow.h"

#include <QApplication>
#include <QTimer>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // High DPI: 디스플레이 배율(100%/200%)에 맞춰 UI 스케일. QApplication 생성 전에 설정.
    // 200%에서도 깨지면: 실행 시 QT_SCALE_FACTOR=1 로 100% 고정하거나, 아래를 AA_DisableHighDpiScaling 로 바꿀 수 있음.
    QApplication::setAttribute(Qt::AA_EnableHighDpiScaling, true);
    QApplication::setAttribute(Qt::AA_UseHighDpiPixmaps, true);

    QApplication a(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("allegro_hand_gui_node");
    MainWindow w(nullptr);
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
