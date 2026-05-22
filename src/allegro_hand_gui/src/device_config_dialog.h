#ifndef DEVICE_CONFIG_DIALOG_H
#define DEVICE_CONFIG_DIALOG_H

#include <QDialog>
#include <QLabel>
#include <QPushButton>
#include <QSpinBox>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "allegro_hand_controllers/srv/set_net_config.hpp"

class DeviceConfigDialog : public QDialog
{
    Q_OBJECT

public:
    explicit DeviceConfigDialog(std::shared_ptr<rclcpp::Node> node, QWidget *parent = nullptr);
    ~DeviceConfigDialog() override = default;

private slots:
    void onCalibrateClicked();
    void onSetNetConfigClicked();

private:
    void setupUi();

    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr lib_cmd_pub_;
    rclcpp::Client<allegro_hand_controllers::srv::SetNetConfig>::SharedPtr net_config_client_;

    QSpinBox* ipOctet3_;   // 3rd octet (192.168.X.x)
    QSpinBox* ipOctet4_;   // 4th octet (192.168.x.X)
    QSpinBox* portSpin_;
    QLabel*   statusLabel_;
};

#endif // DEVICE_CONFIG_DIALOG_H
