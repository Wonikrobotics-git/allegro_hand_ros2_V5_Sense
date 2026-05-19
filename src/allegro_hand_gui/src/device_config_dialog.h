#ifndef DEVICE_CONFIG_DIALOG_H
#define DEVICE_CONFIG_DIALOG_H

#include <QDialog>
#include <QLabel>
#include <QPushButton>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class DeviceConfigDialog : public QDialog
{
    Q_OBJECT

public:
    explicit DeviceConfigDialog(std::shared_ptr<rclcpp::Node> node, QWidget *parent = nullptr);
    ~DeviceConfigDialog() override = default;

private slots:
    void onCalibrateClicked();

private:
    void setupUi();

    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr lib_cmd_pub_;

    QLabel* statusLabel_;
};

#endif // DEVICE_CONFIG_DIALOG_H
