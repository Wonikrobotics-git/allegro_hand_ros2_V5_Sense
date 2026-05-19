#include "device_config_dialog.h"

#include <QVBoxLayout>
#include <QGroupBox>
#include <QLabel>
#include <QPushButton>
#include <QFrame>

DeviceConfigDialog::DeviceConfigDialog(std::shared_ptr<rclcpp::Node> node, QWidget *parent)
    : QDialog(parent), node_(node)
{
    lib_cmd_pub_ = node_->create_publisher<std_msgs::msg::String>(
        "/allegroHand/lib_cmd", 10);

    setupUi();
}

void DeviceConfigDialog::setupUi()
{
    setWindowTitle("Device Configuration");
    setMinimumWidth(400);

    QVBoxLayout* mainLayout = new QVBoxLayout(this);
    mainLayout->setSpacing(14);
    mainLayout->setContentsMargins(20, 20, 20, 20);

    QLabel* titleLabel = new QLabel(
        "<html><body><p><span style='font-size:13pt; font-weight:600;'>"
        "Device Configuration</span></p></body></html>");
    titleLabel->setAlignment(Qt::AlignCenter);
    mainLayout->addWidget(titleLabel);

    // ── Motor Calibration Group ──────────────────────────────
    QGroupBox* calGroup = new QGroupBox("Motor Calibration");
    QVBoxLayout* calLayout = new QVBoxLayout(calGroup);

    QLabel* calWarn = new QLabel(
        "<html><body>"
        "<p style='color:#a94442;'>"
        "<b>⚠ WARNING:</b> Calibration permanently redefines the firmware's angle origin.<br/>"
        "Ensure the hand is <b>torque off</b>, under <b>no load</b>, "
        "and in the <b>intended reference posture</b> before proceeding.<br/>"
        "<b>There is no undo.</b>"
        "</p></body></html>");
    calWarn->setWordWrap(true);
    calWarn->setTextFormat(Qt::RichText);
    calLayout->addWidget(calWarn);

    QPushButton* calBtn = new QPushButton("Calibrate");
    calBtn->setMinimumHeight(36);
    calBtn->setStyleSheet("QPushButton { font-weight: bold; color: #a94442; }");
    calLayout->addWidget(calBtn);

    connect(calBtn, &QPushButton::clicked, this, &DeviceConfigDialog::onCalibrateClicked);
    mainLayout->addWidget(calGroup);

    // ── Status Label ─────────────────────────────────────────
    statusLabel_ = new QLabel("");
    statusLabel_->setAlignment(Qt::AlignCenter);
    statusLabel_->setWordWrap(true);
    mainLayout->addWidget(statusLabel_);

    mainLayout->addStretch();
}

void DeviceConfigDialog::onCalibrateClicked()
{
    std_msgs::msg::String msg;
    msg.data = "calibration";
    lib_cmd_pub_->publish(msg);

    statusLabel_->setStyleSheet("color: green;");
    statusLabel_->setText("[Calibration] Command sent: 'calibration'");
    RCLCPP_INFO(node_->get_logger(), "DeviceConfig: sent lib_cmd 'calibration'");
}
