#include "device_config_dialog.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QLabel>
#include <QPushButton>
#include <QSpinBox>
#include <QFrame>
#include <QMessageBox>

DeviceConfigDialog::DeviceConfigDialog(std::shared_ptr<rclcpp::Node> node, QWidget *parent)
    : QDialog(parent), node_(node)
{
    lib_cmd_pub_ = node_->create_publisher<std_msgs::msg::String>(
        "/allegroHand/lib_cmd", 10);
    net_config_client_ = node_->create_client<allegro_hand_controllers::srv::SetNetConfig>(
        "allegroHand/set_net_config");

    setupUi();
}

void DeviceConfigDialog::setupUi()
{
    setWindowTitle("Device Configuration");
    setMinimumWidth(420);

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

    // ── Network Configuration Group ──────────────────────────
    QGroupBox* netGroup = new QGroupBox("Network Configuration (UDP)");
    QVBoxLayout* netLayout = new QVBoxLayout(netGroup);

    QLabel* netInfo = new QLabel(
        "<html><body><p style='color:#555;'>"
        "Hand IP: <b>192.168.</b> X . X &nbsp;&nbsp; Port: X<br/>"
        "Changes apply after hand reboot."
        "</p></body></html>");
    netInfo->setTextFormat(Qt::RichText);
    netLayout->addWidget(netInfo);

    // IP row: 192.168. [octet3] . [octet4]
    QHBoxLayout* ipRow = new QHBoxLayout();
    ipRow->addWidget(new QLabel("IP:"));
    ipRow->addWidget(new QLabel("192.168."));

    ipOctet3_ = new QSpinBox();
    ipOctet3_->setRange(0, 255);
    ipOctet3_->setValue(0);
    ipOctet3_->setFixedWidth(60);
    ipRow->addWidget(ipOctet3_);
    ipRow->addWidget(new QLabel("."));

    ipOctet4_ = new QSpinBox();
    ipOctet4_->setRange(0, 255);
    ipOctet4_->setValue(50);
    ipOctet4_->setFixedWidth(60);
    ipRow->addWidget(ipOctet4_);
    ipRow->addStretch();
    netLayout->addLayout(ipRow);

    // Port row
    QHBoxLayout* portRow = new QHBoxLayout();
    portRow->addWidget(new QLabel("Port:"));
    portSpin_ = new QSpinBox();
    portSpin_->setRange(1024, 65535);
    portSpin_->setValue(7000);
    portSpin_->setFixedWidth(80);
    portRow->addWidget(portSpin_);
    portRow->addStretch();
    netLayout->addLayout(portRow);

    QPushButton* netBtn = new QPushButton("Apply Network Config");
    netBtn->setMinimumHeight(36);
    netLayout->addWidget(netBtn);
    connect(netBtn, &QPushButton::clicked, this, &DeviceConfigDialog::onSetNetConfigClicked);

    mainLayout->addWidget(netGroup);

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

void DeviceConfigDialog::onSetNetConfigClicked()
{
    if (!net_config_client_->service_is_ready()) {
        statusLabel_->setStyleSheet("color: #a94442;");
        statusLabel_->setText("[Network] Service not available.\nIs the hand connected via UDP?");
        return;
    }

    QString ip = QString("192.168.%1.%2")
                     .arg(ipOctet3_->value())
                     .arg(ipOctet4_->value());

    auto request = std::make_shared<allegro_hand_controllers::srv::SetNetConfig::Request>();
    request->ip      = ip.toStdString();
    request->mask    = "255.255.255.0";
    request->gateway = QString("192.168.%1.1").arg(ipOctet3_->value()).toStdString();
    request->port    = static_cast<uint16_t>(portSpin_->value());

    statusLabel_->setStyleSheet("color: #888;");
    statusLabel_->setText("[Network] Sending...");

    net_config_client_->async_send_request(
        request,
        [this, ip](rclcpp::Client<allegro_hand_controllers::srv::SetNetConfig>::SharedFuture future) {
            if (!rclcpp::ok()) return;
            auto result = future.get();
            QMetaObject::invokeMethod(this, [this, result, ip]() {
                if (result->success) {
                    statusLabel_->setStyleSheet("color: green;");
                    statusLabel_->setText(
                        QString("[Network] Done. New IP: %1\nReboot hand to apply.").arg(ip));
                } else {
                    statusLabel_->setStyleSheet("color: #a94442;");
                    statusLabel_->setText(
                        QString("[Network] Failed: %1").arg(QString::fromStdString(result->message)));
                }
            }, Qt::QueuedConnection);
        });
}
