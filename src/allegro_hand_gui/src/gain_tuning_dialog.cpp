#include "gain_tuning_dialog.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QLabel>
#include <QPushButton>
#include <QScrollArea>
#include <QMessageBox>
#include <QFrame>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>

static const char* fingerNames[] = {"Index", "Middle", "Pinky", "Thumb"};

static const char* jointDescriptions[] = {
    "J0   (Index root)",  "J1   (Index)",       "J2   (Index)",       "J3   (Index tip)",
    "J4   (Middle root)", "J5   (Middle)",      "J6   (Middle)",      "J7   (Middle tip)",
    "J8   (Pinky root)",  "J9   (Pinky)",       "J10  (Pinky)",       "J11  (Pinky tip)",
    "J12  (Thumb root)",  "J13  (Thumb)",       "J14  (Thumb)",       "J15  (Thumb tip)"
};

static const double defaultKp[] = {
    0.8, 0.8, 0.6, 0.5,
    0.8, 0.8, 0.6, 0.5,
    0.8, 0.8, 0.6, 0.5,
    0.6, 0.6, 0.6, 0.4
};

static const double defaultKd[] = {
    0.05, 0.05, 0.04, 0.04,
    0.05, 0.05, 0.04, 0.04,
    0.05, 0.05, 0.04, 0.04,
    0.04, 0.04, 0.04, 0.03
};

GainTuningDialog::GainTuningDialog(std::shared_ptr<rclcpp::Node> node, QWidget *parent)
    : QDialog(parent), node_(node),
      posDirty_(false), gainDirty_(false), initialPositionLoaded_(false)
{
    joint_cmd_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
        "allegroHand/joint_cmd", 10);
    gain_cmd_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
        "allegroHand/gain_cmd", 10);

    posPublishTimer_ = new QTimer(this);
    posPublishTimer_->setInterval(30);
    connect(posPublishTimer_, &QTimer::timeout, this, &GainTuningDialog::posThrottleTick);

    gainPublishTimer_ = new QTimer(this);
    gainPublishTimer_->setInterval(30);
    connect(gainPublishTimer_, &QTimer::timeout, this, &GainTuningDialog::gainThrottleTick);

    setupUi();
    loadFromYaml();

    joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "allegroHand/joint_states", 1,
        std::bind(&GainTuningDialog::jointStateCallback, this, std::placeholders::_1));
}

GainTuningDialog::~GainTuningDialog()
{
    joint_state_sub_.reset();
}

void GainTuningDialog::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    if (initialPositionLoaded_)
        return;

    if ((int)msg->position.size() >= NUM_JOINTS) {
        for (int i = 0; i < NUM_JOINTS; i++) {
            posSliders_[i]->blockSignals(true);
            posSpinBoxes_[i]->blockSignals(true);
            int val = static_cast<int>(msg->position[i] * POS_SCALE);
            posSliders_[i]->setValue(val);
            posSpinBoxes_[i]->setValue(msg->position[i]);
            posSliders_[i]->blockSignals(false);
            posSpinBoxes_[i]->blockSignals(false);
        }
        initialPositionLoaded_ = true;
        RCLCPP_INFO(node_->get_logger(), "Gain panel: loaded current joint positions");
    }
}

QGroupBox* GainTuningDialog::createGainSliderGroup(
    const QString& title,
    QSlider* sliders[], QDoubleSpinBox* spinBoxes[],
    double minVal, double maxVal,
    int scale, int decimals, double step,
    const double defaultVals[])
{
    QGroupBox* group = new QGroupBox(title);
    QHBoxLayout* groupLayout = new QHBoxLayout(group);

    for (int f = 0; f < 4; f++) {
        QGroupBox* fingerBox = new QGroupBox(fingerNames[f]);
        QHBoxLayout* fingerLayout = new QHBoxLayout(fingerBox);

        for (int j = 0; j < 4; j++) {
            int idx = f * 4 + j;

            QVBoxLayout* colLayout = new QVBoxLayout();
            colLayout->setAlignment(Qt::AlignCenter);

            QLabel* jointLabel = new QLabel(QString("J%1").arg(idx));
            jointLabel->setAlignment(Qt::AlignCenter);
            QFont boldFont = jointLabel->font();
            boldFont.setBold(true);
            jointLabel->setFont(boldFont);
            colLayout->addWidget(jointLabel);

            sliders[idx] = new QSlider(Qt::Vertical);
            sliders[idx]->setRange(static_cast<int>(minVal * scale),
                                   static_cast<int>(maxVal * scale));
            sliders[idx]->setValue(static_cast<int>(defaultVals[idx] * scale));
            sliders[idx]->setSingleStep(1);
            sliders[idx]->setPageStep(10);
            sliders[idx]->setMinimumHeight(120);
            sliders[idx]->setTickPosition(QSlider::TicksBothSides);
            sliders[idx]->setTickInterval(static_cast<int>((maxVal - minVal) * scale / 10));
            colLayout->addWidget(sliders[idx], 1, Qt::AlignHCenter);

            spinBoxes[idx] = new QDoubleSpinBox();
            spinBoxes[idx]->setRange(minVal, maxVal);
            spinBoxes[idx]->setDecimals(decimals);
            spinBoxes[idx]->setSingleStep(step);
            spinBoxes[idx]->setValue(defaultVals[idx]);
            spinBoxes[idx]->setFixedWidth(65);
            spinBoxes[idx]->setAlignment(Qt::AlignCenter);
            colLayout->addWidget(spinBoxes[idx]);

            fingerLayout->addLayout(colLayout);

            QDoubleSpinBox* sb = spinBoxes[idx];
            QSlider* sl = sliders[idx];

            connect(sl, &QSlider::valueChanged, this,
                [this, sb, scale](int value) {
                    double dval = value / static_cast<double>(scale);
                    sb->blockSignals(true);
                    sb->setValue(dval);
                    sb->blockSignals(false);
                    onGainChanged();
                });

            connect(sb, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
                [this, sl, scale](double value) {
                    sl->blockSignals(true);
                    sl->setValue(static_cast<int>(value * scale));
                    sl->blockSignals(false);
                    onGainChanged();
                });
        }

        groupLayout->addWidget(fingerBox);
    }

    return group;
}

QWidget* GainTuningDialog::createPositionPanel()
{
    QGroupBox* group = new QGroupBox("Joint Position (rad)");
    QVBoxLayout* layout = new QVBoxLayout(group);

    for (int i = 0; i < NUM_JOINTS; i++) {
        QHBoxLayout* row = new QHBoxLayout();

        QLabel* nameLabel = new QLabel(jointDescriptions[i]);
        nameLabel->setFixedWidth(130);
        QFont boldFont = nameLabel->font();
        boldFont.setBold(true);
        nameLabel->setFont(boldFont);
        row->addWidget(nameLabel);

        posSliders_[i] = new QSlider(Qt::Horizontal);
        posSliders_[i]->setRange(-50, 350);
        posSliders_[i]->setValue(0);
        posSliders_[i]->setSingleStep(1);
        posSliders_[i]->setPageStep(10);
        posSliders_[i]->setMinimumWidth(300);
        posSliders_[i]->setTickPosition(QSlider::TicksBelow);
        posSliders_[i]->setTickInterval(50);
        row->addWidget(posSliders_[i], 1);

        posSpinBoxes_[i] = new QDoubleSpinBox();
        posSpinBoxes_[i]->setRange(-0.50, 3.50);
        posSpinBoxes_[i]->setDecimals(2);
        posSpinBoxes_[i]->setSingleStep(0.01);
        posSpinBoxes_[i]->setValue(0.0);
        posSpinBoxes_[i]->setFixedWidth(75);
        posSpinBoxes_[i]->setAlignment(Qt::AlignCenter);
        row->addWidget(posSpinBoxes_[i]);

        layout->addLayout(row);

        QSlider* sl = posSliders_[i];
        QDoubleSpinBox* sb = posSpinBoxes_[i];

        connect(sl, &QSlider::valueChanged, this,
            [this, sb](int value) {
                double dval = value / static_cast<double>(POS_SCALE);
                sb->blockSignals(true);
                sb->setValue(dval);
                sb->blockSignals(false);
                onPositionChanged();
            });

        connect(sb, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
            [this, sl](double value) {
                sl->blockSignals(true);
                sl->setValue(static_cast<int>(value * POS_SCALE));
                sl->blockSignals(false);
                onPositionChanged();
            });

        if (i == 3 || i == 7 || i == 11) {
            QFrame* sep = new QFrame();
            sep->setFrameShape(QFrame::HLine);
            sep->setFrameShadow(QFrame::Sunken);
            layout->addWidget(sep);
        }
    }

    return group;
}

void GainTuningDialog::setupUi()
{
    setWindowTitle("Gain Tuning Panel");
    resize(1900, 800);
    setMinimumSize(1600, 700);

    QVBoxLayout* outerLayout = new QVBoxLayout(this);

    QLabel* titleLabel = new QLabel(
        "<html><body><p><span style='font-size:14pt; font-weight:600;'>"
        "Joint Gain Tuning Panel</span></p></body></html>");
    titleLabel->setAlignment(Qt::AlignCenter);
    outerLayout->addWidget(titleLabel);

    QHBoxLayout* buttonLayout = new QHBoxLayout();
    QPushButton* loadBtn  = new QPushButton("Load YAML");
    QPushButton* saveBtn  = new QPushButton("Save YAML");
    QPushButton* applyBtn = new QPushButton("Apply All");
    loadBtn->setMinimumHeight(32);
    saveBtn->setMinimumHeight(32);
    applyBtn->setMinimumHeight(32);

    buttonLayout->addWidget(loadBtn);
    buttonLayout->addWidget(saveBtn);
    buttonLayout->addStretch();
    buttonLayout->addWidget(applyBtn);
    outerLayout->addLayout(buttonLayout);

    connect(loadBtn,  &QPushButton::clicked, this, &GainTuningDialog::loadFromYaml);
    connect(saveBtn,  &QPushButton::clicked, this, &GainTuningDialog::saveToYaml);
    connect(applyBtn, &QPushButton::clicked, this, &GainTuningDialog::publishAll);

    QHBoxLayout* contentLayout = new QHBoxLayout();

    // Left: Kp (top) + Kd (bottom) vertical sliders
    QVBoxLayout* gainLayout = new QVBoxLayout();
    gainLayout->addWidget(
        createGainSliderGroup("Kp Gain", kpSliders_, kpSpinBoxes_,
                              0.0, 5.0, KP_SCALE, 2, 0.01, defaultKp));
    gainLayout->addWidget(
        createGainSliderGroup("Kd Gain", kdSliders_, kdSpinBoxes_,
                              0.0, 1.0, KD_SCALE, 3, 0.001, defaultKd));
    contentLayout->addLayout(gainLayout, 4);

    // Right: Position horizontal sliders
    QScrollArea* posScroll = new QScrollArea();
    posScroll->setWidgetResizable(true);
    posScroll->setWidget(createPositionPanel());
    contentLayout->addWidget(posScroll, 6);

    outerLayout->addLayout(contentLayout, 1);
}

void GainTuningDialog::onPositionChanged()
{
    posDirty_ = true;
    if (!posPublishTimer_->isActive()) {
        publishPositions();
        posDirty_ = false;
        posPublishTimer_->start();
    }
}

void GainTuningDialog::onGainChanged()
{
    gainDirty_ = true;
    if (!gainPublishTimer_->isActive()) {
        publishGains();
        gainDirty_ = false;
        gainPublishTimer_->start();
    }
}

void GainTuningDialog::posThrottleTick()
{
    if (posDirty_) {
        publishPositions();
        posDirty_ = false;
    } else {
        posPublishTimer_->stop();
    }
}

void GainTuningDialog::gainThrottleTick()
{
    if (gainDirty_) {
        publishGains();
        gainDirty_ = false;
    } else {
        gainPublishTimer_->stop();
    }
}

void GainTuningDialog::publishPositions()
{
    auto pos_msg = sensor_msgs::msg::JointState();
    pos_msg.header.stamp = node_->get_clock()->now();
    pos_msg.position.resize(NUM_JOINTS);
    for (int i = 0; i < NUM_JOINTS; i++) {
        pos_msg.position[i] = posSliders_[i]->value() / static_cast<double>(POS_SCALE);
    }
    joint_cmd_pub_->publish(pos_msg);
}

void GainTuningDialog::publishGains()
{
    auto gain_msg = std_msgs::msg::Float64MultiArray();
    gain_msg.data.resize(2 * NUM_JOINTS);
    for (int i = 0; i < NUM_JOINTS; i++) {
        gain_msg.data[i]              = kpSliders_[i]->value() / static_cast<double>(KP_SCALE);
        gain_msg.data[NUM_JOINTS + i] = kdSliders_[i]->value() / static_cast<double>(KD_SCALE);
    }
    gain_cmd_pub_->publish(gain_msg);
}

void GainTuningDialog::publishAll()
{
    publishGains();
    publishPositions();
}

static std::string getGainFilePath() {
    return ament_index_cpp::get_package_share_directory("allegro_hand_controllers")
           + "/config/gain.yaml";
}

void GainTuningDialog::loadFromYaml()
{
    try {
        std::string gain_file = getGainFilePath();

        std::ifstream infile(gain_file);
        if (!infile.good()) {
            RCLCPP_WARN(node_->get_logger(), "Gain YAML not found: %s", gain_file.c_str());
            return;
        }

        YAML::Node node = YAML::LoadFile(gain_file);

        if (node["kp"]) {
            auto kp_vals = node["kp"].as<std::vector<double>>();
            for (int i = 0; i < NUM_JOINTS && i < (int)kp_vals.size(); i++) {
                kpSliders_[i]->blockSignals(true);
                kpSpinBoxes_[i]->blockSignals(true);
                kpSliders_[i]->setValue(static_cast<int>(kp_vals[i] * KP_SCALE));
                kpSpinBoxes_[i]->setValue(kp_vals[i]);
                kpSliders_[i]->blockSignals(false);
                kpSpinBoxes_[i]->blockSignals(false);
            }
        }

        if (node["kd"]) {
            auto kd_vals = node["kd"].as<std::vector<double>>();
            for (int i = 0; i < NUM_JOINTS && i < (int)kd_vals.size(); i++) {
                kdSliders_[i]->blockSignals(true);
                kdSpinBoxes_[i]->blockSignals(true);
                kdSliders_[i]->setValue(static_cast<int>(kd_vals[i] * KD_SCALE));
                kdSpinBoxes_[i]->setValue(kd_vals[i]);
                kdSliders_[i]->blockSignals(false);
                kdSpinBoxes_[i]->blockSignals(false);
            }
        }

        RCLCPP_INFO(node_->get_logger(), "Gains loaded from %s", gain_file.c_str());
    } catch (const std::exception& e) {
        RCLCPP_WARN(node_->get_logger(), "Failed to load gain YAML: %s", e.what());
    }
}

void GainTuningDialog::saveToYaml()
{
    try {
        std::string gain_file = getGainFilePath();

        YAML::Emitter out;
        out << YAML::BeginMap;

        out << YAML::Key << "kp" << YAML::Value;
        out << YAML::BeginSeq;
        for (int i = 0; i < NUM_JOINTS; i++)
            out << kpSpinBoxes_[i]->value();
        out << YAML::EndSeq;

        out << YAML::Key << "kd" << YAML::Value;
        out << YAML::BeginSeq;
        for (int i = 0; i < NUM_JOINTS; i++)
            out << kdSpinBoxes_[i]->value();
        out << YAML::EndSeq;

        out << YAML::EndMap;

        std::ofstream fout(gain_file);
        if (!fout) {
            QMessageBox::warning(this, "Error",
                QString("Failed to open file for writing:\n%1")
                    .arg(QString::fromStdString(gain_file)));
            return;
        }
        fout << out.c_str();
        fout.close();

        QMessageBox::information(this, "Success", "Gains saved to YAML.");
    } catch (const std::exception& e) {
        QMessageBox::warning(this, "Error",
            QString("Failed to save YAML: %1").arg(e.what()));
    }
}
