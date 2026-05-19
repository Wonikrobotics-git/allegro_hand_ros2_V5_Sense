#ifndef GAIN_TUNING_DIALOG_H
#define GAIN_TUNING_DIALOG_H

#include <QDialog>
#include <QSlider>
#include <QDoubleSpinBox>
#include <QTimer>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class QGroupBox;

class GainTuningDialog : public QDialog
{
    Q_OBJECT

public:
    explicit GainTuningDialog(std::shared_ptr<rclcpp::Node> node, QWidget *parent = nullptr);
    ~GainTuningDialog() override;

private slots:
    void onPositionChanged();
    void onGainChanged();
    void posThrottleTick();
    void gainThrottleTick();
    void publishPositions();
    void publishGains();
    void publishAll();
    void loadFromYaml();
    void saveToYaml();

private:
    void setupUi();
    QGroupBox* createGainSliderGroup(const QString& title,
                                     QSlider* sliders[], QDoubleSpinBox* spinBoxes[],
                                     double minVal, double maxVal,
                                     int scale, int decimals, double step,
                                     const double defaultVals[]);
    QWidget* createPositionPanel();
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

    static constexpr int NUM_JOINTS = 16;
    static constexpr int POS_SCALE  = 100;
    static constexpr int KP_SCALE   = 100;
    static constexpr int KD_SCALE   = 1000;

    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gain_cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

    QSlider* posSliders_[NUM_JOINTS];
    QSlider* kpSliders_[NUM_JOINTS];
    QSlider* kdSliders_[NUM_JOINTS];

    QDoubleSpinBox* posSpinBoxes_[NUM_JOINTS];
    QDoubleSpinBox* kpSpinBoxes_[NUM_JOINTS];
    QDoubleSpinBox* kdSpinBoxes_[NUM_JOINTS];

    QTimer* posPublishTimer_;
    QTimer* gainPublishTimer_;
    bool posDirty_;
    bool gainDirty_;

    bool initialPositionLoaded_;
};

#endif // GAIN_TUNING_DIALOG_H
