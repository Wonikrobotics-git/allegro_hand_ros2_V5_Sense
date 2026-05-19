#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <sensor_msgs/msg/joint_state.hpp>
#include <QListWidget>
#include <QTimer>

class GainTuningDialog;
class DeviceConfigDialog;

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE


class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr, double uiScale = 1.0);
    ~MainWindow();

    void setNode(std::shared_ptr<rclcpp::Node> node);

private slots:

    void savePose(const std::string& pose_file);
    std::vector<double> readFinalJointStates();

    void ChangeButton();

    void TimeChanged(double arg1);

    void ForceValue(double arg1);
    
    void ForceApply();

    void HomeButton();

    void GraspButton();

    void GravityButton();

    void TorqueoffButton();

    void ClearlogButton();

    void ExitButton();

    void SliderPressed();

    void SliderReleased();

    void SliderValueChanged(int value);

    void SavefileButton();

    void LoadfileButton();

    void RefreshListButton();

    void on_listWidget_itemClicked(QListWidgetItem *item);

    void on_poseCountButton_clicked();

    void on_selectPoseButton_clicked();

    void on_startSequenceButton_clicked();

    void on_refreshButton_clicked();

    void executeSequence();

    void GainTuningPanelButton();

    void DeviceConfigPanelButton();

private:
    Ui::MainWindow *ui;

    int currentSequenceIndex;
    QTimer *sequenceTimer;
    QStringList selectedPoses;
    int repeatCount;
    int poseCount;

    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr time_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr force_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr joint_cmd_pub;
    bool sliderPressed_ = false;

    void listYamlFiles();

    static QString lastSelectedPose;
    bool selectionCompleteLogged;

    GainTuningDialog*    gainTuningDialog_;
    DeviceConfigDialog*  deviceConfigDialog_;
    double               uiScale_;
};
#endif // MAINWINDOW_H
