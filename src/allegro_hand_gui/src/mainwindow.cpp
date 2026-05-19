#include "mainwindow.h"
#include <chrono>
#include <thread>
#include "gain_tuning_dialog.h"
#include "device_config_dialog.h"
#include "./ui_mainwindow.h"
#include <QApplication>
#include <QMessageBox>
#include <QFileDialog>
#include <QDir>
#include <QFileInfoList>
#include <QListWidgetItem>

MainWindow::MainWindow(QWidget *parent, double uiScale)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , uiScale_(uiScale)
{
    ui->setupUi(this);

    ui->listWidget->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Expanding);


    sequenceTimer = new QTimer(this);
    connect(sequenceTimer, &QTimer::timeout, this, &MainWindow::executeSequence);

    connect(ui->listWidget, &QListWidget::itemClicked, this, &MainWindow::on_listWidget_itemClicked);

    /// Time Panel
    connect(ui->pushButton, &QPushButton::clicked, this, &MainWindow::ChangeButton);
    connect(ui->doubleSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MainWindow::TimeChanged);

    /// Grasping force Panel(Button)
    connect(ui->pushButton_2, &QPushButton::clicked, this, &MainWindow::ForceApply);
    connect(ui->doubleSpinBox_2, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MainWindow::ForceValue);

    /// Grasping force Panel(Slider)
    connect(ui->horizontalSlider, &QAbstractSlider::sliderReleased, this, &MainWindow::SliderReleased); 
    connect(ui->horizontalSlider, &QAbstractSlider::sliderPressed, this, &MainWindow::SliderPressed);
    connect(ui->horizontalSlider, &QAbstractSlider::valueChanged, this, &MainWindow::SliderValueChanged);

    /// Pre-defined Hand Pose Panel
    connect(ui->pushButton_3, &QPushButton::clicked, this, &MainWindow::HomeButton);
    connect(ui->pushButton_4, &QPushButton::clicked, this, &MainWindow::GraspButton);
    connect(ui->pushButton_5, &QPushButton::clicked, this, &MainWindow::GravityButton);
    connect(ui->pushButton_6, &QPushButton::clicked, this, &MainWindow::TorqueoffButton);

    /// Save pose Panel
    connect(ui->pushButton_12, &QPushButton::clicked, this, &MainWindow::SavefileButton);  


    ////Load pose Panel
    connect(ui->LoadButton, &QPushButton::clicked, this, &MainWindow::LoadfileButton);
    connect(ui->RefreshButton, &QPushButton::clicked, this, &MainWindow::RefreshListButton);        

    /// Else
    connect(ui->pushButton_10, &QPushButton::clicked, this, &MainWindow::ClearlogButton);
    connect(ui->pushButton_11, &QPushButton::clicked, this, &MainWindow::ExitButton);



    connect(ui->poseCountButton, &QPushButton::clicked, this, &MainWindow::on_poseCountButton_clicked);
    connect(ui->selectPoseButton, SIGNAL(clicked()), this, SLOT(on_selectPoseButton_clicked()));

    connect(ui->startSequenceButton, &QPushButton::clicked, this, &MainWindow::on_startSequenceButton_clicked);
    connect(ui->refreshButton, &QPushButton::clicked, this, &MainWindow::on_refreshButton_clicked);   

    /// Gain Tuning Panel (menu action)
    connect(ui->actionGainTuning, &QAction::triggered, this, &MainWindow::GainTuningPanelButton);
    gainTuningDialog_ = nullptr;

    /// Device Config Panel (menu action)
    connect(ui->actionDeviceConfig, &QAction::triggered, this, &MainWindow::DeviceConfigPanelButton);
    deviceConfigDialog_ = nullptr;

    ui->poseListWidget->hide();
    ui->selectPoseButton->hide();
    ui->repeatCountSpinBox->hide();
    ui->startSequenceButton->hide();
    ui->label_15->hide();

    currentSequenceIndex = 0;
    repeatCount = 0;
    poseCount = 0;
    selectionCompleteLogged = false;

    listYamlFiles();
}

MainWindow::~MainWindow()
{
    delete ui;
}

std::vector<double> MainWindow::readFinalJointStates()
{
    std::vector<double> positions;
    bool received = false;

    auto sub = node_->create_subscription<sensor_msgs::msg::JointState>(
        "allegroHand/joint_states", 1,
        [&positions, &received](const sensor_msgs::msg::JointState::SharedPtr msg) {
            if (!received && msg->position.size() >= 16) {
                positions.assign(msg->position.begin(), msg->position.begin() + 16);
                received = true;
            }
        });

    // Wait up to 2 seconds for a message
    auto start = std::chrono::steady_clock::now();
    while (!received) {
        rclcpp::spin_some(node_);
        auto elapsed = std::chrono::steady_clock::now() - start;
        if (std::chrono::duration_cast<std::chrono::seconds>(elapsed).count() >= 2) {
            ui->logTextEdit->append("Failed to receive joint_states within timeout");
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    sub.reset();
    return positions;
}

void MainWindow::setNode(std::shared_ptr<rclcpp::Node> node)
{
    node_ = node;

    time_pub_ = node_->create_publisher<std_msgs::msg::Float32>("timechange", 1);
    force_pub_ = node_->create_publisher<std_msgs::msg::Float32>("forcechange", 1);
    joint_cmd_pub = node_->create_publisher<std_msgs::msg::String>("allegroHand/lib_cmd", 10);
}

void MainWindow::savePose(const std::string& pose_file)
{
  std::vector<double> positions = readFinalJointStates();

  if (positions.size() < 16) {
    ui->logTextEdit->append("Failed to save pose: could not read 16 joint positions (timeout?)");
    return;
  }

  YAML::Emitter out;
  out << YAML::BeginMap;
  out << YAML::Key << "position" << YAML::Value << positions;
  out << YAML::EndMap;

  std::string pkg_path = ament_index_cpp::get_package_share_directory("allegro_hand_controllers");
  std::string file_path = pkg_path + "/pose/" + pose_file;

  std::ofstream fout(file_path);

    if (!fout) {
    QString logMessage = QString("Failed to open file for writing: %1").arg(QString::fromStdString(file_path));
    ui->logTextEdit->append(logMessage);
    return;
    }

  fout << out.c_str();
  fout.close();

  QString logMessage = QString("Pose saved to %1").arg(QString::fromStdString(pose_file));
  ui->logTextEdit->append(logMessage);
}

void MainWindow::SliderValueChanged(int value)
{
    double double_value = (double)(value/10.0);
    ui->doubleSpinBox_2->setValue(double_value);
    if(sliderPressed_){

        QString logMessage = QString("Slider value changed to %1").arg(double_value);
        ui->logTextEdit->append(logMessage);
    
        auto msg = std_msgs::msg::Float32();
        msg.data = double_value;
        force_pub_->publish(msg);
    }
    
}

void MainWindow::SliderPressed()
{
    sliderPressed_ = true;
}

void MainWindow::SliderReleased()
{
    sliderPressed_ = false;
}

void MainWindow::ChangeButton()
{
    double value = ui->doubleSpinBox->value();
    QString logMessage = QString("Time changed to %1").arg(value);

    ui->logTextEdit->append(logMessage);


    auto msg = std_msgs::msg::Float32();
    msg.data = value;
    time_pub_->publish(msg);
    
}

void MainWindow::TimeChanged(double arg1)
{
}

void MainWindow::ForceValue(double arg1)
{
    int sliderValue = static_cast<int>(arg1 * 10); // Assuming slider values range in integers
    ui->horizontalSlider->setValue(sliderValue);
}

void MainWindow::ForceApply()
{
    double value = ui->doubleSpinBox_2->value();
    QString logMessage = QString("Force Changed to %1").arg(value);

    ui->logTextEdit->append(logMessage);

        auto msg = std_msgs::msg::Float32();
        msg.data = value;
        force_pub_->publish(msg);
}


void MainWindow::HomeButton()
{

    std_msgs::msg::String msg;
    msg.data = "home";

    joint_cmd_pub->publish(msg);

    QString logMessage = QString("Home Position");
    ui->logTextEdit->append(logMessage);
}

void MainWindow::GraspButton()
{

    std_msgs::msg::String msg;
    msg.data = "grasp_4";

    joint_cmd_pub->publish(msg);

    QString logMessage = QString("Grasp");

    ui->logTextEdit->append(logMessage);

}

void MainWindow::GravityButton()
{

    std_msgs::msg::String msg;
    msg.data = "gravcomp";

    joint_cmd_pub->publish(msg);

    QString logMessage = QString("Gravity Compensation");

    ui->logTextEdit->append(logMessage);

}

void MainWindow::TorqueoffButton()
{

    std_msgs::msg::String msg;
    msg.data = "off";

    joint_cmd_pub->publish(msg);

    QString logMessage = QString("Torque off");

    ui->logTextEdit->append(logMessage);

}


void MainWindow::ClearlogButton()
{
    ui->logTextEdit->clear();
}

void MainWindow::SavefileButton()
{
    QString fileName = ui->savefilename->text();

    if (fileName.isEmpty()) {
        QMessageBox::information(this, tr("Input Error"), tr("Please enter a file name."));
        return;
    }

    savePose(fileName.toStdString() + ".yaml");

    listYamlFiles();

}

void MainWindow::LoadfileButton()
{
    QListWidgetItem *selectedItem = ui->listWidget->currentItem();

    if (selectedItem) {
        QString fileName = selectedItem->text();

        std_msgs::msg::String msg;
        msg.data = fileName.toStdString();

        joint_cmd_pub->publish(msg);

        QString logMessage = QString("Selected item: %1").arg(fileName);
        ui->logTextEdit->append(logMessage);
    } else {
        QString logMessage = QString("No item selected");
        ui->logTextEdit->append(logMessage);
    }
}


void MainWindow::RefreshListButton()
{
    listYamlFiles();
}

void MainWindow::on_listWidget_itemClicked(QListWidgetItem *item)
{
    QString fileName = item->text();

}


void MainWindow::listYamlFiles()
{
    ui->listWidget->clear();


    std::string pkg_path = ament_index_cpp::get_package_share_directory("allegro_hand_controllers");
    QString directoryPath = QString::fromStdString(pkg_path + "/pose/");
    QDir directory(directoryPath);


    QStringList yamlFiles = directory.entryList(QStringList() << "*.yaml", QDir::Files);
    foreach (QString filename, yamlFiles) {

        QString displayName = filename;
        displayName.chop(5); 

        QListWidgetItem *item = new QListWidgetItem(displayName);
        ui->listWidget->addItem(item);
    }
}

void MainWindow::on_poseCountButton_clicked()
{
    poseCount = ui->poseCountSpinBox->value();

    ui->poseListWidget->show();
    ui->selectPoseButton->show();

    ui->poseListWidget->clear();

    std::string pkg_path = ament_index_cpp::get_package_share_directory("allegro_hand_controllers");
    QString directoryPath = QString::fromStdString(pkg_path + "/pose/");
    QDir directory(directoryPath);

    QStringList yamlFiles = directory.entryList(QStringList() << "*.yaml", QDir::Files);
    foreach (QString filename, yamlFiles) {

        QString displayName = filename;
        displayName.chop(5);

        QListWidgetItem *item = new QListWidgetItem(displayName);
        ui->poseListWidget->addItem(item);
    }
}

void MainWindow::on_selectPoseButton_clicked()
{

    static QString lastSelectedPose;

    // Check if the current selection is less than the required pose count
    if (selectedPoses.size() < poseCount) {
        // Get the currently selected item from the list widget
        QListWidgetItem *selectedItem = ui->poseListWidget->currentItem();
        
        // Check if an item is selected
        if (selectedItem) {
            QString selectedPose = selectedItem->text(); // Get the text of the selected item

            // Avoid selecting the same pose consecutively
            if (!selectedPoses.isEmpty() && selectedPose == lastSelectedPose) {
            } else {
                // Add the selected pose to the list if it's not already selected
                    selectedPoses.append(selectedPose); // Add to the selected poses list
                    ui->logTextEdit->append("Selected pose: " + selectedPose);

                // Update the last selected pose
                lastSelectedPose = selectedPose;
            }
        }

    }

    // Check if the selection is complete
    if (selectedPoses.size() == poseCount && !selectionCompleteLogged) {
        ui->logTextEdit->append("Selection complete");
        selectionCompleteLogged = true;

        // Show additional UI elements and disable further selection
        ui->repeatCountSpinBox->show();
        ui->startSequenceButton->show();
        ui->refreshButton->show();
        ui->selectPoseButton->setEnabled(false); // Disable the select button
        ui->label_15->show();
    }
}

void MainWindow::on_startSequenceButton_clicked()
{
    double time = ui->doubleSpinBox->value();
    int handtime = (int)(time * 1000) + 200;
    repeatCount = ui->repeatCountSpinBox->value() - 1;
    currentSequenceIndex = 0;
    ui->logTextEdit->append("Starting sequence...");
    sequenceTimer->start(handtime);
}

void MainWindow::on_refreshButton_clicked()
{
    ui->poseCountSpinBox->setValue(1);
    ui->poseListWidget->hide();
    ui->poseListWidget->clear();
    ui->selectPoseButton->hide();
    ui->repeatCountSpinBox->hide();
    ui->repeatCountSpinBox->setValue(1);
    ui->startSequenceButton->hide();
    ui->refreshButton->show();
    selectedPoses.clear();
    ui->logTextEdit->clear();
    ui->label_15->hide();
    currentSequenceIndex = 0;
    repeatCount = 0;
    sequenceTimer->stop();
    ui->logTextEdit->append("Reset complete. Please select poses again.");
    ui->selectPoseButton->setEnabled(true);
    selectionCompleteLogged = false;
}

void MainWindow::executeSequence()
{
    if (currentSequenceIndex < selectedPoses.size()) {
        QString pose = selectedPoses[currentSequenceIndex];
        std_msgs::msg::String msg;
        msg.data = pose.toStdString();

        joint_cmd_pub->publish(msg);

        ui->logTextEdit->append("Executing pose: " + pose);
        ++currentSequenceIndex;
    } else {
        if (repeatCount > 0) {
            currentSequenceIndex = 0;
            --repeatCount;
        } else {
            sequenceTimer->stop();
            ui->logTextEdit->append("Sequence completed.");
        }
    }
}


void MainWindow::GainTuningPanelButton()
{
    if (!gainTuningDialog_) {
        gainTuningDialog_ = new GainTuningDialog(node_, this);
        // Make it a true independent window so it doesn't stay on top of MainWindow.
        gainTuningDialog_->setWindowFlags(Qt::Window);
    }
    gainTuningDialog_->show();
    gainTuningDialog_->raise();
    gainTuningDialog_->activateWindow();

    ui->logTextEdit->append("Gain Tuning Panel opened");
}

void MainWindow::DeviceConfigPanelButton()
{
    if (!deviceConfigDialog_) {
        deviceConfigDialog_ = new DeviceConfigDialog(node_, this);
        deviceConfigDialog_->setWindowFlags(Qt::Window);
    }
    deviceConfigDialog_->show();
    deviceConfigDialog_->raise();
    deviceConfigDialog_->activateWindow();

    ui->logTextEdit->append("Device Configuration Panel opened");
}

void MainWindow::ExitButton()
{
    TorqueoffButton();
    rclcpp::shutdown();
    QApplication::quit();

}