#include <QApplication>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QWidget>
#include <QLabel>
#include <QTimer>
#include <QScreen>

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <string>

float fingertip_sensor[4] = {0, 0, 0, 0};
float madi_sensor[11] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
float palm_sensor_val = 0;

QLabel* sensor1_label;
QLabel* madi_sensor1_1_label;
QLabel* madi_sensor1_2_label;
QLabel* madi_sensor1_3_label;

QLabel* sensor2_label;
QLabel* madi_sensor2_1_label;
QLabel* madi_sensor2_2_label;
QLabel* madi_sensor2_3_label;

QLabel* sensor3_label;
QLabel* madi_sensor3_1_label;
QLabel* madi_sensor3_2_label;
QLabel* madi_sensor3_3_label;

QLabel* sensor4_label;
QLabel* madi_sensor4_1_label;
QLabel* madi_sensor4_2_label;
QLabel* palm_sensor_label;

rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub;
rclcpp::Node::SharedPtr g_node;

std::string whichHand = "right";

void publishTextMarker(const std::string& text, const std::string& frame_id, int marker_id) {
    auto marker = visualization_msgs::msg::Marker();
    marker.header.frame_id = frame_id;
    marker.header.stamp = g_node->get_clock()->now();
    marker.ns = "finger_labels";
    marker.id = marker_id;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.03;
    marker.pose.orientation.w = 1.0;
    marker.text = text;
    marker.scale.z = 0.02;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker_pub->publish(marker);
}

void updateMarkers() {
    publishTextMarker("1", "link_15_0_tip", 0);
    publishTextMarker("2", "link_3_0_tip", 1);
    publishTextMarker("3", "link_7_0_tip", 2);
    publishTextMarker("4", "link_11_0_tip", 3);
}

void tactileSensorCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    if (msg->data.size() < 16) return;

    palm_sensor_val     = msg->data[0];
    fingertip_sensor[0] = msg->data[3];   // thumb
    fingertip_sensor[1] = msg->data[7];   // index
    fingertip_sensor[2] = msg->data[11];  // middle
    fingertip_sensor[3] = msg->data[15];  // ring

    madi_sensor[0]  = msg->data[1];
    madi_sensor[1]  = msg->data[2];
    madi_sensor[2]  = msg->data[4];
    madi_sensor[3]  = msg->data[5];
    madi_sensor[4]  = msg->data[6];
    madi_sensor[5]  = msg->data[8];
    madi_sensor[6]  = msg->data[9];
    madi_sensor[7]  = msg->data[10];
    madi_sensor[8]  = msg->data[12];
    madi_sensor[9]  = msg->data[13];
    madi_sensor[10] = msg->data[14];

    auto fmtTip = [](float v) -> QString {
        return QString("<div style='text-align:center;'>"
                       "<span style='font-size:36pt; font-weight:bold;'>%1</span><br>"
                       "<span style='font-size:9pt;'>kPa</span></div>")
               .arg(QString::number(v, 'f', 1));
    };
    auto fmtMadi = [](float v) -> QString {
        return QString("<div style='text-align:center;'>"
                       "<span style='font-size:28pt; font-weight:bold;'>%1</span><br>"
                       "<span style='font-size:8pt;'>kPa</span></div>")
               .arg(QString::number(v, 'f', 1));
    };
    auto fmtPalm = [](float v) -> QString {
        return QString("<div style='text-align:center;'>"
                       "<span style='font-size:44pt; font-weight:bold;'>%1</span><br>"
                       "<span style='font-size:9pt;'>kPa</span></div>")
               .arg(QString::number(v, 'f', 1));
    };

    if (whichHand == "right") {
        sensor1_label->setText(fmtTip(fingertip_sensor[1]));
        sensor2_label->setText(fmtTip(fingertip_sensor[2]));
        sensor3_label->setText(fmtTip(fingertip_sensor[3]));
        sensor4_label->setText(fmtTip(fingertip_sensor[0]));
        madi_sensor1_1_label->setText(fmtMadi(madi_sensor[4]));
        madi_sensor1_2_label->setText(fmtMadi(madi_sensor[3]));
        madi_sensor1_3_label->setText(fmtMadi(madi_sensor[2]));
        madi_sensor2_1_label->setText(fmtMadi(madi_sensor[7]));
        madi_sensor2_2_label->setText(fmtMadi(madi_sensor[6]));
        madi_sensor2_3_label->setText(fmtMadi(madi_sensor[5]));
        madi_sensor3_1_label->setText(fmtMadi(madi_sensor[10]));
        madi_sensor3_2_label->setText(fmtMadi(madi_sensor[9]));
        madi_sensor3_3_label->setText(fmtMadi(madi_sensor[8]));
        madi_sensor4_1_label->setText(fmtMadi(madi_sensor[1]));
        madi_sensor4_2_label->setText(fmtMadi(madi_sensor[0]));
        palm_sensor_label->setText(fmtPalm(palm_sensor_val));
    } else {
        sensor1_label->setText(fmtTip(fingertip_sensor[3]));
        sensor2_label->setText(fmtTip(fingertip_sensor[2]));
        sensor3_label->setText(fmtTip(fingertip_sensor[1]));
        sensor4_label->setText(fmtTip(fingertip_sensor[0]));
        madi_sensor1_1_label->setText(fmtMadi(madi_sensor[4]));
        madi_sensor1_2_label->setText(fmtMadi(madi_sensor[3]));
        madi_sensor1_3_label->setText(fmtMadi(madi_sensor[2]));
        madi_sensor2_1_label->setText(fmtMadi(madi_sensor[7]));
        madi_sensor2_2_label->setText(fmtMadi(madi_sensor[6]));
        madi_sensor2_3_label->setText(fmtMadi(madi_sensor[5]));
        madi_sensor3_1_label->setText(fmtMadi(madi_sensor[10]));
        madi_sensor3_2_label->setText(fmtMadi(madi_sensor[9]));
        madi_sensor3_3_label->setText(fmtMadi(madi_sensor[8]));
        madi_sensor4_1_label->setText(fmtMadi(madi_sensor[1]));
        madi_sensor4_2_label->setText(fmtMadi(madi_sensor[0]));
        palm_sensor_label->setText(fmtPalm(palm_sensor_val));
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    g_node = std::make_shared<rclcpp::Node>("sensor_gui");

    g_node->declare_parameter("hand_info/which_hand", "right");
    g_node->get_parameter("hand_info/which_hand", whichHand);

    std::string pkg_path = ament_index_cpp::get_package_share_directory("allegro_hand_sensor_visualizer");
    QString bg_path   = QString::fromStdString(pkg_path + "/asset/Logo_transparent.png");
    QString hand_path = QString::fromStdString(pkg_path + "/asset/Hand_transparent.png");
    if (whichHand == "left")
        hand_path = QString::fromStdString(pkg_path + "/asset/Hand_transparent_left.png");

    marker_pub = g_node->create_publisher<visualization_msgs::msg::Marker>("/finger_labels", 10);

    QApplication app(argc, argv);

    const double REF_W = 1920.0;
    const double REF_H = 1080.0;
    QScreen* screen = QApplication::primaryScreen();
    QRect screenGeom = screen->availableGeometry();
    double sx = screenGeom.width()  / REF_W;
    double sy = screenGeom.height() / REF_H;
    double s  = std::min(sx, sy);
    auto SX = [sx](int v) -> int { return (int)(v * sx); };
    auto SY = [sy](int v) -> int { return (int)(v * sy); };
    auto SF = [s](int v)  -> int { return std::max(1, (int)(v * s)); };

    QWidget window;
    window.setObjectName("mainWindow");
    window.setFixedSize(SX(1850), SY(1050));

    QLabel* handLabel = new QLabel(&window);
    QPixmap handPixmap(hand_path);
    QPixmap scaledHand = handPixmap.scaled(SX(1300), SY(1300), Qt::KeepAspectRatio, Qt::SmoothTransformation);
    handLabel->setPixmap(scaledHand);
    handLabel->setFixedSize(scaledHand.size());
    handLabel->setAttribute(Qt::WA_TransparentForMouseEvents);
    if (whichHand == "left") {
        handLabel->move(window.width() - handLabel->width() + SX(250), SY(-50));
    } else {
        handLabel->move(window.width() - handLabel->width(), SY(-50));
    }
    handLabel->show();

    QLabel* logoLabel = new QLabel(&window);
    QPixmap logoPixmap(bg_path);
    QPixmap scaledLogo = logoPixmap.scaled(SX(650), SY(650), Qt::KeepAspectRatio, Qt::SmoothTransformation);
    logoLabel->setPixmap(scaledLogo);
    logoLabel->setFixedSize(scaledLogo.size());
    logoLabel->move((window.width() - logoLabel->width()) / 2, (window.height() - logoLabel->height()) / 2);
    logoLabel->lower();
    logoLabel->show();

    int base_x = SX(830);
    int base_y = SY(120);
    int dy     = SY(70);
    int lw     = SX(320);
    int lws    = SX(260);
    int lh     = SF(100);  // two-line label height (fingertip / palm)
    int lhs    = SF(80);   // two-line label height (madi)
    const int DX = SX(-100); // shift all labels left
    const int DY = SY(-20);  // shift all labels up
    const int FX = SX(-15);  // fingertip labels: additional left shift
    const int FY = SY(-10);  // fingertip labels: additional up shift

    sensor3_label = new QLabel(&window);
    sensor3_label->setGeometry(SX(890)+DX+FX, SY(80)+DY+FY, lw, lh);
    madi_sensor3_1_label = new QLabel(&window);
    madi_sensor3_1_label->setGeometry(SX(910)+DX, SY(280)+DY, lws, lhs);
    madi_sensor3_2_label = new QLabel(&window);
    madi_sensor3_2_label->setGeometry(SX(930)+DX, SY(430)+DY, lws, lhs);
    madi_sensor3_3_label = new QLabel(&window);
    madi_sensor3_3_label->setGeometry(SX(940)+DX, SY(500)+DY, lws, lhs);
    sensor3_label->setTextFormat(Qt::RichText);
    sensor3_label->setAlignment(Qt::AlignCenter);
    sensor3_label->setStyleSheet("color: black;");
    madi_sensor3_1_label->setTextFormat(Qt::RichText);
    madi_sensor3_1_label->setAlignment(Qt::AlignCenter);
    madi_sensor3_1_label->setStyleSheet("color: white;");
    madi_sensor3_2_label->setTextFormat(Qt::RichText);
    madi_sensor3_2_label->setAlignment(Qt::AlignCenter);
    madi_sensor3_2_label->setStyleSheet("color: white;");
    madi_sensor3_3_label->setTextFormat(Qt::RichText);
    madi_sensor3_3_label->setAlignment(Qt::AlignCenter);
    madi_sensor3_3_label->setStyleSheet("color: white;");

    sensor2_label = new QLabel(&window);
    sensor2_label->setGeometry(SX(1090)+DX+FX, SY(70)+DY+FY, lw, lh);
    madi_sensor2_1_label = new QLabel(&window);
    madi_sensor2_1_label->setGeometry(SX(1105)+DX, SY(270)+DY, lws, lhs);
    madi_sensor2_2_label = new QLabel(&window);
    madi_sensor2_2_label->setGeometry(SX(1105)+DX, SY(420)+DY, lws, lhs);
    madi_sensor2_3_label = new QLabel(&window);
    madi_sensor2_3_label->setGeometry(SX(1105)+DX, SY(490)+DY, lws, lhs);
    sensor2_label->setTextFormat(Qt::RichText);
    sensor2_label->setAlignment(Qt::AlignCenter);
    sensor2_label->setStyleSheet("color: black;");
    madi_sensor2_1_label->setTextFormat(Qt::RichText);
    madi_sensor2_1_label->setAlignment(Qt::AlignCenter);
    madi_sensor2_1_label->setStyleSheet("color: white;");
    madi_sensor2_2_label->setTextFormat(Qt::RichText);
    madi_sensor2_2_label->setAlignment(Qt::AlignCenter);
    madi_sensor2_2_label->setStyleSheet("color: white;");
    madi_sensor2_3_label->setTextFormat(Qt::RichText);
    madi_sensor2_3_label->setAlignment(Qt::AlignCenter);
    madi_sensor2_3_label->setStyleSheet("color: white;");

    sensor1_label = new QLabel(&window);
    sensor1_label->setGeometry(SX(1295)+DX+FX, SY(80)+DY+FY, lw, lh);
    madi_sensor1_1_label = new QLabel(&window);
    madi_sensor1_1_label->setGeometry(SX(1295)+DX, SY(280)+DY, lws, lhs);
    madi_sensor1_2_label = new QLabel(&window);
    madi_sensor1_2_label->setGeometry(SX(1285)+DX, SY(430)+DY, lws, lhs);
    madi_sensor1_3_label = new QLabel(&window);
    madi_sensor1_3_label->setGeometry(SX(1280)+DX, SY(500)+DY, lws, lhs);
    sensor1_label->setTextFormat(Qt::RichText);
    sensor1_label->setAlignment(Qt::AlignCenter);
    sensor1_label->setStyleSheet("color: black;");
    madi_sensor1_1_label->setTextFormat(Qt::RichText);
    madi_sensor1_1_label->setAlignment(Qt::AlignCenter);
    madi_sensor1_1_label->setStyleSheet("color: white;");
    madi_sensor1_2_label->setTextFormat(Qt::RichText);
    madi_sensor1_2_label->setAlignment(Qt::AlignCenter);
    madi_sensor1_2_label->setStyleSheet("color: white;");
    madi_sensor1_3_label->setTextFormat(Qt::RichText);
    madi_sensor1_3_label->setAlignment(Qt::AlignCenter);
    madi_sensor1_3_label->setStyleSheet("color: white;");

    int x4 = base_x + SX(750);
    int y4 = base_y + 7 * dy;

    sensor4_label = new QLabel(&window);
    sensor4_label->setGeometry(x4 + SX(145)+DX+FX, SY(860)+DY+FY, lw, lh);
    madi_sensor4_1_label = new QLabel(&window);
    madi_sensor4_1_label->setGeometry(x4+DX, SY(860)+DY, lws, lhs);
    madi_sensor4_2_label = new QLabel(&window);
    madi_sensor4_2_label->setGeometry(x4 - SX(150)+DX, SY(860)+DY, lws, lhs);
    sensor4_label->setTextFormat(Qt::RichText);
    sensor4_label->setAlignment(Qt::AlignCenter);
    sensor4_label->setStyleSheet("color: black;");
    madi_sensor4_1_label->setTextFormat(Qt::RichText);
    madi_sensor4_1_label->setAlignment(Qt::AlignCenter);
    madi_sensor4_1_label->setStyleSheet("color: white;");
    madi_sensor4_2_label->setTextFormat(Qt::RichText);
    madi_sensor4_2_label->setAlignment(Qt::AlignCenter);
    madi_sensor4_2_label->setStyleSheet("color: white;");

    palm_sensor_label = new QLabel(&window);
    palm_sensor_label->setGeometry(x4 - SX(580)+DX, y4 + 2 * dy - SY(40) + SY(100)+DY, lw, lh);
    palm_sensor_label->setTextFormat(Qt::RichText);
    palm_sensor_label->setAlignment(Qt::AlignCenter);
    palm_sensor_label->setStyleSheet("color: white;");

    if (whichHand == "left") {
        int lo = SX(375);
        sensor3_label->setGeometry(SX(890) + lo + DX+FX, SY(80)+DY+FY, lw, lh);
        madi_sensor3_1_label->setGeometry(SX(910) + lo + DX, SY(280)+DY, lws, lhs);
        madi_sensor3_2_label->setGeometry(SX(930) + lo + DX, SY(430)+DY, lws, lhs);
        madi_sensor3_3_label->setGeometry(SX(940) + lo + DX, SY(490)+DY, lws, lhs);
        sensor2_label->setGeometry(SX(1090) + lo + DX+FX, SY(70)+DY+FY, lw, lh);
        madi_sensor2_1_label->setGeometry(SX(1100) + lo + DX, SY(270)+DY, lws, lhs);
        madi_sensor2_2_label->setGeometry(SX(1100) + lo + DX, SY(420)+DY, lws, lhs);
        madi_sensor2_3_label->setGeometry(SX(1100) + lo + DX, SY(490)+DY, lws, lhs);
        sensor1_label->setGeometry(SX(1295) + lo + DX+FX, SY(80)+DY+FY, lw, lh);
        madi_sensor1_1_label->setGeometry(SX(1295) + lo + DX, SY(280)+DY, lws, lhs);
        madi_sensor1_2_label->setGeometry(SX(1285) + lo + DX, SY(430)+DY, lws, lhs);
        madi_sensor1_3_label->setGeometry(SX(1280) + lo + DX, SY(490)+DY, lws, lhs);
        sensor4_label->setGeometry(x4 - SX(745) + DX+FX, SY(860)+DY+FY, lw, lh);
        madi_sensor4_1_label->setGeometry(x4 - SX(590) + DX, SY(860)+DY, lws, lhs);
        madi_sensor4_2_label->setGeometry(x4 - SX(440) + DX, SY(860)+DY, lws, lhs);
        palm_sensor_label->setGeometry(x4 - SX(50) + DX, y4 + 2 * dy - SY(40) + SY(100)+DY, lw, lh);
    }

    window.show();

    auto sensor_sub = g_node->create_subscription<std_msgs::msg::Float32MultiArray>(
        "allegroHand/tactile_sensors", 10, tactileSensorCallback);

    QTimer ros_timer;
    QObject::connect(&ros_timer, &QTimer::timeout, [&]() {
        rclcpp::spin_some(g_node);
        updateMarkers();
    });
    ros_timer.start(1);

    int ret = app.exec();

    g_node.reset();
    rclcpp::shutdown();
    return ret;
}
