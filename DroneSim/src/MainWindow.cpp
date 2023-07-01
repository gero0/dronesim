//
// Created by gero on 6/27/23.
//

#include "MainWindow.h"

template<typename ... Args>
std::string string_format(const std::string &format, Args ... args) {
    int size_s = std::snprintf(nullptr, 0, format.c_str(), args ...) + 1; // Extra space for '\0'
    if (size_s <= 0) { throw std::runtime_error("Error during formatting."); }
    auto size = static_cast<size_t>( size_s );
    std::unique_ptr<char[]> buf(new char[size]);
    std::snprintf(buf.get(), size, format.c_str(), args ...);
    return std::string(buf.get(), buf.get() + size - 1); // We don't want the '\0' inside
}

MainWindow::MainWindow() {
    this->setMinimumSize(1200, 600);
    central_widget = new QWidget(this);

    auto vis_widget = init_vis(central_widget);
    auto bar_widget = init_bars(central_widget);
    auto data_widget = init_data(central_widget);

    QSizePolicy spvis(QSizePolicy::Preferred, QSizePolicy::Preferred);
    spvis.setHorizontalStretch(3);
    QSizePolicy spbar(QSizePolicy::Preferred, QSizePolicy::Preferred);
    spbar.setHorizontalStretch(1);
    QSizePolicy spdata(QSizePolicy::Preferred, QSizePolicy::Preferred);
    spbar.setHorizontalStretch(1);

    vis_widget->setSizePolicy(spvis);
    bar_widget->setSizePolicy(spbar);
    data_widget->setSizePolicy(spdata);

    root_layout = new QHBoxLayout(central_widget);
    root_layout->addWidget(vis_widget);
    root_layout->addWidget(bar_widget);
    root_layout->addWidget(data_widget);

    central_widget->setLayout(root_layout);
    setCentralWidget(central_widget);

    QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(update_sim()));
    timer->start(10);
}


void MainWindow::update_sim_ui() {
    auto [x, y, z] = drone.position;
    auto [pitch, roll, yaw] = drone.rotation;
    auto [vx, vy, vz] = drone.velocity;
    auto [vp, vr, vyaw] = drone.angular_velocity;
    auto r = drone.arm_len;

    position->setText(QString::fromStdString(string_format("Position: X: %.2f; Y: %.2f; Z: %.2f; (m)", x, y, z)));
    rotation->setText(
            QString::fromStdString(string_format("Rotation: P: %.2f; R: %.2f; Y: %.2f; (rad)", pitch, roll, yaw)));
    velocity->setText(QString::fromStdString(string_format("Velocity: X: %.2f; Y: %.2f; Z: %.2f; (m/s)", vx, vy, vz)));
    angular->setText(
            QString::fromStdString(string_format("Angular V.: P: %.2f; R: %.2f; Y: %.2f; (rad/s)", vp, vr, vyaw)));


    Vector3 fr{cos(yaw) - sin(yaw), sin(yaw) + cos(yaw), r * sin(pitch) + r * sin(roll)};
    Vector3 fl{-cos(yaw) - sin(yaw), -sin(yaw) + cos(yaw), r * sin(pitch) - r * sin(roll)};
    Vector3 br{cos(yaw) + sin(yaw), sin(yaw) - cos(yaw), -r * sin(pitch) + r * sin(roll)};
    Vector3 bl{-cos(yaw) + sin(yaw), -sin(yaw) - cos(yaw), -r * sin(pitch) - r * sin(roll)};

    Vector3 front_right{x + fr.x, y + fr.y, z + fr.z};
    Vector3 front_left{x + fl.x, y + fl.y, z + fl.z};
    Vector3 back_left{x + bl.x, y + bl.y, z + bl.z};
    Vector3 back_right{x + br.x, y + br.y, z + br.z};

    Vector3 motors[4] = {front_right, front_left, back_left, back_right};

    vis_data.clear();

    for (auto motor: motors) {
        vis_data << QVector3D(motor.x, motor.z, motor.y);
    }

    vis_data << QVector3D(x, z, y);
    vis_data << QVector3D(x, 0, y);

    vis_series.dataProxy()->resetArray(&vis_data);
}

QWidget *MainWindow::init_vis(QWidget *parent) {
    vis_series.setItemSize(0.1);
    vis_plot = new Q3DScatter();
    vis_plot->scene()->activeCamera()->setCameraPreset(Q3DCamera::CameraPresetIsometricRight);
    vis_plot->scene()->activeCamera()->setZoomLevel(150.0f);

    QWidget *m_container = QWidget::createWindowContainer(vis_plot, parent);
    vis_plot->addSeries(&vis_series);
    vis_plot->setAspectRatio(1.0);
    vis_plot->setHorizontalAspectRatio(1.0);

    auto x_axis = new QValue3DAxis(m_container);
    x_axis->setMin(-10.0);
    x_axis->setMax(10.0);

    auto y_axis = new QValue3DAxis(m_container);
    y_axis->setMin(0.0);
    y_axis->setMax(20.0);

    auto z_axis = new QValue3DAxis(m_container);
    z_axis->setMin(-10.0);
    z_axis->setMax(10.0);

    vis_plot->setAxisX(x_axis);
    vis_plot->setAxisY(y_axis);
    vis_plot->setAxisZ(z_axis);
    vis_plot->setShadowQuality(QAbstract3DGraph::ShadowQualityNone);

    return m_container;
}

QWidget *MainWindow::init_bars(QWidget *parent) {
    auto bar_widget = new QWidget(parent);
    auto bar_layout = new QGridLayout(bar_widget);

    auto Bar_fl = new QProgressBar(bar_widget);
    Bar_fl->setOrientation(Qt::Orientation::Vertical);
    auto Bar_fr = new QProgressBar(bar_widget);
    Bar_fr->setOrientation(Qt::Orientation::Vertical);
    auto Bar_bl = new QProgressBar(bar_widget);
    Bar_bl->setOrientation(Qt::Orientation::Vertical);
    auto Bar_br = new QProgressBar(bar_widget);
    Bar_br->setOrientation(Qt::Orientation::Vertical);

    auto label_fl = new QLabel("Front Left", bar_widget);
    auto label_fr = new QLabel("Front Right", bar_widget);
    auto label_bl = new QLabel("Back Left", bar_widget);
    auto label_br = new QLabel("Back Right", bar_widget);

    bar_layout->addWidget(label_fl, 0, 0);
    bar_layout->addWidget(label_fr, 0, 1);
    bar_layout->addWidget(Bar_fl, 1, 0);
    bar_layout->addWidget(Bar_fr, 1, 1);

    bar_layout->addWidget(label_bl, 2, 0);
    bar_layout->addWidget(label_br, 2, 1);
    bar_layout->addWidget(Bar_bl, 3, 0);
    bar_layout->addWidget(Bar_br, 3, 1);

    bar_widget->setLayout(bar_layout);

    return bar_widget;
}

QWidget *MainWindow::init_data(QWidget *parent) {
    auto data_widget = new QWidget(parent);
    auto layout = new QVBoxLayout(data_widget);

    position = new QLabel("", data_widget);
    rotation = new QLabel("", data_widget);
    velocity = new QLabel("", data_widget);
    angular = new QLabel("", data_widget);

    layout->addWidget(position);
    layout->addWidget(rotation);
    layout->addWidget(velocity);
    layout->addWidget(angular);

    return data_widget;
}

void MainWindow::update_sim() {
    update_sim_ui();
}


