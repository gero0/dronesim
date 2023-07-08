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
//    this->setAttribute(Qt::WA_DeleteOnClose);
    central_widget = new QWidget(this);

    pid_window = new PIDWindow(&drone);
    plot_window = new PlotWindow();

    auto vis_widget = init_visualization(central_widget);
    auto bar_widget = init_bars(central_widget);
    auto menu_widget = init_menu_widgets(central_widget);

    QSizePolicy spvis(QSizePolicy::Preferred, QSizePolicy::Preferred);
    spvis.setHorizontalStretch(3);
    QSizePolicy spbar(QSizePolicy::Preferred, QSizePolicy::Preferred);
    spbar.setHorizontalStretch(1);
    QSizePolicy spdata(QSizePolicy::Preferred, QSizePolicy::Preferred);
    spbar.setHorizontalStretch(1);

    menu_widget->setMinimumWidth(400);
    bar_widget->setMinimumWidth(300);

    vis_widget->setSizePolicy(spvis);
    bar_widget->setSizePolicy(spbar);
    menu_widget->setSizePolicy(spdata);

    root_layout = new QHBoxLayout(central_widget);
    root_layout->addWidget(vis_widget);
    root_layout->addWidget(bar_widget);
    root_layout->addWidget(menu_widget);

    central_widget->setLayout(root_layout);
    setCentralWidget(central_widget);

    QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(update_sim()));
    timer->start(dt * 1000);
}

QWidget *MainWindow::init_visualization(QWidget *parent) {
    vis_data = new QScatterDataArray();
    vis_series = new QScatter3DSeries(this);
    vis_series->setItemSize(0.1);
    vis_plot = new Q3DScatter();
    vis_plot->scene()->activeCamera()->setCameraPreset(Q3DCamera::CameraPresetIsometricRight);
    vis_plot->scene()->activeCamera()->setZoomLevel(150.0f);

    QWidget * m_container = QWidget::createWindowContainer(vis_plot, parent);
    vis_plot->addSeries(vis_series);
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

    bar_fl = new QProgressBar(bar_widget);
    bar_fl->setOrientation(Qt::Orientation::Vertical);
    bar_fr = new QProgressBar(bar_widget);
    bar_fr->setOrientation(Qt::Orientation::Vertical);
    bar_bl = new QProgressBar(bar_widget);
    bar_bl->setOrientation(Qt::Orientation::Vertical);
    bar_br = new QProgressBar(bar_widget);
    bar_br->setOrientation(Qt::Orientation::Vertical);

    label_fl = new QLabel("Front Left", bar_widget);
    label_fr = new QLabel("Front Right", bar_widget);
    label_bl = new QLabel("Back Left", bar_widget);
    label_br = new QLabel("Back Right", bar_widget);

    bar_layout->addWidget(label_fl, 0, 0);
    bar_layout->addWidget(label_fr, 0, 1);
    bar_layout->addWidget(bar_fl, 1, 0);
    bar_layout->addWidget(bar_fr, 1, 1);

    bar_layout->addWidget(label_bl, 2, 0);
    bar_layout->addWidget(label_br, 2, 1);
    bar_layout->addWidget(bar_bl, 3, 0);
    bar_layout->addWidget(bar_br, 3, 1);

    bar_widget->setLayout(bar_layout);

    return bar_widget;
}

QWidget *MainWindow::init_menu_widgets(QWidget *parent) {
    auto data_widget = new QWidget(parent);
    auto layout = new QVBoxLayout(data_widget);

    position = new QLabel("", data_widget);
    rotation = new QLabel("", data_widget);
    velocity = new QLabel("", data_widget);
    angular = new QLabel("", data_widget);
    pid_out = new QLabel("", data_widget);
    pid_setpoints = new QLabel("", data_widget);

    open_pid_window_btn = new QPushButton("PID Settings...", this);
    open_plot_window_btn = new QPushButton("PID Plots...", this);

    layout->addWidget(position);
    layout->addWidget(rotation);
    layout->addWidget(velocity);
    layout->addWidget(angular);
    layout->addWidget(pid_out);
    layout->addWidget(pid_setpoints);
    layout->addWidget(open_pid_window_btn);
    layout->addWidget(open_plot_window_btn);

    connect(open_pid_window_btn, SIGNAL(clicked(bool)), this, SLOT(open_pid_settings()));
    connect(open_plot_window_btn, SIGNAL(clicked(bool)), this, SLOT(open_plot_window()));

    return data_widget;
}

void MainWindow::keyPressEvent(QKeyEvent *event) {
    auto setpoints = drone.controller.get_setpoints();
    if (event->key() == Qt::Key_W) {
        drone.controller.set_pitch(setpoints.v_pitch - 0.05f);
    } else if (event->key() == Qt::Key_S) {
        drone.controller.set_pitch(setpoints.v_pitch + 0.05f);
    } else if (event->key() == Qt::Key_D) {
        drone.controller.set_roll(setpoints.v_roll - 0.05f);
    } else if (event->key() == Qt::Key_A) {
        drone.controller.set_roll(setpoints.v_roll + 0.05f);
    } else if (event->key() == Qt::Key_Q) {
        drone.controller.set_yaw(setpoints.v_yaw + 0.05f);
    } else if (event->key() == Qt::Key_E) {
        drone.controller.set_yaw(setpoints.v_yaw - 0.05f);
    } else if (event->key() == Qt::Key_Shift) {
        drone.controller.set_altitude(setpoints.v_thrust + 0.5f);
    } else if (event->key() == Qt::Key_Control) {
        drone.controller.set_altitude(setpoints.v_thrust - 0.5f);
    }
}

void MainWindow::update_sim() {
    drone.update(dt);

    auto [sp_thrust, sp_pitch, sp_roll, sp_yaw] = drone.controller.get_setpoints();

    plot_window->thrust_plot->append(current_time, sp_thrust, drone.position.z);
    plot_window->pitch_plot->append(current_time, sp_pitch, drone.rotation.pitch);
    plot_window->roll_plot->append(current_time, sp_roll, drone.rotation.roll);
    plot_window->yaw_plot->append(current_time, sp_yaw, drone.rotation.yaw);

    bar_fl->setValue(static_cast<int> (drone.fl_driver.get_speed() * 100.0f));
    bar_fr->setValue(static_cast<int> (drone.fr_driver.get_speed() * 100.0f));
    bar_bl->setValue(static_cast<int> (drone.bl_driver.get_speed() * 100.0f));
    bar_br->setValue(static_cast<int> (drone.br_driver.get_speed() * 100.0f));

    label_fl->setText("Front Left: " + QString::number(bar_fl->value()) + "%");
    label_fr->setText("Front Right: " + QString::number(bar_fr->value()) + "%");
    label_bl->setText("Back Left: " + QString::number(bar_bl->value()) + "%");
    label_br->setText("Back Right: " + QString::number(bar_br->value()) + "%");

    draw_scatter();
    draw_text();
    current_time += dt;
}

void MainWindow::draw_scatter() {
    auto [x, y, z] = drone.position;
    auto [pitch, yaw, roll] = drone.rotation;
    const float r = 2.0f;

    Vector3 fr{cosf(yaw) - sinf(yaw), sinf(yaw) + cosf(yaw), r * sinf(pitch) + r * sinf(roll)};
    Vector3 fl{-cosf(yaw) - sinf(yaw), -sinf(yaw) + cosf(yaw), r * sinf(pitch) - r * sinf(roll)};
    Vector3 br{cosf(yaw) + sinf(yaw), sinf(yaw) - cosf(yaw), -r * sinf(pitch) + r * sinf(roll)};
    Vector3 bl{-cosf(yaw) + sinf(yaw), -sinf(yaw) - cosf(yaw), -r * sinf(pitch) - r * sinf(roll)};

    Vector3 front_right{x + fr.x, y + fr.y, z + fr.z};
    Vector3 front_left{x + fl.x, y + fl.y, z + fl.z};
    Vector3 back_left{x + bl.x, y + bl.y, z + bl.z};
    Vector3 back_right{x + br.x, y + br.y, z + br.z};

    Vector3 motors[4] = {front_right, front_left, back_left, back_right};

    vis_data->clear();

    for (auto motor: motors) {
        *vis_data << QVector3D(motor.x, motor.z, motor.y);
    }

    *vis_data << QVector3D(x, z, y);
    *vis_data << QVector3D(x, 0, y);

    vis_series->dataProxy()->resetArray(vis_data);

    rescale_axes();
}

void MainWindow::rescale_axes() {
    auto [x, y, z] = drone.position;
    float pos[3] = {x, z, y};
    for (int i = 0; i < 3; i++) {
        auto &ax = vis_plot->axes()[i];
        if (pos[i] > ax->max()) {
            auto diff = ax->max() - ax->min();
            ax->setRange(ax->max(), ax->max() + diff);
        } else if (pos[i] < ax->min()) {
            auto diff = ax->max() - ax->min();
            ax->setRange(ax->min() - diff, ax->min());
        }
    }
}

void MainWindow::draw_text() {
    auto [x, y, z] = drone.position;
    auto [pitch, yaw, roll] = drone.rotation;
    auto [vx, vy, vz] = drone.velocity;
    auto [vp, vyaw, vr] = drone.angular_velocity;

    position->setText(QString::fromStdString(string_format("Position: X: %.2f; Y: %.2f; Z: %.2f; (m)", x, y, z)));
    rotation->setText(
            QString::fromStdString(string_format("Rotation: P: %.2f; R: %.2f; Y: %.2f; (rad)", pitch, roll, yaw)));
    velocity->setText(QString::fromStdString(string_format("Velocity: X: %.2f; Y: %.2f; Z: %.2f; (m/s)", vx, vy, vz)));
    angular->setText(
            QString::fromStdString(string_format("Angular V.: P: %.2f; R: %.2f; Y: %.2f; (rad/s)", vp, vr, vyaw)));

    PidValues pid = drone.controller.get_last_pid();
    PidValues setpoints = drone.controller.get_setpoints();

    pid_out->setText(QString::fromStdString(
            string_format("PID: Thrust: %.4f; Pitch: %.4f; Roll:%.4f ; Yaw: %.4f;", pid.v_thrust, pid.v_pitch,
                          pid.v_roll, pid.v_yaw)));

    pid_setpoints->setText(QString::fromStdString(
            string_format("Setpoints: Altitude: %.2f; Pitch: %.2f; Roll:%.2f ; Yaw: %.2f;", setpoints.v_thrust,
                          setpoints.v_pitch,
                          setpoints.v_roll, setpoints.v_yaw)));
}

void MainWindow::open_pid_settings() {
    pid_window->show();
}

void MainWindow::open_plot_window() {
    plot_window->show();
}

MainWindow::~MainWindow() {
    pid_window->close();
    plot_window->close();
    delete pid_window;
    delete plot_window;
}


