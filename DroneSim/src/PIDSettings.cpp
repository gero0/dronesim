//
// Created by gero on 7/6/23.
//


#include "PIDSettings.h"

PIDSettings::PIDSettings(QWidget *parent, QString name){
    this->setParent(parent);
    title_label = new QLabel(name, this);
    auto p_label = new QLabel("Kp", this);
    auto i_label = new QLabel("Ki", this);
    auto d_label = new QLabel("Kd", this);
    p_input = new QLineEdit(this);
    i_input = new QLineEdit(this);
    d_input = new QLineEdit(this);
    submit_btn = new QPushButton("Set", this);
    status = new QLabel(this);

    layout = new QGridLayout(this);
    layout->addWidget(title_label, 0,0,1,6);
    layout->addWidget(p_label, 1,0);
    layout->addWidget(p_input, 1,1);
    layout->addWidget(i_label, 1,2);
    layout->addWidget(i_input, 1,3);
    layout->addWidget(d_label, 1,4);
    layout->addWidget(d_input, 1,5);
    layout->addWidget(d_label, 1,4);
    layout->addWidget(submit_btn, 2, 0);
    layout->addWidget(status);

    connect(submit_btn, SIGNAL(clicked(bool)), this, SLOT(update_values()));
}

void PIDSettings::set_values(PidTunings tunings) {
    p_input->setText(QString::number(tunings.Kp));
    i_input->setText(QString::number(tunings.Ki));
    d_input->setText(QString::number(tunings.Kd));
}


void PIDSettings::update_values(){
    try {
        float Kp = std::stof(p_input->text().toStdString());
        float Ki = std::stof(i_input->text().toStdString());
        float Kd = std::stof(d_input->text().toStdString());
        emit values_updated({Kp, Ki, Kd});
        status->setText("OK");
    } catch (std::exception &e) {
        status->setText("Invalid input!");
    }
}

