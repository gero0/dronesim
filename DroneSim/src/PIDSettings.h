//
// Created by gero on 7/6/23.
//

#ifndef DRONESIMPROJECT_PIDSETTINGS_H
#define DRONESIMPROJECT_PIDSETTINGS_H


#include <QWidget>
#include <QLineEdit>
#include <QPushButton>
#include <QLabel>
#include <QGridLayout>
#include "PID.h"

class PIDSettings : public QWidget {
Q_OBJECT

public:
    explicit PIDSettings(QWidget *parent = nullptr, QString name = "PID");

    void set_values(PidTunings tunings);

signals:
    void values_updated(PidTunings tunings);

private slots:

    void update_values();

private:
    QLabel *title_label;
    QLineEdit *p_input;
    QLineEdit *i_input;
    QLineEdit *d_input;
    QPushButton *submit_btn;
    QGridLayout *layout;
    QLabel* status;
};


#endif //DRONESIMPROJECT_PIDSETTINGS_H
