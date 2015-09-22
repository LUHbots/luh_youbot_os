/* *****************************************************************
 *
 * luh_youbot_gui
 *
 * Copyright (c) 2015,
 * Institute of Mechatronic Systems,
 * Leibniz Universitaet Hannover.
 * (BSD License)
 * All rights reserved.
 *
 * http://www.imes.uni-hannover.de
 *
 * This software is distributed WITHOUT ANY WARRANTY; without
 * even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.
 *
 * For further information see http://www.linfo.org/bsdlicense.html
 *
 * Author: Simon Aden (info@luhbots.de)
 ******************************************************************/

#ifndef LUH_YOUBOT_GUI_MAIN_WINDOW_HPP
#define LUH_YOUBOT_GUI_MAIN_WINDOW_HPP

#include <QMainWindow>
#include <QStatusBar>
#include "arm_control_window.hpp"
#include "qnode.hpp"

namespace luh_youbot_gui
{

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit MainWindow(int argc, char** argv, QWidget *parent = 0);

Q_SIGNALS:

public Q_SLOTS:

protected Q_SLOTS:
    void showMessage(QString msg);

protected:
    ArmControlWindow * arm_control_window_;
    QStatusBar *status_bar_;
    QNode qnode_;
    void closeEvent(QCloseEvent *event);

};
}

#endif // MAIN_WINDOW_HPP
