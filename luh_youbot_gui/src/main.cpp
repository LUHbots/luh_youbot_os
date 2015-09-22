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


/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QApplication>
#include "../include/luh_youbot_gui/main_window.hpp"
#include "../include/luh_youbot_gui/arm_control_window.hpp"
//#include "../include/luh_youbot_gui/render_area.hpp"


/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv)
{
    QApplication app(argc, argv);
    luh_youbot_gui::MainWindow w(argc,argv);
    w.show();
    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
    int result = app.exec();

	return result;
}
