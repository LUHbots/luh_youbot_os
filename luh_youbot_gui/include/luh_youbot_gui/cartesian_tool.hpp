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

#ifndef LUH_YOUBOT_GUI_CARTESIAN_TOOL_HPP
#define LUH_YOUBOT_GUI_CARTESIAN_TOOL_HPP

#include <QObject>
#include <QPoint>
#include <QPainter>

class CartesianTool : public QObject
{
    Q_OBJECT
public:
    explicit CartesianTool(QObject *parent = 0);

    void paint(QPainter &painter, double scale_factor);

    void setMousePosition(QPointF mouse_position){mouse_position_ = mouse_position;}
    void setMousePosition(QPoint mouse_position){mouse_position_ = QPointF(mouse_position);}
    void setPosition(QPointF position){position_ = position;}

Q_SIGNALS:
    void valueChanged(double x, double y);

protected:
    QPointF mouse_position_;
    QPointF position_;
    double max_length_;
    double arrow_width_;
    bool is_active_;

};

#endif // CARTESIAN_TOOL_HPP
