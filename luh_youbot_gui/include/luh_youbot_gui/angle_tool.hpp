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

#ifndef LUH_YOUBOT_GUI_ANGLE_TOOL_HPP
#define LUH_YOUBOT_GUI_ANGLE_TOOL_HPP

#include <QObject>
#include <QPoint>
#include <QPainter>

class AngleTool : public QObject
{
    Q_OBJECT
public:
    explicit AngleTool(QObject *parent = 0);
    explicit AngleTool(QPointF position, double min_angle, double max_angle, double current_angle, double angle_offset,
                       QObject *parent = 0);

    void reset(QPointF position, double min_angle, double max_angle, double current_angle, double angle_offset);
    void paint(QPainter &painter, double scale_factor);
    QPointF getToolPoint();
    void activate();
    void deactivate();
    void setMousePosition(QPointF mouse_position){mouse_position_ = mouse_position;}
    void setMousePosition(QPoint mouse_position){mouse_position_ = QPointF(mouse_position);}
    void setPosition(QPointF position){position_ = position;}
    void setCurrentAngle(double current_angle){current_angle_ = current_angle;}
    void setAngleOffset(double angle_offset){angle_offset_ = angle_offset;}

Q_SIGNALS:
    void angleChanged(double angle);

protected:    
    QPointF mouse_position_;
    QPointF position_;
    QPointF tool_point_;
    double min_angle_;
    double max_angle_;
    double current_angle_;
    double angle_offset_;
    double default_radius_;
    bool is_active_;
};

#endif // ANGLE_TOOL_HPP
