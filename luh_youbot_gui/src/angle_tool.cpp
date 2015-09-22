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

#include "luh_youbot_gui/angle_tool.hpp"
#include <math.h>
#include <iostream>

AngleTool::AngleTool(QObject *parent) :
    QObject(parent)
{
}

AngleTool::AngleTool(QPointF position, double min_angle, double max_angle, double current_angle, double angle_offset, QObject *parent):
    position_(position),
    min_angle_(min_angle),
    max_angle_(max_angle),
    current_angle_(current_angle),
    angle_offset_(angle_offset),
    is_active_(false),
    QObject(parent)
{
    default_radius_ = 0.2;

}

void AngleTool::reset(QPointF position, double min_angle, double max_angle, double current_angle, double angle_offset)
{
    position_ = position;
    min_angle_ = min_angle;
    max_angle_ = max_angle;
    current_angle_ = current_angle;
    angle_offset_ = angle_offset;
    default_radius_ = 0.2;
    is_active_ = false;
}

//void AngleTool::mouseClick(QPoint mouse_position)
//{

//}

//void AngleTool::mousePress(QPoint mouse_position)
//{

//}

//void AngleTool::mouseRelease(QPoint mouse_position)
//{

//}

void AngleTool::activate()
{
    is_active_ = true;
}
void AngleTool::deactivate()
{
    is_active_ = false;
}

void AngleTool::paint(QPainter &painter, double scale_factor)
{
    // === DRAW ANGLE RANGE (PIE) ===
    double radius;
    if(is_active_)
    {
        double dx = -position_.x() + mouse_position_.x();
        double dy =  position_.y() - mouse_position_.y();
        current_angle_ = M_PI/2 - angle_offset_ - atan2(dy, dx);
        if(current_angle_ < M_PI) current_angle_ += 2*M_PI;
        if(current_angle_ > M_PI) current_angle_ -= 2*M_PI;
        current_angle_ = std::min(current_angle_, max_angle_);
        current_angle_ = std::max(current_angle_, min_angle_);
//        radius = sqrt(dx*dx + dy*dy);
        radius = default_radius_ * scale_factor;
        Q_EMIT angleChanged(current_angle_);
    }
    else
        radius = default_radius_ * scale_factor;

    QColor color("blue");
    color.setAlphaF(0.2);
    QPen pen(color);
    QBrush brush(color);
    painter.setPen(pen);
    painter.setBrush(brush);

    int x = position_.x() - radius;
    int y = position_.y() - radius;
    int w = 2 * radius;
    int h = w;

    int start_angle = (90 -(angle_offset_ + max_angle_) * 180.0 / M_PI) * 16;
    int angle_span = (max_angle_ - min_angle_) * 180.0 / M_PI * 16;

    painter.drawPie(x, y, w, h, start_angle, angle_span);

    // === DRAW LEVER ===
    QPointF end_point;

    end_point.setX(position_.x() + radius * sin(angle_offset_ + current_angle_));
    end_point.setY(position_.y() - radius * cos(angle_offset_ + current_angle_));

    color.setNamedColor("blue");
    pen = QPen(QBrush(color), 2);
    painter.setPen(pen);
    painter.setBrush(QBrush("blue"));

    painter.drawLine(position_, end_point);

    double r = 0.015 * scale_factor;
    pen = QPen(QBrush(QColor("black")), 1);
    painter.setPen(pen);
    painter.setBrush(color);
    painter.drawEllipse(position_, r, r);
    painter.drawEllipse(end_point, r, r);

    tool_point_ = end_point;
}

QPointF AngleTool::getToolPoint()
{
   return tool_point_;
}
