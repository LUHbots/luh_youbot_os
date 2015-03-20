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
 * Author: Simon Aden (simon.aden@mailbox.org)
 ******************************************************************/

#include "luh_youbot_gui/vertical_angle_tool.hpp"
#include <math.h>
#include <iostream>

VerticalAngleTool::VerticalAngleTool(QObject *parent) :
    QObject(parent)
{
}

VerticalAngleTool::VerticalAngleTool(QPointF position, double min_angle, double max_angle, double current_angle, double angle_offset, QObject *parent):
    position_(position),
    min_angle_(min_angle),
    max_angle_(max_angle),
    current_angle_(current_angle),
    angle_offset_(angle_offset),
    is_active_(false),
    QObject(parent)
{
    if(current_angle_ < M_PI) current_angle_ += 2*M_PI;
    if(current_angle_ > M_PI) current_angle_ -= 2*M_PI;
    default_radius_ = 0.14; // TODO: as parameter, scalable?
    shrink_factor_ = 0.2;
    slider_length_ = 0.36;
    slider_pos_ =  (current_angle - min_angle) / (max_angle - min_angle);

}

void VerticalAngleTool::reset(QPointF position, double min_angle, double max_angle, double current_angle, double angle_offset)
{
    if(current_angle_ < M_PI) current_angle_ += 2*M_PI;
    if(current_angle_ > M_PI) current_angle_ -= 2*M_PI;
    position_ = position;
    min_angle_ = min_angle;
    max_angle_ = max_angle;
    current_angle_ = current_angle;
    angle_offset_ = angle_offset;
    default_radius_ = 0.14; // TODO: as parameter, scalable?
    shrink_factor_ = 0.2;
    slider_length_ = 0.36;
    slider_pos_ = slider_pos_ =  (current_angle - min_angle) / (max_angle - min_angle);
    is_active_ = false;
}

//void VerticalAngleTool::mouseClick(QPoint mouse_position)
//{

//}

//void VerticalAngleTool::mousePress(QPoint mouse_position)
//{

//}

//void VerticalAngleTool::mouseRelease(QPoint mouse_position)
//{

//}

void VerticalAngleTool::activate()
{
    is_active_ = true;
}
void VerticalAngleTool::deactivate()
{
    is_active_ = false;
}

void VerticalAngleTool::paint(QPainter &painter, double scale_factor)
{
    // === DRAW ANGLE RANGE (PIE) ===
    double radius = default_radius_ * scale_factor;

    if(is_active_)
    {
        double dx = mouse_position_.x() -position_.x();
        double dy = mouse_position_.y() - position_.y();
        slider_pos_ = dx * cos(angle_offset_) + dy * sin(angle_offset_);
        slider_pos_ /= scale_factor;
        slider_pos_ = std::min(slider_pos_, 0.5 * slider_length_);
        slider_pos_ = std::max(slider_pos_, -0.5 * slider_length_);
        slider_pos_ /= slider_length_;
        slider_pos_ += 0.5;

        current_angle_ = min_angle_ + slider_pos_ * (max_angle_ - min_angle_);
        if(current_angle_ < M_PI) current_angle_ += 2*M_PI;
        if(current_angle_ > M_PI) current_angle_ -= 2*M_PI;
        Q_EMIT angleChanged(current_angle_);
    }


    painter.translate(position_);
    painter.rotate(angle_offset_ * 180.0 / M_PI);


    QColor color("blue");
    QPen pen (QBrush(color), 2);
    painter.setPen(pen);
    painter.setBrush(QBrush("blue"));


    // === DRAW SLIDER ===
    QPointF slider_start(-0.5 * scale_factor * slider_length_, 0);
    QPointF slider_end(0.5 * scale_factor * slider_length_, 0);
    painter.drawLine(slider_start, slider_end);

    double r = 0.015 * scale_factor;
    pen = QPen(QBrush(QColor("black")), 1);
    painter.setPen(pen);
    painter.setBrush(color);
    painter.drawEllipse(QPointF(slider_start.x() + slider_pos_ * slider_length_ * scale_factor ,0), r, r);
    //    painter.drawEllipse(end_point, 0.5*r, 0.5*r);

    tool_point_.setX(position_.x() + (slider_pos_ - 0.5) * cos(angle_offset_) * slider_length_ * scale_factor);
    tool_point_.setY(position_.y() + (slider_pos_ - 0.5) * sin(angle_offset_) * slider_length_ * scale_factor);

    painter.resetTransform();
}

QPointF VerticalAngleTool::getToolPoint()
{
    return tool_point_;
}

void VerticalAngleTool::setCurrentAngle(double current_angle)
{
    current_angle_ = current_angle;

    if(current_angle_ < M_PI) current_angle_ += 2*M_PI;
    if(current_angle_ > M_PI) current_angle_ -= 2*M_PI;


    slider_pos_ = (current_angle_ - min_angle_) / (max_angle_ - min_angle_);
}
