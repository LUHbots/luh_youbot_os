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

#include "luh_youbot_gui/cartesian_tool.hpp"
#include <math.h>

CartesianTool::CartesianTool(QObject *parent) :
    QObject(parent)
{
    max_length_ = 0.2;
    arrow_width_ = 0.02;
}

void CartesianTool::paint(QPainter &painter, double scale_factor)
{
    double w = arrow_width_ * scale_factor;
    QPen pen(QBrush(QColor("black")), 1, Qt::SolidLine, Qt::RoundCap, Qt::MiterJoin);
    painter.setPen(pen);
    painter.setBrush(QColor(0, 255, 0));

    double dx = mouse_position_.x() - position_.x();
    double dy = mouse_position_.y() - position_.y();
    double angle = atan2(dy, dx);
    double r = std::min(max_length_ * scale_factor, sqrt(dx*dx + dy*dy));

    double x1 = - w/2;
    double x2 = std::max(x1, r - 2*w);
    double x3 = r;
    double y1 = -w;
    double y2 = -w/2;
    double y3 = 0;
    double y4 = w/2;
    double y5 = w;

    QPolygon arrow;
    arrow.append(QPoint(x1, y2));
    arrow.append(QPoint(x1, y4));
    arrow.append(QPoint(x2, y4));
    arrow.append(QPoint(x2, y5));
    arrow.append(QPoint(x3, y3));
    arrow.append(QPoint(x2, y1));
    arrow.append(QPoint(x2, y2));

    painter.translate(position_);
    painter.rotate(angle * 180/M_PI);

    if(r>w)
        painter.drawPolygon(arrow);

    painter.drawEllipse(QPointF(0,0), 0.7*w, 0.7*w);

    dx /= scale_factor;
    dy /= scale_factor;
    dx = std::min(dx, max_length_);
    dx = std::max(dx, -max_length_);
    dy = std::min(dy, max_length_);
    dy = std::max(dy, -max_length_);

    Q_EMIT valueChanged(dx / max_length_, dy / max_length_);

    painter.resetTransform();
}
