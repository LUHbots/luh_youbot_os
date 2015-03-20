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

#include "luh_youbot_gui/arm_side_view.hpp"
#include <QLinearGradient>

using namespace luh_youbot_gui;
//using namespace luh_youbot_kinematics;
namespace ykin = luh_youbot_kinematics;


//########## CONSTRUCTOR ###############################################################################################
ArmSideView::ArmSideView(QObject *parent)
{
    // parameters
    segment_width_ = 0.07;
    joint_radius_ = 0.02;
    gripper_width_ = 0.015;

    socket_radius_ = 0.08;
    socket_height_ = 0.075;
    joint_2_height_over_socket_ = 0.075;

    gripper_finger_length_ = 0.07;
    gripper_socket_length_ = 0.07;
    gripper_socket_height_ = segment_width_;
    gripper_socket_width_ = 0.104;

    l0_ = ykin::L0;
    l0_ = ykin::L0;
    l1_ = ykin::L1;
    l2_ = ykin::L2;
    l3_ = ykin::L3;
    l4_ = ykin::L4 - gripper_finger_length_ - gripper_socket_length_;

    luh_youbot_kinematics::JointPosition home_pos;
//    home_pos.setQ1(luh_youbot_kinematics::JointPosition::MIN_JNT_POSITIONS[0]);
//    home_pos.setQ2(luh_youbot_kinematics::JointPosition::MIN_JNT_POSITIONS[1]);
//    home_pos.setQ3(luh_youbot_kinematics::JointPosition::MAX_JNT_POSITIONS[2]);
//    home_pos.setQ4(luh_youbot_kinematics::JointPosition::MIN_JNT_POSITIONS[3]);
//    home_pos.setQ5(luh_youbot_kinematics::JointPosition::MIN_JNT_POSITIONS[4]);

    virtual_jointstate_ = home_pos;
    current_jointstate_ = home_pos;
    current_gripper_state_ = 0.0;
    virtual_gripper_state_ = 0.0;

    connect(&angle_tool_, SIGNAL(angleChanged(double)), this, SLOT(angleToolValueChanged(double)));
    connect(&cartesian_tool_, SIGNAL(valueChanged(double, double)),
            this, SLOT(cartesianToolValueChanged(double, double)));
    connect(&vert_angle_tool_, SIGNAL(angleChanged(double)),
            this, SLOT(vertAngleToolValueChanged(double)));

    active_joint_ = NONE;
    manipulate_virtual_ = true;

    gripper_state_max_ = 0.06;
    gripper_state_min_ = 0.0;
    gripper_offset_ = 0.015;

    scale_factor_ = 0;
    text_height_ = 0.01;
    text_top_view_.setTextFormat(Qt::RichText);
    text_q5_view_.setTextFormat(Qt::RichText);
    text_gripper_view_.setTextFormat(Qt::RichText);

    line_width_ = 0.005;

    virtual_is_not_initialized_ = true;
}

//########## PAINT #####################################################################################################
void ArmSideView::paint(QPainter &painter, int width, int height)
{
    double old_scale_factor = scale_factor_;
    setLayout(width, height);

    // check if scale changed
    if(scale_factor_ != old_scale_factor)
    {
        int font_size = (int)(text_height_ * scale_factor_);
        QString tag1 = "<font size=\"" + QString::number(font_size) + "\">";
        QString tag2 = "</font>";
        text_top_view_.setText(tag1 + "Joint 1" + tag2);
        text_q5_view_.setText(tag1 + "Joint 5" + tag2);
        text_gripper_view_.setText(tag1 + "Gripper" + tag2);

        line_width_scaled_ = std::max(1.0, (line_width_ * scale_factor_));
    }

    current_points_ = calculatePoints(current_jointstate_);
    virtual_points_ = calculatePoints(virtual_jointstate_);

    drawWorkspaceBoundaries(painter);

    drawArm(painter, virtual_points_, 0.4);
    drawArm(painter, current_points_, 1);

    drawGripper(painter, virtual_points_, virtual_jointstate_.q5(), virtual_gripper_state_,  0.4);
    drawGripper(painter, current_points_, current_jointstate_.q5(), current_gripper_state_, 1);

    if(manipulate_virtual_)
        drawSocket(painter, virtual_jointstate_.q1());
    else
        drawSocket(painter, current_jointstate_.q1());

    if(active_joint_ == CARTESIAN)
    {
        if(manipulate_virtual_)
            cartesian_tool_.setPosition(virtual_points_[6]);
        else
            cartesian_tool_.setPosition(current_points_[6]);
        cartesian_tool_.paint(painter, scale_factor_);
    }
    else if(active_joint_ == JOINT_1 || active_joint_ == JOINT_5 || active_joint_ == GRIPPER)
    {
        updateAngleTool();
        vert_angle_tool_.paint(painter, scale_factor_);
    }
    else if(active_joint_ != NONE)
    {
        updateAngleTool();
        angle_tool_.paint(painter, scale_factor_);
    }


    drawTopView(painter, side_point_1_, current_jointstate_, 1.0);
    drawQ5View(painter, side_point_2_, current_jointstate_, 1.0);
    drawGripperView(painter, side_point_3_, current_gripper_state_, 1.0);

    drawTopView(painter, side_point_1_, virtual_jointstate_, 0.4);
    drawQ5View(painter, side_point_2_, virtual_jointstate_, 0.4);
    drawGripperView(painter, side_point_3_, virtual_gripper_state_, 0.4);
}

//########## SET PARAMETERS ############################################################################################
void ArmSideView::setParameters(FileReader &file_reader)
{
    gripper_state_max_ = file_reader.getDoubleParam("gripper_max");
    gripper_state_min_ = file_reader.getDoubleParam("gripper_min");
    gripper_offset_ = file_reader.getDoubleParam("gripper_offset");
}

//########## SET LAYOUT ################################################################################################
void ArmSideView::setLayout(int width, int height)
{
    double diameter = (2 * (l2_ + l3_ + l4_ + gripper_finger_length_ + gripper_socket_length_));
    double shrink_factor = 0.95;

    if(width > height)
    {
        if(0.75 * width > height)
        {
            scale_factor_ = height * shrink_factor / diameter;
            side_view_height_ = shrink_factor * height / 3.0;
        }
        else
        {
            scale_factor_ = width * shrink_factor * 0.75 / diameter;
            side_view_height_ = shrink_factor * width / 4.0;

        }

        center_point_.setX(width * 0.625);
        center_point_.setY(height * 0.5);

        side_point_1_.setX(width * 0.125);
        side_point_1_.setY(height * 0.16666666);

        side_point_2_.setX(width * 0.125);
        side_point_2_.setY(height * 0.5);

        side_point_3_.setX(width * 0.125);
        side_point_3_.setY(height * 0.83333333);
    }
    else
    {
        if(0.75 * height > width)
        {
            scale_factor_ = width * shrink_factor / diameter;
            side_view_height_ = shrink_factor * width / 3.0;
        }
        else
        {
            scale_factor_ = height * shrink_factor * 0.75 / diameter;
            side_view_height_ = shrink_factor* height / 4.0;
        }

        center_point_.setY(height * 0.625);
        center_point_.setX(width * 0.5);

        side_point_1_.setY(height * 0.125);
        side_point_1_.setX(width * 0.16666666);

        side_point_2_.setY(height * 0.125);
        side_point_2_.setX(width * 0.5);

        side_point_3_.setY(height * 0.125);
        side_point_3_.setX(width * 0.83333333);
    }

}

//########## CALCULATE POINTS ##########################################################################################
QVector<QPointF> ArmSideView::calculatePoints(ykin::JointPosition jointstate)
{
    QVector<QPointF> points;

    // === CALCULATE POINTS ===
    points.resize(7);
    double a1 = jointstate.q2();
    double a2 = a1 + jointstate.q3();
    double a3 = a2 + jointstate.q4();
    points[0].setX(0);
    points[0].setY(0);
    points[1].setX(l1_);
    points[1].setY(l0_);
    points[2].setX(points[1].x() + l2_ * sin(a1));
    points[2].setY(points[1].y() + l2_ * cos(a1));
    points[3].setX(points[2].x() + l3_ * sin(a2));
    points[3].setY(points[2].y() + l3_ * cos(a2));
    points[4].setX(points[3].x() + l4_ * sin(a3));
    points[4].setY(points[3].y() + l4_ * cos(a3));
    points[5].setX(points[4].x() + gripper_socket_length_ * sin(a3));
    points[5].setY(points[4].y() + gripper_socket_length_ * cos(a3));
    points[6].setX(points[5].x() + gripper_finger_length_ * sin(a3));
    points[6].setY(points[5].y() + gripper_finger_length_ * cos(a3));

    // === TRANSFORM POINTS TO PIXEL COORDINATES ===
    for(int i=0; i<points.size(); i++)
    {
        points[i].setX(points[i].x() - l1_);
        points[i].setY(points[i].y() - l0_);

        points[i].rx() *= scale_factor_;
        points[i].ry() *= -scale_factor_;
        points[i].rx() += center_point_.x();
        points[i].ry() += center_point_.y();
    }

    return points;
}

//########## DRAW ARM ##################################################################################################
void ArmSideView::drawArm(QPainter &painter, QVector<QPointF> points, double alpha)
{
    QColor color;
    color.setAlphaF(alpha);

    // === DRAW ARM SEGMENTS ===
    color.setNamedColor("darkorange");
    color.setAlphaF(alpha);
    QPen pen(QBrush(color), segment_width_ * scale_factor_, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
    painter.setPen(pen);
    painter.drawLine(points[0], points[1]);

    QPointF p = points[0];
    p.rx() -= segment_width_ * scale_factor_ * 0.65;
    painter.drawLine(p, points[1]);
    painter.drawLine(points[2], points[3]);

    color.setNamedColor("orange");
    color.setAlphaF(alpha);
    pen.setColor(color);
    painter.setPen(pen);
    painter.drawLine(points[1], points[2]);
    painter.drawLine(points[3], points[4]);

    // === DRAW JOINTS ===
    color.setNamedColor("black");
    color.setAlphaF(alpha);

    painter.setPen(color);
    painter.setBrush(color);
    double radius = joint_radius_ * scale_factor_;
    for(int i=1; i<3; i++)
    {
        painter.drawEllipse(points[i], radius, radius);
    }
}

//########## DRAW SOCKET ###############################################################################################
void ArmSideView::drawSocket(QPainter &painter, double q1)
{
    double x_flare_1 = socket_radius_ * sin(q1);
    double x_flare_2 = socket_radius_ * sin(q1 + M_PI);
    double y_flare_1 = socket_radius_ * cos(q1);
    double y_flare_2 = socket_radius_ * cos(q1 + M_PI);
    if(y_flare_1 < 0)
        if(x_flare_1 > 0)
            x_flare_1 = 2 * socket_radius_ - x_flare_1;
        else
            x_flare_1 = -2 * socket_radius_ - x_flare_1;
    if(y_flare_2 < 0)
        if(x_flare_2 > 0)
            x_flare_2 = 2 * socket_radius_ - x_flare_2;
        else
            x_flare_2 = -2 * socket_radius_ - x_flare_2;

    QLinearGradient gradient(-2*socket_radius_ * scale_factor_, 0, 2*socket_radius_*scale_factor_, 0);
    x_flare_1 = x_flare_1 / (4 *socket_radius_) + 0.5;
    x_flare_2 = x_flare_2 / (4 *socket_radius_) + 0.5;
    gradient.setColorAt(x_flare_1, QColor(100, 100, 100));
    gradient.setColorAt(x_flare_2, QColor(100, 100, 100));
    double offset = 0.15;
    gradient.setColorAt(std::max(x_flare_1 - offset, 0.0), QColor("black"));
    gradient.setColorAt(std::max(x_flare_2 - offset, 0.0), QColor("black"));
    gradient.setColorAt(std::min(x_flare_1 + offset, 1.0), QColor("black"));
    gradient.setColorAt(std::min(x_flare_2 + offset, 1.0), QColor("black"));
    QBrush brush(gradient);

    // === DRAW SOCKET ===
    QPointF p1, p2;
    QColor color("black");
    QPen pen(brush, 0, Qt::SolidLine, Qt::FlatCap, Qt::BevelJoin);
//    QPen pen;
    painter.setPen(pen);
    painter.setBrush(brush);
    painter.translate(current_points_[0].x(), current_points_[1].y());
//    p1.setX(current_points_[0].x() - socket_radius_ * scale_factor_);
//    p1.setY(current_points_[1].y() + joint_2_height_over_socket_ * scale_factor_);
//    p2.setX(p1.x() + 2*socket_radius_ * scale_factor_);
//    p2.setY(p1.y() + socket_height_ * scale_factor_);
    p1.setX(-socket_radius_ * scale_factor_);
    p1.setY(joint_2_height_over_socket_ * scale_factor_);
    p2.setX(socket_radius_ * scale_factor_);
    p2.setY(p1.y() + socket_height_ * scale_factor_);
    painter.drawRect(QRectF(p1, p2));

    p2.setY(p1.y());
    color.setNamedColor("orange");
    pen = QPen(QBrush(color), 0.2 * socket_height_ * scale_factor_, Qt::SolidLine, Qt::SquareCap, Qt::RoundJoin);
    painter.setPen(pen);
    painter.drawLine(p1, p2);

    painter.resetTransform();
}

//########## DRAW GRIPPER ##############################################################################################
void ArmSideView::drawGripper(QPainter &painter, QVector<QPointF> points, double q5, double gripper_state, double alpha)
{
    gripper_state += gripper_offset_;

    // "normalize" angle to the needed range
    q5 += M_PI / 2;
    while(q5 > M_PI / 2)
        q5 -= M_PI;
    while(q5 < -M_PI / 2)
        q5 += M_PI;

    double edge_x = 1;
    if(q5 < 0)
    {
        edge_x = -1;
        q5 *= -1;
    }
    edge_x *= 0.5 * gripper_socket_width_ * cos(q5) - 0.5 * gripper_socket_height_ * sin(q5);

    // === DRAW GRIPPER SOCKET ===
    double phi = q5 - atan(gripper_socket_height_ / gripper_socket_width_);

    double width = sqrt(gripper_socket_height_ * gripper_socket_height_
                        + gripper_socket_width_ * gripper_socket_width_) * cos(phi);


    QColor color("black");
    color.setAlphaF(alpha);
    QPen pen(QBrush(color), width * scale_factor_, Qt::SolidLine, Qt::FlatCap, Qt::BevelJoin);
    painter.setPen(pen);
    painter.drawLine(points[4], points[5]);

    // === DRAW GRIPPER FINGERS ===
    double dx = points[6].x() - points[5].x();
    double dy = points[6].y() - points[5].y();
    double angle = atan2(dy, dx) * 180.0 / M_PI;

    phi = q5 - M_PI / 4;
    width = sqrt(2) * gripper_width_ * cos(phi);

    QPointF p1(0, 0);
    QPointF p2(0, gripper_finger_length_ * scale_factor_);
    pen.setWidthF(width * scale_factor_);
    painter.setPen(pen);
    painter.translate(points[5]);
    painter.rotate(angle - 90);
    double s = (gripper_state + gripper_width_) * cos(q5) * scale_factor_;
    painter.translate(s / 2, 0);
    painter.drawLine(p1, p2);
    painter.translate(-s, 0);
    painter.drawLine(p1, p2);

    if(alpha == 1)
    {
        color.setNamedColor("gray");
    }
    else
    {
        color.setNamedColor("black");
        color.setAlphaF(0.5);
    }
    pen.setColor(color);
    pen.setWidth(1);
    painter.setPen(pen);
    painter.translate(s/2, 0);
    p1.setX(edge_x * scale_factor_);
    p2.setX(p1.x());
    p1.setY(-gripper_socket_length_ * scale_factor_);
    p2.setY(0);
    painter.drawLine(p1, p2);

    painter.resetTransform();
}

//########## DRAW WORKSPACE BOUNDARIES #################################################################################
void ArmSideView::drawWorkspaceBoundaries(QPainter &painter)
{
    QPen pen(QBrush("white"), line_width_scaled_, Qt::DashDotLine);
    pen.setWidth(1);

    painter.setPen(pen);
//    painter.setBrush(QColor("lightGray"));
    painter.setBrush(QBrush());
    double radius = scale_factor_ * (l2_ + l3_ + l4_ + gripper_finger_length_ + gripper_socket_length_);
    painter.drawEllipse(current_points_[1], radius, radius);
}

//########## MOUSE CLICK ###############################################################################################
void ArmSideView::click(QPoint point)
{
    // check side views
    if(isClicked(side_point_1_, point, 0.5 * side_view_height_ / scale_factor_))
    {
        jointClicked(1);
        return;
    }
    else if(isClicked(side_point_2_, point, 0.5 * side_view_height_ / scale_factor_))
    {
        jointClicked(5);
        return;
    }
    else if(isClicked(side_point_3_, point, 0.5 * side_view_height_ / scale_factor_))
    {
        jointClicked(6);
        return;
    }

    // check joints
    QPointF ref_point;
    for(int i=0; i<6; i++)
    {
        if(manipulate_virtual_)
            ref_point = virtual_points_[i];
        else
            ref_point = current_points_[i];

        if(isClicked(ref_point, point))
        {
            jointClicked(i+1);
            return;
        }
    }

    // check endeffector
    if(manipulate_virtual_)
        ref_point = virtual_points_[6];
    else
        ref_point = current_points_[6];

    if(isClicked(ref_point, point))
        endEffectorClicked();
    else
        active_joint_ = NONE;
}

//########## MOUSE PRESSED #############################################################################################
void ArmSideView::press(QPoint point)
{
    mouse_position_ = point;
    angle_tool_.setMousePosition(mouse_position_);
    vert_angle_tool_.setMousePosition(mouse_position_);
    cartesian_tool_.setMousePosition(mouse_position_);

    // check tool point
    if(active_joint_ != NONE)
    {
        QPointF tool_point;
        if(active_joint_ == JOINT_1 || active_joint_ == JOINT_5 || active_joint_ == GRIPPER)
            tool_point = vert_angle_tool_.getToolPoint();
        else
            tool_point = angle_tool_.getToolPoint();

        if(isClicked(tool_point, point))
        {
            toolPressed();
            return;
        }
    }

    // check joints
    QPointF ref_point;
    for(int i=0; i<6; i++)
    {
        if(manipulate_virtual_)
            ref_point = virtual_points_[i];
        else
            ref_point = current_points_[i];

        if(isClicked(ref_point, point))
        {
            jointPressed(i+1);
            return;
        }
    }

    // check endeffector
    if(manipulate_virtual_)
        ref_point = virtual_points_[6];
    else
        ref_point = current_points_[6];

    if(isClicked(ref_point, point))
        endEffectorPressed();
    else
        active_joint_ = NONE;
}


//########## MOUSE RELEASED ############################################################################################
void ArmSideView::release(QPoint point)
{
    //    active_joint_ = NONE;
//    if(active_joint_ == CARTESIAN)
//        active_joint_ = NONE;
//    else
//    {
//        if(!manipulate_virtual_)
//        {
//            angle_tool_.setCurrentAngle(0);
//            vert_angle_tool_.setCurrentAngle(0);
//        }
//        angle_tool_.deactivate();
//        vert_angle_tool_.deactivate();
//    }

    ykin::JointPosition jointstate;
    double gripper_state;
    if(manipulate_virtual_)
    {
        jointstate = virtual_jointstate_;
        gripper_state = virtual_gripper_state_;
    }
    else
    {
        jointstate = current_jointstate_;
        gripper_state = current_gripper_state_;

        Q_EMIT jointVelocityCommand(0, 0.0);
    }

    switch(active_joint_)
    {
    case JOINT_1:
    {
        vert_angle_tool_.deactivate();
        vert_angle_tool_.setCurrentAngle(jointstate.q1());
        break;
    }
    case JOINT_2:
    {
        angle_tool_.deactivate();
        angle_tool_.setCurrentAngle(jointstate.q2());
        break;
    }
    case JOINT_3:
    {
        angle_tool_.deactivate();
        angle_tool_.setCurrentAngle(jointstate.q3());
        break;
    }
    case JOINT_4:
    {
        angle_tool_.deactivate();
        angle_tool_.setCurrentAngle(jointstate.q4());
        break;
    }
    case JOINT_5:
    {
        vert_angle_tool_.deactivate();
        vert_angle_tool_.setCurrentAngle(jointstate.q5());
        break;
    }
    case ENDEFFECTOR:
    {
        angle_tool_.deactivate();
        angle_tool_.setCurrentAngle(0);
        break;
    }
    case CARTESIAN:
    {
        active_joint_ = NONE;
        break;
    }
    case GRIPPER:
    {
//        if(!manipulate_virtual_)
//            Q_EMIT gripperPositionCommand(vert_angle_tool_.getCurrentValue());

        vert_angle_tool_.deactivate();
        vert_angle_tool_.setCurrentAngle(gripper_state);

        break;
    }
    case NONE:
    {
        break;
    }
    }
}

//########## MOUSE MOVED ###############################################################################################
void ArmSideView::move(QPoint point)
{
    mouse_position_ = point;
    angle_tool_.setMousePosition(mouse_position_);
    vert_angle_tool_.setMousePosition(mouse_position_);
    cartesian_tool_.setMousePosition(mouse_position_);
}

//########## JOINT CLICKED #############################################################################################
void ArmSideView::jointClicked(int number)
{
    if(number == 1)
        active_joint_ = JOINT_1;
    if(number == 2)
        active_joint_ = JOINT_2;
    if(number == 3)
        active_joint_ = JOINT_3;
    if(number == 4)
        active_joint_ = JOINT_4;
    if(number == 5)
        active_joint_ = JOINT_5;
    if(number == 6)
        active_joint_ = GRIPPER;

    createTool();
}

//########## ENDEFFECTOR CLICKED #######################################################################################
void ArmSideView::endEffectorClicked()
{
    active_joint_ = ENDEFFECTOR;
    createTool();
}

//########## JOINT PRESSED #############################################################################################
void ArmSideView::jointPressed(int number)
{
    //    std::cout << "Joint " << number << " pressed." << std::endl;
    if(number == 1)
        active_joint_ = JOINT_1;
    if(number == 2)
        active_joint_ = JOINT_2;
    if(number == 3)
        active_joint_ = JOINT_3;
    if(number == 4)
        active_joint_ = JOINT_4;
    if(number == 5)
        active_joint_ = JOINT_5;
    if(number == 6)
        active_joint_ = GRIPPER;

    createTool();
    //    angle_tool_.activate();
}

//########## ENDEFFECTOR PRESSED #######################################################################################
void ArmSideView::endEffectorPressed()
{
    active_joint_ = CARTESIAN;

    if(manipulate_virtual_)
    {
        cartesian_tool_.setPosition(virtual_points_[6]);
        cartesian_tool_.setMousePosition(virtual_points_[6]);
    }
    else
    {
        cartesian_tool_.setPosition(current_points_[6]);
        cartesian_tool_.setMousePosition(current_points_[6]);
    }
}

//########## TOOL PRESSED ##############################################################################################
void ArmSideView::toolPressed()
{    
    if(active_joint_ == JOINT_1 || active_joint_ == JOINT_5 || active_joint_ == GRIPPER)
        vert_angle_tool_.activate();
    else
        angle_tool_.activate();
}
//########## IS CLICKED ################################################################################################
bool ArmSideView::isClicked(QPointF point, QPoint clicked_point, double radius)
{
    double radius_factor = 1.8;
    double dx, dy;

    dx = point.x() - clicked_point.x();
    dy = point.y() - clicked_point.y();

    double r = sqrt(dx*dx + dy*dy) / scale_factor_;
    if(radius == -1)
        return r < joint_radius_ * radius_factor;

    else return r < radius;
}

//########## CREATE TOOL ###############################################################################################
void ArmSideView::createTool()
{
    double angle_offset, current_angle, angle_max, angle_min;
    QPointF joint_position;

    ykin::JointPosition jointstate;
    double gripper_state;
    QVector<QPointF> points;
    if(manipulate_virtual_)
    {
        jointstate = virtual_jointstate_;
        points = virtual_points_;
        gripper_state = virtual_gripper_state_;
    }
    else
    {
        jointstate = current_jointstate_;
        points = current_points_;
        gripper_state = current_gripper_state_;
    }

    bool vertical = false;
    if(active_joint_ == JOINT_1)
    {
        angle_offset = 0;
        current_angle = jointstate.q1();
        angle_max = ykin::MAX_JNT_POSITIONS[0];
        angle_min = ykin::MIN_JNT_POSITIONS[0];
        joint_position = points[0];
        vertical = true;
    }
    else if(active_joint_ == JOINT_2)
    {
        angle_offset = 0;
        current_angle = jointstate.q2();
        angle_max = ykin::MAX_JNT_POSITIONS[1];
        angle_min = ykin::MIN_JNT_POSITIONS[1];
        joint_position = points[1];
    }
    else if(active_joint_ == JOINT_3)
    {
        angle_offset = jointstate.q2();
        current_angle = jointstate.q3();
        angle_max = ykin::MAX_JNT_POSITIONS[2];
        angle_min = ykin::MIN_JNT_POSITIONS[2];
        joint_position = points[2];
    }
    else if(active_joint_ == JOINT_4)
    {
        angle_offset = jointstate.q2() + jointstate.q3();
        current_angle = jointstate.q4();
        angle_max = ykin::MAX_JNT_POSITIONS[3];
        angle_min = ykin::MIN_JNT_POSITIONS[3];
        joint_position = points[3];
    }
    else if(active_joint_ == JOINT_5)
    {
        angle_offset = jointstate.q2() + jointstate.q3() + jointstate.q4();
        current_angle = jointstate.q5();
        angle_max = ykin::MAX_JNT_POSITIONS[4];
        angle_min = ykin::MIN_JNT_POSITIONS[4];
        joint_position = points[4];
        vertical = true;
    }
    else if(active_joint_ == GRIPPER)
    {
        angle_offset = jointstate.q2() + jointstate.q3() + jointstate.q4();
        current_angle = gripper_state;
        angle_max = gripper_state_max_;
        angle_min = gripper_state_min_;
        joint_position = points[5];
        vertical = true;
    }
    else if(active_joint_ == ENDEFFECTOR)
    {
        angle_offset = jointstate.q2() + jointstate.q3() + jointstate.q4() - M_PI;
        current_angle = 0;
        angle_max = M_PI/2;
        angle_min = -M_PI/2;
        joint_position = points[6];
    }
    else if(active_joint_ == NONE)
        return;

    if(vertical)
    {
        vert_angle_tool_.reset(joint_position, angle_min, angle_max,current_angle, angle_offset);
        vert_angle_tool_.setMousePosition(joint_position);
        if(active_joint_ == JOINT_1)
            vert_angle_tool_.setSliderLength(0.6);
    }
    else
    {
        angle_tool_.reset(joint_position, angle_min, angle_max,current_angle, angle_offset);
        angle_tool_.setMousePosition(joint_position);
    }
}

//########## UPDATE ANGLE TOOL #########################################################################################
void ArmSideView::updateAngleTool()
{
    double angle_offset, current_angle;
    QPointF joint_position;

    ykin::JointPosition jointstate;
    double gripper_state;
    QVector<QPointF> points;
    if(manipulate_virtual_)
    {
        jointstate = virtual_jointstate_;
        points = virtual_points_;
        gripper_state = virtual_gripper_state_;
    }
    else
    {
        jointstate = current_jointstate_;
        points = current_points_;
        gripper_state = current_gripper_state_;
    }

    bool vertical = false;
    if(active_joint_ == JOINT_1)
    {
        angle_offset = 0;
        current_angle = jointstate.q1();
        joint_position = points[0];
        vertical = true;
    }
    else if(active_joint_ == JOINT_2)
    {
        angle_offset = 0;
        current_angle = jointstate.q2();
        joint_position = points[1];
    }
    else if(active_joint_ == JOINT_3)
    {
        angle_offset = jointstate.q2();
        current_angle = jointstate.q3();
        joint_position = points[2];
    }
    else if(active_joint_ == JOINT_4)
    {
        angle_offset = jointstate.q2() + jointstate.q3();
        current_angle = jointstate.q4();
        joint_position = points[3];
    }
    else if(active_joint_ == JOINT_5)
    {
        angle_offset = jointstate.q2() + jointstate.q3() + jointstate.q4();
        current_angle = jointstate.q5();
        joint_position = points[4];
        vertical = true;
    }
    else if(active_joint_ == GRIPPER)
    {
        angle_offset = jointstate.q2() + jointstate.q3() + jointstate.q4();
        current_angle = gripper_state;
        joint_position = points[5];
        vertical = true;
    }
    else if(active_joint_ == ENDEFFECTOR)
    {
        angle_offset = jointstate.q2() + jointstate.q3() + jointstate.q4() - M_PI;
        current_angle = 0;
        joint_position = points[6];
    }
    else if(active_joint_ == NONE)
        return;

    if(vertical)
    {
        vert_angle_tool_.setAngleOffset(angle_offset);
        vert_angle_tool_.setPosition(joint_position);
    }
    else
    {
        angle_tool_.setAngleOffset(angle_offset);
        angle_tool_.setPosition(joint_position);
    }

}

//########## ANGLE TOOL VALUE CHANGED ##################################################################################
void ArmSideView::angleToolValueChanged(double angle)
{
    int joint_idx = 0;
    double max_angle = M_PI/2;
//    if(!manipulate_virtual_)
//    {
//        angle = std::max(angle, -max_angle);
//        angle = std::min(angle, max_angle);
//        angle /= max_angle;
//    }

    if(active_joint_ == JOINT_2)
    {
        if(manipulate_virtual_)
            virtual_jointstate_.setQ2(angle);
        else
        {
            joint_idx = 1;
            angle = (angle - current_jointstate_.q2())/max_angle;
        }
    }
    else if(active_joint_ == JOINT_3)
    {
        if(manipulate_virtual_)
            virtual_jointstate_.setQ3(angle);
        else
        {
            joint_idx = 2;
            angle = (angle - current_jointstate_.q3())/max_angle;
        }
    }
    else if(active_joint_ == JOINT_4)
    {
        if(manipulate_virtual_)
            virtual_jointstate_.setQ4(angle);
        else
        {
            joint_idx = 3;
            angle = (angle - current_jointstate_.q4())/max_angle;
        }
    }
    else if(active_joint_ == ENDEFFECTOR)
    {
        if(manipulate_virtual_)
        {
            ykin::CylindricPosition cyl_pos = virtual_jointstate_.toCylindric();
            cyl_pos.setTheta(cyl_pos.theta() + angle);
            ykin::JointPosition jnt_pos = cyl_pos.toJointspace(virtual_jointstate_);
            if(cyl_pos.isReachable() && jnt_pos.isReachable())
                virtual_jointstate_ = jnt_pos;
        }
        else
        {
            angle = std::min(1.0, std::max(-1.0, angle / max_angle));
            Q_EMIT cartesianVelocityCommand(0, 0, angle);
            return;
        }
    }

    angle = std::min(1.0, std::max(-1.0, angle));

    if(active_joint_ != NONE)
    {
        if(manipulate_virtual_)
            Q_EMIT virtualJointStateChanged(virtual_jointstate_, virtual_gripper_state_);
        else
            Q_EMIT jointVelocityCommand(joint_idx, angle);
    }
}

//########## VERTICAL ANGLE TOOL VALUE CHANGED #########################################################################
void ArmSideView::vertAngleToolValueChanged(double angle)
{
    int joint_idx = 0;
    double max_angle = M_PI/2;
    double scaled_angle = angle;
//    if(!manipulate_virtual_)
//    {
//        scaled_angle = std::max(angle, -max_angle);
//        scaled_angle = std::min(angle, max_angle);
//        scaled_angle /= max_angle;
//    }


    if(active_joint_ == JOINT_1)
    {
        if(manipulate_virtual_)
            virtual_jointstate_.setQ1(angle);
        else
        {
            joint_idx = 0;
            scaled_angle = (angle - current_jointstate_.q1()) / max_angle;
        }

    }
    else if(active_joint_ == JOINT_5)
    {
        if(manipulate_virtual_)
            virtual_jointstate_.setQ5(angle);
        else
        {
            joint_idx = 4;
            scaled_angle = (angle - current_jointstate_.q5()) / max_angle;
        }
    }
    else if(active_joint_ == GRIPPER)
    {
        if(manipulate_virtual_)
            virtual_gripper_state_ = angle;
        else
        {
            Q_EMIT gripperPositionCommand(angle);
            return;
        }

    }

    scaled_angle = std::min(1.0, std::max(-1.0, scaled_angle));

    if(active_joint_ != NONE )
    {
        if(manipulate_virtual_)
            Q_EMIT virtualJointStateChanged(virtual_jointstate_, virtual_gripper_state_);
        else
            Q_EMIT jointVelocityCommand(joint_idx, scaled_angle);
    }

}

//########## CARTESIAN TOOL VALUE CHANGED ##############################################################################
void ArmSideView::cartesianToolValueChanged(double dx, double dy)
{
    if(manipulate_virtual_)
    {
        ykin::CylindricPosition cyl_pos = virtual_jointstate_.toCylindric();
        cyl_pos.setR((mouse_position_.x() - virtual_points_[0].x()) / scale_factor_);
        cyl_pos.setZ((-mouse_position_.y() + virtual_points_[0].y()) / scale_factor_);
        ykin::JointPosition jnt_pos = cyl_pos.toJointspace(virtual_jointstate_);
        if(cyl_pos.isReachable() && jnt_pos.isReachable())
            virtual_jointstate_ = jnt_pos;

        if(active_joint_ != NONE)
            Q_EMIT virtualJointStateChanged(virtual_jointstate_, virtual_gripper_state_);
    }
    else
    {
        Q_EMIT cartesianVelocityCommand(dx, -dy, 0);
    }
}

//########## DRAW TOP VIEW #############################################################################################
void ArmSideView::drawTopView(QPainter &painter, QPointF position,
                              luh_youbot_kinematics::JointPosition jointstate, double alpha)
{
    double scale = side_view_height_ / (2 * (l1_ + l2_ + l3_ + l4_ + gripper_socket_length_ + gripper_finger_length_));
    double width = 0.05;

    double w2 = width / 2 * scale;

    double x1 = -w2;
    double x2 = 0;
    double x3 = w2;

    double phi = jointstate.q2();
    double y1 = 0;
    double y2 = y1 -l1_ * scale;
    double y3 = y2 -l2_ * scale * sin(phi);
    phi += jointstate.q3();
    double y4 = y3 -l3_ * scale * sin(phi);
    phi += jointstate.q4();
    double y5 = y4 -l4_ * scale * sin(phi);
    double y6 = y5 - gripper_socket_length_ * scale * sin(phi);
    double y7 = y6 - gripper_finger_length_ * scale * sin(phi);

    QPointF p1(x1, y1);
    QPointF p2(x1, y2);
    QPointF p3(x3, y2);
    QPointF p4(x3, y3);
    QPointF p5(x1, y3);
    QPointF p6(x1, y4);
    QPointF p7(x3, y4);
    QPointF p8(x3, y5);
    QPointF p9(x2, y5);
    QPointF p10(x2, y6);
    QPointF p11(x2, y7);

    painter.translate(position);

    // draw text
    painter.setBrush(QBrush());
    QColor color("white");
    QPen pen(color);
    pen.setWidthF(line_width_scaled_);
    painter.setPen(pen);
    painter.drawStaticText(-(int)side_view_height_/2, -(int)side_view_height_/2, text_top_view_);

    color.setAlphaF(alpha);
    pen.setColor(color);    
    pen.setStyle(Qt::DashDotLine);
    pen.setWidth(1);
    painter.setPen(pen);
    double radius = (l1_ + l2_ + l3_ + l4_ + gripper_socket_length_ + gripper_finger_length_) * scale;
    int angle_start = (90 - (ykin::MAX_JNT_POSITIONS[0] * 180/M_PI)) * 16;
    int angle_span = (ykin::MAX_JNT_POSITIONS[0] - ykin::MIN_JNT_POSITIONS[0]) * 180/M_PI * 16;
    if(alpha == 1)
        painter.drawPie(-radius, -radius, 2*radius, 2*radius, angle_start, angle_span);

    painter.rotate(jointstate.q1() * 180.0/M_PI);

    pen.setStyle(Qt::SolidLine);
    pen.setWidthF(line_width_scaled_);
    painter.setPen(pen);
    painter.drawLine(p1, p2);
    painter.drawLine(p3, p4);
    painter.drawLine(p5, p6);
    painter.drawLine(p7, p8);
    painter.drawLine(p8, p9);
    painter.drawLine(p10, p11);    

    double sradius = socket_radius_ * scale;
    if(alpha == 1)
        painter.drawEllipse(QPointF(0,0), sradius, sradius);

    pen.setStyle(Qt::DotLine);
    painter.setPen(pen);
    painter.drawLine(QPointF(0.0,0.0), QPointF(0.0, -radius));

    pen.setStyle(Qt::SolidLine);
    pen.setWidthF(3 * line_width_scaled_);
    painter.setPen(pen);
    painter.drawLine(p2, p3);
    painter.drawLine(p4, p5);
    painter.drawLine(p6, p7);
    painter.drawLine(p9,p10);

    painter.resetTransform();
}

//########## DRAW Q5 VIEW ##############################################################################################
void ArmSideView::drawQ5View(QPainter &painter, QPointF position,
                             luh_youbot_kinematics::JointPosition jointstate, double alpha)
{
    double shrink_factor = 0.8;
    double scale = shrink_factor *side_view_height_ / sqrt(gripper_socket_width_*gripper_socket_width_ +
                                                           gripper_socket_height_ * gripper_socket_height_);
    double w = gripper_socket_width_ * scale;
    double h = gripper_socket_height_ * scale;

    double reduction_ratio = 0.65;

    double dx = w/2;
    double dy = (1-reduction_ratio) * h / 2;

    double a = M_PI / 2 - atan(dy/dx);
    double b = M_PI - 2 * a;

    double r = sqrt(dx*dx + dy*dy) * sin(a) / sin(b);

    painter.translate(position);
    painter.setBrush(QBrush());
    QColor color("white");
    QPen pen(color);
    pen.setWidthF(line_width_scaled_);
    painter.setPen(pen);
    painter.drawStaticText(-(int)side_view_height_/2, -(int)side_view_height_/2, text_q5_view_);

    color.setAlphaF(alpha);
    pen.setColor(color);
    pen.setStyle(Qt::DashDotLine);
    pen.setWidth(1);
    painter.setPen(pen);

    double l = 0.8 * side_view_height_ / 2;

    painter.setPen(pen);
    if(alpha == 1)
    {
        painter.drawLine(0, l, 0, -l);
        painter.drawLine(-l, 0, l, 0);
    }
//    int angle_start = (90 - (JointVector::MAX_JNT_POSITIONS[4] * 180/M_PI)) * 16;
//    int angle_span = (JointVector::MAX_JNT_POSITIONS[4] - JointVector::MIN_JNT_POSITIONS[4]) * 180/M_PI * 16;
//    painter.drawPie(-l, -l, 2*l, 2*l, angle_start, angle_span);

    pen.setStyle(Qt::SolidLine);
    pen.setWidth(line_width_scaled_);
    painter.setPen(pen);

    if(alpha == 1)
        painter.drawEllipse(QPointF(0,0), joint_radius_ * scale, joint_radius_ * scale);

    painter.rotate(-jointstate.q5() * 180/M_PI);

    double beta_degree = b * 180/M_PI;
    painter.drawArc(-r, -h/2 , 2*r, 2*r, (90 - beta_degree) * 16, 32 * beta_degree);
    painter.drawArc(-r, h/2 - 2*r, 2*r, 2*r, (270 - beta_degree) * 16, 32 * beta_degree);

    QPointF p1(-w/2, -h/2 * reduction_ratio);
    QPointF p2(-w/2,  h/2 * reduction_ratio);
    QPointF p3( w/2,  h/2 * reduction_ratio);
    QPointF p4( w/2, -h/2 * reduction_ratio);
    painter.drawLine(p1, p2);
    painter.drawLine(p3, p4);

    painter.translate(0, -h/2);
    double rect_factor_x = 0.15;
    double rect_factor_y = 0.05;
    painter.drawRect(-rect_factor_x*w, -rect_factor_y*h, 2*rect_factor_x*w, 2*rect_factor_y*h);

    painter.resetTransform();
}

//########## DRAW GRIPPER VIEW #########################################################################################
void ArmSideView::drawGripperView(QPainter &painter, QPointF position, double gripper_state, double alpha)
{
    gripper_state += gripper_offset_;

    double scale = 0.7 * side_view_height_ / (1.5 * gripper_finger_length_);

    QRect rect(-0.5 * gripper_width_ * scale, -0.75 * gripper_finger_length_ * scale,
               gripper_width_ * scale, gripper_finger_length_ * scale);

    painter.translate(position);

    painter.setBrush(QBrush());
    QColor color("white");
    QPen pen(color);
    painter.setPen(pen);

    painter.drawStaticText(-(int)side_view_height_/2, -(int)side_view_height_/2, text_gripper_view_);

    color.setAlphaF(alpha);
    pen.setColor(color);    
    pen.setStyle(Qt::DashDotLine);
    pen.setWidth(1);
    painter.setPen(pen);

    double l = 0.8 * side_view_height_ /2;
    if(alpha == 1)
        painter.drawLine(0, - l, 0, l);

    pen.setStyle(Qt::SolidLine);
    pen.setWidthF(line_width_scaled_);
    painter.setPen(pen);
    if(alpha == 1)
        painter.drawRect(-0.5 * gripper_socket_width_ * scale, 0.25 * gripper_finger_length_ * scale,
                         gripper_socket_width_ * scale, 0.5 * gripper_finger_length_ * scale);

    painter.translate((gripper_state + gripper_width_) * scale / 2, 0);
    painter.drawRect(rect);
    painter.translate(-(gripper_state + gripper_width_) * scale, 0);
    painter.drawRect(rect);

    painter.resetTransform();
}

//########## SLOT: CHANGE MANIPULATION MODE ############################################################################
void ArmSideView::changeManipulationMode(bool manipulate_virtual)
{
    virtual_jointstate_ = current_jointstate_;
    virtual_gripper_state_ = current_gripper_state_;
    virtual_points_ = current_points_;

    manipulate_virtual_ = manipulate_virtual;

    active_joint_ = NONE;

    Q_EMIT virtualJointStateChanged(virtual_jointstate_, virtual_gripper_state_);
    Q_EMIT updateRequest();
}

//########## SET VIRTUAL JOINTSTATE ####################################################################################
void ArmSideView::setVirtualJointstate(luh_youbot_kinematics::JointPosition jointstate, double gripper_state)
{
    virtual_jointstate_ = jointstate;
    virtual_gripper_state_ = gripper_state;

    Q_EMIT updateRequest();
}

//########## SET CURRENT JOINTSTATE ####################################################################################
void ArmSideView::setCurrentJointstate(luh_youbot_kinematics::JointPosition jointstate, double gripper_state)
{
    current_jointstate_ = jointstate;
    current_gripper_state_ = gripper_state;

    if(!manipulate_virtual_ || virtual_is_not_initialized_)
    {
        virtual_jointstate_ = current_jointstate_;
        virtual_gripper_state_ = current_gripper_state_;

        Q_EMIT virtualJointStateChanged(virtual_jointstate_, virtual_gripper_state_);
    }

    if(virtual_is_not_initialized_)
        virtual_is_not_initialized_ = false;

//    Q_EMIT updateRequest();
}

//########## SLOT: CLEAR ###############################################################################################
void ArmSideView::clear()
{
    active_joint_ = NONE;

    Q_EMIT updateRequest();
}

