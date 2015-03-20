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

#ifndef LUH_YOUBOT_GUI_ARM_SIDE_VIEW_HPP
#define LUH_YOUBOT_GUI_ARM_SIDE_VIEW_HPP

#include <QVector>
#include <QObject>
#include <QPainter>
#include <QPoint>
#include <QStaticText>
#include <vector>
#include "luh_youbot_kinematics/arm_kinematics.h"
#include "angle_tool.hpp"
#include "vertical_angle_tool.hpp"
#include "cartesian_tool.hpp"
#include "file_reader.hpp"

namespace luh_youbot_gui
{

class ArmSideView : public QObject
{
    Q_OBJECT
public:
    explicit ArmSideView(QObject *parent = 0);

    void paint(QPainter &painter, int width, int height);
    void click(QPoint point);
    void press(QPoint point);
    void release(QPoint point);
    void move(QPoint point);

    void setVirtualJointstate(luh_youbot_kinematics::JointPosition jointstate, double gripper_state);
    void setCurrentJointstate(luh_youbot_kinematics::JointPosition jointstate, double gripper_state);

    void setParameters(FileReader &file_reader);

Q_SIGNALS:
    void jointVelocityCommand(int joint_idx, double vel);
    void cartesianVelocityCommand(double v_r, double v_z, double v_theta);
    void gripperPositionCommand(double width);
    void virtualJointStateChanged(luh_youbot_kinematics::JointPosition jointstate, double gripper_state);
    void updateRequest();

public Q_SLOTS:
    void changeManipulationMode(bool manipulate_virtual);
    void clear();

protected Q_SLOTS:
    void angleToolValueChanged(double angle);
    void vertAngleToolValueChanged(double angle);
    void cartesianToolValueChanged(double dx, double dy);

private:

    QVector<QPointF> current_points_;
    QVector<QPointF> virtual_points_;

    luh_youbot_kinematics::JointPosition current_jointstate_;
    luh_youbot_kinematics::JointPosition virtual_jointstate_;
    double current_gripper_state_;
    double virtual_gripper_state_;

    double scale_factor_;

    bool manipulate_virtual_;

    QPointF mouse_position_;

    AngleTool angle_tool_;
    CartesianTool cartesian_tool_;
    VerticalAngleTool vert_angle_tool_;

    double segment_width_;
    double joint_radius_;
    double gripper_width_;
    double socket_radius_;
    double socket_height_;
    double joint_2_height_over_socket_;
    double l0_, l1_, l2_, l3_, l4_, l5_;
    double gripper_finger_length_;
    double gripper_socket_length_;
    double gripper_socket_width_;
    double gripper_socket_height_;

    QPointF center_point_;
    QPointF side_point_1_;
    QPointF side_point_2_;
    QPointF side_point_3_;
    double side_view_height_;

    double gripper_state_max_, gripper_state_min_, gripper_offset_;

    QStaticText text_top_view_, text_q5_view_, text_gripper_view_;
    double text_height_;

    double line_width_;
    double line_width_scaled_;

    bool virtual_is_not_initialized_;


    enum Joint{NONE, JOINT_1, JOINT_2, JOINT_3, JOINT_4, JOINT_5, ENDEFFECTOR, CARTESIAN, GRIPPER} active_joint_;

    QVector<QPointF> calculatePoints(luh_youbot_kinematics::JointPosition jointstate);
    void setLayout(int width, int height);
    void drawArm(QPainter &painter, QVector<QPointF> points, double alpha);
    void drawSocket(QPainter &painter, double q1);
    void drawGripper(QPainter &painter, QVector<QPointF> points, double q5, double gripper_state, double alpha);
    void drawWorkspaceBoundaries(QPainter &painter);
    void createTool();
    void updateAngleTool();

    void drawTopView(QPainter &painter, QPointF position,
                     luh_youbot_kinematics::JointPosition jointstate, double alpha);
    void drawQ5View(QPainter &painter, QPointF position, luh_youbot_kinematics::JointPosition jointstate, double alpha);
    void drawGripperView(QPainter &painter, QPointF position, double gripper_state, double alpha);

    void jointClicked(int number);
    void endEffectorClicked();
    void jointPressed(int number);
    void endEffectorPressed();
    void toolPressed();
    bool isClicked(QPointF point, QPoint clicked_point, double radius=-1);

};

}
#endif // ARM_SIDE_VIEW_HPP
