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

#ifndef LUH_YOUBOT_GUI_ARM_CONTROL_WINDOW_HPP
#define LUH_YOUBOT_GUI_ARM_CONTROL_WINDOW_HPP

#include <QCheckBox>
#include <QComboBox>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QListView>
#include <QMenu>
#include <QPushButton>
#include <QRadioButton>
#include <QScrollArea>
#include <QSlider>
#include <QStringListModel>
#include <QTableWidget>
#include <QTabWidget>
#include <QVBoxLayout>
#include <QWidget>

#include "render_area.hpp"
#include "common.hpp"
#include "file_reader.hpp"

namespace luh_youbot_gui
{

class ArmControlWindow : public QWidget
{
    Q_OBJECT
public:
    explicit ArmControlWindow(QWidget *parent = 0);
    RenderArea* render_area(){return render_area_;}
    ~ArmControlWindow();

Q_SIGNALS:
    void logMessage(QString msg);
    void cartesianVelocity(double v_x, double v_y, double v_theta);
    void jointVelocity(int joint_idx, double vel);
    void jointPositionSet(luh_youbot_kinematics::JointPosition pos, double gripper, int mode, bool is_relative);
    void cartesianPositionSet(luh_youbot_kinematics::CartesianPosition pos, double gripper, int mode, bool is_relative);
    void cylindricPositionSet(luh_youbot_kinematics::CylindricPosition pos, double gripper, int mode, bool is_relative);
    void clearModel();    
    void compensateGravityRequested(bool do_compensation);
    void velocitiesChanged(luh_youbot_kinematics::JointVelocity joint_vel,
                           luh_youbot_kinematics::CartesianVelocity cart_vel,
                           luh_youbot_kinematics::CylindricVelocity cyl_vel);

public Q_SLOTS:
    void onRosInitialised();

protected Q_SLOTS:
    void hideTabs(bool hide);
    void changeJointstateCoordinates(int index);
    void changeMovementMode(int mode);
    void onVirtualJointstateChanged(luh_youbot_kinematics::JointPosition jointstate, double gripper_position);
//    void onCurrentJointstateChanged(luh_youbot_kinematics::JointPosition jointstate, double gripper_position);
    void onCurrentJointstateChanged(double q1, double q2, double q3, double q4, double q5, double gripper_state);
    void onRadioButtonPositionToggled(bool checked);
    void onRadioButtonVelocityToggled(bool checked);
    void onButtonGoClicked(bool checked);
    void onTableCellChanged(int row, int col);
    void onPoseSelected(QModelIndex index1, QModelIndex index2);
    void onPoseClicked(QModelIndex index);
    void onPoseNameEdited(QModelIndex index1, QModelIndex index2);
    void onAddPose();
    void onSavePosesAs();
    void onLoadPoses();
    void deleteSelectedPose();
    void renameSelectedPose();
    void onCartesianVelocitySet(double v_x, double v_y, double v_theta);
    void onJointVelocitySet(int joint_idx, double vel);
    void onMaxVelocitiesEdited();
    void onCheckboxRelativeChanged(bool is_relative);
    void onSliderMoved(int value);
    void onButtonGravityCompensationClicked(bool is_checked);
//    void onButtonGravityCompensationReleased();
    void onButtonApplyClicked();

protected:
    QHBoxLayout *main_horizontal_layout_;
    QVBoxLayout *left_vertical_layout_;
    QVBoxLayout *right_vertical_layout_;
    RenderArea *render_area_;
    QTabWidget *toolbox_;
    QWidget *toolbox_page_poses_;
    QWidget *toolbox_page_parameters_;
    QPushButton *button_velocity_;
    QPushButton *button_position_;
    QPushButton *button_go_;
    QHBoxLayout *layout_move_arm_;    
    QTableWidget *table_jointstate_;
    QHBoxLayout *layout_jointstate_;
    QPushButton *button_hide_tabs_;    
    QGridLayout *layout_poses_tab_;
    QListView *list_poses_;
    QStringListModel *model_poses_;
    QPushButton *button_load_poses_;
//    QPushButton *button_save_poses_;
    QPushButton *button_save_poses_as_;
    QLineEdit *edit_add_pose_;
    QPushButton *button_add_pose_;
    QLabel *label_coordinates_;
    QLabel *label_movement_mode_;
    QComboBox *combo_movement_mode_;
    QComboBox *combo_jointstate_coordinates_;
    QLabel *label_coordinates_help_;
    QVBoxLayout *layout_parameters_tab_;
    QGridLayout *layout_position_settings_;
    QGridLayout *layout_velocity_settings_;
    QGroupBox *box_position_settings_;
    QGroupBox *box_velocity_settings_;
    QLabel *label_movement_mode_help_;
    QVector<QLabel*> labelvector_max_velocity_;
    QVector<QLineEdit*> editvector_max_velocity_;
    QVector<QSlider*> slidervector_max_velocity_;
    QLabel *label_relative_pose_;
    QCheckBox *checkbox_relative_pose_;
    QPushButton *button_gravity_comp_;
//    QScrollArea *scroll_area_params_;
//    QVBoxLayout *layout_scroll_area_;
    QPushButton *button_apply_velocities_;

    QString movement_direct_hint_, movement_inter_hint_, movement_plan_hint_;

    luh_youbot_kinematics::JointPosition joint_setpoint_;
    double gripper_setpoint_;
    luh_youbot_kinematics::JointPosition current_jointstate_;
    double current_gripper_state_;

    luh_youbot_kinematics::JointVelocity max_joint_velocity_;
    luh_youbot_kinematics::CylindricVelocity max_cyl_velocity_;
    luh_youbot_kinematics::CartesianVelocity max_cart_velocity_;

    double gripper_state_max_;
    double gripper_state_min_;

    QColor default_text_color_;

    FileReader file_reader_;
    PoseMap predef_poses_;
//    std::vector<PredefinedPose> predef_poses_;

    QString selected_pose_name_;

    bool velocity_is_activated_;

    int debug_counter_;

    bool ros_is_initialised_;

    QVector<double> max_velocity_values_;

    void updateJointstateTable(bool setpoint);
    void updateJointstateTable(double gripper_state, bool setpoint);
    void updateJointstateTable(luh_youbot_kinematics::JointPosition jointstate, bool setpoint);
    void updatePosesList();
    bool showSafetyDialog();

};
}

#endif // ARM_CONTROL_WINDOW_HPP
