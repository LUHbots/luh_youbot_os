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

#include "luh_youbot_gui/arm_control_window.hpp"
#include <QFileDialog>
#include <QHeaderView>
#include <QMessageBox>
#include <QTime>
#include <ros/package.h>
#include <QSpacerItem>


using namespace luh_youbot_gui;

ArmControlWindow::ArmControlWindow(QWidget *parent) :
    QWidget(parent)
{
    debug_counter_ = 0;

    // === PARAMETERS ===
    std::string param_file = ros::package::getPath("luh_youbot_gui");
    param_file.append("/cfg/params.yaml");
    if(!file_reader_.loadParameters(param_file))
    {
        std::cout << "Failed to read parameter file." << std::endl;
        Q_EMIT logMessage("Unable to read parameter file " + QString::fromStdString(param_file));
        gripper_state_max_ = 0.06;
        gripper_state_min_ = 0.00;
    }
    else
    {
        gripper_state_min_ = file_reader_.getDoubleParam("gripper_min");
        gripper_state_max_ = file_reader_.getDoubleParam("gripper_max");
    }

    std::string poses_file = ros::package::getPath("luh_youbot_poses");
    if(!poses_file.empty())
    {
        poses_file.append("/poses/poses.yaml");
        file_reader_.setPosesFile(poses_file);
        if(!file_reader_.loadPoses(predef_poses_))
            Q_EMIT logMessage("Unable to read pose file " + QString::fromStdString(poses_file));
    }
    ros_is_initialised_ = false;

    Q_EMIT logMessage("Loaded "+ QString::number(predef_poses_.size()) + " predefined poses.");

    movement_direct_hint_ = QString("Direct:\n")
            + "All joints are independently controlled to\n"
            + "reach the goal position as fast as possible.";
    movement_inter_hint_ = QString("Interpolated: \n")
            + "The trajectory from start to goal position is\n"
            + "interpolated in the given coordinate system.";
    movement_plan_hint_ = QString("Path Planning (PLAN):\n")
            + "A collision-free trajectory from start to goal\n"
            + "position is computed using a set of previously\n"
            + "definded poses and neighborhood pairs.";

    // === RIGHT SIDE ===

    // buttons on top
    button_velocity_ = new QPushButton();
    button_position_ = new QPushButton();
    button_velocity_->setCheckable(true);
    button_position_->setCheckable(true);
    button_velocity_->setChecked(false);
    button_position_->setChecked(true);
    button_velocity_->setToolTip("Velocity Control");
    button_position_->setToolTip("Position Control");
    velocity_is_activated_ = false;

    button_gravity_comp_ = new QPushButton();
    button_gravity_comp_->setToolTip("gravity compensation");

    button_go_ = new QPushButton();
    button_go_->setToolTip("Move Arm");
    button_velocity_->setIcon(QIcon(":/images/vel_button_icon.png"));
    button_position_->setIcon(QIcon(":/images/pos_button_icon.png"));
    button_go_->setIcon(QIcon(":/images/go_button_icon.png"));
    button_gravity_comp_->setIcon(QIcon(":/images/gravity_icon.png"));
    button_gravity_comp_->setCheckable(true);

    layout_move_arm_ = new QHBoxLayout();
    layout_move_arm_->addWidget(button_velocity_);
    layout_move_arm_->addWidget(button_position_);
    layout_move_arm_->addWidget(button_go_);
    layout_move_arm_->addSpacerItem(new QSpacerItem(0,0, QSizePolicy::Expanding));
    layout_move_arm_->addWidget(button_gravity_comp_);

    // arm model
    render_area_ = new RenderArea();
    render_area_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    if(file_reader_.parametersLoaded())
        render_area_->model()->setParameters(file_reader_);

    // jointstate
    table_jointstate_ = new QTableWidget(2, 6);
    table_jointstate_->setItem(0, 0, new QTableWidgetItem("0.00"));
    table_jointstate_->setItem(0, 1, new QTableWidgetItem("0.00"));
    table_jointstate_->setItem(0, 2, new QTableWidgetItem("0.00"));
    table_jointstate_->setItem(0, 3, new QTableWidgetItem("0.00"));
    table_jointstate_->setItem(0, 4, new QTableWidgetItem("0.00"));
    table_jointstate_->setItem(0, 5, new QTableWidgetItem("0.00"));
    table_jointstate_->setItem(1, 0, new QTableWidgetItem("0.00"));
    table_jointstate_->setItem(1, 1, new QTableWidgetItem("0.00"));
    table_jointstate_->setItem(1, 2, new QTableWidgetItem("0.00"));
    table_jointstate_->setItem(1, 3, new QTableWidgetItem("0.00"));
    table_jointstate_->setItem(1, 4, new QTableWidgetItem("0.00"));
    table_jointstate_->setItem(1, 5, new QTableWidgetItem("0.00"));
    QStringList labels;
    labels.append("q1");
    labels.append("q2");
    labels.append("q3");
    labels.append("q4");
    labels.append("q5");
    labels.append("gripper");
    table_jointstate_->setHorizontalHeaderLabels(labels);
    labels.clear();
    labels.append("Current");
    labels.append("Setpoint");
    table_jointstate_->setVerticalHeaderLabels(labels);
    table_jointstate_->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);
    table_jointstate_->resizeColumnsToContents();
    table_jointstate_->resizeRowsToContents();
    table_jointstate_->setFixedHeight(table_jointstate_->rowHeight(0)*3);
    table_jointstate_->setFixedWidth(table_jointstate_->columnWidth(5)*7);
    table_jointstate_->horizontalHeader()->setResizeMode(QHeaderView::Stretch);
    table_jointstate_->verticalHeader()->setResizeMode(QHeaderView::Stretch);
    table_jointstate_->setCornerButtonEnabled(false);
    for(int i=0; i<6; i++)
    {
        table_jointstate_->item(1, i)->setFlags(Qt::ItemIsEnabled | Qt::ItemIsEditable);
        table_jointstate_->item(0, i)->setFlags(Qt::ItemIsEnabled);
        table_jointstate_->item(0, i)->setBackgroundColor(QColor("lightGray"));
    }

    default_text_color_ = table_jointstate_->item(0,0)->textColor();

    //    layout_jointstate_ = new QHBoxLayout();
    //    layout_jointstate_->addWidget(combo_jointstate_coordinates_);
    //    layout_jointstate_->addWidget(label_coordinates_help_);
    //    layout_jointstate_->addSpacerItem(new QSpacerItem(0,0, QSizePolicy::Expanding));

    // layout
    right_vertical_layout_ = new QVBoxLayout();
    right_vertical_layout_->addLayout(layout_move_arm_);
    right_vertical_layout_->addWidget(render_area_);
    right_vertical_layout_->addWidget(table_jointstate_);
    //    right_vertical_layout_->addLayout(layout_jointstate_);

    // === LEFT SIDE ===

    // == POSES TAB ==

    // poses buttons
    button_save_poses_as_ = new QPushButton("Save...");
    button_load_poses_ = new QPushButton("Load...");

    // list view for poses
    list_poses_ = new QListView();
    model_poses_ = new QStringListModel();
    list_poses_->setModel(model_poses_);
    list_poses_->setContextMenuPolicy(Qt::ActionsContextMenu);
    QAction *delete_action = new QAction("delete", 0);
    QAction *rename_action = new QAction("rename", 0);
    list_poses_->addAction(delete_action);
    list_poses_->addAction(rename_action);

    updatePosesList();

    // add pose edit and button
    edit_add_pose_ = new QLineEdit();
    QRegExp rx("[A-Za-z0-9_]*");
    QValidator *validator = new QRegExpValidator(rx);
    edit_add_pose_->setValidator(validator);
    button_add_pose_ = new QPushButton("Add");
    button_add_pose_->setToolTip("Add new or modify existing pose");

    // poses tab
    layout_poses_tab_ = new QGridLayout();
    layout_poses_tab_->addWidget(button_load_poses_, 0, 0);
    layout_poses_tab_->addWidget(button_save_poses_as_, 0, 1);
    layout_poses_tab_->addWidget(list_poses_, 1, 0, 1, 3);
    layout_poses_tab_->addWidget(edit_add_pose_, 2, 0, 1, 2);
    layout_poses_tab_->addWidget(button_add_pose_, 2, 2, 1, 1);
    toolbox_page_poses_ = new QWidget();
    toolbox_page_poses_->setLayout(layout_poses_tab_);

    // == PARAMETERS TAB ==

    label_coordinates_ = new QLabel("Coordinates");

    combo_jointstate_coordinates_ = new QComboBox();
    combo_jointstate_coordinates_->addItem("Jointpace");
    combo_jointstate_coordinates_->addItem("Cartesian");
    combo_jointstate_coordinates_->addItem("Cylindrical");
    combo_jointstate_coordinates_->setCurrentIndex(JOINTSPACE);

    label_coordinates_help_ = new QLabel(" ?");
    label_coordinates_help_->setToolTip("<html><img src=\":/images/jointspace_klein.png\"/></html>");
    label_coordinates_help_->setFrameStyle(QFrame::StyledPanel);

    label_movement_mode_ = new QLabel("Motion Mode");

    combo_movement_mode_ = new QComboBox();
    combo_movement_mode_->addItem("Direct");
    combo_movement_mode_->addItem("Interpolated");
    combo_movement_mode_->addItem("Path Planning");
    combo_movement_mode_->setCurrentIndex(MODE_PLAN);

    label_movement_mode_help_ = new QLabel(" ?");
    label_movement_mode_help_->setToolTip(movement_plan_hint_);
    label_movement_mode_help_->setFrameStyle(QFrame::StyledPanel);

    label_relative_pose_ = new QLabel("Pose Setpoint");

    checkbox_relative_pose_ = new QCheckBox("Relative");

    layout_position_settings_ = new QGridLayout();
    layout_position_settings_->addWidget(label_coordinates_, 0, 0, 1, 1, Qt::AlignRight);
    layout_position_settings_->addWidget(combo_jointstate_coordinates_, 0, 1);
    layout_position_settings_->addWidget(label_coordinates_help_, 0, 2, 1, 1, Qt::AlignRight);
    layout_position_settings_->addWidget(label_movement_mode_, 1, 0);
    layout_position_settings_->addWidget(combo_movement_mode_, 1, 1);
    layout_position_settings_->addWidget(label_movement_mode_help_, 1, 2);
    layout_position_settings_->addWidget(label_relative_pose_, 2, 0);
    layout_position_settings_->addWidget(checkbox_relative_pose_, 2, 1);

    labelvector_max_velocity_.push_back(new QLabel("q1"));
    labelvector_max_velocity_.push_back(new QLabel("q2"));
    labelvector_max_velocity_.push_back(new QLabel("q3"));
    labelvector_max_velocity_.push_back(new QLabel("q4"));
    labelvector_max_velocity_.push_back(new QLabel("q5"));
    labelvector_max_velocity_.push_back(new QLabel("r"));
    labelvector_max_velocity_.push_back(new QLabel("x"));
    labelvector_max_velocity_.push_back(new QLabel("y"));
    labelvector_max_velocity_.push_back(new QLabel("z"));
    labelvector_max_velocity_.push_back(new QLabel("theta"));

    max_velocity_values_.push_back(2.5);
    max_velocity_values_.push_back(2.5);
    max_velocity_values_.push_back(2.5);
    max_velocity_values_.push_back(2.5);
    max_velocity_values_.push_back(1.2);
    max_velocity_values_.push_back(0.2);
    max_velocity_values_.push_back(0.2);
    max_velocity_values_.push_back(0.2);
    max_velocity_values_.push_back(0.2);
    max_velocity_values_.push_back(1.5);

    layout_velocity_settings_ = new QGridLayout();
    QValidator *double_validator = new QDoubleValidator(0.0, 1e6, 10);

    int initial_percentage = 20;
    for(int i=0; i<labelvector_max_velocity_.size(); i++)
    {
        slidervector_max_velocity_.push_back(new QSlider(Qt::Horizontal));
        slidervector_max_velocity_.back()->setMinimum(0);
        slidervector_max_velocity_.back()->setMaximum(100);
        slidervector_max_velocity_.back()->setValue(initial_percentage);

        editvector_max_velocity_.push_back(
                    new QLineEdit(QString::number(max_velocity_values_[i] *(initial_percentage/100.00), 'f', 3)));
        editvector_max_velocity_.back()->setFixedWidth(65);
        editvector_max_velocity_.back()->setValidator(double_validator);

        layout_velocity_settings_->addWidget(labelvector_max_velocity_[i], i, 0, 1, 1, Qt::AlignRight);
        layout_velocity_settings_->addWidget(editvector_max_velocity_[i], i, 2);
        layout_velocity_settings_->addWidget(slidervector_max_velocity_[i], i, 1);
    }

    // initialise max velocities
    onMaxVelocitiesEdited();

    button_apply_velocities_ = new QPushButton("Apply");
    button_apply_velocities_->setFixedWidth(65);
    layout_velocity_settings_->addWidget(button_apply_velocities_, labelvector_max_velocity_.size(), 2);

    //    layout_velocity_settings_->setColumnStretch(layout_velocity_settings_->columnCount(), 1);

    box_position_settings_ = new QGroupBox("Position Control Settings");
    box_position_settings_->setLayout(layout_position_settings_);

    box_velocity_settings_ = new QGroupBox("Maximum Velocities");
    box_velocity_settings_->setLayout(layout_velocity_settings_);


    layout_parameters_tab_ = new QVBoxLayout();
    layout_parameters_tab_->addWidget(box_position_settings_);
    layout_parameters_tab_->addWidget(box_velocity_settings_);
    layout_parameters_tab_->addSpacerItem(new QSpacerItem(0, 0, QSizePolicy::Minimum, QSizePolicy::Expanding));

//    scroll_area_params_ = new QScrollArea();
//    scroll_area_params_->setLayout(layout_scroll_area_);

//    layout_parameters_tab_ = new QVBoxLayout();
//    layout_parameters_tab_->addWidget(scroll_area_params_);

    toolbox_page_parameters_ = new QWidget();
    toolbox_page_parameters_->setLayout(layout_parameters_tab_);


    // tab widget
    toolbox_ = new QTabWidget();
    toolbox_->addTab(toolbox_page_poses_, "Poses");
    toolbox_->addTab(toolbox_page_parameters_, "Parameters");
    toolbox_->setCurrentIndex(0);
    toolbox_->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);

    // layout
    left_vertical_layout_ = new QVBoxLayout();
    left_vertical_layout_->addWidget(toolbox_);

    // === CENTER BUTTON ===
    button_hide_tabs_ = new QPushButton("<");
    button_hide_tabs_->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Expanding);
    button_hide_tabs_->setFixedWidth(15);
    button_hide_tabs_->setCheckable(true);

    // === MAIN LAYOUT ===
    main_horizontal_layout_ = new QHBoxLayout();
    main_horizontal_layout_->addLayout(left_vertical_layout_);
    main_horizontal_layout_->addWidget(button_hide_tabs_);
    main_horizontal_layout_->addLayout(right_vertical_layout_);
    setLayout(main_horizontal_layout_);


    // === SIGNALS AND SLOTS ===
    connect(button_hide_tabs_, SIGNAL(clicked(bool)),
            this, SLOT(hideTabs(bool)));
    connect(combo_jointstate_coordinates_, SIGNAL(currentIndexChanged(int)),
            this, SLOT(changeJointstateCoordinates(int)));
    connect(button_position_, SIGNAL(toggled(bool)),
            render_area_->model(), SLOT(changeManipulationMode(bool)));
    connect(button_position_, SIGNAL(toggled(bool)),
            this, SLOT(onRadioButtonPositionToggled(bool)));
    connect(button_velocity_, SIGNAL(toggled(bool)),
            this, SLOT(onRadioButtonVelocityToggled(bool)));
    connect(button_go_, SIGNAL(clicked(bool)),
            this, SLOT(onButtonGoClicked(bool)));
    connect(render_area_->model(), SIGNAL(virtualJointStateChanged(luh_youbot_kinematics::JointPosition,double)),
            this, SLOT(onVirtualJointstateChanged(luh_youbot_kinematics::JointPosition,double)));
    connect(table_jointstate_, SIGNAL(cellChanged(int,int)),
            this, SLOT(onTableCellChanged(int, int)));
    connect(list_poses_->selectionModel(), SIGNAL(currentRowChanged(QModelIndex,QModelIndex)),
            this, SLOT(onPoseSelected(QModelIndex, QModelIndex)));
    connect(list_poses_, SIGNAL(clicked(QModelIndex)),
            this, SLOT(onPoseClicked(QModelIndex)));
    connect(model_poses_, SIGNAL(dataChanged(QModelIndex,QModelIndex)),
            this, SLOT(onPoseNameEdited(QModelIndex, QModelIndex)));
    connect(button_load_poses_, SIGNAL(clicked()),
            this, SLOT(onLoadPoses()));
    connect(button_save_poses_as_, SIGNAL(clicked()),
            this, SLOT(onSavePosesAs()));
    connect(button_add_pose_, SIGNAL(clicked()),
            this, SLOT(onAddPose()));
    connect(delete_action, SIGNAL(triggered()),
            this, SLOT(deleteSelectedPose()));
    connect(rename_action, SIGNAL(triggered()),
            this, SLOT(renameSelectedPose()));
    connect(render_area_->model(), SIGNAL(cartesianVelocityCommand(double,double,double)),
            this, SLOT(onCartesianVelocitySet(double,double,double)));
    connect(render_area_->model(), SIGNAL(jointVelocityCommand(int,double)),
            this, SLOT(onJointVelocitySet(int,double)));
    connect(combo_movement_mode_, SIGNAL(currentIndexChanged(int)),
            this, SLOT(changeMovementMode(int)));
    connect(checkbox_relative_pose_, SIGNAL(toggled(bool)),
            this, SLOT(onCheckboxRelativeChanged(bool)));
    connect(this, SIGNAL(clearModel()),
            render_area_->model(), SLOT(clear()));
    connect(button_gravity_comp_, SIGNAL(clicked(bool)), this, SLOT(onButtonGravityCompensationClicked(bool)));
//    connect(button_gravity_comp_, SIGNAL(released()), this, SLOT(onButtonGravityCompensationReleased()));
    connect(button_apply_velocities_, SIGNAL(clicked()), this, SLOT(onButtonApplyClicked()));

    for(int i=0; i<editvector_max_velocity_.size(); i++)
    {
        connect(editvector_max_velocity_[i], SIGNAL(editingFinished()), this, SLOT(onMaxVelocitiesEdited()));
        connect(slidervector_max_velocity_[i], SIGNAL(valueChanged(int)), this, SLOT(onSliderMoved(int)));
    }


    changeJointstateCoordinates(0);
}

ArmControlWindow::~ArmControlWindow()
{

}

void ArmControlWindow::hideTabs(bool hide)
{
    toolbox_->setVisible(!hide);
    if(hide)
        button_hide_tabs_->setText(">");
    else
        button_hide_tabs_->setText("<");
}

void ArmControlWindow::changeJointstateCoordinates(int index)
{
    if(index == JOINTSPACE)
    {
        table_jointstate_->horizontalHeaderItem(0)->setText("q1");
        table_jointstate_->horizontalHeaderItem(0)->setToolTip("Joint angle in degrees");
        table_jointstate_->horizontalHeaderItem(1)->setText("q2");
        table_jointstate_->horizontalHeaderItem(1)->setToolTip("Joint angle in degrees");
        table_jointstate_->horizontalHeaderItem(2)->setText("q3");
        table_jointstate_->horizontalHeaderItem(2)->setToolTip("Joint angle in degrees");
        table_jointstate_->horizontalHeaderItem(3)->setText("q4");
        table_jointstate_->horizontalHeaderItem(4)->setToolTip("Joint angle in degrees");

        //        label_coordinate_image_->setPixmap(QPixmap(":/images/jointspace_klein.png"));
        label_coordinates_help_->setToolTip("<html><img src=\":/images/jointspace_klein.png\"/></html>");
    }
    else if(index == CARTESIAN)
    {
        table_jointstate_->horizontalHeaderItem(0)->setText("x");
        table_jointstate_->horizontalHeaderItem(0)->setToolTip("Endeffector position im meters");
        table_jointstate_->horizontalHeaderItem(1)->setText("y");
        table_jointstate_->horizontalHeaderItem(1)->setToolTip("Endeffector position im meters");
        table_jointstate_->horizontalHeaderItem(2)->setText("z");
        table_jointstate_->horizontalHeaderItem(2)->setToolTip("Endeffector position im meters");
        table_jointstate_->horizontalHeaderItem(3)->setText("theta");
        table_jointstate_->horizontalHeaderItem(3)->setToolTip("Endeffector angle in degrees");

        //        label_coordinate_image_->setPixmap(QPixmap(":/images/kartesisch_base_klein.png"));
        label_coordinates_help_->setToolTip("<html><img src=\":/images/kartesisch_base_klein.png\"/></html>");
    }
    else
    {
        table_jointstate_->horizontalHeaderItem(0)->setText("q1");
        table_jointstate_->horizontalHeaderItem(0)->setToolTip("Joint angle in degrees");
        table_jointstate_->horizontalHeaderItem(1)->setText("r");
        table_jointstate_->horizontalHeaderItem(1)->setToolTip("Endeffector position im meters");
        table_jointstate_->horizontalHeaderItem(2)->setText("z");
        table_jointstate_->horizontalHeaderItem(2)->setToolTip("Endeffector position im meters");
        table_jointstate_->horizontalHeaderItem(3)->setText("theta");
        table_jointstate_->horizontalHeaderItem(3)->setToolTip("Endeffector angle in degrees");

        //        label_coordinate_image_->setPixmap(QPixmap(":/images/zylinderkoordinaten_klein.png"));
        label_coordinates_help_->setToolTip("<html><img src=\":/images/zylinderkoordinaten_klein.png\"/></html>");
    }

    table_jointstate_->horizontalHeaderItem(4)->setText("q5");
    table_jointstate_->horizontalHeaderItem(4)->setToolTip("Joint angle in degrees");
    table_jointstate_->horizontalHeaderItem(5)->setText("gripper");
    table_jointstate_->horizontalHeaderItem(5)->setToolTip("Gripper width im meters");

    updateJointstateTable(true);
    updateJointstateTable(false);
}

//########## SLOT: VIRTUAL JOINTSTATE CHANGED ##########################################################################
void ArmControlWindow::onVirtualJointstateChanged(luh_youbot_kinematics::JointPosition jointstate,
                                                  double gripper_position)
{
    joint_setpoint_ = jointstate;
    gripper_setpoint_ = gripper_position;

    updateJointstateTable(true);
}

//########## SLOT: CURRENT JOINTSTATE CHANGED ##########################################################################
//void ArmControlWindow::onCurrentJointstateChanged(luh_youbot_kinematics::JointPosition jointstate,
//                                                  double gripper_position)
void ArmControlWindow::onCurrentJointstateChanged(double q1, double q2, double q3, double q4, double q5,
                                                  double gripper_state)
{
    //    std::cout << debug_counter_++ << std::endl;
    //    Q_EMIT logMessage(QString::number(debug_counter_));
    //    jointstate.printValues("jointstate");

    //    current_jointstate_= jointstate;


    double gripper_tolerance = 0.001;
    double joint_tolerance = M_PI / 180.0;
    double max_joint_diff = std::max(std::abs(q1 - current_jointstate_.q1()),
                                     std::max(std::abs(q2 - current_jointstate_.q2()),
                                              std::max(std::abs(q3 - current_jointstate_.q3()),
                                                       std::max(std::abs(q4 - current_jointstate_.q4()),
                                                                std::abs(q5 - current_jointstate_.q5())))));

    double gripper_diff = std::abs(gripper_state - current_gripper_state_);

    if(max_joint_diff < joint_tolerance && gripper_diff < gripper_tolerance)
        return;

    current_jointstate_.setQ1(q1);
    current_jointstate_.setQ2(q2);
    current_jointstate_.setQ3(q3);
    current_jointstate_.setQ4(q4);
    current_jointstate_.setQ5(q5);
    current_gripper_state_ = gripper_state;

    render_area_->model()->setCurrentJointstate(current_jointstate_, current_gripper_state_);
    render_area_->update();

    updateJointstateTable(false);

    if(checkbox_relative_pose_->isChecked())
    {
        updateJointstateTable(true);
    }
}

//########## UPDATE JOINTSTATE TABLE ###################################################################################
void ArmControlWindow::updateJointstateTable(double gripper_state, bool setpoint)
{
    table_jointstate_->blockSignals(true);

    int row_index;

    if(setpoint)
        row_index = 1;
    else
        row_index = 0;

    // check if pose is set to relative
    if(setpoint && checkbox_relative_pose_->isChecked())
    {
        gripper_state -= current_gripper_state_;
        QString sign;
        if(gripper_state >= 0)
            sign.append("+");
        table_jointstate_->item(row_index, 5)->setText(sign + QString::number(gripper_state, 'f', 3));
    }
    else
        table_jointstate_->item(row_index, 5)->setText(QString::number(gripper_state, 'f', 3));

    table_jointstate_->item(row_index, 5)->setTextColor(default_text_color_);

    table_jointstate_->blockSignals(false);
}

//########## UPDATE JOINTSTATE TABLE ###################################################################################
void ArmControlWindow::updateJointstateTable(luh_youbot_kinematics::JointPosition jointstate, bool setpoint)
{
    table_jointstate_->blockSignals(true);

    int row_index;

    if(setpoint)
        row_index = 1;
    else
        row_index = 0;

    int index = combo_jointstate_coordinates_->currentIndex();
    if(index == JOINTSPACE)
    {
        // check if pose is set to relative
        QVector<QString> signs(5, "");
        if(setpoint && checkbox_relative_pose_->isChecked())
        {
            jointstate -= current_jointstate_;
            for(int i=0; i<5; i++)
            {
                if(jointstate[i] >= 0)
                    signs[i] = "+";
            }
        }
        table_jointstate_->item(row_index, 0)->setText(signs[0] + QString::number(jointstate.q1() * 180/M_PI, 'f', 1));
        table_jointstate_->item(row_index, 1)->setText(signs[1] + QString::number(jointstate.q2() * 180/M_PI, 'f', 1));
        table_jointstate_->item(row_index, 2)->setText(signs[2] + QString::number(jointstate.q3() * 180/M_PI, 'f', 1));
        table_jointstate_->item(row_index, 3)->setText(signs[3] + QString::number(jointstate.q4() * 180/M_PI, 'f', 1));
        table_jointstate_->item(row_index, 4)->setText(signs[4] + QString::number(jointstate.q5() * 180/M_PI, 'f', 1));
    }
    else if(index == CARTESIAN)
    {
        luh_youbot_kinematics::CartesianPosition cart_pos = jointstate.toCartesian();

        // check if pose is set to relative
        QVector<QString> signs(5, "");
        if(setpoint && checkbox_relative_pose_->isChecked())
        {
            cart_pos -= current_jointstate_.toCartesian();
            for(int i=0; i<5; i++)
            {
                if(cart_pos[i] >= 0)
                    signs[i] = "+";
            }
        }

        table_jointstate_->item(row_index, 0)->setText(signs[0] + QString::number(cart_pos.x(), 'f', 3));
        table_jointstate_->item(row_index, 1)->setText(signs[1] + QString::number(cart_pos.y(), 'f', 3));
        table_jointstate_->item(row_index, 2)->setText(signs[2] + QString::number(cart_pos.z(), 'f', 3));
        table_jointstate_->item(row_index, 3)->setText(signs[3] + QString::number(cart_pos.theta() * 180/M_PI, 'f', 1));
        table_jointstate_->item(row_index, 4)->setText(signs[4] + QString::number(cart_pos.q5() * 180/M_PI, 'f', 1));
    }
    else
    {
        luh_youbot_kinematics::CylindricPosition cyl_pos = jointstate.toCylindric();
        // check if pose is set to relative
        QVector<QString> signs(5, "");
        if(setpoint && checkbox_relative_pose_->isChecked())
        {
            cyl_pos -= current_jointstate_.toCylindric();
            for(int i=0; i<5; i++)
            {
                if(cyl_pos[i] >= 0)
                    signs[i] = "+";
            }
        }
        table_jointstate_->item(row_index, 0)->setText(signs[0] + QString::number(cyl_pos.q1() * 180/M_PI, 'f', 1));
        table_jointstate_->item(row_index, 1)->setText(signs[1] + QString::number(cyl_pos.r(), 'f', 3));
        table_jointstate_->item(row_index, 2)->setText(signs[2] + QString::number(cyl_pos.z(), 'f', 3));
        table_jointstate_->item(row_index, 3)->setText(signs[3] + QString::number(cyl_pos.theta() * 180/M_PI, 'f', 1));
        table_jointstate_->item(row_index, 4)->setText(signs[4] + QString::number(cyl_pos.q5() * 180/M_PI, 'f', 1));
    }


    for(int i=0; i<5; i++)
        table_jointstate_->item(row_index, i)->setTextColor(default_text_color_);

    table_jointstate_->blockSignals(false);
}

//########## UPDATE JOINTSTATE TABLE ###################################################################################
void ArmControlWindow::updateJointstateTable(bool setpoint)
{
    if(setpoint)
    {
        updateJointstateTable(gripper_setpoint_, setpoint);
        updateJointstateTable(joint_setpoint_, setpoint);
    }
    else
    {
        updateJointstateTable(current_jointstate_, setpoint);
        updateJointstateTable(current_gripper_state_, setpoint);
    }
}

//########## SLOT: RADIO BUTTON TOGGLED ################################################################################
void ArmControlWindow::onRadioButtonPositionToggled(bool checked)
{
    button_go_->setCheckable(!checked);
    button_go_->setChecked(false);
    button_velocity_->setChecked(!checked);
    velocity_is_activated_ = false;
}

//########## SLOT: RADIO BUTTON TOGGLED ################################################################################
void ArmControlWindow::onRadioButtonVelocityToggled(bool checked)
{
    button_go_->setCheckable(checked);
    button_go_->setChecked(false);
    button_position_->setChecked(!checked);
    velocity_is_activated_ = false;
}

//########## SLOT: BUTTON GO CLICKED ###################################################################################
void ArmControlWindow::onButtonGoClicked(bool checked)
{
    if(!ros_is_initialised_)
    {
        button_go_->setChecked(false);
        Q_EMIT logMessage("ROS is not initialized.");
        return;
    }

    button_gravity_comp_->setChecked(false);

    if(button_velocity_->isChecked())
    {
        velocity_is_activated_ = checked;
    }
    else
    {
        int mode = combo_movement_mode_->currentIndex();

        int coordinates = combo_jointstate_coordinates_->currentIndex();
        bool is_relative = checkbox_relative_pose_->isChecked();

        if(!showSafetyDialog())
            return;

        if(coordinates == JOINTSPACE)
        {
            if(is_relative)
                Q_EMIT jointPositionSet(joint_setpoint_ - current_jointstate_,
                                        gripper_setpoint_ - current_gripper_state_, mode, true);
            else
                Q_EMIT jointPositionSet(joint_setpoint_, gripper_setpoint_, mode, false);
        }
        else if(coordinates == CARTESIAN)
        {
            luh_youbot_kinematics::CartesianPosition cart_pos = joint_setpoint_.toCartesian();
            double gripper = gripper_setpoint_;
            if(is_relative)
            {
                cart_pos -= current_jointstate_.toCartesian();
                gripper -= current_gripper_state_;
            }
            Q_EMIT cartesianPositionSet(cart_pos, gripper, mode, is_relative);
        }
        else if(coordinates == CYLINDRIC)
        {
            luh_youbot_kinematics::CylindricPosition cyl_pos = joint_setpoint_.toCylindric();
            double gripper = gripper_setpoint_;
            if(is_relative)
            {
                cyl_pos -= current_jointstate_.toCylindric();
                gripper -= current_gripper_state_;
            }
            Q_EMIT cylindricPositionSet(cyl_pos, gripper, mode, is_relative);
        }

    }
}

//########## SLOT: ON TABLE CELL CHANGED ###############################################################################
void ArmControlWindow::onTableCellChanged(int row, int col)
{
    luh_youbot_kinematics::JointPosition jointstate;
    int index = combo_jointstate_coordinates_->currentIndex();
    bool reachable = false;
    bool is_relative = checkbox_relative_pose_->isChecked();
    if(index == JOINTSPACE)
    {
        jointstate.setQ1(table_jointstate_->item(1, 0)->text().toDouble() * M_PI/180);
        jointstate.setQ2(table_jointstate_->item(1, 1)->text().toDouble() * M_PI/180);
        jointstate.setQ3(table_jointstate_->item(1, 2)->text().toDouble() * M_PI/180);
        jointstate.setQ4(table_jointstate_->item(1, 3)->text().toDouble() * M_PI/180);
        jointstate.setQ5(table_jointstate_->item(1, 4)->text().toDouble() * M_PI/180);

        if(is_relative)
        {
            //jointstate.printValues("relative jointstate");
            jointstate += current_jointstate_;
            //jointstate.printValues("absolute jointstate");
        }

        reachable = jointstate.isReachable();
    }
    else if(index == CARTESIAN)
    {
        luh_youbot_kinematics::CartesianPosition cart_pos;
        cart_pos.setX(table_jointstate_->item(1, 0)->text().toDouble());
        cart_pos.setY(table_jointstate_->item(1, 1)->text().toDouble());
        cart_pos.setZ(table_jointstate_->item(1, 2)->text().toDouble());
        cart_pos.setTheta(table_jointstate_->item(1, 3)->text().toDouble() * M_PI/180);
        cart_pos.setQ5(table_jointstate_->item(1, 4)->text().toDouble() * M_PI/180);

        if(is_relative)
            cart_pos += current_jointstate_.toCartesian();

        jointstate = cart_pos.toJointspace(joint_setpoint_);
        if(!jointstate.isReachable())
            jointstate = cart_pos.toJointspace();

        reachable = jointstate.isReachable() && cart_pos.isReachable();
    }
    else
    {
        luh_youbot_kinematics::CylindricPosition cyl_pos;
        cyl_pos.setQ1(table_jointstate_->item(1, 0)->text().toDouble() * M_PI/180);
        cyl_pos.setR(table_jointstate_->item(1, 1)->text().toDouble());
        cyl_pos.setZ(table_jointstate_->item(1, 2)->text().toDouble());
        cyl_pos.setTheta(table_jointstate_->item(1, 3)->text().toDouble() * M_PI/180);
        cyl_pos.setQ5(table_jointstate_->item(1, 4)->text().toDouble() * M_PI/180);

        if(is_relative)
            cyl_pos += current_jointstate_.toCylindric();

        jointstate = cyl_pos.toJointspace(joint_setpoint_);
        if(!jointstate.isReachable())
            jointstate = cyl_pos.toJointspace();

        reachable = jointstate.isReachable() && cyl_pos.isReachable();
    }

    double gripper_state = table_jointstate_->item(1, 5)->text().toDouble();

    if(is_relative)
        gripper_state += current_gripper_state_;

    if(gripper_state >= gripper_state_min_ && gripper_state <= gripper_state_max_)
    {
        gripper_setpoint_ = gripper_state;
        updateJointstateTable(gripper_setpoint_, true);
    }
    else
    {
        table_jointstate_->item(1, 5)->setTextColor(QColor("red"));
    }


    if(reachable)
    {
        joint_setpoint_ = jointstate;
        render_area_->model()->setVirtualJointstate(jointstate, gripper_setpoint_);
        updateJointstateTable(jointstate, true);
    }
    else
    {
        for(int i=0; i<5; i++)
            table_jointstate_->item(1, i)->setTextColor(QColor("red"));
    }
}

//########## SLOT: ON POSE SELECTED ####################################################################################
void ArmControlWindow::onPoseSelected(QModelIndex index1, QModelIndex index2)
{
    onPoseClicked(index1);
}

//########## SLOT: ON POSE CLICKED #####################################################################################
void ArmControlWindow::onPoseClicked(QModelIndex index)
{
    selected_pose_name_ = model_poses_->data(index, Qt::DisplayRole).toString();

    joint_setpoint_ = predef_poses_[selected_pose_name_];

    edit_add_pose_->setText(selected_pose_name_);

    render_area_->model()->setVirtualJointstate(joint_setpoint_, gripper_setpoint_);
    updateJointstateTable(joint_setpoint_, true);

    Q_EMIT clearModel();
}

//########## SLOT: ON POSE EDITED ######################################################################################
void ArmControlWindow::onPoseNameEdited(QModelIndex index1, QModelIndex index2)
{
    QString name = model_poses_->data(index1, Qt::DisplayRole).toString();

    if(name == selected_pose_name_)
        return;

    predef_poses_[name] = predef_poses_[selected_pose_name_];
    predef_poses_.remove(selected_pose_name_);

    updatePosesList();

    Q_EMIT logMessage(selected_pose_name_ + " changed to " + name + ".");

    selected_pose_name_ = name;
    edit_add_pose_->setText(selected_pose_name_);

    // find new index
    int index_row = 0;
    for(PoseMap::const_iterator it = predef_poses_.begin(); it != predef_poses_.end(); it++)
    {
        if(it.key() == name)
            break;

        index_row++;
    }

    list_poses_->setCurrentIndex(model_poses_->index(index_row));
}

//########## SLOT: ON ADD POSE #########################################################################################
void ArmControlWindow::onAddPose()
{
    QString pose_name = edit_add_pose_->text();

    if(pose_name.isEmpty())
        return;

    if(predef_poses_.find(pose_name) != predef_poses_.end())
    {
        QMessageBox msgBox;
        msgBox.setWindowTitle("Pose already exists");
        msgBox.setText("Overwrite existing pose " + pose_name + "?");
        msgBox.setStandardButtons(QMessageBox::Yes | QMessageBox::No);
        msgBox.setDefaultButton(QMessageBox::No);
        int ret = msgBox.exec();

        if(ret == QMessageBox::No)
            return;
    }

    predef_poses_[pose_name] = this->current_jointstate_;

    updatePosesList();
}

//########## SLOT: ON SAVE POSES #######################################################################################
//void ArmControlWindow::onSavePoses()
//{
//    file_reader_.savePoses(predef_poses_);
//}

//########## SLOT: ON SAVE POSES AS ####################################################################################
void ArmControlWindow::onSavePosesAs()
{
    QString default_path = QString::fromStdString(file_reader_.getFileName());
    QString file_name = QFileDialog::getSaveFileName(this,
                                                     tr("Save Pose File"), default_path, tr("YAML Files (*.yaml)"));

    if(file_name.isEmpty())
        return;

    if(!file_name.endsWith(".yaml"))
        file_name.append(".yaml");

    QRegExp rx(".*/[A-Za-z0-9]+[A-Za-z0-9_]*.yaml");
    QRegExpValidator validator(rx);

    int pos = 0;
    if(validator.validate(file_name, pos) != QValidator::Acceptable)
    {
        QMessageBox msgBox;
        msgBox.setWindowTitle("Invalid Filename");
        msgBox.setText(file_name);
        msgBox.setStandardButtons(QMessageBox::Ok);
        int ret = msgBox.exec();
        return;
    }

    file_reader_.setPosesFile(file_name.toStdString());
    file_reader_.savePoses(predef_poses_);
}

//########## SLOT: ON LOAD POSES #######################################################################################
void ArmControlWindow::onLoadPoses()
{
    QString default_path = QString::fromStdString(ros::package::getPath("luh_youbot_gui"));
    QString file_name = QFileDialog::getOpenFileName(this,
                                                     tr("Save Pose File"), default_path, tr("YAML Files (*.yaml)"));

    int ret = 0;

    if(!predef_poses_.isEmpty())
    {

        QMessageBox msgBox;
        msgBox.setWindowTitle("Load Poses");
        msgBox.setText("There are already poses in the list.");
        QString s("You can merge the loaded poses into the list or replace the current list with new poses.");
        msgBox.setInformativeText(s);

        QPushButton *add_button = msgBox.addButton("Merge", QMessageBox::ApplyRole);
        QPushButton *replace_button = msgBox.addButton("Replace", QMessageBox::ResetRole);
        QPushButton *cancel_button = msgBox.addButton("Cancel", QMessageBox::RejectRole);

        msgBox.setDefaultButton(replace_button);

        msgBox.exec();

        if(msgBox.clickedButton() == add_button)
        {
            ret = 0;
        }
        else if(msgBox.clickedButton() == replace_button)
        {
            ret = 1;
        }
        else
        {
            ret = 2;
        }

    }

    if(ret == 1)
    {
        predef_poses_.clear();
    }
    else if (ret == 2)
        return;

    file_reader_.setPosesFile(file_name.toStdString());
    file_reader_.loadPoses(predef_poses_);
    updatePosesList();
}

//########## UPDATE POSES LIST #########################################################################################
void ArmControlWindow::updatePosesList()
{
    QStringList pose_names;

    for(PoseMap::iterator it = predef_poses_.begin(); it != predef_poses_.end(); ++it)
    {
        pose_names.append(it.key());
    }

    model_poses_->setStringList(pose_names);
}

//########## SLOT: DELETE SELECTED POSE ################################################################################
void ArmControlWindow::deleteSelectedPose()
{
    if(predef_poses_.isEmpty())
        return;

    QModelIndex index = list_poses_->selectionModel()->currentIndex();
    QString name = model_poses_->data(index, Qt::DisplayRole).toString();

    predef_poses_.remove(name);

    updatePosesList();

    Q_EMIT logMessage(name + " deleted.");
}

//########## SLOT: RENAME SELECTED POSE ################################################################################
void ArmControlWindow::renameSelectedPose()
{
    if(predef_poses_.isEmpty())
        return;

    QModelIndex index = list_poses_->selectionModel()->currentIndex();
    list_poses_->edit(index);
}

//########## SLOT: RENAME SELECTED POSE ################################################################################
void ArmControlWindow::onCartesianVelocitySet(double v_x, double v_y, double v_theta)
{
    v_x *= max_cyl_velocity_.r();
    v_y *= max_cyl_velocity_.z();
    v_theta *= max_cyl_velocity_.theta();

    if(velocity_is_activated_ && ros_is_initialised_)
        Q_EMIT cartesianVelocity(v_x, v_y, v_theta);
}

//########## SLOT: RENAME SELECTED POSE ################################################################################
void ArmControlWindow::onJointVelocitySet(int joint_idx, double vel)
{
    vel *= max_joint_velocity_[joint_idx];

    if(velocity_is_activated_ && ros_is_initialised_)
        Q_EMIT jointVelocity(joint_idx, vel);
}

//########## SLOT: ON ROS INITIALISED ##################################################################################
void ArmControlWindow::onRosInitialised()
{
    ros_is_initialised_ = true;
}

//########## SLOT: CHANGE MOVEMENT MODE ################################################################################
void ArmControlWindow::changeMovementMode(int mode)
{
    if(mode == MODE_DIRECT)
    {
        label_movement_mode_help_->setToolTip(movement_direct_hint_);
    }
    else if(mode == MODE_INTER)
    {
        label_movement_mode_help_->setToolTip(movement_inter_hint_);
    }
    else if(mode == MODE_PLAN)
    {
        label_movement_mode_help_->setToolTip(movement_plan_hint_);
    }
}

//########## SLOT: ON MAX VELOCITIES EDITED ############################################################################
void ArmControlWindow::onMaxVelocitiesEdited()
{
    QVector<double> velocity_values;
    for(int i=0; i<editvector_max_velocity_.size(); i++)
    {
        slidervector_max_velocity_[i]->blockSignals(true);

        double value = editvector_max_velocity_[i]->text().toDouble();
        value = std::max(0.0, std::min(max_velocity_values_[i], value));
        editvector_max_velocity_[i]->setText(QString::number(value, 'f', 3));
        velocity_values.push_back(value);

        slidervector_max_velocity_[i]->setValue(value/max_velocity_values_[i] * 100);

        slidervector_max_velocity_[i]->blockSignals(false);
    }

    max_joint_velocity_.setQ1(velocity_values[0]);
    max_joint_velocity_.setQ2(velocity_values[1]);
    max_joint_velocity_.setQ3(velocity_values[2]);
    max_joint_velocity_.setQ4(velocity_values[3]);
    max_joint_velocity_.setQ5(velocity_values[4]);
    max_cyl_velocity_.setQ1(velocity_values[0]);
    max_cyl_velocity_.setR(velocity_values[5]);
    max_cyl_velocity_.setZ(velocity_values[8]);
    max_cyl_velocity_.setTheta(velocity_values[9]);
    max_cyl_velocity_.setQ5(velocity_values[4]);
    max_cart_velocity_.setX(velocity_values[6]);
    max_cart_velocity_.setY(velocity_values[7]);
    max_cart_velocity_.setZ(velocity_values[8]);
    max_cart_velocity_.setTheta(velocity_values[9]);
    max_cart_velocity_.setQ5(velocity_values[4]);
}

//########## SLOT: ON CHECK BOX RELATIVE CHANGED #######################################################################
void ArmControlWindow::onCheckboxRelativeChanged(bool is_relative)
{
    updateJointstateTable(true);
}

//########## SLOT: ON SLIDER MOVED #####################################################################################
void ArmControlWindow::onSliderMoved(int value)
{
    QVector<double> velocity_values;

    for(int i=0; i<slidervector_max_velocity_.size(); i++)
    {
        editvector_max_velocity_[i]->blockSignals(true);
        double v = 0.01 * slidervector_max_velocity_[i]->value() * max_velocity_values_[i];
        editvector_max_velocity_[i]->setText(QString::number(v, 'f', 3));
        editvector_max_velocity_[i]->blockSignals(false);

        velocity_values.push_back(v);
    }

    max_joint_velocity_.setQ1(velocity_values[0]);
    max_joint_velocity_.setQ2(velocity_values[1]);
    max_joint_velocity_.setQ3(velocity_values[2]);
    max_joint_velocity_.setQ4(velocity_values[3]);
    max_joint_velocity_.setQ5(velocity_values[4]);
    max_cyl_velocity_.setQ1(velocity_values[0]);
    max_cyl_velocity_.setR(velocity_values[5]);
    max_cyl_velocity_.setZ(velocity_values[8]);
    max_cyl_velocity_.setTheta(velocity_values[9]);
    max_cyl_velocity_.setQ5(velocity_values[4]);
    max_cart_velocity_.setX(velocity_values[6]);
    max_cart_velocity_.setY(velocity_values[7]);
    max_cart_velocity_.setZ(velocity_values[8]);
    max_cart_velocity_.setTheta(velocity_values[9]);
    max_cart_velocity_.setQ5(velocity_values[4]);
}

//########## SHOW SAFETY DIALOG ########################################################################################
bool ArmControlWindow::showSafetyDialog()
{
    double distance_thresh = M_PI / 2;

    int mode = combo_movement_mode_->currentIndex();

    if(mode == MODE_PLAN)
        return true;

    luh_youbot_kinematics::JointPosition pose_diff = current_jointstate_ - joint_setpoint_;
    pose_diff *= pose_diff;

    double distance = 0;
    for(uint i=0; i<pose_diff.size(); i++)
    {
        distance += pose_diff[i];
    }

    distance = sqrt(distance);

    if(distance > distance_thresh)
    {
        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Warning);
        msgBox.setWindowTitle("Warning");
        if(mode == MODE_DIRECT)
            msgBox.setText(QString("Direct movements over large distances might cause collisions.\n")
                           + QString("Only proceed if you are sure the movement is save."));
        else if(mode == MODE_INTER)
            msgBox.setText(QString("Interpolated movements over large distances might cause collisions.\n")
                           + QString("Only proceed if you are sure the movement is save."));

        msgBox.setInformativeText("Proceed?");
        QPushButton *proceed_button = msgBox.addButton("Proceed", QMessageBox::AcceptRole);
        QPushButton *cancel_button = msgBox.addButton("Cancel", QMessageBox::RejectRole);

        msgBox.setDefaultButton(cancel_button);

        msgBox.exec();

        return (msgBox.clickedButton() == proceed_button);
    }
    else return true;

}

//########## ON BUTTON GRAVITY COMPENSATION PRESSED ####################################################################
void ArmControlWindow::onButtonGravityCompensationClicked(bool is_checked)
{
    Q_EMIT compensateGravityRequested(is_checked);

    render_area_->model()->setVirtualJointstate(current_jointstate_, current_gripper_state_);
    updateJointstateTable(current_jointstate_, true);

    Q_EMIT clearModel();

}

//########## ON BUTTON GRAVITY COMPENSATION RELEASED ###################################################################
//void ArmControlWindow::onButtonGravityCompensationReleased()
//{
//    Q_EMIT stiffenArmRequested();
//}

//########## ON BUTTON APPLY CLICKED ###################################################################################
void ArmControlWindow::onButtonApplyClicked()
{
    Q_EMIT velocitiesChanged(max_joint_velocity_, max_cart_velocity_, max_cyl_velocity_);
}
