#ifndef LUH_YOUBOT_KINEMATICS_CONSTANTS_H
#define LUH_YOUBOT_KINEMATICS_CONSTANTS_H

namespace luh_youbot_kinematics
{

// constants
/// number of joints
const unsigned int N_JOINTS = 5;

/// height of joint 2 (in relation to arm_link_0)
const double L0 = 0.115;

/// radial offset of joint 2 (in relation to axis of joint 1)
const double L1 = 0.033;

/// length of first arm segment
const double L2 = 0.155;

/// length of second arm segment
const double L3 = 0.135;

/// length of third arm segment including gripper
const double L4 = 0.220;

/// distance between arm_link_0 origin and axis of joint 1
const double ARM_LINK_0_X_OFFSET = 0.024;

/// maximum joint accelerations
//const double MAX_JNT_ACCELERATIONS[] = {0.7, 0.7, 0.7, 0.7, 0.7};
//const double MAX_JNT_ACCELERATIONS[] = {1.34, 1.34, 2.09, 2.94, 2.94};
const double MAX_JNT_ACCELERATIONS[] = {2.0, 2.0, 2.0, 2.0, 2.0};

/// maximum joint velocities
//const double MAX_JNT_VELOCITIES[] = {0.8, 0.8, 0.8, 0.8, 0.8};
//const double MAX_JNT_VELOCITIES[] = {2.68, 2.68, 4.18, 5.89, 5.89};
const double MAX_JNT_VELOCITIES[] = {2.6, 2.6, 2.6, 2.6, 1.3};

/// joint angle offsets (difference between desired and actual zero positions)
const double JOINT_OFFSETS[] =
{
    -2.949606436,
    -1.1344673701987218,
     2.5481772172532176,
    -1.7889600250839740,
    -3.0019466477485340
};

/// maximum joint positions (in relation to desired zero positions)
const double MAX_JNT_POSITIONS[] =
{
    2.8905337534,
    1.4835265078,
    2.532469254,
    1.6402432236,
    2.6396457936
};

/// minimum joint positions (in relation to desired zero positions)
const double MIN_JNT_POSITIONS[] =
{
    -2.9395372288,
    -1.124398163,
    -2.4783710285,
    -1.7668361332,
    -2.8913271881
};

}
#endif // LUH_YOUBOT_KINEMATICS_CONSTANTS_H
