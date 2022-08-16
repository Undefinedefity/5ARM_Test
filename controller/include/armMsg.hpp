#ifndef _ARMMSG_HPP_
#define _ARMMSG_HPP_

struct JointState
{
    int16_t v;
    int16_t t;
    uint16_t p;
    int rot = 0;
    uint16_t motorAngle;
    int64_t motorMultiAngle;
};
struct ArmJointState
{
    JointState j1, j2, j3;
    bool switchCmd = false;
};

struct JointControl
{
    float p_des = 0;
    float v_des = 0;
    float kp = 0;
    float kd = 0;
    float tau = 0;
    // int mode = 0; // 0--torque control mode, 1--speed control mode
};
struct ArmJointControl
{
    JointControl j1, j2, j3;
};

struct JointMsg
{
    unsigned char data[19];
};
struct ArmJointMsg
{
    JointMsg j1,j2,j3;
};

#endif