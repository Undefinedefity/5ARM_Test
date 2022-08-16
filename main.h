// ServoControll.cpp
// Two XM540 serial connected with a USB2RDXL
// Using simple Protocol2PacketHandler::write4ByteTxRx to control

#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0

#include <stdlib.h>
#include <stdio.h>
#include <iostream>

#include "dynamixel_sdk.h" // Uses DYNAMIXEL SDK library

#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_POSITION 116
#define ADDR_PRESENT_POSITION 132
#define MINIMUM_POSITION_LIMIT 0        // Refer to the Minimum Position Limit of product eManual
#define MAXIMUM_POSITION_LIMIT_ID1 4095 // ID 1, 360 degree
#define MAXIMUM_POSITION_LIMIT_ID2 1000 // ID 2,  88 degree
#define BAUDRATE 57600

// https://emanual.robotis.com/docs/en/dxl/protocol2/
#define PROTOCOL_VERSION 2.0

// Factory default ID of all DYNAMIXEL is 1
#define DXL1_ID 1 // J4
#define DXL2_ID 2 // J5 gripper

// Use the actual port assigned to the U2D2.
// Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
#define DEVICENAME "/dev/ttyUSB0"

#define TORQUE_ENABLE 1
#define TORQUE_DISABLE 0
#define DXL_MOVING_STATUS_THRESHOLD 20 // DYNAMIXEL moving status threshold
#define ESC_ASCII_VALUE 0x1b           // KEY ESC

#define J4Step 20
#define J5Step 5

// For Logi_Controller
#include "joystick.h"
#include <memory>
#include <unistd.h>

// For RMD motors
#include "uart.hpp"
#define UART_PATH (char *)"/dev/ttyACM0"

ArmJointState armJointState;
ArmJointState armState_init;
ArmJointControl armJointControl;
ArmJointMsg armJointMsg;

#define J1Step 1000
#define J2Step 1000
#define J3Step 200

const int angleRateS2 = 3600;
const int angleRate = 800;

void showArmState(ArmJointState *state)
{
    std::cout << "j1: " << state->j1.p << "(p) " << state->j1.v << "(v) " << state->j1.t << "(t) "
              << state->j1.motorMultiAngle << "(angle)" << std::endl;
    std::cout << "j2: " << state->j2.p << "(p) " << state->j2.v << "(v) " << state->j2.t << "(t) "
              << state->j2.motorMultiAngle << "(angle)" << std::endl;
    std::cout << "j3: " << state->j3.p << "(p) " << state->j3.v << "(v) " << state->j3.t << "(t) "
              << state->j3.motorMultiAngle << "(angle)" << std::endl;

}

void shut_motorPackAll()
{
    arm_can_msg_mut.lock();
    shut_motor(&armJointMsg.j1);
    shut_motor(&armJointMsg.j2);
    shut_motor(&armJointMsg.j3);
    arm_can_msg_mut.unlock();
    std::cout << "!!! The Motor Shut Off !!!" << std::endl;
}

void pressKeytoContinue(int *key, int ASCII)
{
    while (*key != ASCII)
    {
        usleep(1000);
    }
}

void keyBoardDetector(int *key)
{
    system("stty -icanon");
    system("stty -echo");
    while (1)
    {
        *key = getchar();
        usleep(1000);
    }
}

int getch()
{
#if defined(__linux__) || defined(__APPLE__)
    struct termios oldt, newt;
    int ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
#elif defined(_WIN32) || defined(_WIN64)
    return _getch();
#endif
}