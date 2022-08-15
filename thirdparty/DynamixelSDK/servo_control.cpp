// ServoControll.cpp
// Two XM540 serial connected with a USB2RDXL
// Using simple Protocol2PacketHandler::write4ByteTxRx to control

#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0

#include <stdlib.h>
#include <stdio.h>
#include <iostream>

#include "dynamixel_sdk.h" // Uses DYNAMIXEL SDK library#define ADDR_TORQUE_ENABLE 64

#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_POSITION 116
#define ADDR_PRESENT_POSITION 132
#define MINIMUM_POSITION_LIMIT 0    // Refer to the Minimum Position Limit of product eManual
#define MAXIMUM_POSITION_LIMIT 4095 // Refer to the Maximum Position Limit of product eManual
#define BAUDRATE 57600

// DYNAMIXEL Protocol Version (1.0 / 2.0)
// https://emanual.robotis.com/docs/en/dxl/protocol2/
#define PROTOCOL_VERSION 2.0

// Factory default ID of all DYNAMIXEL is 1
#define DXL1_ID 1 // J4
#define DXL2_ID 2 // J5 gripper

// Use the actual port assigned to the U2D2.
// ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
#define DEVICENAME "/dev/ttyUSB0"

#define TORQUE_ENABLE 1
#define TORQUE_DISABLE 0
#define DXL_MOVING_STATUS_THRESHOLD 20 // DYNAMIXEL moving status threshold
#define ESC_ASCII_VALUE 0x1b           // KEY ESC

// For Logi_Controller
#include "joystick.h"
#include <memory.h>
#include <unistd.h>

int getch(); //

int main(int argc, char const *argv[])
{
    // Dynamxiel initialization
    {
        // Initialize PortHandler instance
        // Set the port path
        // Get methods and members of PortHandlerLinux or PortHandlerWindows
        dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

        // Initialize PacketHandler instance
        // Set the protocol version
        // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
        dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

        int dxl_comm_result = COMM_RX_FAIL;                           // Communication result
        int dxl1_goal_position = 0;                                   // For ID 1 J4
        int dxl2_goal_position = 0;                                   // For ID 2 gripper
        uint8_t dxl_error = 0;                                        // DYNAMIXEL error
        int32_t dxl1_present_position = 0, dxl2_present_position = 0; // Read 4 byte Position data
    }

    // Enable Dynamixel Torque
    {
        // Enable DYNAMIXEL#1 Torque
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        }
        else
        {
            printf("Dynamixel#%d has been successfully connected \n", DXL1_ID);
        }

        // Enable Dynamixel#2 Torque
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        }
        else
        {
            printf("Dynamixel#%d has been successfully connected \n", DXL2_ID);
        }
    }

    // Logi game controller initialization
    GamepadCommand gameCmd;
    JoystickEvent event;
    std::shared_ptr<Joystick> joystick; // why?
    int iter = 0;                       // iteration counter

    // necessary param
    // b  -- mins, plus
    // J1 -- LB, RB
    // J2 -- LT, RT
    // J3 -- BACK, START
    // J4 -- A, B
    // J5 -- X, Y
    double J1 = 0, J2 = 0, J3 = 0, J4 = 0, J5 = 0;

    // Create an instance of Joystick
    joystick = std::make_shared<Joystick>("/dev/input/js0");

    // Ensure that it was found and that we can use it
    if (!joystick->isFound())
    {
        printf("joystick not found!\n");
        exit(1);
    }
    gameCmd.zero();

    while (1)
    {
        break;
    }

    return 0;
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