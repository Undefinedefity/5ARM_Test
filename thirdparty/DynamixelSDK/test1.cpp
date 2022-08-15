// test 1
// Two XM540 serial connected with a USB2RDXL
// Modified from Dynamixel protocol2.0/read_write
// Set two values of each servos and then they reach the value

#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <stdlib.h>
#include <stdio.h>
#include <iostream>

#include "dynamixel_sdk.h" // Uses DYNAMIXEL SDK library

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

int getch(); // 

int main(int argc, char const *argv[])
{
    // Initialize PortHandler instance
    // Set the port path
    // Get methods and members of PortHandlerLinux or PortHandlerWindows
    dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

    // Initialize PacketHandler instance
    // Set the protocol version
    // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
    dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    int dxl_comm_result = COMM_RX_FAIL; // Communication result
    int dxl1_goal_position = 0;         // For ID 1 J4
    int dxl2_goal_position = 0;         // For ID 2 gripper

    uint8_t dxl_error = 0;                                        // DYNAMIXEL error
    int32_t dxl1_present_position = 0, dxl2_present_position = 0; // Read 4 byte Position data

    // Open port
    if (portHandler->openPort())
    {
        printf("Succeeded to open the port!\n");
    }
    else
    {
        printf("Failed to open the port!\n");
        printf("Press any key to terminate...\n");
        getch();
        return 0;
    }

    // Set port baudrate
    if (portHandler->setBaudRate(BAUDRATE))
    {
        printf("Succeeded to change the baudrate!\n");
    }
    else
    {
        printf("Failed to change the baudrate!\n");
        printf("Press any key to terminate...\n");
        getch();
        return 0;
    }

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

    while (1)
    {
        printf("Press any key to continue. (Press [ESC] to exit)\n");
        if (getch() == ESC_ASCII_VALUE)
            break;

        // Write goal position
        // Servo ID1
        std::cout << "Write ID1 servo, range[0, 4095] input the value:  ";
        std::cin >> dxl1_goal_position;

        // Servo ID2
        std::cout << "Write ID2 servo, range[0, 4095] input the value:  ";
        std::cin >> dxl2_goal_position;
        getchar(); // If no getchar(), then the enter

        dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL1_ID, ADDR_GOAL_POSITION, dxl1_goal_position, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        }

        dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL2_ID, ADDR_GOAL_POSITION, dxl2_goal_position, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        }

        do
        {
            // Get Dynamixel#1 present position value
            dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL1_ID, ADDR_PRESENT_POSITION, (uint32_t *)&dxl1_present_position, &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS)
            {
                printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
            }
            else if (dxl_error != 0)
            {
                printf("%s\n", packetHandler->getRxPacketError(dxl_error));
            }

            // Get Dynamixel#2 present position value
            dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL2_ID, ADDR_PRESENT_POSITION, (uint32_t *)&dxl2_present_position, &dxl_error);
            if(dxl_comm_result != COMM_SUCCESS)
            {
                printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
            }
            else if (dxl_error != 0)
            {
                printf("%s\n", packetHandler->getRxPacketError(dxl_error));
            }
            

            printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d\n", DXL1_ID, dxl1_goal_position, dxl1_present_position, DXL2_ID, dxl2_goal_position, dxl2_present_position);

        } while ((abs(dxl1_goal_position - dxl1_present_position) > DXL_MOVING_STATUS_THRESHOLD) || (abs(dxl2_goal_position - dxl2_present_position) > DXL_MOVING_STATUS_THRESHOLD));
    }

    // Disable DYNAMIXEL#1 Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);

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
        printf("Succeeded disabling DYNAMIXEL Torque.\n");
    }

    // Disable DYNAMIXEL#2 Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);

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
        printf("Succeeded disabling DYNAMIXEL Torque.\n");
    }

    // Close port
    portHandler->closePort();

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