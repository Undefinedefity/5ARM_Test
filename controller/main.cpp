//
// Created by yuran on 2022/1/17.
//
#include "uart.hpp"
#include <fstream>

#define UART_PATH (char *)"/dev/ttyACM0"
#define J1_CTRL
//#define J3_CTRL
//#define J3CTRL2
#define SAVE_DATA

#ifdef SAVE_DATA
std::fstream save_state("state.txt", std::ios::ate | std::ios::out);
#endif

using namespace std;

ArmJointState armJointState;
ArmJointState armJointLast;
ArmJointState armState_init;
ArmJointControl armJointControl;
ArmJointMsg armJointMsg;
int64_t angleOffset[3];
double posOffset[3];
const int CPR = 65535;

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

int main(int argc, char const *argv[])
{
    int uart = uart_init(BAUD, UART_PATH);
    int key;

    int xunhuan = 0;
    int count = 0;
    initMsg(&armJointMsg);
    thread recv_thread(uart_recv, uart, &armJointState);
    recv_thread.detach();
    thread send_thread(uart_send, uart, &armJointMsg);
    send_thread.detach();
    thread detect_keyboard(keyBoardDetector, &key);
    detect_keyboard.detach();

    cout << "Press c to continue..." << endl;
    pressKeytoContinue(&key, 99);
    cout << "\033[2J\033[1;1H";

    int k = 1;

    while (key != 101)
    {
        cout << "\033[2J\033[1;1H";
        cout << "arm: " << endl;
        arm_can_state_mut.lock();
        showArmState(&armJointState);
        arm_can_state_mut.unlock();

        std::cout << "k: " << k << std::endl;
//        cout << "Press e to exit..." << endl;

//        packAll(&armJointMsg,armJointControl);

#ifdef READ_ANGLE
        cmdReadPos1(&(armJointMsg.j1));
        cmdReadPos1(&(armJointMsg.j2));
        cmdReadPos1(&(armJointMsg.j3));
#endif

#ifdef SEND_CTRL_CMD
        if (k > 80)
        {
            // joint 1
            if ((armJointState.j1.p - armJointLast.j1.p) > CPR / 2)
            {
                armJointState.j1.rot -= 1;
            } else if ((armJointState.j1.p - armJointLast.j1.p) < -CPR / 2)
            {
                armJointState.j1.rot += 1;
            }
            // joint 2
            if ((armJointState.j2.p - armJointLast.j2.p) > CPR / 2)
            {
                armJointState.j2.rot -= 1;
            } else if ((armJointState.j2.p - armJointLast.j2.p) < -CPR / 2)
            {
                armJointState.j2.rot += 1;
            }
            // joint 3
            if ((armJointState.j3.p - armJointLast.j3.p) > CPR / 2)
            {
                armJointState.j3.rot -= 1;
            } else if ((armJointState.j3.p - armJointLast.j3.p) < -CPR / 2)
            {
                armJointState.j3.rot += 1;
            }
        }

        armJointLast = armJointState;

        if (k < 50)
        {
            xunhuan = 1;
            cmdReadPos1(&(armJointMsg.j1));
            cmdReadPos1(&(armJointMsg.j2));
            cmdReadPos1(&(armJointMsg.j3));
            armJointState.switchCmd = false;
            angleOffset[0] = armJointState.j1.motorMultiAngle;
            angleOffset[1] = armJointState.j2.motorMultiAngle;
            angleOffset[2] = armJointState.j3.motorMultiAngle;
        } else if (k < 100)
        {
            xunhuan = 2;
            armJointControl.j1.p_des = angleOffset[0];
            armJointControl.j2.p_des = angleOffset[1];
            armJointControl.j3.p_des = angleOffset[2];
            packAll(&armJointMsg, armJointControl);
            armJointState.switchCmd = true;
            posOffset[0] = armJointState.j1.p;
            posOffset[1] = armJointState.j2.p;
            posOffset[2] = armJointState.j3.p;
        }
//        else if (k < 100+3600)
//        {
//            xunhuan = 3;
//            count++;
//            armJointControl.j2.p_des = angleOffset[1] + count * 40 * angleRateS2 / 3600;
//            packCmd(&(armJointMsg.j2), armJointControl.j2);
//        }
        else if (200 < k && k < 202)
        {
            armJointControl.j2.p_des = angleOffset[1] + 1 * angleRateS2;
            packCmd(&(armJointMsg.j2), armJointControl.j2);
        }
//        else if (300 < k && k < 302)
//        {
//            armJointControl.j2.p_des = angleOffset[1] + 40 * angleRateS2;
//            packCmd(&(armJointMsg.j2), armJointControl.j2);
//        }
//        else if (300 < k && k < 302)
//        {
//            armJointControl.j3.p_des = angleOffset[2] - 2 * angleRate;
//            packCmd(&(armJointMsg.j3), armJointControl.j3);
//        }
//        else if (400 < k && k < 402)
//        {
//
//        }

        else
        {
            xunhuan = 5;
            cout << "Press e to exit..." << endl;
        }

        if (count > 3600)
            count = 3600;

#endif

#ifdef SAVE_DATA
        save_state
                << k << " "
                << armJointState.j1.motorMultiAngle << " "
                << armJointState.j2.motorMultiAngle << " "
                << armJointState.j3.motorMultiAngle << " "
                << armJointState.j1.t << " "
                << armJointState.j2.t << " "
                << armJointState.j3.t << " "
                << armJointControl.j1.p_des << " "
                << armJointControl.j2.p_des << " "
                << armJointControl.j3.p_des << " "
                << std::endl;
#endif
        usleep(10000);
        k++;
    }

    shut_motorPackAll();
    std::cout << "!!! The Motor Shut Off !!!" << std::endl;
    usleep(2000);

    system("stty echo");
    system("stty icanon");
    sleep(1);

    exit(0);

}