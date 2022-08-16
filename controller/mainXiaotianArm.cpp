//
// Created by yuran on 2022/1/17.
//
#include "uart.hpp"

#define UART_PATH (char *)"/dev/ttyACM0"
#define J1_CTRL
//#define J3_CTRL
//#define J3CTRL2

using namespace std;

ArmJointState armJointState;
ArmJointState armState_init;
ArmJointControl armJointControl;
ArmJointMsg armJointMsg;

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
    int sign = 1;

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

        int test_rate = 5;
        int userDesAngle = 5 / test_rate;

        if (k < 50)
        {
            cmdReadPos1(&(armJointMsg.j1));
            cmdReadPos1(&(armJointMsg.j2));
            cmdReadPos1(&(armJointMsg.j3));
        } else
        {
            int res = k / (200 / test_rate);
            cout << "res is: " << res << endl;
#ifdef J1_CTRL
            if (res > (39-1)*test_rate)
            {
                std::cout << "Hold the arm, and press e to exit..." << std::endl;
            }

            else if (res <= 2*test_rate)
            {
                armJointControl.j2.p_des = armJointState.j2.motorMultiAngle + res * userDesAngle * angleRateS2;
                packCmd(&(armJointMsg.j2), armJointControl.j2);
                armJointState.switchCmd = true;
            }
            else
            {
//                cout << - (res-2) * userDesAngle << endl;
                armJointControl.j1.p_des = armJointState.j1.motorMultiAngle - (res - 2*test_rate) * userDesAngle * angleRateS2;
                cout << "send angle cmd is: " << armJointControl.j1.p_des << endl;
                packCmd(&(armJointMsg.j1), armJointControl.j1);
                armJointState.switchCmd = true;
            }
#endif

#ifdef J3_CTRL
            if (res > (21-1)*test_rate)
            {
                std::cout << "Hold the arm, and press e to exit..." << std::endl;
            }

            else if (res <= 2*test_rate)
            {
                armJointControl.j2.p_des = armJointState.j2.motorMultiAngle + res * 0 * angleRateS2;
                packCmd(&(armJointMsg.j2), armJointControl.j2);
                armJointState.switchCmd = true;
            }
            else
            {
//                cout << - (res-2) * userDesAngle << endl;
                armJointControl.j3.p_des = armJointState.j3.motorMultiAngle + (res - 2*test_rate) * userDesAngle * angleRate;
                cout << "send angle cmd is: " << armJointControl.j3.p_des << endl;
                packCmd(&(armJointMsg.j3), armJointControl.j3);
                armJointState.switchCmd = true;
            }
#endif

#ifdef J3CTRL2
            int judge1 = 18 + 2;
            int judge2 = 18;
            if (res > (judge1+judge2) * test_rate)
            {
                std::cout << "Hold the arm, and press e to exit..." << std::endl;
            } else if (res <= judge1 * test_rate)
            {
                armJointControl.j2.p_des = armJointState.j2.motorMultiAngle + res * userDesAngle * angleRateS2;
                packCmd(&(armJointMsg.j2), armJointControl.j2);
                armJointState.switchCmd = true;
            } else if (res <= (judge1 + judge2) * test_rate)
            {
                armJointControl.j3.p_des =
                        armJointState.j3.motorMultiAngle - (res - judge1 * test_rate) * userDesAngle * angleRate;
                cout << "send angle cmd is: " << armJointControl.j3.p_des << endl;
                packCmd(&(armJointMsg.j3), armJointControl.j3);
                armJointState.switchCmd = true;
            } else
            {
                armJointControl.j3.p_des = armJointState.j3.motorMultiAngle +
                                           (res - (judge1 + judge2) * test_rate) * userDesAngle * angleRate;
                cout << "send angle cmd is: " << armJointControl.j3.p_des << endl;
                packCmd(&(armJointMsg.j3), armJointControl.j3);
                armJointState.switchCmd = true;
            }
#endif

        }
#endif

        usleep(2000);
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