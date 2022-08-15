/*! @file GamepadCommand.h
 *  @brief The GamepadCommand type containing joystick information
 */

#ifndef PROJECT_GAMEPADCOMMAND_H
#define PROJECT_GAMEPADCOMMAND_H

#include "utilities.h"
/*!
 * The state of the gamepad
 */
struct GamepadCommand
{
    /*!
     * Construct a gamepad and set to zero.
     */
    GamepadCommand() { zero(); }

    bool LB, RB, BACK, START, A, B, X, Y, LT, RT;

    Eigen::Vector2d leftStickAnalog, rightStickAnalog, Dpad;

    /*!
     * Set all values to zero
     */
    void zero()
    {
        LB = false;
        RB = false;
        LT = false;
        RT = false;
        BACK = false;
        START = false;
        A = false;
        B = false;
        X = false;
        Y = false;

        leftStickAnalog = Eigen::Vector2d::Zero();
        rightStickAnalog = Eigen::Vector2d::Zero();
        Dpad = Eigen::Vector2d::Zero();
    }

    /*!
     * The Logitech F310's seem to do a bad job of returning to zero exactly, so a
     * deadband around zero is useful when integrating joystick commands
     * @param f : The deadband
     */
    void applyDeadband(float f)
    {
        eigenDeadband(leftStickAnalog, f);
        eigenDeadband(rightStickAnalog, f);
        eigenDeadband(Dpad, f);
    }

    /*!
     * Represent as human-readable string.
     * @return string representing state
     */
    std::string toString()
    {
        std::string result =
            "Result:\nLB: " + boolToString(LB) + "\n" +
            "RB: " + boolToString(RB) + "\n" +
            "LT: " + boolToString(LT) + "\n" +
            "RT: " + boolToString(RT) + "\n" +
            "BACK: " + boolToString(BACK) + "\n" +
            "START: " + boolToString(START) + "\n" +
            "A: " + boolToString(A) + "\n" +
            "B: " + boolToString(B) + "\n" +
            "X: " + boolToString(X) + "\n" +
            "Y: " + boolToString(Y) + "\n" +
            "Dpad: " + eigenToString(Dpad) + "\n" +
            "leftStickAnalog: " + eigenToString(leftStickAnalog) + "\n" +
            "rightStickAnalog: " + eigenToString(rightStickAnalog) + "\n";
        return result;
    }
};

#endif // PROJECT_DRIVERCOMMAND_H
