#include "joystick.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <iostream>
#include <string>
#include <sstream>
#include <unistd.h>

Joystick::Joystick()
{
  openPath("/dev/input/js0");
}

Joystick::Joystick(int joystickNumber)
{
  std::stringstream sstm;
  sstm << "/dev/input/js" << joystickNumber;
  openPath(sstm.str());
}

Joystick::Joystick(std::string devicePath)
{
  openPath(devicePath);
}

Joystick::Joystick(std::string devicePath, bool blocking)
{
  openPath(devicePath, blocking);
}

void Joystick::openPath(std::string devicePath, bool blocking)
{
  // Open the device using either blocking or non-blocking
  _fd = open(devicePath.c_str(), blocking ? O_RDONLY : O_RDONLY | O_NONBLOCK);
}

bool Joystick::sample(JoystickEvent *event)
{
  int bytes = read(_fd, event, sizeof(*event));

  if (bytes == -1)
    return false;

  // NOTE if this condition is not met, we're probably out of sync and this
  // Joystick instance is likely unusable
  return bytes == sizeof(*event);
}

bool Joystick::isFound()
{
  return _fd >= 0;
}

Joystick::~Joystick()
{
  close(_fd);
}

void Joystick::updateCommand(JoystickEvent *event, GamepadCommand &cmd)
{
  if (sample(event))
  { 
    // XInput mode
    // Attempt to sample an event from the joystick
    // button
    // A - 0
    // B  -  1
    // X  -  2
    // Y  -  3
    // LB -  4
    // RB -  5
    if (event->isButton() && event->number == 0)
      cmd.X = event->value;

    if (event->isButton() && event->number == 1)
      cmd.A = event->value;

    if (event->isButton() && event->number == 2)
      cmd.B = event->value;

    if (event->isButton() && event->number == 3)
      cmd.Y = event->value;

    if (event->isButton() && event->number == 4)
      cmd.LB = event->value;

    if (event->isButton() && event->number == 5)
      cmd.RB = event->value;

    if (event->isButton() && event->number == 6)
      cmd.LT = event->value;

    if (event->isButton() && event->number == 7)
      cmd.RT = event->value;

    if (event->isButton() && event->number == 8)
      cmd.BACK = event->value;

    if (event->isButton() && event->number == 9)
      cmd.START = event->value;

    // Axis
    if (event->isAxis() && event->number == 0)
      cmd.leftStickAnalog[0] = -(event->value) / double(event->MAX_AXES_VALUE);

    if (event->isAxis() && event->number == 1)
      cmd.leftStickAnalog[1] = -(event->value) / double(event->MAX_AXES_VALUE);

    if (event->isAxis() && event->number == 2)
      cmd.rightStickAnalog[0] = -(event->value) / double(event->MAX_AXES_VALUE);

    if (event->isAxis() && event->number == 3)
      cmd.rightStickAnalog[1] = -(event->value) / double(event->MAX_AXES_VALUE);

    if (event->isAxis() && event->number == 4)
      cmd.Dpad[0] = -(event->value) / double(event->MAX_AXES_VALUE);

    if (event->isAxis() && event->number == 5)
      cmd.Dpad[1] = -(event->value) / double(event->MAX_AXES_VALUE);

    cmd.applyDeadband(0.001);
    printf("%s\n", cmd.toString().c_str());
  }
}

std::ostream &operator<<(std::ostream &os, const JoystickEvent &e)
{
  os << "type=" << static_cast<int>(e.type)
     << " number=" << static_cast<int>(e.number)
     << " value=" << static_cast<int>(e.value);
  return os;
}