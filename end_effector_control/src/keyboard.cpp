#include "end_effector_control/keyboard.h"

#include <termios.h>
#include <unistd.h>
#include <stdio.h>

static struct termios oldt;
static struct termios newt;

Keyboard::Keyboard()
{
    init();
}

Keyboard::~Keyboard()
{
    restore();
}

void Keyboard::init()
{
    tcgetattr(STDIN_FILENO, &oldt);

    newt = oldt;

    newt.c_lflag &= ~(ICANON | ECHO);

    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
}

void Keyboard::restore()
{
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
}

char Keyboard::getKey()
{
    return getchar();
}