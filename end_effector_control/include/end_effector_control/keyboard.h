#ifndef KEYBOARD_H
#define KEYBOARD_H

class Keyboard
{
public:

    Keyboard();

    ~Keyboard();

    char getKey();

private:

    void init();

    void restore();

};

#endif