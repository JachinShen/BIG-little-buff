#include <iostream>
#include <stdio.h>
#include <time.h>

#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include "Headers.h"

using namespace std;
class Serial {
private:
//#if PLATFORM == MANIFOLD
    int fd;
//#endif

private:
    int set_opt(int, int, int, char, int);
    char receive_buf[30];

public:
    void init(string port);
    void sendString(char* data, int length);
    void sendTarget(int, int, int);
    int receive();
};
