#include "Serial.h"
int Serial::set_opt(int fd, int nSpeed, int nBits, char nEvent, int nStop)
{
    struct termios newtio, oldtio;
    if (tcgetattr(fd, &oldtio) != 0) {
        perror("SetupSerial 1");
        return -1;
    }
    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag |= CLOCAL | CREAD;
    newtio.c_cflag &= ~CSIZE;

    switch (nBits) {
    case 7:
        newtio.c_cflag |= CS7;
        break;
    case 8:
        newtio.c_cflag |= CS8;
        break;
    }

    switch (nEvent) {
    case 'O': //奇校验
        newtio.c_cflag |= PARENB;
        newtio.c_cflag |= PARODD;
        newtio.c_iflag |= (INPCK | ISTRIP);
        break;
    case 'E': //偶校验
        newtio.c_iflag |= (INPCK | ISTRIP);
        newtio.c_cflag |= PARENB;
        newtio.c_cflag &= ~PARODD;
        break;
    case 'N': //无校验
        newtio.c_cflag &= ~PARENB;
        break;
    }

    switch (nSpeed) {
    case 2400:
        cfsetispeed(&newtio, B2400);
        cfsetospeed(&newtio, B2400);
        break;
    case 4800:
        cfsetispeed(&newtio, B4800);
        cfsetospeed(&newtio, B4800);
        break;
    case 9600:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    case 115200:
        cfsetispeed(&newtio, B115200);
        cfsetospeed(&newtio, B115200);
        break;
    default:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    }
    if (nStop == 1) {
        newtio.c_cflag &= ~CSTOPB;
    } else if (nStop == 2) {
        newtio.c_cflag |= CSTOPB;
    }
    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = 0;
    tcflush(fd, TCIFLUSH);
    if ((tcsetattr(fd, TCSANOW, &newtio)) != 0) {
        perror("com set error");
        return -1;
    }
    printf("set done!\n");
    return 0;
}

void Serial::init(string port)
{
//#if PLATFORM == MANIFOLD
    fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY); //妙算串口设备描述符看说明书，这个应该就是
    if (fd < 0) {
        perror("open_port error");
        return;
    }
    int i;
    if ((i = set_opt(fd, 115200, 8, 'N', 1)) < 0) //串口参数设置
    {
        perror("set_opt error");
        return;
    }
//#endif
}

void Serial::sendString(char* data, int length)
{
    int n = write(fd, data, length);
    if (n<0) {
        cout << "Write Error! Sign: " << n << endl;
    }
    //char receive[30] = { 0 };
    //n = read(fd, receive, 50);
    //if (n<0) {
        //cout << "Read Error!" << endl;
    //} else {
        //cout << "Receive:" << receive << endl;
    //}
}

int Serial::receive()
{
    for (int i=0; i<30; ++i) {
        receive_buf[i] = 0;
    }
    return read(fd, receive_buf, 30);
    //cout << "Receive: " << receive_buf << endl;
}

void Serial::sendTarget(int target_x, int target_y, int is_found)
{

    if (target_x < 0 || target_x > 640 || target_y < 0 || target_y > 480) {
        return;
        is_found = 0;
        target_x = 320;
        target_y = 240;
    }
    cout << dec << "x:" << target_x << " y:" << target_y << endl;

    char buf[7];
    buf[0] = 0xA5;
    buf[1] = (target_x >> 8) & 0xFF;
    buf[2] = target_x & 0xFF;
    if (is_found == 11) {
        buf[3] = 0xB1;
    } else if (is_found == 12) {
        buf[3] = 0xB2;
    } else if (is_found == 13) {
        buf[3] = 0xB3;
    } else if (is_found == 14) {
        buf[3] = 0xB4;
    } else if (is_found == 4) {
        buf[3] = 0xA2;
    } else if (is_found == 3) {
        buf[3] = 0xA1;
    } else if (is_found == 2) {
        buf[3] = 0xA8;
    } else if (is_found == 1) {
        buf[3] = 0xA6;
    } else if (is_found == 0) {
        buf[3] = 0xA4;
    } else if (is_found == -1) {
        buf[3] = 0xA3;
    } else {
        cout << "Invalid Instruction!" << endl;
        buf[3] = 0xA4;
    }
    buf[4] = (target_y >> 8) & 0xFF;
    buf[5] = target_y & 0xFF;
    buf[6] = 0xA7;
    for (int i = 0; i < 7; ++i) {
        cout << hex << (unsigned int)(unsigned char)buf[i] << " ";
    }
    cout << endl;

    sendString(buf, 7);
};

int Serial::buffMode()
{
    static int stm_state = 0;
    int buff_mode = -1;
    //cout << "Receive: ";
    for (int i=0; i<30; ++i) {
        //cout << " " << hex << (unsigned int)(unsigned char)receive_buf[i];
        switch(stm_state) {
            case 0: if ((unsigned int)(unsigned char) receive_buf[i] == 0xff) {
                        stm_state = 1;
                    }
                    break;
            case 1: if ((unsigned int)(unsigned char) receive_buf[i] == 0x00
                            || (unsigned int)(unsigned char) receive_buf[i] == 0x01
                            || (unsigned int)(unsigned char) receive_buf[i] == 0x02) {
                        buff_mode = (unsigned int)(unsigned char)receive_buf[i];
                        cout << "Get Buff Mode: " << (unsigned int)(unsigned char) receive_buf[i];
                        stm_state = 2;
                    }
                    break;
            case 2: if ((unsigned int)(unsigned char) receive_buf[i] == 0x00) {
                        stm_state = 3;
                    }
                    break;
            case 3: if ((unsigned int)(unsigned char) receive_buf[i] == 0xfe) {
                        stm_state = 4;
                    }
                    break;
            default: break;
        }
        if (stm_state == 4) {
            stm_state = 0;
            //cout << endl;
            return buff_mode;
        }

    }
    return -1;
        //<< (int)receive_buf[1] << (int)receive_buf[2] << (int)receive_buf[3] << endl;
    //if ((int)receive_buf[0] == 0xff 
            //&& (int)receive_buf[1] == 0x01
            //&& (int)receive_buf[2] == 0x00
            //&& (int)receive_buf[3] == 0xfe) {
        //return 1;
    //}
    //if ((int)receive_buf[0] == 0xff 
            //&& (int)receive_buf[1] == 0x02
            //&& (int)receive_buf[2] == 0x00
            //&& (int)receive_buf[3] == 0xfe) {
        //return 2;
    //}
    //return 0;
}
