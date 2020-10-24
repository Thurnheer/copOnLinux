#ifndef COP_ON_LINUX_UART_HPP
#define COP_ON_LINUX_UART_HPP

#include "uart.hpp"
#include <stdio.h>
#include <string.h>

#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <system_error>
#include <iostream>
#include <vector>

template<class Iterator>
class Uart
{
public:
    Uart()
    : port_handle_(open("/dev/ttyACM0", O_RDWR))
    , read_buf_()
    {
        if(port_handle_ < 0) {
            throw std::system_error();
        }
        if(tcgetattr(port_handle_, &tty_) != 0){
            throw std::system_error();
        }
        tty_.c_cflag &= ~PARENB; // clear parity bit
        tty_.c_cflag &= ~CSTOPB; // only one stop bit
        tty_.c_cflag &= ~CSIZE; // clear bits per byte field
        tty_.c_cflag |= CS8; // 8 bits per byte
        tty_.c_cflag &= ~CRTSCTS; // disable HW flowcontrol
        tty_.c_cflag |= CREAD | CLOCAL;
        tty_.c_lflag &= ~ICANON; // disable canonical mode (write on newline)
        tty_.c_lflag &= ~ECHO;
        tty_.c_lflag &= ~ECHOE;
        tty_.c_lflag &= ~ECHONL;
        tty_.c_lflag &= ~ISIG; // disable interpretation of INT, QUIT and SUSP
        tty_.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off sw flow control
        tty_.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);
        tty_.c_oflag &= ~OPOST;
        tty_.c_oflag &= ~ONLCR;
        tty_.c_cc[VTIME] = 0;
        tty_.c_cc[VMIN] = 0;
        cfsetispeed(&tty_, B115200);
        cfsetospeed(&tty_, B115200);
        if(tcsetattr(port_handle_, TCSANOW, &tty_) != 0) {
            throw std::system_error();
        }
        read_buf_.reserve(BUF_SIZE);
    }

    ~Uart() {
        close(port_handle_);
    }
    void send(Iterator begin, Iterator end) {

    }

    void read() {
        std::byte buf[BUF_SIZE];
        int n = ::read(port_handle_, buf, BUF_SIZE);
        read_buf_ = std::vector<std::byte>(buf, buf + n);        
    }

private:
    int port_handle_;
    struct termios tty_;
    static const size_t BUF_SIZE = 512;
public:
    std::vector<std::byte> read_buf_;
};

#endif // COP_ON_LINUX_UART_HPP

