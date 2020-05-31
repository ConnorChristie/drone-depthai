// File: ceSerial.cpp
// Description: ceSerial communication class implementation for Windows and Linux
// WebSite: http://cool-emerald.blogspot.sg/2017/05/serial-port-programming-in-c-with.html
// MIT License (https://opensource.org/licenses/MIT)
// Copyright (c) 2018 Yan Naing Aye

// References
// https://en.wikibooks.org/wiki/Serial_Programming/termios
// http://www.silabs.com/documents/public/application-notes/an197.pdf
// https://msdn.microsoft.com/en-us/library/ff802693.aspx
// http://www.cplusplus.com/forum/unices/10491/

#include "ceSerial.h"

using namespace std;

namespace ce
{

    ceSerial::ceSerial(string Device, long BaudRate, long DataSize, char ParityType, float NStopBits)
    {
        fd = -1;
        port = Device;
        SetBaudRate(BaudRate);
        SetDataSize(DataSize);
        SetParity(ParityType);
        SetStopBits(NStopBits);
    }

    ceSerial::~ceSerial()
    {
        Close();
    }

    void ceSerial::SetPort(string Device) {
        port = Device;
    }

    string ceSerial::GetPort() {
        return port;
    }

    void ceSerial::SetDataSize(long nbits) {
        if ((nbits < 5) || (nbits > 8)) nbits = 8;
        dsize = nbits;
    }

    long ceSerial::GetDataSize() {
        return dsize;
    }

    void ceSerial::SetParity(char p) {
        if ((p != 'N') && (p != 'E') && (p != 'O')) {
            p = 'N';
        }
        parity = p;
    }

    char ceSerial::GetParity() {
        return parity;
    }

    void ceSerial::SetStopBits(float nbits) {
        if (nbits >= 2) stopbits = 2;
        else stopbits = 1;
    }

    float ceSerial::GetStopBits() {
        return stopbits;
    }

    long ceSerial::Open(void)
    {
        struct termios settings;

        fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);

        if (tcgetattr(fd, &settings) < 0)
        {
            printf("Error from tcgetattr %s: %s\n", port.c_str(), strerror(errno));
            return -1;
        }

        settings.c_oflag = 0;
        settings.c_lflag = 0;

        settings.c_cflag = CREAD | CLOCAL;

        if (dsize == 5)
            settings.c_cflag |= CS5;
        else if (dsize == 6)
            settings.c_cflag |= CS6;
        else if (dsize == 7)
            settings.c_cflag |= CS7;
        else
            settings.c_cflag |= CS8;

        if (stopbits == 2) settings.c_cflag |= CSTOPB;
        if (parity == 'O') settings.c_cflag |= PARODD;
        if (parity == 'N') settings.c_iflag = IGNPAR;

        settings.c_cc[VMIN] = 0;
        settings.c_cc[VTIME] = 1;

        cfsetospeed(&settings, baud);
        cfsetispeed(&settings, baud);

        tcflush(fd, TCIFLUSH);
        tcsetattr(fd, TCSANOW, &settings);

        return 0;
    }

    void ceSerial::Close() {
        if (IsOpened()) close(fd);
        fd = -1;
    }

    bool ceSerial::IsOpened()
    {
        if (fd == (-1)) return false;
        else return true;
    }

    void ceSerial::SetBaudRate(long baudrate) {
        if (baudrate < 50) baud = B0;
        else if (baudrate < 75) baud = B50;
        else if (baudrate < 110) baud = B75;
        else if (baudrate < 134) baud = B110;
        else if (baudrate < 150) baud = B134;
        else if (baudrate < 200) baud = B150;
        else if (baudrate < 300) baud = B200;
        else if (baudrate < 600) baud = B300;
        else if (baudrate < 1200) baud = B600;
        else if (baudrate < 2400) baud = B1200;
        else if (baudrate < 4800) baud = B2400;
        else if (baudrate < 9600) baud = B4800;
        else if (baudrate < 19200) baud = B9600;
        else if (baudrate < 38400) baud = B19200;
        else if (baudrate < 57600) baud = B38400;
        else if (baudrate < 115200) baud = B57600;
        else if (baudrate < 230400) baud = B115200;
        else baud = B230400;
    }

    long ceSerial::GetBaudRate() {
        long baudrate = 9600;
        if (baud < B50) baudrate = 0;
        else if (baud < B75) baudrate = 50;
        else if (baud < B110) baudrate = 75;
        else if (baud < B134) baudrate = 110;
        else if (baud < B150) baudrate = 134;
        else if (baud < B200) baudrate = 150;
        else if (baud < B300) baudrate = 200;
        else if (baud < B600) baudrate = 300;
        else if (baud < B1200) baudrate = 600;
        else if (baud < B2400) baudrate = 1200;
        else if (baud < B4800) baudrate = 2400;
        else if (baud < B9600) baudrate = 4800;
        else if (baud < B19200) baudrate = 9600;
        else if (baud < B38400) baudrate = 19200;
        else if (baud < B57600) baudrate = 38400;
        else if (baud < B115200) baudrate = 57600;
        else if (baud < B230400) baudrate = 115200;
        else baudrate = 230400;
        return baudrate;
    }

    size_t ceSerial::Write(char* data, size_t length)
    {
        if (!IsOpened())
        {
            return false;
        }

        return write(fd, data, length);
    }

    size_t ceSerial::Read(char* buf, size_t size)
    {
        return read(fd, buf, size);
    }

    bool ceSerial::Flush()
    {
        if (!IsOpened())
        {
            return false;
        }
        return tcdrain(fd) == 0;
    }

    bool ceSerial::SetRTS(bool value) {
        long RTS_flag = TIOCM_RTS;
        bool success = true;
        if (value) {//Set RTS pin
            if (ioctl(fd, TIOCMBIS, &RTS_flag) == -1) success = false;
        }
        else {//Clear RTS pin
            if (ioctl(fd, TIOCMBIC, &RTS_flag) == -1) success = false;
        }
        return success;
    }

    bool ceSerial::SetDTR(bool value) {
        long DTR_flag = TIOCM_DTR;
        bool success = true;
        if (value) {//Set DTR pin
            if (ioctl(fd, TIOCMBIS, &DTR_flag) == -1) success = false;
        }
        else {//Clear DTR pin
            if (ioctl(fd, TIOCMBIC, &DTR_flag) == -1) success = false;
        }
        return success;
    }

    bool ceSerial::GetCTS(bool& success) {
        success = true;
        long status;
        if (ioctl(fd, TIOCMGET, &status) == -1) success = false;
        return ((status & TIOCM_CTS) != 0);
    }

    bool ceSerial::GetDSR(bool& success) {
        success = true;
        long status;
        if (ioctl(fd, TIOCMGET, &status) == -1) success = false;
        return ((status & TIOCM_DSR) != 0);
    }

    bool ceSerial::GetRI(bool& success) {
        success = true;
        long status;
        if (ioctl(fd, TIOCMGET, &status) == -1) success = false;
        return ((status & TIOCM_RI) != 0);
    }

    bool ceSerial::GetCD(bool& success) {
        success = true;
        long status;
        if (ioctl(fd, TIOCMGET, &status) == -1) success = false;
        return ((status & TIOCM_CD) != 0);
    }

}