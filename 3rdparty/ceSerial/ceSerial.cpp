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
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <string.h>

using namespace std;

#ifdef ceWINDOWS
#define READ_TIMEOUT 10      // milliseconds
#else
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <errno.h>
#endif

namespace ce {

    void ceSerial::Delay(unsigned long ms) {
#ifdef ceWINDOWS
        Sleep(ms);
#else
        usleep(ms * 1000);
#endif
    }

    ceSerial::ceSerial()
    {
#ifdef ceWINDOWS
        hComm = INVALID_HANDLE_VALUE;
        port = "\\\\.\\COM1";
#else
        fd = -1;
        port = "/dev/serial0";
#endif // defined
        SetBaudRate(9600);
        SetDataSize(8);
        SetParity('N');
        SetStopBits(1);
    }

    ceSerial::ceSerial(string Device, long BaudRate, long DataSize, char ParityType, float NStopBits)
    {
#ifdef ceWINDOWS
        hComm = INVALID_HANDLE_VALUE;
#else
        fd = -1;
#endif // defined
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
#ifdef ceWINDOWS
            if ((p != 'M') && (p != 'S')) p = 'N';
#else
            p = 'N';
#endif
        }
        parity = p;
    }

    char ceSerial::GetParity() {
        return parity;
    }

    void ceSerial::SetStopBits(float nbits) {
        if (nbits >= 2) stopbits = 2;
#ifdef ceWINDOWS
        else if (nbits >= 1.5) stopbits = 1.5;
#endif
        else stopbits = 1;
    }

    float ceSerial::GetStopBits() {
        return stopbits;
    }


#ifdef ceWINDOWS

    void ceSerial::SetBaudRate(long baudrate) {
        if (baudrate < 300) baud = CBR_110;
        else if (baudrate < 600) baud = CBR_300;
        else if (baudrate < 1200) baud = CBR_600;
        else if (baudrate < 2400) baud = CBR_1200;
        else if (baudrate < 4800) baud = CBR_2400;
        else if (baudrate < 9600) baud = CBR_4800;
        else if (baudrate < 14400) baud = CBR_9600;
        else if (baudrate < 19200) baud = CBR_14400;
        else if (baudrate < 38400) baud = CBR_19200;
        else if (baudrate < 57600) baud = CBR_38400;
        else if (baudrate < 115200) baud = CBR_57600;
        else if (baudrate < 128000) baud = CBR_115200;
        else if (baudrate < 256000) baud = CBR_128000;
        else baud = CBR_256000;
    }

    long ceSerial::GetBaudRate() {
        return baud;
    }

    long ceSerial::Open()
    {
        if (IsOpened()) return 0;
#ifdef UNICODE
        wstring wtext(port.begin(), port.end());
#else
        string wtext = port;
#endif
        hComm = CreateFile(wtext.c_str(),
            GENERIC_READ | GENERIC_WRITE,
            0,
            0,
            OPEN_EXISTING,
            FILE_ATTRIBUTE_NORMAL,
            0);
        if (hComm == INVALID_HANDLE_VALUE) { return 1; }

        //get initial state
        DCB dcbOri;
        bool fSuccess;
        fSuccess = GetCommState(hComm, &dcbOri);
        if (!fSuccess) { return 3; }

        DCB dcb1 = dcbOri;

        dcb1.BaudRate = baud;

        if (parity == 'E') dcb1.Parity = EVENPARITY;
        else if (parity == 'O') dcb1.Parity = ODDPARITY;
        else if (parity == 'M') dcb1.Parity = MARKPARITY;
        else if (parity == 'S') dcb1.Parity = SPACEPARITY;
        else dcb1.Parity = NOPARITY;

        dcb1.ByteSize = (BYTE)dsize;

        if (stopbits == 2) dcb1.StopBits = TWOSTOPBITS;
        else if (stopbits == 1.5) dcb1.StopBits = ONE5STOPBITS;
        else dcb1.StopBits = ONESTOPBIT;

        dcb1.fOutxCtsFlow = false;
        dcb1.fOutxDsrFlow = false;
        dcb1.fOutX = false;
        dcb1.fDtrControl = DTR_CONTROL_DISABLE;
        dcb1.fRtsControl = RTS_CONTROL_DISABLE;
        fSuccess = SetCommState(hComm, &dcb1);
        this->Delay(60);
        if (!fSuccess) { return 4; }

        fSuccess = GetCommState(hComm, &dcb1);
        if (!fSuccess) { return 5; }

        if (!GetCommTimeouts(hComm, &timeouts_ori)) { return 8; } // Error getting time-outs.
        COMMTIMEOUTS timeouts;
        timeouts.ReadIntervalTimeout = 20;
        timeouts.ReadTotalTimeoutMultiplier = 15;
        timeouts.ReadTotalTimeoutConstant = 100;
        timeouts.WriteTotalTimeoutMultiplier = 15;
        timeouts.WriteTotalTimeoutConstant = 100;
        if (!SetCommTimeouts(hComm, &timeouts)) { return 9; } // Error setting time-outs.
        return 0;
    }

    void ceSerial::Close()
    {
        if (IsOpened())
        {
            SetCommTimeouts(hComm, &timeouts_ori);
            CloseHandle(hComm);//close comm port
            hComm = INVALID_HANDLE_VALUE;
        }
    }

    bool ceSerial::IsOpened()
    {
        if (hComm == INVALID_HANDLE_VALUE) return false;
        else return true;
    }

    bool ceSerial::Flush()
    {
        if (!IsOpened()) {
            return false;
        }
        return FlushFileBuffers(hComm);
    }

    size_t ceSerial::Write(char* data, size_t length)
    {
        if (!IsOpened()) {
            return false;
        }
        DWORD bytes_written;

        // Issue write.
        if (!WriteFile(hComm, data, static_cast<DWORD>(length), &bytes_written, NULL))
        {
            return -1;
        }

        return (size_t)bytes_written;
    }

    size_t ceSerial::Read(char* buf, size_t size)
    {
        if (!IsOpened())
        {
            return 0;
        }

        DWORD bytes_read;
        if (!ReadFile(hComm, buf, static_cast<DWORD>(size), &bytes_read, NULL))
        {
            return -1;
        }
        return (size_t)bytes_read;
    }

    bool ceSerial::SetRTS(bool value)
    {
        bool r = false;
        if (IsOpened()) {
            if (value) {
                if (EscapeCommFunction(hComm, SETRTS)) r = true;
            }
            else {
                if (EscapeCommFunction(hComm, CLRRTS)) r = true;
            }
        }
        return r;
    }

    bool ceSerial::SetDTR(bool value)
    {
        bool r = false;
        if (IsOpened()) {
            if (value) {
                if (EscapeCommFunction(hComm, SETDTR)) r = true;
            }
            else {
                if (EscapeCommFunction(hComm, CLRDTR)) r = true;
            }
        }
        return r;
    }

    bool ceSerial::GetCTS(bool& success)
    {
        success = false;
        bool r = false;
        if (IsOpened()) {
            DWORD dwModemStatus;
            if (GetCommModemStatus(hComm, &dwModemStatus)) {
                r = MS_CTS_ON & dwModemStatus;
                success = true;
            }
        }
        return r;
    }

    bool ceSerial::GetDSR(bool& success)
    {
        success = false;
        bool r = false;
        if (IsOpened()) {
            DWORD dwModemStatus;
            if (GetCommModemStatus(hComm, &dwModemStatus)) {
                r = MS_DSR_ON & dwModemStatus;
                success = true;
            }
        }
        return r;
    }

    bool ceSerial::GetRI(bool& success)
    {
        success = false;
        bool r = false;
        if (IsOpened()) {
            DWORD dwModemStatus;
            if (GetCommModemStatus(hComm, &dwModemStatus)) {
                r = MS_RING_ON & dwModemStatus;
                success = true;
            }
        }
        return r;
    }

    bool ceSerial::GetCD(bool& success)
    {
        success = false;
        bool r = false;
        if (IsOpened()) {
            DWORD dwModemStatus;
            if (GetCommModemStatus(hComm, &dwModemStatus)) {
                r = MS_RLSD_ON & dwModemStatus;
                success = true;
            }
        }
        return r;
    }

#else  //for POSIX

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

        settings.c_cc[VMIN] = 1;
        settings.c_cc[VTIME] = 0;

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
#endif

} // namespace ce 