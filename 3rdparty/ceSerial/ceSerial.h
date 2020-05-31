#ifndef CESERIAL_H
#define CESERIAL_H

#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <errno.h>

#define THROW(exceptionClass, message) throw exceptionClass(__FILE__, \
__LINE__, (message) )

namespace ce {

class ceSerial {
    char rxchar;
    std::string port;
    long baud;
    long dsize;
    char parity;
    float stopbits;
    long fd;
public:
    static void Delay(unsigned long ms);
    ceSerial(std::string Device, long BaudRate, long DataSize, char ParityType, float NStopBits);
    ~ceSerial();
    long Open(void);//return 0 if success
    void Close();
    bool Flush();
    size_t Read(char* buf, size_t size);
    size_t Write(char *data, size_t length);
    bool SetRTS(bool value);//return success flag
    bool SetDTR(bool value);//return success flag
    bool GetCTS(bool& success);
    bool GetDSR(bool& success);
    bool GetRI(bool& success);
    bool GetCD(bool& success);
    bool IsOpened();
    void SetPort(std::string Port);
    std::string GetPort();
    void SetBaudRate(long baudrate);
    long GetBaudRate();
    void SetDataSize(long nbits);
    long GetDataSize();
    void SetParity(char p);
    char GetParity();
    void SetStopBits(float nbits);
    float GetStopBits();
};

}

#endif