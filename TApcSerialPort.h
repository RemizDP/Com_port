#pragma once
#include <string>       // работа с типом string

enum baud_t {b1200, b2400, b4800, b9600};

class TApcSerialPort{

  public:
    TApcSerialPort();;
  TApcSerialPort(uint32_t adwFileHandle);

    int get_handle();
    int file_open(const std::string astrPortPathName);
    int file_close();
    int configure_settings(baud_t adwBaudRate);
    int write(uint8_t* apBuf, size_t astSize, uint32_t adwTimeout, size_t& astWritten);
    int read(uint8_t* apBuf, size_t astSize, uint32_t adwTimeout, size_t& astRlen);
  private:
   int m_FileHandle;
};
