#pragma once
#include <cerrno>
#include <cstdlib>     
#include <string>
#include <cstring>
#include <termios.h>    // выставление флагов в структуре  (режим работы  порта)
#include <unistd.h>     // usleep, close(), write(), read();
#include <poll.h>       // набор дескрипторов
#include <fcntl.h>      // open()
#include <stdint.h>     // uint8_t и uint32_t
#include <iostream>

speed_t ConvertBaudRate(uint32_t adwBaudRate);

class TApcSerialPort{

  public:
    TApcSerialPort(){
    m_FileHandle =-1;
  };
  TApcSerialPort(uint32_t fd){
    m_FileHandle =fd;
  };
  ~TApcSerialPort(){};

    int get_FH();
    int file_open(const std::string astrPortPathName);
    int file_close();
    int SetDefaultSettings(uint32_t adwBaudRate);
    int write(uint8_t* apBuf, size_t astSize, uint32_t adwTimeout, size_t& astWritten);
    int read(uint8_t* apBuf, size_t astSize, uint32_t adwTimeout, size_t& astRlen);
  private:
   int m_FileHandle;
};
