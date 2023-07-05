#include "tApcSerialPort.h" //enBaudRate
class TApcSerialTest{
public:
  int openTest(const std::string astrPortName);
  int baudrateTest(const std::string astrPortName, enBaudRate adwBaudRate);
  int sendTest(const std::string astrPortName, enBaudRate adwBaudRate, uint32_t adwWsize, uint32_t adwRsize, uint32_t adwWriteTimeout, uint32_t adwReadTimeout);
  int run(const std::string astrPortName, enBaudRate adwBaudRate);
};