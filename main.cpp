#include "tApcSerialPort.h"
#include "tApcSerialTest.h"
int main(){
  TApcSerialTest T;
  T.run("/dev/ttyS0",enBaudRate::b9600);
  return 0; 
}
