#include "TApcSerialPort.h"
#include "TApcSerialTest.h"
int main(){
  TApcSerialTest T;
  T.run("/dev/ttyS0",b9600);
  return 0; 
}
