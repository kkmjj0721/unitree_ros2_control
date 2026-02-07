#include <unistd.h>
#include "unitree_actuator_sdk/serialPort/SerialPort.h"
#include "unitree_actuator_sdk/unitreeMotor/unitreeMotor.h"


int main(int argc,char** argv) 
{

  SerialPort  serial("/dev/ttyUSB0");
  MotorCmd    cmd;
  MotorData   data;

  while(true) 
  {
    cmd.motorType = MotorType::GO_M8010_6;
    data.motorType = MotorType::GO_M8010_6;
    cmd.mode = queryMotorMode(MotorType::GO_M8010_6,MotorMode::FOC);
    cmd.id   = 0;                                                         // id
    cmd.kp   = 0.0;                                                       // p
    cmd.kd   = 0.01;                                                      // d
    cmd.q    = 0.0;                                                       // pos
    cmd.dq   = -6.28*queryGearRatio(MotorType::GO_M8010_6);               // 速度
    cmd.tau  = 0.0;                                                       // 力矩
    serial.sendRecv(&cmd,&data);

    std::cout <<  std::endl;
    std::cout <<  "motor.q: "    << data.q    <<  std::endl;
    std::cout <<  "motor.temp: "   << data.temp   <<  std::endl;
    std::cout <<  "motor.W: "      << data.dq      <<  std::endl;
    std::cout <<  "motor.merror: " << data.merror <<  std::endl;
    std::cout <<  std::endl;

    usleep(1000);
  }

}