#include <ServoSmooth.h>
#include <modbus.h>
#include <modbusDevice.h>
#include <modbusRegBank.h>
#include <modbusSlave.h>

ServoSmooth servo;
modbusDevice regBank;
modbusSlave slave;

uint32_t tmr;
boolean flag;

void setup() 
{
  regBank.setId(1);
  regBank.add(30001);
  regBank.add(30002);

  slave._device = &regBank;  
  slave.setBaud(115200); 

  servo.attach(3);        // подключить
  servo.setSpeed(5); //5   // ограничить скорость
  servo.setAccel(0.5);   	  // установить ускорение (разгон и торможение)

}

int lowerPos = 67;
int upperPos = 127;

void loop() {
  servo.tick();

  if (millis() - tmr >= 10000) {   // 10// каждые 3 сек
    tmr = millis();
    flag = !flag;
    servo.setTargetDeg(flag ? lowerPos : upperPos);  // двигаем на углы 50 и 120
  }

  regBank.set(30001, (word)(servo.getCurrentDeg()-7));//-0.122173)); // -7 гр

  slave.run();
}