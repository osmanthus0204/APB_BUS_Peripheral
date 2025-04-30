### ultrasonic_sensor 모듈 


  <img src="https:https://github.com/osmanthus0204/APB_BUS_Peripheral/tree/main/250429_Peripherals_UltrasonicSensor.srcs/ultrasonic_sensor_ASM.png" width=800px>
TRIGGER FSM module과 ECHO FSM module을 별도로 구현하지 않고
TRIGGER와 ECHO가 하나의 FSM module로 구현한 ver
실시간성을 강조하기 위해 start신호의 입력을 항상 1로 받는 상황을 가정함.

---
### verilog


APB_SlaveIntf_Ultrasonic_Sensor내에는 

slv_reg0 ,slv_reg1 두 개의 register 생성

usr //ultrasonic start register
udr //ultrasonic distance register

---

### c언어 
int main() {
    Switch_init(GPIOD);

    uint32_t temp;
    ULT_init(ULT,ON); 					//start 신호를 주기 위해 ULT_init(ULT,ON); 사용
    while(1) {
            if(Switch_read(GPIOD) == 0x00) { //GPIOD 입력이 없으면 slv_reg0(USR)에 저장된 데이터 FND에 출력
            temp = ULT_read_USR(ULT);
            FND_writeData(FND, temp);
            FND_writeDOT(FND,(1<<3));
            } 
            else {					//GPIOD 입력이 있으면 slv_reg1(UDR)에 저장된 데이터 FND에 출력
            temp = ULT_distance(ULT);
            FND_writeData(FND, temp);
            FND_writeDOT(FND,(1<<1));
            }
        }

}
 
