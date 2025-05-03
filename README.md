# APB_BUS_Peripheral
250429 ~ 250506

---
##주요 기능

1. RX통해 초음파센서1,2 , 온습도센서 키고 끄기 제어(FND등은 켜둠)

2. 초음파1(입구), 초음파2(출구) 통해 터널 내 차량 대수 UART TX통해 컴퓨터 화면에 출력

3. 초음파1(입구) 쪽 실시간 거리 측정 값 FND에 출력

4. 온습도 값 TX 통해 컴퓨터 화면에 출력

---
##메모리 매핑

0x1000_0000 ROM
0x1000_0FFF RESERVERD
0x1000_1000 GPOA
0x1000_2000 GPIB
0x1000_3000 GPIOC
0x1000_4000 GPIOD
0x1000_5000 FND
0x1000_6000 TIM
0x1000_7000 uart rx
0x1000_8000 uart tx
0x1000_9000 Temp/Humid
0x1001_0000 UltraSonic1
0x1001_1000 UltraSonic2

---
##FPGA BOARD Port

### Switches
GPIB[0] ~ GPIB[7] : Switch[0] ~ Switch[7]
GPIOD[0] ~ GPIOD[7] : Switch[8] ~ Switch[15]

### LEDs
GPOA[0] ~ GPOA[7] : LED[0] ~ LED[7]
GPIOC[0] ~ GPIOC[7] : LED[8] ~ LED[15]

###7 Segment Display
fndFont[0] ~ fndFont[7]
fndCom[0] ~ fndCom[3]

###Buttons
 reset          : U18  (BTNC)
 GPIOD[4]   : T18   (BTNU)
 GPIOD[5]   : W19  (BTNL)
 GPIOD[6]   : T17   (BTNR)
 GPIOD[7]   : U17  (BTND)


###USB-RS232 Interface
rx
tx

###Pmod_PinOut
<img src="https://github.com/osmanthus0204/APB_BUS_Peripheral/tree/main/Pmod_Pin-Out.png" width=500px>