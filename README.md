
# RISC_V_APB_BUS_Peripheral_TEAM_PROJECT 
- 250429 ~ 250506

---
## 주요 기능

1. RX통해 초음파센서1,2 , 온습도센서 키고 끄기 제어(FND등은 켜둠)

2. 초음파1(입구), 초음파2(출구) 통해 터널 내 차량 대수 UART TX통해 컴퓨터 화면에 출력

3. 초음파1(입구) 쪽 실시간 거리 측정 값 FND에 출력

4. 온습도 값 TX 통해 컴퓨터 화면에 출력

---
## 메모리 매핑

- 0x1000_0000 RAM 
    - logic [31:0] mem[0:2**10-1]
- 0x1000_0FFF RESERVERD </br> </br></br></br>

- 0x1000_1000 GPOA 
    - logic [31:0] slv_reg0, slv_reg1; </br>
    - assign moder = slv_reg0[7:0]; </br>
    - assign odr   = slv_reg1[7:0];</br> // assign outPort[i] = moder[i] ? odr[i] : 1'bz;  </br> </br>
- 0x1000_2000 GPIB
    - logic [31:0] slv_reg0, slv_reg1; </br> 
    - assign moder = slv_reg0[7:0]; </br>
    - assign slv_reg1[7:0] = idr; </br> // assign idr[i] = ~moder[i] ? inPort[i] : 1'bz; </br> </br>
- 0x1000_3000 GPIOC
    - logic [31:0] slv_reg0, slv_reg1, slv_reg2; </br> 
    - assign moder = slv_reg0[7:0]; </br>
    - assign slv_reg1[7:0] = idr; </br>
    - assign odr           = slv_reg2[7:0]; </br> // assign outPort[i] = moder[i] ? odr[i] : 1'bz;</br>// assign idr[i] = ~moder[i] ? inPort[i] : 1'bz; </br> </br>
- 0x1000_4000 GPIOD
    -  상동</br></br>
- 0x1000_5000 FND
    - logic [31:0] slv_reg0, slv_reg1, slv_reg2; </br>
    - assign fcr = slv_reg0[0];  // ON,OFF</br>
    - assign fdr = slv_reg1[13:0]; // DATA</br>
    - assign fpr = slv_reg2[ 3:0]; //Decimal point </br> </br>
- 0x1000_6000 TIM
    - logic [31:0] slv_reg0, slv_reg1, slv_reg2, slv_reg3; </br> 
    - assign en       = slv_reg0[0]; //enable </br>
    - assign clear    = slv_reg0[1]; //clear </br>
    - assign slv_reg1 = tcnt;        //time_counter_data </br>
    - assign psc      = slv_reg2;    //prescaler </br>
    - assign arr      = slv_reg3;    //Auto-Reload  Register - ounter's maximum value </br></br>
- 0x1000_7000 uart rx
    - logic [31:0] slv_reg0, slv_reg1, slv_reg2;</br>
    - assign slv_reg0[31:0] = {30'b0, fsr}; //{full, empty} - FIFO state register </br>
    - assign slv_reg1[31:0] = {24'b0, frd}; // read data register </br></br>
- 0x1000_8000 uart tx
    - logic [31:0] slv_reg0, slv_reg1, slv_reg2, slv_reg3, slv_reg4; </br>
    - assign slv_reg0[2:0] = FSR; // {tx_done, full, empty} - FIFO state register </br>
    - assign FWD = slv_reg1[7:0]; // write data 
    - assign slv_reg2[7:0] = FRD; // read data </br>
    - assign BRR = slv_reg3[15:0]; </br>// Baud Rate Register -> [15:4] DIV_Mantissa(정수부), [3:0] DIV_Fraction </br>
    - assign UCR = slv_reg4[3:0]; </br>// UART Control Register -> 0: UART enable, 1: UART_tx enable, 2: UART_rx enalbe, 3: UART_tx Trigger </br></br>

- 0x1000_9000 Temp/Humid
    - logic [31:0] slv_reg0, slv_reg1, slv_reg2, slv_reg3, slv_reg4, slv_reg5, slv_reg6;</br>
    - assign dcr = slv_reg0[9:0];</br> //{empty_rx_b, uart_trig[7:0], modesel, start} </br>
    - assign slv_reg1[4:0] = dlr; // led input
    -     if (dht_done) begin
                slv_reg0[10:0]  <= 2'b00;  
                // start 비트 자동 클리어
                slv_reg2[15:0] <= ddr; 
                // sensor data(temp or hum)  
                dht_data_valid <= 1'b1;  
                // 유효 플래그 설정
                slv_reg3       <= digit_1000;
                slv_reg4       <= digit_100;
                slv_reg5       <= digit_10;
                slv_reg6       <= digit_1;
            end
- 0x1001_0000 UltraSonic1
    - reg [31:0] slv_reg0, slv_reg1, slv_reg2;
    - assign usr = slv_reg0[0]; 
    //ultrasonic start register
    - assign slv_reg1[11:0] = udr;
    //ultrasonic distance register
    - assign slv_reg2[31:0] = uar;
    //ultrasonic ascii registe </br>uar = {ascii_digit_1000, ascii_digit_100, ascii_digit_10, ascii_digit_1};</br></br>
- 0x1001_1000 UltraSonic2
    -  상동</br></br>
---
## FPGA BOARD Port

### Switches
- GPIB[0] ~ GPIB[7] : Switch[0] ~ Switch[7]
- GPIOD[0] ~ GPIOD[3] : Switch[8] ~ Switch[11]

### LEDs
- GPOA[0] ~ GPOA[7] : LED[0] ~ LED[7]
- GPIOC[0] ~ GPIOC[7] : LED[8] ~ LED[15]

### 7 Segment Display
- fndFont[0] ~ fndFont[7]
- fndCom[0] ~ fndCom[3]

### Buttons
- reset          : U18  (BTNC)
- GPIOD[4]   : T18   (BTNU)
- GPIOD[5]   : W19  (BTNL)
- GPIOD[6]   : T17   (BTNR)
- GPIOD[7]   : U17  (BTND)


### USB-RS232 Interface
- rx
- tx

### Pmod_PinOut
- <img src="https://github.com/osmanthus0204/APB_BUS_Peripheral/blob/main/Pmod_Pin-Out.png" width=500px>
