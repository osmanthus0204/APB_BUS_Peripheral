#include <stdint.h>

#define __IO volatile

typedef struct {
    __IO uint32_t MODER;
    __IO uint32_t ODR;
} GPO_TypeDef;

typedef struct {
    __IO uint32_t MODER;
    __IO uint32_t IDR;
} GPI_TypeDef;

typedef struct {
    __IO uint32_t MODER;
    __IO uint32_t IDR;
    __IO uint32_t ODR;
} GPIO_TypeDef;

typedef struct {
    __IO uint32_t FCR;
    __IO uint32_t FDR;
    __IO uint32_t FPR;
} FND_TypeDef;

typedef struct {
    __IO uint32_t TCR;
    __IO uint32_t TCNT;
    __IO uint32_t PSC;
    __IO uint32_t ARR;
} TIM_TypeDef;

typedef struct {
    __IO uint32_t FSR;
    __IO uint32_t FWD;
    __IO uint32_t FRD;
    __IO uint32_t BRR;
    __IO uint32_t UCR;
} UART_TX_TypeDef;

typedef struct {
    __IO uint32_t DCR;
    __IO uint32_t DLR;
    __IO uint32_t DDR;
    __IO uint32_t digit_1000;
    __IO uint32_t digit_100;
    __IO uint32_t digit_10;
    __IO uint32_t digit_1;
} DHT11_TypeDef;

typedef struct {
    __IO uint32_t USR;
    __IO uint32_t UDR;
    __IO uint32_t UAR;
} ULT_TypeDef;

// ult_vehicle struct 
typedef struct {
    uint32_t vehicle_count;
    uint32_t entrance_state;
    uint32_t exit_state;
    uint32_t entrance_stable_count;
    uint32_t exit_stable_count;
} VehicleCounter;

#define APB_BASEADDR        0x10000000
#define GPOA_BASEADDR       (APB_BASEADDR + 0x1000)
#define GPIB_BASEADDR       (APB_BASEADDR + 0x2000)
#define GPIOC_BASEADDR      (APB_BASEADDR + 0x3000)
#define GPIOD_BASEADDR      (APB_BASEADDR + 0x4000)
#define FNDE_BASEADDR       (APB_BASEADDR + 0x5000)
#define TIM_BASEADDR        (APB_BASEADDR + 0x6000)
#define UART_RX_BASEADDR    (APB_BASEADDR + 0x7000)
#define UART_TX_BASEADDR    (APB_BASEADDR + 0x8000)
#define DHT11F_BASEADDR     (APB_BASEADDR + 0x9000)
#define ULT1_BASEADDR       (APB_BASEADDR + 0x10000)
#define ULT2_BASEADDR       (APB_BASEADDR + 0x11000)

#define GPOA              ((GPO_TypeDef *) GPOA_BASEADDR)
#define GPIB              ((GPI_TypeDef *) GPIB_BASEADDR)
#define GPIOC             ((GPIO_TypeDef *) GPIOC_BASEADDR)
#define GPIOD             ((GPIO_TypeDef *) GPIOD_BASEADDR)
#define FNDE              ((FND_TypeDef *) FNDE_BASEADDR)
#define TIM               ((TIM_TypeDef *) TIM_BASEADDR)
#define UART_TX           ((UART_TX_TypeDef *) UART_TX_BASEADDR)
#define DHT11F            ((DHT11_TypeDef *) DHT11F_BASEADDR)
#define ULT1              ((ULT_TypeDef *) ULT1_BASEADDR)
#define ULT2              ((ULT_TypeDef *) ULT2_BASEADDR)

#define FND_OFF           0
#define FND_ON            1
#define FND_DP            4

#define FND_CTRL_PIN      0   // GPIOC[0] - FND ON/OFF
#define MODE_SEL_PIN      1   // GPIOC[1] - 0: 온도 / 1: 습도
#define START_BTN_PIN     4   // GPIOC[4] - Start 버튼

// Setting Value
#define THRESHOLD_mm 150  // 차량 감지 임계값 (mm)
#define MIN_STABLE_COUNT 2 // 

// -------------delay------------
void delay(int n);
// -------------LED------------
void LED_init(GPIO_TypeDef * GPIOx);
void LED_write(GPIO_TypeDef * GPIOx, uint32_t data);
// -------------Switch------------
uint32_t Switch_read(GPIO_TypeDef *GPIOx); // sinik
void Switch_init(GPIO_TypeDef * GPIOx);
uint32_t get_fnd_ctrl(GPIO_TypeDef * GPIOx);
uint32_t get_mode_sel(GPIO_TypeDef * GPIOx);
void Button_init(GPIO_TypeDef * GPIOx);
uint32_t get_start_trigger(GPIO_TypeDef * GPIOx, uint32_t *prev_state);
// -------------FND------------
void FND_init(FND_TypeDef * FND, uint32_t ON_OFF);
void FND_writeData(FND_TypeDef * FND, uint32_t ctrl, uint32_t data);
// --------- UART_TX------------------
void uart_tx_init();
void uart_tx_send_byte(uint8_t byte);
void uart_send_string(const char* str);
// -------------TIM------------
void TIM_start(TIM_TypeDef * tim);
void TIM_stop(TIM_TypeDef * tim);
uint32_t TIM_readCounter(TIM_TypeDef * tim);
void TIM_writePrescaler(TIM_TypeDef * tim, uint32_t psc);
void TIM_writeAutoReload(TIM_TypeDef * tim, uint32_t arr);
void TIM_clear(TIM_TypeDef * tim);
uint32_t TIM_DHT11_AUTO_10sec(uint32_t *prevTime);
// --------- Ultrasonic sensor------------------
void ULT_init(ULT_TypeDef *ult, uint32_t ON_OFF);
uint32_t ULT_read_USR(ULT_TypeDef *ult);
uint32_t ULT_distance(ULT_TypeDef *ult);
uint32_t ULT_read_UAR(ULT_TypeDef *ult);
void ULT_uart_tx_data(ULT_TypeDef *ult);
void VehicleCounter_init(VehicleCounter *vc);
uint32_t ULT_vehicle(VehicleCounter *vc, ULT_TypeDef *ult_ent, ULT_TypeDef *ult_out);
// -------------DHT11-------------
uint32_t DHT11_led_read(DHT11_TypeDef * DHT11);
uint32_t DHT11_data_read(DHT11_TypeDef * DHT11);
void DHT11_write(DHT11_TypeDef * DHT11, uint32_t dht11_trig);
uint32_t DHT11_to_fnd(uint32_t data);
uint32_t DHT11_digit_1000(DHT11_TypeDef * DHT11);
uint32_t DHT11_digit_100(DHT11_TypeDef * DHT11);
uint32_t DHT11_digit_10(DHT11_TypeDef * DHT11);
uint32_t DHT11_digit_1(DHT11_TypeDef * DHT11);
uint32_t DHT11_uart_data(DHT11_TypeDef * DHT11);

int main()
{
    uint8_t DHT11_ASCII_BYTE[4];
    uint32_t fnd_ctrl = 0;
    uint32_t mode_sel = 0;
    uint32_t start_btn = 0;
    uint32_t start_auto = 0;
    uint32_t prev_start_btn = 0;
    uint32_t dht11_PrevTime = 0;
    uint32_t temp_hum_data = 0;
    uint32_t dht11_led = 0;
    uint32_t dht11_digit_1000 = 0;
    uint32_t dht11_digit_100 = 0;
    uint32_t dht11_digit_10 = 0;
    uint32_t dht11_digit_1 = 0;

    uint32_t ascii = 0;

    LED_init(GPIOD);
    Switch_init(GPIOC);
    Button_init(GPIOC);
    FND_init(FNDE, FND_ON); // 초기화는 ON으로 시작

    TIM_writePrescaler(TIM, 100000-1);
    TIM_writeAutoReload(TIM, 0xFFFFFFFF);
    TIM_start(TIM);

    uart_tx_init();

    // Ultra sensor
    VehicleCounter vc;
    VehicleCounter_init(&vc);
    ULT_init(ULT1,1);
    ULT_init(ULT2,1);
    uint32_t temp;

    while (1) {
        uint32_t ten_over = 0;
        fnd_ctrl = get_fnd_ctrl(GPIOC);          // GPIOC[0]
        mode_sel = get_mode_sel(GPIOC);          // GPIOC[1]
        // start_btn = get_start_trigger(GPIOC, &prev_start_btn);    // GPIOC[4]
        start_auto = TIM_DHT11_AUTO_10sec(&dht11_PrevTime);
        if (fnd_ctrl) {
            ascii = ULT_vehicle(&vc, ULT1, ULT2);
            FND_init(FNDE, FND_ON);

            // ---------ULT--------
            uart_tx_send_byte('C');
            uart_tx_send_byte('U');
            uart_tx_send_byte('R');
            uart_tx_send_byte('R'); 
            uart_tx_send_byte('E');
            uart_tx_send_byte('N');
            uart_tx_send_byte('T');     
            uart_tx_send_byte(' ');
            uart_tx_send_byte('V');
            uart_tx_send_byte('E');
            uart_tx_send_byte('H');
            uart_tx_send_byte('I');
            uart_tx_send_byte('C');
            uart_tx_send_byte('L');
            uart_tx_send_byte('E');
            uart_tx_send_byte(':');
            uart_tx_send_byte(' ');
            while(ascii>=10) {
                ascii = ascii - 10;
                ten_over++;
            }
            uart_tx_send_byte(ten_over+'0');
            uart_tx_send_byte(ascii+'0');
            uart_tx_send_byte('\n');
            // ---------------------

            if (start_auto) { 
                uint32_t dht_ctrl = (mode_sel << 1) | 0x01;
                DHT11_write(DHT11F, dht_ctrl);
                temp_hum_data = (DHT11_data_read(DHT11F));
                // DHT11_uart_data(DHT11F);
                FND_writeData(FNDE, FND_DP, temp_hum_data);
                dht11_led = DHT11_led_read(DHT11F);
                LED_write(GPIOD, dht11_led);
            }
        } 
        else {
            FND_init(FNDE, FND_OFF);
            FND_writeData(FNDE, 0, 0x0000);
            LED_write(GPIOD, 0x00); // 모든 LED OFF
        }
    }
    return 0;
}

void delay(int n)
{
    for (int i = 0; i < n; i++)
        for (int j = 0; j < 1000; j++);
}

void LED_init(GPIO_TypeDef * GPIOx) {
    GPIOx->MODER = 0xFF;
}

void LED_write(GPIO_TypeDef * GPIOx, uint32_t data) {
    GPIOx->ODR = data;
}

// void Switch_init(GPIO_TypeDef * GPIOx) {
//     GPIOx->MODER &= ~((1 << (FND_CTRL_PIN * 2)) | (1 << (MODE_SEL_PIN * 2)));  // 둘 다 입력
// }

void Switch_init(GPIO_TypeDef *GPIOx)
{
    GPIOx->MODER = 0x00;
}

// sinik
uint32_t Switch_read(GPIO_TypeDef *GPIOx)
{
    return GPIOx->IDR;
}

uint32_t get_fnd_ctrl(GPIO_TypeDef * GPIOx) {
    return (GPIOx->IDR >> FND_CTRL_PIN) & 0x1;
}

uint32_t get_mode_sel(GPIO_TypeDef * GPIOx) {
    return (GPIOx->IDR >> MODE_SEL_PIN) & 0x1;
}

void Button_init(GPIO_TypeDef * GPIOx) {
    GPIOx->MODER &= ~(1 << (START_BTN_PIN * 2));  // input mode
}

uint32_t get_start_trigger(GPIO_TypeDef * GPIOx, uint32_t *prev_state) {
    uint32_t curr_state = (GPIOx->IDR >> START_BTN_PIN) & 0x1;

    uint32_t triggered = (curr_state == 1 && *prev_state == 0) ? 1 : 0;

    *prev_state = curr_state;
    return triggered;
}

void FND_init(FND_TypeDef * FND, uint32_t ON_OFF) {
    FND->FCR = ON_OFF;
}

void FND_writeData(FND_TypeDef * FND, uint32_t ctrl, uint32_t data) {
    FND->FPR = ctrl;
    FND->FDR = data;
}

// ------------TIMER----------------
void TIM_start(TIM_TypeDef * tim){
    tim->TCR |= (1<<0); // set enable bit
}
void TIM_stop(TIM_TypeDef * tim){
    tim->TCR &= ~(1<<0); // reset enable bit
}
uint32_t TIM_readCounter(TIM_TypeDef * tim){
    return tim->TCNT;
}
void TIM_writePrescaler(TIM_TypeDef * tim, uint32_t psc){
    tim->PSC = psc;
}
void TIM_writeAutoReload(TIM_TypeDef * tim, uint32_t arr){
    tim->ARR = arr;
}
void TIM_clear(TIM_TypeDef * tim){
    tim->TCR |= (1<<1); // set clear bit;
    tim->TCR &= ~(1<<1); // reset clear bit;
}

// ----------------UART_TX---------------------
void uart_tx_init()
{
    UART_TX->UCR = 0x03;         
}

void uart_tx_send_byte(uint8_t byte)
{
    // while (UART_TX->FSR & (1 << 1));  // FSR[1] == full

    while (UART_TX->FSR & (1 << 3));  // FSR[3] == tx_busy

    UART_TX->FWD = byte;
}

void uart_send_string(const char* str)
{
    while (*str) {
        uart_tx_send_byte(*str++);
    }
}

// ------------DHT11----------------
// 0507
uint32_t TIM_DHT11_AUTO_10sec(uint32_t *prevTime){
    uint32_t curTime = TIM_readCounter(TIM);
    if (curTime - *prevTime >= 10000) {
        *prevTime = curTime;
        return 1;
    }
    return 0;
}

uint32_t DHT11_led_read(DHT11_TypeDef * DHT11) {
    return DHT11->DLR;
}
uint32_t DHT11_data_read(DHT11_TypeDef * DHT11) {
    return DHT11->DDR;
}
void DHT11_write(DHT11_TypeDef * DHT11, uint32_t dht11_trig) {
    DHT11->DCR = dht11_trig;
}
uint32_t DHT11_digit_1000(DHT11_TypeDef * DHT11) {
    return DHT11->digit_1000;
}
uint32_t DHT11_digit_100(DHT11_TypeDef * DHT11) {
    return DHT11->digit_100;
}
uint32_t DHT11_digit_10(DHT11_TypeDef * DHT11) {
    return DHT11->digit_10;
}
uint32_t DHT11_digit_1(DHT11_TypeDef * DHT11) {
    return DHT11->digit_1;
}
uint32_t DHT11_uart_data(DHT11_TypeDef * DHT11) {
    if(get_mode_sel(GPIOC)) {
                    uart_tx_send_byte('H');
                    uart_tx_send_byte('u');
                    uart_tx_send_byte('m');
                    uart_tx_send_byte('i');
                    uart_tx_send_byte('d');
                    uart_tx_send_byte(' ');
                    uart_tx_send_byte('=');
                    uart_tx_send_byte(' ');
                }
                else{
                    uart_tx_send_byte('T');
                    uart_tx_send_byte('e');
                    uart_tx_send_byte('m');
                    uart_tx_send_byte('p');
                    uart_tx_send_byte('e');
                    uart_tx_send_byte('r');
                    uart_tx_send_byte('a');
                    uart_tx_send_byte('t');
                    uart_tx_send_byte('u');
                    uart_tx_send_byte('r');
                    uart_tx_send_byte('e');
                    uart_tx_send_byte(' ');
                    uart_tx_send_byte('=');
                    uart_tx_send_byte(' ');

                }
                uart_tx_send_byte('0'+(DHT11_digit_1000(DHT11F)));
                uart_tx_send_byte('0'+(DHT11_digit_100(DHT11F)));
                uart_tx_send_byte('.');
                uart_tx_send_byte('0'+(DHT11_digit_10(DHT11F)));
                uart_tx_send_byte('0'+(DHT11_digit_1(DHT11F)));
                if(get_mode_sel(GPIOC)) {
                    uart_tx_send_byte('%');
                }
                else{
                    uart_tx_send_byte('C');
                }
                uart_tx_send_byte('\n');
}

// ------------UltraSensor----------------
void ULT_init(ULT_TypeDef *ult, uint32_t ON_OFF)
{
    ult->USR = ON_OFF;
}

uint32_t ULT_distance(ULT_TypeDef *ult){
    return ult->UDR;
}

uint32_t ULT_read_USR(ULT_TypeDef *ult){
    return ult->USR;
}
uint32_t ULT_read_UAR(ULT_TypeDef *ult){
    return ult->UAR;
}

void ULT_uart_tx_data(ULT_TypeDef *ult)
{
    uart_tx_send_byte('D');
    uart_tx_send_byte('S');
    uart_tx_send_byte('T');
    uart_tx_send_byte(':');
    uart_tx_send_byte(ULT_read_UAR(ult)>>(24) & 0xff);
    uart_tx_send_byte(ULT_read_UAR(ult)>>(16) & 0xff);
    uart_tx_send_byte(ULT_read_UAR(ult)>>(8) & 0xff);
    uart_tx_send_byte(ULT_read_UAR(ult) & 0xff);
    uart_tx_send_byte('m');
    uart_tx_send_byte('m');
    uart_tx_send_byte('\n'); 
}

// struct initial function
void VehicleCounter_init(VehicleCounter *vc) {
    vc -> vehicle_count = 0;
    vc -> entrance_state = 0;
    vc -> exit_state = 0;
    vc -> entrance_stable_count = 0;
    vc -> exit_stable_count = 0;
}

uint32_t ULT_vehicle(VehicleCounter *vc, ULT_TypeDef *ult_ent, ULT_TypeDef *ult_out)
{           uint32_t entrance_dst = ULT_distance(ult_ent); // 센서 거리 읽기
            uint32_t exit_dst = ULT_distance(ult_out);

            // 입구 센서 처리
            if (entrance_dst < THRESHOLD_mm) {
            // 물체 감지됨
                if (vc -> entrance_state == 0) { 
                    vc -> entrance_stable_count++;
                    if (vc -> entrance_stable_count >= MIN_STABLE_COUNT) {
                        vc -> entrance_state = 1;  // 상태 변경: 차량 감지
                        vc -> entrance_stable_count = 0;
                    }
                } else {
                    vc -> entrance_stable_count = 0;
                }
            } else { // 물체가 감지되지 않음
                if (vc -> entrance_state == 1) {
                    vc -> entrance_stable_count++;
                    if (vc -> entrance_stable_count >= MIN_STABLE_COUNT) {
                        vc -> entrance_state = 0;  // 상태 변경: 차량 통과
                        vc -> entrance_stable_count = 0;
                        
                        // 차량이 완전히 통과함 -> 카운트 증가
                        vc -> vehicle_count++;
                    }
                } else {
                    vc -> entrance_stable_count = 0;
                }
            }
        
            // 출구 센서 처리
            if (exit_dst < THRESHOLD_mm) {
            // 물체 감지됨
                if (vc -> exit_state == 0) {
                    vc -> exit_stable_count++;
                    if (vc -> exit_stable_count >= MIN_STABLE_COUNT) {
                        vc -> exit_state = 1;  // 상태 변경: 차량 감지
                        vc -> exit_stable_count = 0;
                    }
                } else {
                    vc -> exit_stable_count = 0;
                }
            } else {
                // 물체가 감지되지 않음
                if (vc -> exit_state == 1) {
                    vc -> exit_stable_count++;
                    if (vc -> exit_stable_count >= MIN_STABLE_COUNT) {
                        vc -> exit_state = 0;  // 상태 변경: 차량 통과
                        vc -> exit_stable_count = 0;
                        
                        // 차량이 완전히 통과함 -> 카운트 감소
                            if(vc -> vehicle_count > 0) { vc -> vehicle_count--;}
                    }
                } else {
                    vc -> exit_stable_count = 0;
                }
            }
    return vc -> vehicle_count;
}