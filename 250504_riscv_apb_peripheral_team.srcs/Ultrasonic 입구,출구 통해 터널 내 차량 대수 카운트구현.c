#include <stdint.h>


#define __IO            volatile

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

} UART_RX_TypeDef;

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

// typedef struct {
//     __IO uint32_t USR;
//     __IO uint32_t UDR;
//     __IO uint32_t UAR_1000;
//     __IO uint32_t UAR_100;
//     __IO uint32_t UAR_10;
//     __IO uint32_t UAR_1;
// } ULT_TypeDef;

// ult_vehicle struct 
typedef struct {
    uint32_t vehicle_count;
    uint32_t entrance_state;
    uint32_t exit_state;
    uint32_t entrance_stable_count;
    uint32_t exit_stable_count;
} VehicleCounter;

#define APB_BASEADDR    0x10000000
#define GPOA_BASEADDR   (APB_BASEADDR + 0x1000)
#define GPIB_BASEADDR   (APB_BASEADDR + 0x2000)
#define GPIOC_BASEADDR   (APB_BASEADDR + 0x3000)
#define GPIOD_BASEADDR   (APB_BASEADDR + 0x4000)
#define FND_BASEADDR    (APB_BASEADDR + 0x5000)
#define TIM_BASEADDR    (APB_BASEADDR + 0x6000)
#define UART_RX_BASEADDR    (APB_BASEADDR + 0x7000)
#define UART_TX_BASEADDR    (APB_BASEADDR + 0x8000)
#define DHT11F_BASEADDR    (APB_BASEADDR + 0x9000)
#define ULT1_BASEADDR    (APB_BASEADDR + 0x10000)
#define ULT2_BASEADDR    (APB_BASEADDR + 0x11000)


#define GPOA            ((GPO_TypeDef *) GPOA_BASEADDR)
#define GPIB            ((GPI_TypeDef *) GPIB_BASEADDR)
#define GPIOC            ((GPIO_TypeDef *) GPIOC_BASEADDR)
#define GPIOD            ((GPIO_TypeDef *) GPIOD_BASEADDR)
#define FND             ((FND_TypeDef *) FND_BASEADDR)
#define TIM             ((TIM_TypeDef *) TIM_BASEADDR)

#define UART_RX             ((UART_RX_TypeDef *) UART_RX_BASEADDR)
#define UART_TX             ((UART_TX_TypeDef *) UART_TX_BASEADDR)
#define DHT11F            ((DHT11_TypeDef *) DHT11F_BASEADDR)
#define ULT1             ((ULT_TypeDef *) ULT1_BASEADDR)
#define ULT2             ((ULT_TypeDef *) ULT2_BASEADDR)

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
void delay (int n);
// -------------LED------------
void LED_init(GPIO_TypeDef *GPIOx);
void LED_write(GPIO_TypeDef *GPIOx, uint32_t data);

// -------------Switch------------
void Switch_init(GPIO_TypeDef *GPIOx);
uint32_t Switch_read(GPIO_TypeDef *GPIOx);

// -------------FND------------
void FND_init(FND_TypeDef *fnd, uint32_t ON_OFF);
void FND_writeCom(FND_TypeDef *fnd, uint32_t comport);
void FND_writeData(FND_TypeDef *fnd, uint32_t data);
void FND_writeDOT(FND_TypeDef *fnd, uint32_t DOT);

// -------------TIM------------
void TIM_start(TIM_TypeDef *tim);
void TIM_stop(TIM_TypeDef *tim);
uint32_t TIM_readCounter(TIM_TypeDef *tim);
void TIM_writePrescaler(TIM_TypeDef *tim, uint32_t psc);
void TIM_writeAutoReload(TIM_TypeDef *tim,uint32_t arr);
void TIM_clear(TIM_TypeDef *tim);

// -------------Button-----------
void button_init(GPIO_TypeDef *GPIOx);
uint32_t button_read(GPIO_TypeDef *GPIOx);
uint32_t button_toggle(GPIO_TypeDef *GPIOx, uint32_t Button, uint32_t toggle);

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

// --------- UART_TX------------------
void uart_tx_init();
void uart_tx_send_byte(uint8_t byte);
void uart_tx_send_string(const char* str);





int main()
{

    VehicleCounter vc;
    VehicleCounter_init(&vc);
    LED_init(GPIOC);
    Switch_init(GPIOD);
    ULT_init(ULT1,1);
    ULT_init(ULT2,1);
    FND_init(FND, FND_ON);
    uint32_t temp;
    uart_tx_init();
    while(1) { 
            if( Switch_read(GPIOD) & (1<<1)) {
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
                uart_tx_send_byte(ULT_vehicle(&vc, ULT1, ULT2)+'0');
                uart_tx_send_byte('\n');
            } 

            if(Switch_read(GPIOD) & (1<<0)) {
                FND_writeData(FND, ULT_distance(ULT1));
                FND_writeDOT(FND,(1<<1));
                LED_write(GPIOC,(1<<7));
            }else {             
                FND_writeData(FND, ULT_distance(ULT2));
                FND_writeDOT(FND,(1<<1));
                LED_write(GPIOC,(1<<0));
            }
           

        
            // if(Switch_read(GPIOD) == 0x00) {
            // temp = ULT_read_USR(ULT1);
            // FND_writeData(FND, temp);
            // FND_writeDOT(FND,(1<<3));
            // } 
            // else {
                
            // temp = ULT_distance(ULT1);
            // FND_writeData(FND, temp);
            // FND_writeDOT(FND,(1<<1));
            // ULT_uart_tx_data(ULT1);
            // }     
    
    }
    
    
    return 0;

}




/*=================================
 ULT function
 =================================*/

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
/*=================================
 FND function
 =================================*/

void FND_init(FND_TypeDef *fnd, uint32_t ON_OFF)
{
    fnd->FCR = ON_OFF;
}

void FND_writeData(FND_TypeDef *fnd, uint32_t data)
{
    fnd->FDR = data;
}

void FND_writeDOT(FND_TypeDef *fnd, uint32_t DOT)
{
    fnd->FPR = DOT;
}

/*=================================
 timer function
 =================================*/


void TIM_start(TIM_TypeDef *tim){
    tim-> TCR |= (1<<0); //set enable bit
}

void TIM_stop(TIM_TypeDef *tim){
    tim -> TCR &= ~(1<<0); // reset enable bit
}

uint32_t TIM_readCounter(TIM_TypeDef *tim){
    return tim->TCNT;
}

void TIM_writePrescaler(TIM_TypeDef *tim, uint32_t psc){
    tim -> PSC = psc;
}

void TIM_writeAutoReload(TIM_TypeDef *tim,uint32_t arr){
    tim -> ARR = arr;
}

void TIM_clear(TIM_TypeDef *tim){
    tim -> TCR = (1<<1); //set clear bit;
    tim -> TCR &= ~(1<<1); //reset clear bit;
}

/*=================================
 button driver
 =================================*/


void button_init(GPIO_TypeDef *GPIOx)
{
    GPIOx->MODER &= (0xdf);
}

uint32_t button_read(GPIO_TypeDef *GPIOx){
    return GPIOx->IDR;
}


uint32_t button_toggle(GPIO_TypeDef *GPIOx, uint32_t Button, uint32_t toggle){
    enum {IDLE, SETUP, TOGGLE};
    uint32_t state = IDLE;
    switch(state)
		{ 	case IDLE:
				if (button_read(GPIOx) == Button) state = SETUP;
				else state = IDLE;
			break;
			case SETUP:
				if (button_read(GPIOx) == 0) state = TOGGLE;
				else state = SETUP;
			break;
			case TOGGLE:
                state = IDLE;
                return ~toggle;
			break;
		}
}


/*=================================
 LED function
 =================================*/
void LED_init(GPIO_TypeDef *GPIOx)
{
    GPIOx->MODER = 0xff;
}



void LED_write(GPIO_TypeDef *GPIOx, uint32_t data)
{
    GPIOx->ODR = data;
}


/*=================================
 Switch function
 =================================*/
void Switch_init(GPIO_TypeDef *GPIOx)
{
    GPIOx->MODER = 0x00;
}

uint32_t Switch_read(GPIO_TypeDef *GPIOx)
{
    return GPIOx->IDR;
}

/*=================================
UART_TX function
 =================================*/


void uart_tx_init()
{
    // UART->BRR = 0x28C1;  // 9600 bps @ 100MHz PCLK
    UART_TX->UCR = 0x03;          // UART Enable + TX Enable (bit 0, 1)
}

void uart_tx_send_byte(uint8_t byte)
{
    // FIFO full일 경우 대기 
    while (UART_TX->FSR & (1 << 1));  // FSR[1] == full

    UART_TX->FWD = byte;
}

void uart_tx_send_string(const char* str)
{
    while (*str) {
        uart_tx_send_byte(*str++);
    }
}

/*=================================
 delay function
 =================================*/
void delay (int n)
{
    uint32_t temp = 0;
    for (int i=0; i<n; i++) {
        for (int j=0; j<1000; j++){
            temp++;
        }
    }
}
/*=================================
 DHT-11 function etc
 =================================*/
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


/*=================================
 DHT-11 function
 =================================*/
uint32_t TIM_DHT11_AUTO_10sec(uint32_t *prevTime){
    uint32_t curTime = TIM_readCounter(TIM);
    uint32_t data = 0;
    if(curTime - *prevTime == 10000) {
        *prevTime = curTime;
        data ^= 1<<0;
    }
    else{
        data ^= 0<<0;
    }
    return data; 
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