#include <stdint.h>

#define __IO            volatile

typedef struct {
    __IO uint32_t MODER;
    __IO uint32_t IDR;
    __IO uint32_t ODR;
} GPIO_TypeDef;

typedef struct {
    __IO uint32_t FSR;
    __IO uint32_t FWD;
    __IO uint32_t FRD;
    __IO uint32_t BRR;
    __IO uint32_t UCR;
} UART_TypeDef;

#define APB_BASEADDR     0x10000000
#define GPIOC_BASEADDR   (APB_BASEADDR + 0x3000)
#define GPIOD_BASEADDR   (APB_BASEADDR + 0x4000)
#define UART_BASEADDR    (APB_BASEADDR + 0x7000)

#define GPIOC            ((GPIO_TypeDef *) GPIOC_BASEADDR)
#define GPIOD            ((GPIO_TypeDef *) GPIOD_BASEADDR)
#define UART             ((UART_TypeDef *) UART_BASEADDR)

void delay(int n);
void uart_init();
void uart_send_byte(uint8_t byte);
void uart_send_string(const char* str);

int main()
{
    char hi_str[] = "Hi!\n";
    uart_init();
    uart_send_byte('B');
    uart_send_byte('C');
    uart_send_byte('D');
    uart_send_byte('A');
    uart_send_string(hi_str); // string 출력은 아직 더 해봐야함

    while (1) {
    }
}

void uart_init()
{
    // UART->BRR = 0x28C1;  // 9600 bps @ 100MHz PCLK
    UART->UCR = 0x03;          // UART Enable + TX Enable (bit 0, 1)
}

void uart_send_byte(uint8_t byte)
{
    // FIFO full일 경우 대기 (선택적)
    while (UART->FSR & (1 << 1));  // FSR[1] == full

    UART->FWD = byte;
}

void uart_send_string(const char* str)
{
    while (*str) {
        uart_send_byte(*str++);
    }
}

void delay(int n)
{
    uint32_t temp = 0;
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < 1000; j++) {
            temp++;
        }
    }
}
