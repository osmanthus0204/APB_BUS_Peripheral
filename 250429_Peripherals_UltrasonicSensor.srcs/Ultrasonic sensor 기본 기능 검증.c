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
    __IO uint32_t USR;
    __IO uint32_t UDR;
} ULT_TypeDef;


#define APB_BASEADDR    0x10000000
#define GPOA_BASEADDR   (APB_BASEADDR + 0x1000)
#define GPIB_BASEADDR   (APB_BASEADDR + 0x2000)
#define GPIOC_BASEADDR   (APB_BASEADDR + 0x3000)
#define GPIOD_BASEADDR   (APB_BASEADDR + 0x4000)
#define FND_BASEADDR    (APB_BASEADDR + 0x5000)
#define TIM0_BASEADDR    (APB_BASEADDR + 0x6000)
#define ULT_BASEADDR    (APB_BASEADDR + 0x7000)


#define GPOA            ((GPO_TypeDef *) GPOA_BASEADDR)
#define GPIB            ((GPI_TypeDef *) GPIB_BASEADDR)
#define GPIOC            ((GPIO_TypeDef *) GPIOC_BASEADDR)
#define GPIOD            ((GPIO_TypeDef *) GPIOD_BASEADDR)
#define FND             ((FND_TypeDef *) FND_BASEADDR)
#define TIM0             ((TIM_TypeDef *) TIM0_BASEADDR)
#define ULT             ((ULT_TypeDef *) ULT_BASEADDR)


#define OFF             0
#define ON              1



void delay (int n);
void LED_init(GPIO_TypeDef *GPIOx);
void LED_write(GPIO_TypeDef *GPIOx, uint32_t data);
void Switch_init(GPIO_TypeDef *GPIOx);
uint32_t Switch_read(GPIO_TypeDef *GPIOx);

void FND_init(FND_TypeDef *fnd, uint32_t ON_OFF);
void FND_writeCom(FND_TypeDef *fnd, uint32_t comport);
void FND_writeData(FND_TypeDef *fnd, uint32_t data);
void FND_writeDOT(FND_TypeDef *fnd, uint32_t DOT);

void TIM_start(TIM_TypeDef *tim);
void TIM_stop(TIM_TypeDef *tim);
uint32_t TIM_readCounter(TIM_TypeDef *tim);
void TIM_writePrescaler(TIM_TypeDef *tim, uint32_t psc);
void TIM_writeAutoReload(TIM_TypeDef *tim,uint32_t arr);
void TIM_clear(TIM_TypeDef *tim);

void button_init(GPIO_TypeDef *GPIOx);
uint32_t button_read(GPIO_TypeDef *GPIOx);

uint32_t button_toggle(GPIO_TypeDef *GPIOx, uint32_t Button, uint32_t toggle);


void ULT_init(ULT_TypeDef *ult, uint32_t ON_OFF);
uint32_t ULT_read_USR(ULT_TypeDef *ult);
uint32_t ULT_distance(ULT_TypeDef *ult);

int main()
{
    Switch_init(GPIOD);
    ULT_init(ULT,ON);
    uint32_t temp;

    while(1) {
        FND_init(FND, ON);
        
            if(Switch_read(GPIOD) == 0x00) {
            temp = ULT_read_USR(ULT);
            FND_writeData(FND, temp);
            FND_writeDOT(FND,(1<<3));
            } 
            else {
            temp = ULT_distance(ULT);
            FND_writeData(FND, temp);
            FND_writeDOT(FND,(1<<1));
            }
        
        }

        


        // temp = Switch_read(GPIOC);
        // delay(100);
        // if (temp & (1<<0)) {
        //     LED_write(GPIOD, temp);
        // }
        // else if (temp & (1<<1)) {
        //     LED_write(GPIOD, one);
        //     one = (one << 1) | (one >> 7);
        //     delay(500);
        // }
        // else if (temp & (1<<2)) {
        //     LED_write(GPIOD, one);
        //     one = (one >> 1) | (one << 7);
        //     delay(500);
        // }
        // else {
        //     LED_write(GPIOD, 0xff);
        //     delay(500); 
        //     LED_write(GPIOD, 0x00);
        //     delay(500); 
        // }
    
    return 0;

}


/*=================================
 delay function
 =================================*/
// void delay (int n)
// {
//     uint32_t temp = 0;
//     for (int i=0; i<n; i++) {
//         for (int j=0; j<1000; j++){
//             temp++;
//         }
//     }
// }

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


// void TIM_start(TIM_TypeDef *tim){
//     tim-> TCR |= (1<<0); //set enable bit
// }

// void TIM_stop(TIM_TypeDef *tim){
//     tim -> TCR &= ~(1<<0); // reset enable bit
// }

// uint32_t TIM_readCounter(TIM_TypeDef *tim){
//     return tim->TCNT;
// }

// void TIM_writePrescaler(TIM_TypeDef *tim, uint32_t psc){
//     tim -> PSC = psc;
// }

// void TIM_writeAutoReload(TIM_TypeDef *tim,uint32_t arr){
//     tim -> ARR = arr;
// }

// void TIM_clear(TIM_TypeDef *tim){
//     tim -> TCR = (1<<1); //set clear bit;
//     tim -> TCR &= ~(1<<1); //reset clear bit;
// }

/*=================================
 button driver
 =================================*/


// void button_init(GPIO_TypeDef *GPIOx)
// {
//     GPIOx->MODER &= (0xdf);
// }

// uint32_t button_read(GPIO_TypeDef *GPIOx){
//     return GPIOx->IDR;
// }


// uint32_t button_toggle(GPIO_TypeDef *GPIOx, uint32_t Button, uint32_t toggle){
//     enum {IDLE, SETUP, TOGGLE};
//     uint32_t state = IDLE;
//     switch(state)
// 		{ 	case IDLE:
// 				if (button_read(GPIOx) == Button) state = SETUP;
// 				else state = IDLE;
// 			break;
// 			case SETUP:
// 				if (button_read(GPIOx) == 0) state = TOGGLE;
// 				else state = SETUP;
// 			break;
// 			case TOGGLE:
//                 state = IDLE;
//                 return ~toggle;
// 			break;
// 		}
// }


/*=================================
 LED function
 =================================*/
// void LED_init(GPIO_TypeDef *GPIOx)
// {
//     GPIOx->MODER = 0x01;
// }



// void LED_write(GPIO_TypeDef *GPIOx, uint32_t data)
// {
//     GPIOx->ODR = data;
// }


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