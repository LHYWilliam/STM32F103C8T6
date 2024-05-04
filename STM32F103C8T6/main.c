#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

#include "contrast_sensor.h"
#include "oled.h"

int main() {
    OLED_Init();

    ContrastSensor sensor = {GPIOB, GPIO_Pin_13};
    ContrastSensor_Init(RCC_APB2Periph_GPIOB, &sensor);

    for (;;) {
        OLED_ShowNum(1, 1, ContrastSensor_Get(), 3);
    }
}
