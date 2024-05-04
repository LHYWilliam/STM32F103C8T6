#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

#include "key.h"
#include "led.h"
#include "light_sensor.h"

int main() {
    LED led = {GPIOA, GPIO_Pin_0, HIGH};
    LED_Init(RCC_APB2Periph_GPIOA, &led);

    Key led_key = {GPIOB, GPIO_Pin_11, LOW};
    Key_Init(RCC_APB2Periph_GPIOB, &led_key);

    LightSensor light_sensor = {GPIOB, GPIO_Pin_12, HIGH};
    LightSensor_Init(RCC_APB2Periph_GPIOB, &light_sensor);

    for (;;) {
        if (LightSensor_Get(&light_sensor)) {
            LED_On(&led);
        } else {
            LED_Off(&led);
        }
    }
}
