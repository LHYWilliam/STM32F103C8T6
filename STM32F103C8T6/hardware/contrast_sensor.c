#include "contrast_sensor.h"

void ContrastSensor_Init(ContrastSensor *sensor) { GPIO_Init_(sensor->gpio); }