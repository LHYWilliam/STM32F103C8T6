#include "string.h"

#include "controller.h"
#include <stdint.h>

void Controller_Init(Controller *controller) {
    controller->data_count = 0;
    controller->function_count = 0;
}

void Controller_AddData(Controller *controller, char *data_name,
                        uint8_t *data_address) {
    strcpy(controller->data_names[controller->data_count], data_name);
    controller->data_addresses[controller->data_count] = data_address;
    controller->data_count++;
}

void Controller_AddFunction(Controller *controller, char *function_name,
                            void (*function_address)(void)) {
    strcpy(controller->function_names[controller->function_count],
           function_name);
    controller->function_addresses[controller->function_count] =
        function_address;
    controller->function_count++;
}

uint8_t Controller_Set(Controller *controller, char *data_name, uint8_t value) {
    uint8_t i = 0;
    while (i < controller->data_count) {
        if (strcmp(controller->data_names[i], data_name) == 0) {
            *controller->data_addresses[i] = value;
            break;
        }
        i++;
    }

    return i == controller->data_count;
}

uint8_t Controller_Call(Controller *controller, char *function_name) {
    uint8_t i = 0;
    while (i < controller->function_count) {
        if (strcmp(controller->function_names[i], function_name) == 0) {
            controller->function_addresses[i]();
            break;
        }
        i++;
    }

    return i == controller->function_count;
}
