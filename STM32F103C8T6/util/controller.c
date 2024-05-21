#include <stdint.h>
#include <string.h>

#include "controller.h"

void Controller_Init(Controller *controller) {
    controller->data_count = 0;
    controller->function_count = 0;
}

void Controller_Add(Controller *controller, char *name, void *address,
                    EvalType type) {
    if (type == DATA) {
        strcpy(controller->data_names[controller->data_count], name);
        controller->data_addresses[controller->data_count] = address;
        controller->data_count++;
    } else if (type == FUNCTION) {
        strcpy(controller->function_names[controller->function_count], name);
        controller->function_addresses[controller->function_count] = address;
        controller->function_count++;
    }
}

void *Controller_Eval(Controller *controller, char *name, EvalType type) {
    void *goal;
    uint8_t i = 0, length;
    if (type == DATA) {
        length = controller->data_count;
    } else if (type == FUNCTION) {
        length = controller->function_count;
    }

    while (i < length) {
        if (type == DATA && strcmp(controller->data_names[i], name) == 0) {
            goal = controller->data_addresses[i];
            break;
        } else if (type == FUNCTION &&
                   strcmp(controller->function_names[i], name) == 0) {
            goal = controller->function_addresses[i];
            break;
        }
        i++;
    }

    return goal;
}