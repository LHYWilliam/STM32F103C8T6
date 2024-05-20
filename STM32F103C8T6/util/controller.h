#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#include <stdint.h>

typedef enum { DATA, FUNCTION } EvalType;

#define MAX_SIZE 32

typedef struct {
    uint8_t data_count;
    char data_names[MAX_SIZE][MAX_SIZE];
    void *data_addresses[MAX_SIZE];

    uint8_t function_count;
    char function_names[MAX_SIZE][MAX_SIZE];
    void *function_addresses[MAX_SIZE];
} Controller;

void Controller_Init(Controller *controller);

void Controller_Add(Controller *controller, char *name, void *address,
                    EvalType type);
void *Controller_Eval(Controller *controller, char *name, EvalType type);

#endif