#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#include <stdint.h>

#define MAX_SIZE 32

typedef struct {
    uint8_t data_count;
    char data_names[MAX_SIZE][MAX_SIZE];
    uint8_t *data_addresses[MAX_SIZE];

    uint8_t function_count;
    char function_names[MAX_SIZE][MAX_SIZE];
    void (*function_addresses[MAX_SIZE])(void);
} Controller;

void Controller_Init(Controller *controller);
void Controller_AddData(Controller *controller, char *data_name,
                        uint8_t *data_address);
void Controller_AddFunction(Controller *controller, char *function_name,
                            void (*function_address)(void));
uint8_t Controller_Set(Controller *controller, char *data_name, uint8_t value);
uint8_t Controller_Call(Controller *controller, char *function_name);

#endif