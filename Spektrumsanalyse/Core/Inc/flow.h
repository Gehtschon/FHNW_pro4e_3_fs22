#ifndef FLOW_H
#define FLOW_H
#include <stdbool.h>
#include "../../Core/Inc/stm32-hal-rfm95/rfm95.h"



void Flow(rfm95_handle_t *handle);
void FlowInit(rfm95_handle_t *handle);


#endif
