/*
 * queue.c
 *
 *  Created on: Jul 25, 2024
 *      Author: Ethan
 */

#include "queue.h"

Queue_TypeDef can_message_queue_viper;
Queue_TypeDef can_message_queue_global;

void MX_Queue_1_Init(void)
{
    Queue_TypeDef *viper_queue_ptr = create_queue();
    Queue_TypeDef *global_queue_ptr = create_queue();
    if (viper_queue_ptr == NULL)
    {
//       todo handle error
        return;
    }
    if (global_queue_ptr == NULL)
    {
//       todo handle error
        return;
    }
    can_message_queue_viper = *viper_queue_ptr;
    can_message_queue_global = *global_queue_ptr;
}
