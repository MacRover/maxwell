/*
 * queue.c
 *
 *  Created on: Jul 25, 2024
 *      Author: Ethan
 */

#include "queue.h"

Queue_TypeDef can_message_queue_1;

void MX_Queue_1_Init(void)
{
    Queue_TypeDef *queue_ptr = create_queue();
    if (queue_ptr == NULL)
    {
//       todo handle error
        return;
    }
    can_message_queue_1 = *queue_ptr;
}
