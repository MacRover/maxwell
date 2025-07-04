/*
 * queue.h
 *
 *  Created on: Jul 25, 2024
 *      Author: Ethan
 */

#ifndef INC_QUEUE_H_
#define INC_QUEUE_H_

#include <queue_lib.h>

extern Queue_TypeDef can_message_queue_viper;
extern Queue_TypeDef can_message_queue_global;
extern Queue_TypeDef can_message_queue_tx;

void MX_Queue_1_Init(void);

#endif /* INC_QUEUE_H_ */
