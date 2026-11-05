/*
 * viper_queue.h
 *
 *  Created on: Oct 14, 2026
 *      Author: Manil Ferr
 */

#ifndef INC_VIPER_QUEUE_H_
#define INC_VIPER_QUEUE_H_

#include <queue_lib.h>

extern Queue_TypeDef can_message_queue_viper;
extern Queue_TypeDef can_message_queue_global;
extern Queue_TypeDef can_message_queue_tx;

void MX_Queue_1_Init(void);



#endif /* INC_VIPER_QUEUE_H_ */
