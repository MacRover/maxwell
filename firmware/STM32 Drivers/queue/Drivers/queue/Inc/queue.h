/*
 * queue.h
 *
 *  Created on: Jul 24, 2024
 *      Author: Ethan
 */

#ifndef QUEUE_INC_QUEUE_H_
#define QUEUE_INC_QUEUE_H_

#include <stdlib.h>
#include <stdint.h>

typedef struct node
{
    void *data;
    struct node *next;
} Node_TypeDef;

typedef struct queue
{
    Node_TypeDef *head;
    Node_TypeDef *tail;
} Queue_TypeDef;

Queue_TypeDef* create_queue();
void destroy_queue(Queue_TypeDef *queue);
uint8_t queue_empty(Queue_TypeDef *queue);
void queue_enqueue(Queue_TypeDef *queue, void *data);
void queue_dequeue(Queue_TypeDef *queue);
void* queue_front(Queue_TypeDef *queue);
void* queue_tail(Queue_TypeDef *queue);

#endif /* QUEUE_INC_QUEUE_H_ */
