/*
 * queue.c
 *
 *  Created on: Jul 24, 2024
 *      Author: Ethan
 *
 */

#include "queue_lib.h"

Queue_TypeDef* create_queue()
{
    Queue_TypeDef *new_queue = malloc(sizeof(Queue_TypeDef));
    if (new_queue == NULL)
    {
        return NULL;
    }
    new_queue->head = NULL;
    new_queue->tail = NULL;
    new_queue->size = 0;
    return new_queue;
}

void destroy_queue(Queue_TypeDef *queue)
{
    while (!queue_empty(queue))
    {
        queue_dequeue(queue);
    }
//    free(queue);
}

uint8_t queue_empty(Queue_TypeDef *queue)
{
    if (queue->head == NULL && queue->tail == NULL)
    {
        return 1;
    }
    return 0;
}

void queue_enqueue(Queue_TypeDef *queue, void *new_data)
{
    if (queue->size >= MAX_QUEUE_SIZE)
    {
    	return;
    }
    
    queue->size++;

    if (queue->tail == NULL)
    {
        // handle empty queue
        queue->tail = malloc(sizeof(Node_TypeDef));
        if (queue->tail == NULL)
        {
            // todo handle mem error
            return;
        }
        queue->tail->data = new_data;
        queue->tail->next = NULL;
        // update head too
        queue->head = queue->tail;
        return;
    }
    queue->tail->next = malloc(sizeof(Node_TypeDef));
    if (queue->tail->next == NULL)
    {
        // todo handle mem error
        return;
    }
    queue->tail->next->data = new_data;
    queue->tail->next->next = NULL;
    queue->tail = queue->tail->next;

}

void queue_dequeue(Queue_TypeDef *queue)
{
    if (queue->head != NULL)
    {
        Node_TypeDef *temp = queue->head;
        queue->head = queue->head->next;
        // if the queue is now empty, update the tail too
        if (queue->head == NULL)
        {
            queue->tail = NULL;
        }
        free(temp->data);
        free(temp);

        queue->size--;
    }
}

void* queue_front(Queue_TypeDef *queue)
{
    if (queue->head == NULL)
    {
        return NULL;
    }

    return queue->head->data;
}

void* queue_tail(Queue_TypeDef *queue)
{
    if (queue->tail == NULL)
    {
        return NULL;
    }

    return queue->tail->data;
}
