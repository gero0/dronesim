#ifndef TIETO_TASK_QUEUE_H
#define TIETO_TASK_QUEUE_H

#include <stddef.h>
#include <malloc.h>
#include <stdbool.h>
#include <stdint.h>

/**
 * Queue allows storing and receiving data in FIFO order.
 * Queue receives type size as a parameter, allowing it to store any type.
 * The size of queue is fixed at creation and it will not accept more elements when it's full.
 * Members of this struct should not be modified directly.
 * Use appropriate functions to interact with the queue.
 */
typedef struct Queue {
    uint8_t *data;
    size_t front_idx;
    size_t back_idx;
    size_t capacity;
    size_t size;
    size_t type_size;
} Queue;

/**
 * Creates new queue on the heap.
 * @param capacity - number of elements queue can store at once. Must be greater than 0.
 * @param type_size - size of type to be stored. Must be greater than 0.
 * @return pointer to created queue. May return NULL if any of the required allocations fail.
 */
Queue *queue_create(size_t capacity, size_t type_size);

/**
 * Checks if queue is empty.
 * @param q pointer to queue
 * @return true if queue is empty, false otherwise.
 */
bool queue_empty(const Queue *q);

/**
 * Checks if queue is full.
 * @param q pointer to queue
 * @return true if queue is full, false otherwise.
 */
bool queue_full(const Queue *q);

/**
 * Inserts a copy of the element into the queue.
 * @param q pointer to queue
 * @param element pointer to element to be inserted
 * @return true if element was inserted, false if insertion failed (queue is full)
 */
bool queue_push(Queue *q, const void *element);

/**
 * Returns pointer to first element in the queue.
 * Make sure to check if the queue isn't empty before calling this function!
 * @param q pointer to queue
 * @return pointer to first element in the queue
 */
void *queue_get(Queue *q);

/**
 * Remove first element from the queue
 * @param q pointer to queue
 * @return true if an element was removed, false otherwise (queue empty)
 */
bool queue_pop(Queue *q);

/**
 * Deletes the queue and frees allocated resources
 * @param q pointer to the queue
 */
void queue_destroy(Queue *q);


#endif //TIETO_TASK_QUEUE_H
