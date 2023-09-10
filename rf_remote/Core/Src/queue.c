#include <assert.h>
#include <memory.h>
#include "queue.h"

Queue *queue_create(size_t capacity, size_t type_size) {
    assert(capacity > 0);
    assert(type_size > 0);
    Queue *q = malloc(sizeof(Queue));
    if(q == NULL){
        return NULL;
    }
    q->data = malloc(capacity * type_size);
    if(q->data == NULL){
        free(q);
        return NULL;
    }
    q->front_idx = 0;
    q->back_idx = 0;
    q->capacity = capacity;
    q->size = 0;
    q->type_size = type_size;
    return q;
}

bool queue_empty(const Queue *q) {
    return q->size == 0;
}

bool queue_full(const Queue *q) {
    return q->size >= q->capacity;
}

bool queue_push(Queue *q, const void *element) {
    if (queue_full(q)) {
        return false;
    }

    memcpy(q->data + (q->back_idx * q->type_size), element, q->type_size);
    q->back_idx = (q->back_idx + 1) % q->capacity;
    q->size++;
    return true;
}

void *queue_get(Queue *q) {
    assert(!queue_empty(q));
    return (q->data + (q->front_idx * q->type_size));
}

bool queue_pop(Queue *q) {
    if (queue_empty(q)) {
        return false;
    }
    q->size--;
    q->front_idx = (q->front_idx + 1) % q->capacity;
    return true;
}

void queue_destroy(Queue *q) {
    free(q->data);
    free(q);
}
