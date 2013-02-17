#include <stdlib.h>
#include "queue.h"

struct queue_t *
queue_init(int size)
{
	struct queue_t *queue;
	
	/* Create the queue description structure */
	queue = malloc(sizeof(struct queue_t ));
	
	queue->length = 0;
	queue->size = size;
	queue->queue = malloc(sizeof(void *)*size);
	queue->start = 0;
	
	return queue;
}

int
queue_dequeue(struct queue_t *queue, void **data)
{
	/* There must be at least one element to dequeue */
	if(queue->length < 1) return -1;

	/* Get its data */
	if(data != NULL)
		*data = queue->queue[queue->start];

	/* Remove it */
    queue->start++;
    /* Wrap back to the front */
    if(queue->start >= queue->size) queue->start = 0;
	queue->length--;

	return 0;
}

int
queue_queue(struct queue_t *queue, void *data)
{

	/* Make sure that there is space in the queue to
	 * put a new element */
	if(queue->length >= queue->size) return -1;

	queue->queue[(queue->start+queue->length) % (queue->size)] = data;
	queue->length++;

	return 0;
}

void
queue_free(struct queue_t *queue)
{
	free(queue->queue);
	free(queue);

	return;
}
