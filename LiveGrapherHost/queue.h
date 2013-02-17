#ifndef _QUEUE_H
#define _QUEUE_H

struct queue_t {
	void **queue;
	int size;
	int length;
	int start;
};

struct queue_t *
queue_init(int size);

int
queue_dequeue(struct queue_t *queue, void **data);

int
queue_queue(struct queue_t *queue, void *data);

void
queue_free(struct queue_t *queue);

#endif
