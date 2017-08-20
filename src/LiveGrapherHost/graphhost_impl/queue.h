// Copyright (c) 2017 FRC Team 3512. All Rights Reserved.

#pragma once

#include "list.h"

struct queue_t {
    struct list_t* list;
    struct list_elem_t* last;
    int size;
    int length;
};

struct queue_t* queue_init(int size);

int queue_dequeue(struct queue_t* queue, void** data);

int queue_queue(struct queue_t* queue, void* data);

void queue_free(struct queue_t* queue);
