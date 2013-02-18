#ifndef _GRAPHHOST_H
#define _GRAPHHOST_H

/* #define VxWorks */

#ifdef __cplusplus
extern "C" {
#endif

#ifndef VxWorks
#include <stdint.h>
#endif

#include "list.h"
#include "queue.h"
#include <sys/socket.h>
#include <sys/types.h>
#include <pthread.h>

struct graphhost_t {
	pthread_t thread;
	pthread_mutex_t mutex;
	uint8_t running;
	int ipcfd_r;
	int ipcfd_w;
	int port;
	struct list_t *connlist;
};

struct socketconn_t {
	int fd;
	struct list_elem_t *elem;

	/* char *dataset; */
	struct list_t *datasets;
	uint8_t selectflags;

	int orphan;

	/* Write buffer currently being written */
	uint8_t *writebuf; /* The buffer that needs to be written into the socket */
	size_t writebuflength; /* The length of the buffer to be written */
	size_t writebufoffset; /* How much has been written so far */
	struct queue_t *writequeue;

	/* Read buffer currently being read */
	uint8_t *readbuf;
	size_t readbuflength;
	size_t readbufoffset;
};

struct writebuf_t {
	/* Write buffer */
	uint8_t *buf; /* The buffer that needs to be written into the socket */
	uint32_t buflength; /* The length of the buffer to be written */
};

struct graph_payload_t {
	char dataset[16];
	float x;
	float y;
};

#define SOCKET_READ 1
#define SOCKET_WRITE 2
#define SOCKET_ERROR 4

struct graphhost_t *
GraphHost_create(int port);

void
GraphHost_destroy(struct graphhost_t *inst);

int
GraphHost_graphData(float x, float y, const char *dataset, struct graphhost_t *graphhost);

/* Internal functions */

int
sockets_listen_int(int port, sa_family_t sin_family, uint32_t s_addr);

void
sockets_listremovesocket(struct list_t *list, struct socketconn_t *conn);

void
sockets_accept(struct list_t *connlist, int listenfd);

void
sockets_close(struct list_t *list, struct list_elem_t *elem);

void
sockets_remove_orphan(struct socketconn_t *conn);

void
sockets_clear_orphans(struct list_t *list);

int
sockets_readh(struct list_t *list, struct list_elem_t *elem);

int
sockets_readdoneh(uint8_t *inbuf, size_t bufsize, struct list_t *list, struct list_elem_t *elem);

int
sockets_writeh(struct list_t *list, struct list_elem_t *elem);

int
sockets_queuewrite(struct graphhost_t *inst, struct socketconn_t *conn, uint8_t *buf, size_t buflength);

void *
sockets_threadmain(void *arg);

#ifdef __cplusplus
}
#endif

#endif /* _GRAPHHOST_H */
