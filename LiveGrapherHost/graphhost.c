#include <stdlib.h>
#include "list.h"
#include "queue.h"
#include "graphhost.h"
#include <string.h>
#include <strings.h>
#include <errno.h>
#include <stdio.h>
#include <pthread.h>
/* #include <stdint.h> */

#ifdef VxWorks

#include <ioLib.h>
#include <pipeDrv.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>

#include <sockLib.h>
#include <hostLib.h>
#include <selectLib.h>

#else

#include <assert.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <signal.h>
#include <sys/select.h>
#include <stropts.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>

#endif

/* Listens on a specified port (listenport), and returns the file
 * descriptor to the listening socket.
 */
int
sockets_listen_int(int port, sa_family_t sin_family, uint32_t s_addr)
{
	struct sockaddr_in serv_addr;
	int error;
	int sd;

	/* Create a TCP socket */
	sd = socket(sin_family, SOCK_STREAM, 0);
	if(sd == -1){
		perror("");
		return -1;
	}

	/* Zero out the serv_addr struct */
	bzero((char *) &serv_addr, sizeof(struct sockaddr_in));

	/* Set up the listener sockaddr_in struct */
	serv_addr.sin_family = sin_family;
	serv_addr.sin_addr.s_addr = s_addr;
	serv_addr.sin_port = htons(port);

	/* Bind the socket to the listener sockaddr_in */
	error = bind(sd, (struct sockaddr *) &serv_addr,
		sizeof(struct sockaddr_in));
	if(error != 0){
		perror("");
		close(sd);
		return -1;
	}

	/* Listen on the socket for incoming conncetions */
	error = listen(sd, 5);
	if(error != 0){
		perror("");
		close(sd);
		return -1;
	}

	/* Make sure we aren't killed by SIGPIPE */
	signal(SIGPIPE, SIG_IGN);

	return sd;
}

struct graphhost_t *
GraphHost_create(int port)
{
	/* int i; */
	struct graphhost_t *inst;
	int pipefd[2];
	int error;

	/* Allocate memory for the graphhost_t structure */
	inst = malloc(sizeof(struct graphhost_t));

	/* Mark the thread as not running, this will be set to 1 by the thread */
	inst->running = 0;

	/* Store the port to listen on */
	inst->port = port;

	/* Create a pipe for IPC with the thread */
#ifdef VxWorks
	pipeDevCreate("/pipe/graphhost", 10, 100);
	pipefd[0] = open("/pipe/graphhost", O_RDONLY, 0644);
	pipefd[1] = open("/pipe/graphhost", O_WRONLY, 0644);

	if(pipefd[0] == -1 || pipefd[1] == -1) {
		perror("");
		free(inst);
		return NULL;
	}
#else
	error = pipe(pipefd);
	if(error == -1) {
		perror("");
		free(inst);
		return NULL;
	}
#endif

	inst->ipcfd_r = pipefd[0];
	inst->ipcfd_w = pipefd[1];

	/* Launch the thread */
	if(pthread_create(&inst->thread, NULL, sockets_threadmain, (void *)inst) != 0) {
		fprintf(stderr, "pthread_create(3) failed\n");
		free(inst);
		return NULL;
	}

	return inst;
}

void
GraphHost_destroy(struct graphhost_t *inst)
{
	/* Tell the other thread to stop */
	write(inst->ipcfd_w, "x", 1);

	/* Join to the other thread */
	pthread_join(inst->thread, NULL);

	/* Close file descriptors and clean up */
	close(inst->ipcfd_r);
	close(inst->ipcfd_w);
	free(inst);
	
	return;
}

/*
struct socketconn_t *
sockets_listaddsocket(struct list_t *list, struct socketconn_t *conn)
{
	struct socketconn_t *conn;

	conn = malloc(sizeof(struct socketconn_t));
	conn->fd = fd;

	conn->selectflags = 0;
	conn->dataset = NULL;
	conn->elem = list_add_after(conn, NULL, conn);

	return conn;
}
*/

void
sockets_listremovesocket(struct list_t *list, struct socketconn_t *conn)
{
	list_delete(list, conn->elem);

	return;
}

void
sockets_accept(struct list_t *connlist, int listenfd)
{
	int new_fd;
	int on;
	socklen_t clilen;
	struct socketconn_t *conn;
	struct sockaddr_in cli_addr;
	int error;
	int flags;

	clilen = sizeof(struct sockaddr_in);

	/* Accept a new connection */
	new_fd = accept(listenfd, (struct sockaddr *) &cli_addr,
		&clilen);

	/* Make sure that the file descriptor is valid */
	if(new_fd < 1) {
		perror("");
		return;
	}

#ifdef VxWorks
	/* Set the socket non-blocking. */
	on = 1;
	error = ioctl(new_fd, (int)FIONBIO, (const int *)&on);
	if(error == -1){
		perror("");
		close(new_fd);
		return;
	}

#else

	/* Set the socket non-blocking. */
	flags = fcntl(new_fd, F_GETFL, 0);
	if(flags == -1) {
		perror("");
		close(new_fd);
		return;
	}

	error = fcntl(new_fd, F_SETFL, flags | O_NONBLOCK);
	if(error == -1) {
		perror("");
		close(new_fd);
		return;
	}

#endif

	conn = malloc(sizeof(struct socketconn_t));
	conn->fd = new_fd;
	conn->selectflags = SOCKET_READ | SOCKET_ERROR;

	conn->datasets = list_create();

	conn->writequeue = queue_init(20);
	conn->writebuf = NULL;
	conn->writebuflength = 0;
	conn->writebufoffset = 0;

	conn->readbuf = NULL;
	conn->readbuflength = 0;
	conn->readbufoffset = 0;

	conn->orphan = 0;

	/* Add it to the list, this makes it a bit non-thread-safe */
	conn->elem = list_add_after(connlist, NULL, conn);

	return;
}

/* Mark the socket for closing and deleting */
void
sockets_close(struct list_t *list, struct list_elem_t *elem)
{
	struct socketconn_t *conn = elem->data;

	conn->orphan = 1;

	return;
}

/* NOTE: Does not remove the element from the list */
void
sockets_remove_orphan(struct socketconn_t *conn)
{
	struct writebuf_t *writebuf;
	struct list_elem_t *dataset;

	/* Give up on the current write buffer */
	if(conn->writebuf != NULL) {
		free(conn->writebuf);
	}

	/* Give up on the current read buffer */
	if(conn->readbuf != NULL) {
		free(conn->readbuf);
	}

	/* Give up on all other queued buffers too */
	while(queue_dequeue(conn->writequeue, (void **)&writebuf) == 0) {
		free(writebuf->buf);
		free(writebuf);
	}

	queue_free(conn->writequeue);

	for(dataset = conn->datasets->start; dataset != NULL; dataset = dataset->next) {
		free(dataset->data);
	}

	list_destroy(conn->datasets);

	/* Free it when we get back to it, this is a hack */
	conn->orphan = 1;

	/* free(conn->dataset); */
	close(conn->fd);
	free(conn);

	return;
}

/* Closes and clears orpahans from the list */
void
sockets_clear_orphans(struct list_t *list)
{
	struct list_elem_t *elem;
	struct list_elem_t *last = NULL;
	struct socketconn_t *conn;

	for(elem = list->start; elem != NULL; ) {
		last = elem;
		elem = elem->next;

		if(last != NULL) {
			conn = last->data;
			if(conn->orphan == 1) {
				conn = last->data;
				sockets_remove_orphan(conn);
				list_delete(list, last);
			}
		}
	}

	return;
}

#if 0
int
sockets_readh(struct list_t *list, struct list_elem_t *elem)
{
	struct socketconn_t *conn = elem->data;
	char *buf;
	size_t length;
	int error;

	error = recv(conn->fd, &length, 4, 0);
	if(error < 1) {
		/* Clean up the socket here */
		sockets_close(list, elem);
		return 0;
	}

	/* Swap byte order */
	length = ntohl(length);

	/* Sanity check on the size */
	if(length > 64) {
		/* Clean up the socket here */
		sockets_close(list, elem);
		return 0;
	}

	buf = malloc(length+1);

	error = recv(conn->fd, buf, length, 0);
	if(error < 1) {
		/* Clean up the socket here */
		free(buf);
		sockets_close(list, elem);

		return 0;
	}

	buf[length] = '\0';

	conn->dataset = buf;

	return 0;
}

#endif

int
sockets_readh(struct list_t *list, struct list_elem_t *elem)
{
	struct socketconn_t *conn = elem->data;
	int error;

	if(conn->readbuflength == 0) {
		conn->readbufoffset = 0;
		conn->readbuflength = 16; /* This should be configurable somewhere */
		conn->readbuf = malloc(conn->readbuflength);
	}

	error = recv(conn->fd, conn->readbuf, conn->readbuflength - conn->readbufoffset, 0);
	if(error < 1) {
		/* Clean up the socket here */
		sockets_close(list, elem);
		return 0;
	}
	conn->readbufoffset += error;

	if(conn->readbufoffset == conn->readbuflength) {
		sockets_readdoneh(conn->readbuf, conn->readbuflength, list, elem);
		conn->readbufoffset = 0;
		conn->readbuflength = 0;
		free(conn->readbuf);
		conn->readbuf = NULL;
	}


	return 0;
}

/* Recieves 16 byte buffers which will be freed upon return */
int
sockets_readdoneh(uint8_t *inbuf, size_t bufsize, struct list_t *list, struct list_elem_t *elem)
{
	struct socketconn_t *conn = elem->data;
	char *buf;

	assert(bufsize == 16);

	inbuf[15] = 0;

	buf = strdup((char *)inbuf);
	/* buf = malloc(strlen(inbuf)+1);
	strcpy(buf, inbuf); */

	list_add_after(conn->datasets, NULL, buf);
	/* conn->dataset = buf; */

	return 0;
}


int
sockets_writeh(struct list_t *list, struct list_elem_t *elem)
{
	int error;
	struct socketconn_t *conn = elem->data;
	struct writebuf_t *writebuf;
	
	while(1) {

		/* Get another buffer to send */
		if(conn->writebuflength == 0) {
			error = queue_dequeue(conn->writequeue, (void **)&writebuf);
			/* There are no more buffers in the queue */
			if(error != 0) {
				/* Call the write finished callback in the upper layer */
				/*if(conninfo->listener->wdoneh != NULL)
					conninfo->listener->wdoneh(conninfo); */

				/* Stop selecting on write */
				conn->selectflags &= ~(SOCKET_WRITE);

				return 0;
			}
			conn->writebuf = writebuf->buf;
			conn->writebuflength = writebuf->buflength;
			conn->writebufoffset = 0;
			free(writebuf);
		}

		/* These descriptors are ready for writing */
		conn->writebufoffset += send(conn->fd, conn->writebuf, conn->writebuflength - conn->writebufoffset, 0);

		/* Have we finished writing the buffer? */
		if(conn->writebufoffset == conn->writebuflength) {

			/* Reset the write buffer */
			conn->writebuflength = 0;
			conn->writebufoffset = 0;
			free(conn->writebuf);
			conn->writebuf = NULL;
		}else{
			/* We haven't finished writing, keep selecting. */
			return 0;
		}
	
	}

	/* We always return from within the loop, this is unreachable */
	return -1;
}

/* Queue a buffer for writing. Returns 0 on success, returns -1 if buffer
 * wasn't queued. Only one buffer can be queued for writing at a time.
 */
int
sockets_queuewrite(struct graphhost_t *inst, struct socketconn_t *conn, uint8_t *buf, size_t buflength)
{
	int error;
	struct writebuf_t *writebuf;

	writebuf = malloc(sizeof(struct writebuf_t));
	writebuf->buf = malloc(buflength);
	writebuf->buflength = buflength;
	memcpy(writebuf->buf, buf, buflength);
	error = queue_queue(conn->writequeue, writebuf);
	if(error != 0) {
		free(writebuf->buf);
		free(writebuf);
		return 0;
	}

	/* Select on write */
	conn->selectflags |= SOCKET_WRITE;
	write(inst->ipcfd_w, "r", 1);

	return 0;
}

void *
sockets_threadmain(void *arg)
{
	int listenfd;
	struct graphhost_t *inst = arg;
	struct socketconn_t *conn;
	struct list_elem_t *elem;
	int maxfd;
	int fd;
	uint8_t ipccmd;

	fd_set readfds;
	fd_set writefds;
	fd_set errorfds;

	pthread_mutex_init(&inst->mutex, NULL);

	/* Create a list to store all the open connections */
	inst->connlist = list_create();

	/* Listen on a socket */
	listenfd = sockets_listen_int(inst->port, AF_INET, 0x00000000);
	if(listenfd == -1) {
		pthread_mutex_destroy(&inst->mutex);
		list_destroy(inst->connlist);
		pthread_exit(NULL);
		return NULL;
	}

	/* Set the running flag after we've finished initializing everything */
	inst->running = 1;

	while(1) {

		/* Clear the fdsets */
		FD_ZERO(&readfds);
		FD_ZERO(&writefds);
		FD_ZERO(&errorfds);

		/* Reset the maxfd */
		maxfd = listenfd;

		/* Add the file descriptors to the list */
		pthread_mutex_lock(&inst->mutex);
		for(elem = inst->connlist->start; elem != NULL;
			elem = elem->next) {
			conn = elem->data;
			fd = conn->fd;

			if(conn->orphan == 1) continue;

			if(maxfd < fd)
				maxfd = fd;
			if(conn->selectflags & SOCKET_READ)
				FD_SET(fd, &readfds);
			if(conn->selectflags & SOCKET_WRITE)
				FD_SET(fd, &writefds);
			if(conn->selectflags & SOCKET_ERROR)
				FD_SET(fd, &errorfds);
		}
		pthread_mutex_unlock(&inst->mutex);

		/* Select on the listener fd */
		FD_SET(listenfd, &readfds);

		/* ipcfd will recieve data when the thread needs to exit */
		FD_SET(inst->ipcfd_r, &readfds);

		/* Select on the file descrpitors */
		select(maxfd+1, &readfds, &writefds, &errorfds, NULL);

		pthread_mutex_lock(&inst->mutex);
		for(elem = inst->connlist->start; elem != NULL;
			elem = elem->next) {
			conn = elem->data;
			fd = conn->fd;

			if(conn->orphan == 1) continue;

			if(FD_ISSET(fd, &readfds)) {
				/* Handle reading */
				sockets_readh(inst->connlist, elem);
			}
			if(FD_ISSET(fd, &writefds)) {
				/* Handle writing */
				sockets_writeh(inst->connlist, elem);
			}
			if(FD_ISSET(fd, &errorfds)) {
				/* Handle errors */
				sockets_close(inst->connlist, elem);
			}
		}

		/* Close all the file descriptors marked for closing */
		sockets_clear_orphans(inst->connlist);
		pthread_mutex_unlock(&inst->mutex);

		/* Check for listener condition */
		if(FD_ISSET(listenfd, &readfds)) {
			/* Accept connections */
			sockets_accept(inst->connlist, listenfd);
		}

		/* Handle IPC commands */
		if(FD_ISSET(inst->ipcfd_r, &readfds)) {
			read(inst->ipcfd_r, &ipccmd, 1);
			if(ipccmd == 'x') {
				break;
			}
		}
	}

	/* We're done, clear the running flag and clean up */
	inst->running = 0;
	pthread_mutex_lock(&inst->mutex);

	/* Mark all the open file descriptors for closing */
	for(elem = inst->connlist->start; elem != NULL;
		elem = elem->next) {
		sockets_close(inst->connlist, elem);
		/* We don't need to delete the element form the
		   because we just delete all of them below. */
	}

	/* Actually close all the open file descriptors */
	sockets_clear_orphans(inst->connlist);

	/* Free the list */
	list_destroy(inst->connlist);

	/* Close the listener file descriptor */
	close(listenfd);

	/* Destroy the mutex */
	pthread_mutex_unlock(&inst->mutex);
	pthread_mutex_destroy(&inst->mutex);

	pthread_exit(NULL);
	return NULL;
}

int
GraphHost_graphData(float x, float y, const char *dataset, struct graphhost_t *graphhost)
{
	struct list_elem_t *elem;
	struct list_elem_t *datasetp;
	struct socketconn_t *conn;
	struct graph_payload_t payload;
	/* union floatint_t floatint; */

	/* struct graph_payload_t* qpayload; */
	char *dataset_str;
	uint32_t tmp;

	/* Safety first */
	assert(sizeof(float) == sizeof(uint32_t));

	if(!graphhost->running) return -1;

	/* Zero the payload structure */
	memset((void *)&payload, 0x00, sizeof(struct graph_payload_t));

	/* Change to network byte order */
	memcpy(&tmp, &x, sizeof(float));
	tmp = htonl(tmp);
	memcpy(&payload.x, &tmp, sizeof(float));

	memcpy(&tmp, &y, sizeof(float));
	tmp = htonl(tmp);
	memcpy(&payload.y, &tmp, sizeof(float));

	/*
	floatint.f = x;
	floatint.i = htonl(floatint.i);
	payload.x = floatint.f;

	floatint.f = y;
	floatint.i = htonl(floatint.i);
	payload.y = floatint.f;
	*/

	/*
	tmp = htonl(*((uint32_t *)&x));
	payload.x = *((float *)&tmp);
	tmp = htonl(*((uint32_t *)&y));
	payload.y = *((float *)&tmp);
	*/

	strncpy(payload.dataset, dataset, 15);

	/* Giant lock approach */
	pthread_mutex_lock(&graphhost->mutex);

	for(elem = graphhost->connlist->start; elem != NULL; elem = elem->next) {
		conn = elem->data;
		for(datasetp = conn->datasets->start; datasetp != NULL; datasetp = datasetp->next) {
			dataset_str = datasetp->data;
			if(dataset_str != NULL && strcmp(dataset_str, dataset) == 0) {
				/* Send the value off */
				/* qpayload = malloc(sizeof(struct graph_payload_t));
				memcpy(qpayload, &payload, sizeof(struct graph_payload_t)); */
				sockets_queuewrite(graphhost, conn, (void *)&payload, sizeof(struct graph_payload_t));
			}
		}
	}

	pthread_mutex_unlock(&graphhost->mutex);

	return 0;
}
