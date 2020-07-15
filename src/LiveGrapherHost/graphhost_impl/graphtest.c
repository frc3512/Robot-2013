// Copyright (c) 2017-2020 FRC Team 3512. All Rights Reserved.

/* A very simple test harness for GraphHost */

#ifndef VxWorks

#include <signal.h>
#include <stdlib.h>
#include <unistd.h>

#include "graphhost.h"

/* main function */
int main(void) {
    struct graphhost_t* gh;

    /* Ignore SIGPIPE */
    signal(SIGPIPE, SIG_IGN);

    /* Create a GraphHost */
    gh = GraphHost_create(4098);

    /* Send some bogus data */
    while (1) {
        GraphHost_graphData(0, 0, "PID0", gh);
        sleep(1);
    }

    /* NOTREACHED */
    GraphHost_destroy(gh);

    return 0;
}

#endif /* VxWorks */
