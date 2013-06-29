#include "GraphHost.hpp"
#include "graphhost_impl/graphhost.h"

// Create entry in symbol table for gettimeofday function for runtime linkage
int gettimeofday (struct timeval *tv_ptr, void *ptr);

GraphHost::GraphHost(int port) :
        m_inst( GraphHost_create(port) ) ,
        m_sendInterval( 5 ) {
    resetTime();
}

GraphHost::~GraphHost() {
	GraphHost_destroy(m_inst);
}

int GraphHost::graphData(float value, std::string dataset) {
    gettimeofday( &m_rawTime , NULL );
    m_currentTime = m_rawTime.tv_usec / 1000 + m_rawTime.tv_sec * 1000;

    return GraphHost_graphData(m_currentTime - m_startTime, value, dataset.c_str(), m_inst);
}

void GraphHost::setSendInterval( uint32_t milliseconds ) {
    m_sendInterval = milliseconds;
}

bool GraphHost::hasIntervalPassed() {
    gettimeofday( &m_rawTime , NULL );
    m_currentTime = m_rawTime.tv_usec / 1000 + m_rawTime.tv_sec * 1000;

    return m_currentTime - m_lastTime > m_sendInterval;
}

void GraphHost::resetInterval() {
    m_lastTime = m_currentTime;
}

void GraphHost::resetTime() {
    /* Store the current time into startTime as the fixed starting point
     * for our graph.
     */
    gettimeofday( &m_rawTime , NULL );
    m_startTime = m_rawTime.tv_usec / 1000 + m_rawTime.tv_sec * 1000;
    m_lastTime = m_startTime;
}
