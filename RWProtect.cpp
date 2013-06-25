//=============================================================================
//File Name: RWProtect.cpp
//Description: Allows resource to be read by many threads but only written to
//             by one thread when no thread is reading
//Author: Tyler Veness
//=============================================================================

#include "RWProtect.hpp"

RWProtect::RWProtect() {
	m_roomEmpty = semMCreate( SEM_Q_PRIORITY | SEM_DELETE_SAFE );
	m_numReadMutex = semMCreate( SEM_Q_PRIORITY | SEM_DELETE_SAFE );

	m_numReaders = 0;
}

RWProtect::~RWProtect() {
    semDelete( m_roomEmpty );
    semDelete( m_numReadMutex );
}

void RWProtect::startRead() {
    // Lock m_numReaders mutex
    semTake( m_numReadMutex , WAIT_FOREVER );

    // Add reader to room
    m_numReaders++;

    // If this is the first reader, wait for the room to be empty
    if ( m_numReaders == 1 ) {
        semTake( m_roomEmpty , WAIT_FOREVER );
    }

    // Unlock m_numReaders mutex
    semGive( m_numReadMutex );
}

void RWProtect::stopRead() {
    // Lock m_numReaders mutex
    semTake( m_numReadMutex , WAIT_FOREVER );

    // Remove reader from room
    m_numReaders--;

    /* If this is the last reader exiting the room, no one else is there so
     * mark the room as empty
     */
    if ( m_numReaders == 0 ) {
        semGive( m_roomEmpty );
    }

    // Unlock m_numReaders mutex
    semGive( m_numReadMutex );
}

void RWProtect::startWrite() {
    // Wait for room to be empty
    semTake( m_roomEmpty , WAIT_FOREVER );
}

void RWProtect::stopWrite() {
    /* Mark room as empty since only one writer and no readers are allowed in
     * the room at one time
     */
    semGive( m_roomEmpty );
}
