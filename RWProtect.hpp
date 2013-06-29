//=============================================================================
//File Name: RWProtect.hpp
//Description: Allows resource to be read by many threads but only written to
//             by one thread when no thread is reading
//Author: FRC Team 3512, Spartatroniks
//=============================================================================

#ifndef RW_PROTECT_HPP
#define RW_PROTECT_HPP

#include <semLib.h>

class RWProtect {
public:
    RWProtect();
    virtual ~RWProtect();

    void startRead();
    void stopRead();

    void startWrite();
    void stopWrite();

private:
    SEM_ID m_roomEmpty;

    // Protects m_numReaders
    SEM_ID m_numReadMutex;

    unsigned int m_numReaders;
};

#endif // RW_PROTECT_HPP
