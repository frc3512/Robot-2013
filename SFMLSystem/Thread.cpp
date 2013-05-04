////////////////////////////////////////////////////////////
//
// SFML - Simple and Fast Multimedia Library
// Copyright (C) 2007-2012 Laurent Gomila (laurent.gom@gmail.com)
//
// This software is provided 'as-is', without any express or implied warranty.
// In no event will the authors be held liable for any damages arising from the use of this software.
//
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it freely,
// subject to the following restrictions:
//
// 1. The origin of this software must not be misrepresented;
//    you must not claim that you wrote the original software.
//    If you use this software in a product, an acknowledgment
//    in the product documentation would be appreciated but is not required.
//
// 2. Altered source versions must be plainly marked as such,
//    and must not be misrepresented as being the original software.
//
// 3. This notice may not be removed or altered from any source distribution.
//
////////////////////////////////////////////////////////////

/* !!! THIS IS AN EXTREMELY ALTERED AND PURPOSE-BUILT VERSION OF SFML !!!
 * This distribution is designed to possess only a limited subset of the
 * original library's functionality and to only build on VxWorks 6.3.
 * The original distribution of this software has many more features and
 * supports more platforms.
 */

#include "../SFML/System/Thread.hpp"

#include <cassert>
#include <iostream>

namespace sf {

Thread::~Thread() {
    wait();
    delete m_entryPoint;
}

void Thread::launch() {
    wait();
    m_isActive = true;
    m_isActive = pthread_create(&m_thread, NULL, &Thread::entryPoint, &m_thread) == 0;

    if (!m_isActive) {
        std::cerr << "Failed to create thread" << std::endl;
    }
}

void Thread::wait() {
    if ( m_isActive ) {
        // A thread cannot wait for itself!
        if (pthread_equal(pthread_self(), m_thread) != 0) {
            std::cerr << "A thread cannot wait for itself! (Thread.cpp:58)" << "\n";
            abort();
        }
    }
}

void Thread::terminate() {
    if ( m_isActive ) {
        pthread_cancel( m_thread );
    }
}

void Thread::run() {
    m_entryPoint->run();
}

void* Thread::entryPoint(void* userData) {
    // The Thread instance is stored in the user data
    Thread* owner = static_cast<Thread*>(userData);

    // Tell the thread to handle cancel requests immediately
    pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);

    // Forward to the owner
    owner->run();

    return NULL;
}

} // namespace sf
