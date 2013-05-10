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

#ifndef SFML_THREADLOCALPTR_HPP
#define SFML_THREADLOCALPTR_HPP

#include "ThreadLocal.hpp"


namespace sf
{
// Pointer to a thread-local variable
template <typename T>
class ThreadLocalPtr : private ThreadLocal {
public:
    ThreadLocalPtr(T* value = NULL);

    /* Like raw pointers, applying the * operator returns a reference to the
     * pointed object
     */
    T& operator *() const;

    // Like raw pointers, applying the -> operator returns the pointed object
    T* operator ->() const;

    /* Cast operator to implicitly convert the pointer to its raw pointer type
     * (T*)
     * Returns Pointer to the actual object
     */
    operator T*() const;

    ThreadLocalPtr<T>& operator =(T* value);

    ThreadLocalPtr<T>& operator =(const ThreadLocalPtr<T>& right);
};

} // namespace sf

#include "ThreadLocalPtr.inl"


#endif // SFML_THREADLOCALPTR_HPP
