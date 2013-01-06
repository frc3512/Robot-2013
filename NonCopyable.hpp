//=============================================================================
//File Name: NonCopyable.hpp
//Description: Prevents derived class from being copyable
//Author: FRC Team 3512, Spartatroniks
//=============================================================================

#ifndef NONCOPYABLE_HPP
#define NONCOPYABLE_HPP

class NonCopyable {
protected:
    /* Because this class has a copy constructor, the compiler will not
     * automatically generate the default constructor. We must define it
     * explicitly.
     */
    NonCopyable() {}

private:
    /* By making the copy constructor and assignment operator private, the
     * compiler will trow an error if anyone outside tries to use it. To
     * prevent NonCopyable or friend classes from using it, we also provide no
     * definition, so that the linker will throw an error upon use.
     */

    NonCopyable( const NonCopyable& );

    NonCopyable& operator=( const NonCopyable& );
};

#endif // NONCOPYABLE_HPP
