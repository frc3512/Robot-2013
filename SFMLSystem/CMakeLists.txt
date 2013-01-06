set(SFML_SYS_SRC
    Clock.cpp
    Err.cpp
    Lock.cpp
    Mutex.cpp
    String.cpp
    Thread.cpp
    ThreadLocal.cpp
    Time.cpp
    VxWorks/ClockImpl.cpp
    VxWorks/MutexImpl.cpp
    VxWorks/SleepImpl.cpp
    VxWorks/ThreadImpl.cpp
    VxWorks/ThreadLocalImpl.cpp
)

add_library(SFMLSystem ${SFML_SYS_SRC})
