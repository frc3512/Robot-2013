//=============================================================================
//File Name: ReadWriteProtector.hpp
//Description: Allows resource to be read by many threads but only written to
//             by one thread when no thread is reading
//Author: Tyler Veness
//=============================================================================

#ifndef READ_WRITE_PROTECTOR_HPP
#define READ_WRITE_PROTECTOR_HPP

#include <atomic>

class ReadWriteProtector {
public:
    ReadWriteProtector();
    virtual ~ReadWriteProtector();

    void startReading();
    void stopReading();

    void startWriting();
    void stopWriting();

private:
	std::atomic<int> m_readingMutex;
	std::atomic<int> m_writingMutex;
};

#endif // READ_WRITE_PROTECTOR_HPP
