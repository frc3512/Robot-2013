//=============================================================================
//File Name: ReadWriteProtector.cpp
//Description: Allows resource to be read by many threads but only written to
//             by one thread when no thread is reading
//Author: Tyler Veness
//=============================================================================

#include "ReadWriteProtector.hpp"
#include "SFML/System/Sleep.hpp"

ReadWriteProtector::ReadWriteProtector() {
	m_readingMutex = 0;
	m_writingMutex = 0;
}

ReadWriteProtector::~ReadWriteProtector() {

}

void ReadWriteProtector::startReading() {
	while ( m_writingMutex > 0 ) {
		sf::sleep( sf::milliseconds( 10 ) );
	}

	m_readingMutex++;
}

void ReadWriteProtector::stopReading() {
	m_readingMutex--;
}

void ReadWriteProtector::startWriting() {
	while ( m_readingMutex > 0 && m_writingMutex > 0 ) {
		sf::sleep( sf::milliseconds( 10 ) );
	}

	m_writingMutex++;
}

void ReadWriteProtector::stopWriting() {
	m_writingMutex--;
}
