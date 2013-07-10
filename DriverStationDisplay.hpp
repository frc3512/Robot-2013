//=============================================================================
//File Name: DriverStationDisplay.hpp
//Description: Receives IP address from remote host then sends HUD data there
//Author: FRC Team 3512, Spartatroniks
//=============================================================================

#ifndef DRIVER_STATION_DISPLAY_HPP
#define DRIVER_STATION_DISPLAY_HPP

/* This class allows you to pack data into an SFML packet and send it to an
 * application on the DriverStation that displays it in a GUI.
 *
 * USAGE:
 * 1) Call DriverStationDisplay::getInstance() to create an instance of this
 *    class
 * 2) Call clear() on the pointer to empty the packet before adding new data
 * 3) Add new data with the << operator (e.g. *dsPtr << 4.f; *dsPtr << myVar;)
 * 4) After all data is packed, call sendToDS() to send the data to the Driver
 *    Station
 *
 * Extract the received packet on the DriverStation with the >> operator like
 * any other SFML packet.
 *
 * Note: Make sure that the application on the Driver Station extracts the same
 *       types from the received packet in the same order in which you inserted
 *       them.
 *
 * The packets are always sent to 10.35.12.42 for testing purposes
 */

#include "NonCopyable.hpp"
#include "SFML/Network/IpAddress.hpp"
#include "SFML/Network/Packet.hpp"
#include "SFML/Network/UdpSocket.hpp"

class DriverStationDisplayInit;

class DriverStationDisplay : public NonCopyable , public sf::Packet {
public:
    virtual ~DriverStationDisplay();

    static DriverStationDisplay* getInstance( unsigned short dsPort );

    /* Sends data currently in packet to Driver Station
     * 'userData' holds the packet to be sent to the Driver Station
     * If the pointer is NULL, this class's internal packet is sent instead
     */
    void sendToDS( sf::Packet* userData = NULL );

    /* receive control commands from Driver Station
     * 'userData' will hold extra data sent after received command
     */
    const std::string receiveFromDS( void* userData );

    // Add UI element data to packet
    template <class T>
    void addElementData( unsigned char type , std::basic_string<wchar_t> ID , T data );

private:
    DriverStationDisplay( unsigned short portNumber );

    static DriverStationDisplay* m_dsDisplay;

    sf::UdpSocket m_socket;

    sf::IpAddress m_dsIP; // IP address of Driver Station
    unsigned short m_dsPort; // port to which to send data

    sf::IpAddress m_recvIP; // stores IP address temporarily during receive
    unsigned short m_recvPort; // stores port temporarily during receive
    char* m_recvBuffer; // buffer for Driver Station requests
    size_t m_recvAmount; // holds number of bytes received from Driver Station

    friend class DriverStationDisplayInit;
};

#include "DriverStationDisplay.inl"

#endif // DRIVER_STATION_DISPLAY_HPP
