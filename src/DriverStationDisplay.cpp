// Copyright (c) 2012-2017 FRC Team 3512. All Rights Reserved.

#include "DriverStationDisplay.hpp"

sf::UdpSocket* SocketInit::m_socket;

SocketInit::SocketInit() {}

SocketInit::~SocketInit() {
    m_socket->unbind();
    delete m_socket;
}

sf::UdpSocket& SocketInit::getInstance(uint16_t portNumber) {
    if (m_socket == NULL) {
        m_socket = new sf::UdpSocket;
        if (portNumber == 0) {
            m_socket->bind(1130);
        } else {
            m_socket->bind(portNumber);
        }
        m_socket->setBlocking(false);
    }

    return *m_socket;
}

SocketInit gSocket;
