//=============================================================================
//File Name: DriverStationDisplay.inl
//Description: Receives IP address from remote host then sends HUD data there
//Author: FRC Team 3512, Spartatroniks
//=============================================================================

template <class T>
void DriverStationDisplay::addElementData( unsigned char type , std::wstring ID , T data ) {
    *this << type;
    *this << ID;
    *this << data;
}
