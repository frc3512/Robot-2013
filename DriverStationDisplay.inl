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

// Make sure every std::string is converted to a std::wstring before it's sent
template <>
inline void DriverStationDisplay::addElementData( unsigned char type , std::wstring ID , std::string data ) {
    *this << type;
    *this << ID;

    // Convert std::string to std::wstring
    wchar_t cStr[data.length() + 1];
    std::memset( cStr , 0 , sizeof(cStr) );

    for ( unsigned int i = 0 ; i < sizeof(cStr) / sizeof(wchar_t) ; i++ ) {
        cStr[i] = data[i];
    }

    std::wstring tempStr( cStr );

    *this << tempStr;
}
