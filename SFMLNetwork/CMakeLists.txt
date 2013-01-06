set(SFML_NET_SRC
    Ftp.cpp
    Http.cpp
    IpAddress.cpp
    Packet.cpp
    Socket.cpp
    SocketSelector.cpp
    TcpListener.cpp
    TcpSocket.cpp
    UdpSocket.cpp
    VxWorks/SocketImpl.cpp
)

add_library(SFMLNetwork ${SFML_NET_SRC})
