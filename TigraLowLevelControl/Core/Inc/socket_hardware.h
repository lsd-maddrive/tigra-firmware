#ifndef SOCKET_HARDWARE_H
#define SOCKET_HARDWARE_H

#include <iostream>
using namespace std;

#ifdef __cplusplus
extern "C"
{
#endif

#include <math.h>
// #include "lwip/opt.h"
// #include "lwip/timeouts.h"
// #include "lwip/tcpip.h"
// #include "netif/ethernet.h"
// #include "netif/etharp.h"
// #include "lwip/tcp.h"
// #include "ethernetif.h"
// #include "lwip/arch.h"
// #include "lwip/api.h"
// #include "lwip/apps/fs.h"
// #include "ethernetConf.h"

    class SocketHardware
    {
    public:

        void init()
        {
            // TODO - reimplement with uC stack
            // cout << "Socker connection" << endl;

            // socket = tcp::socket(io_context);
            // socket.connect(tcp::endpoint(boost::asio::ip::address::from_string("127.0.0.1"), 23456));

            // cout << "Socker connected: " << socket.is_open() << endl;

            // start_time = boost::posix_time::microsec_clock::local_time();
        }

        void init(char *dst)
        {
            // TODO - reimplement with uC stack
            // Init with destination definition (like 10.143.123.454:11411)
            cout << "Attempt to initialize finction with destination argument - not supported, call init() with default" << endl;
            init();
        }

        int read()
        {
            // TODO - reimplement with uC stack
            uint8_t buffer[1];
            // boost::system::error_code error;

            // if (socket.available() < 1)
            // {
            //     return -1;
            // }

            // size_t bytesRead = socket.read_some(boost::asio::buffer(buffer, 1), error);
            // if (error)
            // {
            //     cout << "read failed: " << error.message() << " " << error.value() << endl;
            //     exit(1);
            // }

            // if (bytesRead < 1)
            // {
            //     return -1;
            // }
            return buffer[0];
        }

        void write(uint8_t *data, int length)
        {
            // TODO - reimplement with uC stack
            // boost::system::error_code error;
            // boost::asio::write(socket, boost::asio::buffer(data, length), error);
            // if (error)
            // {
            //     cout << "send failed: " << error.message() << endl;
            // }
        }

        uint32_t time()
        {
            //return HAL_GetTick();
            return 0;
        }

    // protected:
    //     boost::asio::io_context io_context;
    //     tcp::socket socket;
    //     boost::posix_time::ptime start_time;
    // };

#ifdef __cplusplus
}
#endif

#endif /* SOCKET_HARDWARE_H */
