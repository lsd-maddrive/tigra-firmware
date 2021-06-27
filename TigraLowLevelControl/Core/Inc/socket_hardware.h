#ifndef SOCKET_HARDWARE_H
#define SOCKET_HARDWARE_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <math.h>
#include "main.h"

#define CONNECTION_PORT 80
#define SERVER_PORT 1234

    class SocketHardware
    {
    private:
        struct netconn *tcpConnection;
        err_t error;
        ip_addr_t ServerIPaddr;
        uint32_t startTime; 

    public:

        void init()
        {
            this->startTime=HAL_GetTick();
            printDebugMessage((uint8_t*)"TCP connection...\n\r");
            IP4_ADDR(&this->ServerIPaddr, 192,168,0,1);
            this->tcpConnection = netconn_new(NETCONN_TCP);
            if(this->tcpConnection!=NULL)
            {
                this->error = netconn_bind(this->tcpConnection, NULL, 80);
                if(this->error==ERR_OK)
                {
                    this->error=netconn_connect(this->tcpConnection, &this->ServerIPaddr, SERVER_PORT);
                    if(this->error==ERR_OK)
                        printDebugMessage((uint8_t*)"Connected\n\r");
                    else
                        printDebugMessage((uint8_t*)"Server not connected\n\r");
                }
                else
                {
                    netconn_delete(this->tcpConnection);
                    printDebugMessage((uint8_t*)"Connection failed\n\r");
                }
            }
        }

        void init(char *dst)
        {
            // TODO - reimplement with uC stack
            // Init with destination definition (like 10.143.123.454:11411)
            printDebugMessage((uint8_t*)"Attempt to initialize finction with destination argument - not supported, call init() with default\n\r");
            init();
        }

        int read()
        {
            // TODO - reimplement with uC stack
            err_t reciveError;
            uint8_t retBuff;
            static struct netbuf *inbuf = NULL;
            static uint8_t* buf = NULL;
            static uint16_t buflen = NULL;
            static uint16_t readStep = 0;
            if(readStep==0)
            {
                reciveError = netconn_recv(this->tcpConnection, &inbuf);
                if(reciveError==ERR_OK)
                {
                    netbuf_data(inbuf, (void**)&buf, &buflen);
                    retBuff=buf[readStep];
                    readStep++;
                } 
                else
                    printDebugMessage((uint8_t*)"Recive error\n\r");   
            }
            else
            {
                retBuff=buf[readStep];
                readStep++;
                if(readStep==buflen) 
                {
                    readStep=0;
                    if(!(netbuf_next(inbuf) >= 0))
                        netbuf_delete(inbuf);
                }
            }
            return retBuff;
        }

        void write(uint8_t *data, int length)
        {
            // TODO - reimplement with uC stack
            err_t sentError;
            sentError=netconn_write(this->tcpConnection,data,length,NETCONN_COPY);
            if(sentError!=ERR_OK)
            {
               printDebugMessage((uint8_t*)"Transmitt failed\n\r"); 
            }
        }

        uint32_t time()
        {
            return HAL_GetTick()-this->startTime;
        }

    };

#ifdef __cplusplus
}
#endif

#endif /* SOCKET_HARDWARE_H */
