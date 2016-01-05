/**************************************************************************
	Souliss - vNet Virtualized Network
    Copyright (C) 2011  Veseo

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
	
	Originally developed by Dario Di Maio
	
***************************************************************************/
/*!
    \file 
    \ingroup

*/
/**************************************************************************/
#include <stdio.h>
#include <string.h>
#include <iostream>

#include "GetConfig.h"				// need : ethUsrCfg.h
#include "vNetDriver_eth.h"

#include "frame/vNet/tools/UserMode.c"

#include <sys/socket.h> 
#include <arpa/inet.h> //inet_addr
#include <netdb.h> //hostent
#include <fcntl.h>

int sockfd = -1;
U8 vNetM1_header;										// Header for output frame
oFrame vNetM1_oFrame;									// Data structure for output frame
uint8_t ripadrr[4];
uint16_t rport;
unsigned char recvbuf[256];     /* receive buffer */

void vNet_Init_M1()
{
    sockfd = socket(AF_INET , SOCK_DGRAM | O_NONBLOCK  , 0);
 
    //fcntl(sockfd, O_NONBLOCK );

}

/**************************************************************************/
/*!
	Start listening the vNet used ports
*/
/**************************************************************************/
void vNet_Begin_M1(uint8_t sock)
{
    struct sockaddr_in myaddr;

    memset((char *)&myaddr, 0, sizeof(myaddr));
    myaddr.sin_family = AF_INET;
    myaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    myaddr.sin_port = htons(230);
    
    bind(sockfd, (struct sockaddr *)&myaddr, sizeof(myaddr));
}


uint8_t vNet_DataAvailable_M1()
{
    struct sockaddr_in remaddr;     /* remote address */
    int recvlen;                    /* # bytes received */
    
    socklen_t addrlen = sizeof(remaddr);            /* length of addresses */
    
    recvlen = recvfrom(sockfd, recvbuf, 256, 0, (struct sockaddr *)&remaddr, &addrlen);
    
    // If the incoming size is bigger than the UDP header
    if((recvlen >= 8  ) && (recvlen <= VNET_MAX_FRAME))
    {
        int len = recvbuf[0];
        if (len <= recvlen)
        {
            rport = remaddr.sin_port;
            memcpy((void *)&ripadrr[0], (void *) &remaddr.sin_addr.s_addr, 4);
    
            return ETH_SUCCESS;
        }
    }
   /* // Reset the socket
    close(UDP_SOCK);
    while(W5x00.readSnSR(UDP_SOCK) != SnSR::CLOSED);

    // Init the socket
    socket(UDP_SOCK, SnMR::UDP, ETH_PORT, 0);		// Open the socket*/

    return ETH_FAIL;
 }

uint8_t vNet_RetrieveData_M1(uint8_t *data)
{
    struct sockaddr_in remaddr;     /* remote address */
    int recvlen;                    /* # bytes received */
    socklen_t addrlen = sizeof(remaddr);            /* length of addresses */
    
    //recvlen = recvfrom(sockfd, buf, 256, 0, (struct sockaddr *)&remaddr, &addrlen); 
        
    int len = recvbuf[0];

    // Retrieve the complete message
    if(len>0  && len <= (VNET_MAX_FRAME))
    {	
            memmove(data, recvbuf+1, len-1);

            // Verify the incoming address, is a not conventional procedure at this layer
            // but is required to record the IP address in case of User Mode addresses
            #if(UMODE_ENABLE)
            // Get the IP source address for the last frame, ripadrr and sportnumber are processed
            // in vNet_UDP_callback() with a callback from the uIP stack.

            // Is an UserMode frame, record the incoming source information
            if(((*(U16*)&data[4]) & 0xFF00) != 0x0000)
            {
                    uint16_t sportnumber = HTONS(rport);										// Swap byte before record the source port
                    UserMode_Record((*(U16*)&data[4]), ripadrr, (uint8_t*)&sportnumber);	
            }												
            #endif
    }
    else
    {
            //vnetlenght = 0;
            return ETH_FAIL;
    }

    return len;
    
}

uint8_t vNet_Send_M1(uint16_t addr, oFrame *frame, uint8_t len)
{
    
    uint8_t sock, ip_addr[4];
    uint16_t count = 0, vNet_port;

    // Check message length
    if ((len == 0) || (len >= UIP_PAYLOADSIZE))
        return ETH_FAIL;

    // If the frame is not empty, there are waiting data 	
    oFrame_Define(&vNetM1_oFrame);
    if(oFrame_isBusy())
        return ETH_FAIL;

    // Build a frame with len of payload as first byte
    vNetM1_header = len+1;
    oFrame_Set(&vNetM1_header, 0, 1, 0, frame);

    // Define the standard vNet port
    vNet_port = ETH_PORT;

    // Define the IP address to be used
    if((addr == VNET_ADDR_BRDC) ||  (addr == VNET_ADDR_wBRDC) ||  (addr == VNET_ADDR_nBRDC) || ((addr > VNET_ADDR_L_M3) && (addr < VNET_ADDR_H_M3)))
    {
        // Set the IP broadcast address
        for(U8 i=0;i<4;i++)
            ip_addr[i]=0xFF;
    }	
    else
    {
        // Verify the User Mode	
        #if(UMODE_ENABLE)
        if ((addr & 0xFF00) != 0x0000)
        {	
            // The first byte is the User Mode Index, if in range 0x01 - 0x64
            // a standard client/server connection is used with the user interface
            // this give routing and NATting passthrough
            if(!UserMode_Get(addr, &ip_addr[0], (uint8_t*)(&vNet_port)))
            {		
                // Flag the error
                oFrame_Reset();

                return ETH_FAIL;
            }	
        }
        else
        #endif
            eth_vNettoIP(addr, &ip_addr[0]);	// Get the IP address
    }

    struct sockaddr_in servaddr;    /* server address */

    /* fill in the server's address and data */
    memset((char*)&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(vNet_port);

    memcpy((void *)&servaddr.sin_addr, (void *) &ip_addr[0], 4);

    uint8_t data_len = oFrame_GetLenght();
    
    U8 tempBuf[data_len];
    int i=0;
    while ( i < data_len)
    {
        if(oFrame_Available())
            tempBuf[i]=oFrame_GetByte();
        else	
            return 0;
        i++;
    }
    

    
    sendto(sockfd, tempBuf, data_len, 0, (struct sockaddr *)&servaddr, sizeof(servaddr));  
            
    // At this stage data are processed or socket is failed, so we can
    // securely reset the oFrame
    oFrame_Reset();	

    return ETH_SUCCESS;

}


uint16_t vNet_GetSourceAddress_M1()
{
    	uint16_t addr;
	
	// Address translation	
	eth_IPtovNet(&addr, ripadrr);

	return addr;    
}

void vNet_SetAddress_M1(uint16_t addr)
{
    uint8_t ip_addr[4];

    // Translate and set the address
    eth_vNettoIP(addr, &ip_addr[0]);
    eth_SetIPAddress(&ip_addr[0]);

    /*// Set the MAC Address	
    #if(AUTO_MAC)
            eth_vNettoMAC(addr, mac_addr);
            nic_init(mac_addr);
    #else
            eth_vNettoMAC(0, mac_addr);
            nic_init(mac_addr);
    #endif*/

    vNet_Begin_M1(sockfd);	 
}





/**************************************************************************/
/*!
    Translate a vNet address (2 bytes) in MAC address (6 bytes)
*/
/**************************************************************************/
void eth_vNettoMAC(const uint16_t addr, uint8_t *mac_addr)
{
	uint8_t *vNet_addr;
	vNet_addr = (uint8_t *)&addr;
	
	mac_addr[5] = MAC_ADDRESS[5]+*vNet_addr++;
	mac_addr[4] = MAC_ADDRESS[4]+*vNet_addr;	
	mac_addr[3] = MAC_ADDRESS[3];
	mac_addr[2] = MAC_ADDRESS[2];	
	mac_addr[1] = MAC_ADDRESS[1];
	mac_addr[0] = MAC_ADDRESS[0];
	
	#if(MAC_DEBUG)
		// Set the MAC address as broadcast one
		mac_addr[0] |= 0x01;
	#endif
}

/**************************************************************************/
/*!
    Translate a vNet address (2 bytes) in IP address (4 bytes)
*/
/**************************************************************************/
void eth_vNettoIP(const uint16_t addr, uint8_t *ip_addr)
{
	uint8_t *vNet_addr;
	vNet_addr = (uint8_t *)&addr;
	
	ip_addr[3] = stack.base_ip[3]+*vNet_addr++;
	ip_addr[2] = stack.base_ip[2]+*vNet_addr;
	ip_addr[1] = stack.base_ip[1];
	ip_addr[0] = stack.base_ip[0];

}

/**************************************************************************/
/*!
    Translate an  IP address (4 bytes) in vNet address (2 bytes)
*/
/**************************************************************************/
void eth_IPtovNet(uint16_t *addr, const uint8_t *ip_addr)
{
	uint8_t *vNet_addr, brdcast[4] = {0x00};
	vNet_addr = (uint8_t *)addr;
	
	*vNet_addr++ = ip_addr[3] - stack.base_ip[3];
	*vNet_addr = ip_addr[2] - stack.base_ip[2];

}

/**************************************************************************/
/*!
    Set the base IP address
*/
/**************************************************************************/
void eth_SetBaseIP(uint8_t *ip_addr)
{
    uint8_t i;
    for(i=0;i<4;i++)
            stack.base_ip[i] = *ip_addr++;
}

/**************************************************************************/
/*!
    Set the IP address
*/
/**************************************************************************/
void eth_SetIPAddress(uint8_t *ip_addr)
{
    uint8_t i;
    for(i=0;i<4;i++)
            stack.ip[i] = *ip_addr++;
}

/**************************************************************************/
/*!
    Set the Subnet mask
*/
/**************************************************************************/
void eth_SetSubnetMask(uint8_t *submask)
{
    uint8_t i;
    for(i=0;i<4;i++)
            stack.subnetmask[i] = *submask++;
}

/**************************************************************************/
/*!
    Set the Gateway
*/
/**************************************************************************/
void eth_SetGateway(uint8_t *gateway)
{
    uint8_t i;
    for(i=0;i<4;i++)
            stack.gateway[i] = *gateway++;
}

/**************************************************************************/
/*!
    Get the IP address
*/
/**************************************************************************/
void eth_GetIP(uint8_t *ip_addr)
{
    *(ip_addr+0) = stack.ip[0];
    *(ip_addr+1) = stack.ip[1];
    *(ip_addr+2) = stack.ip[2];
    *(ip_addr+3) = stack.ip[3];	
}

/**************************************************************************/
/*!
    Get the base IP address
*/
/**************************************************************************/
void eth_GetBaseIP(uint8_t *ip_addr)
{
    *(ip_addr+0) = stack.base_ip[0];
    *(ip_addr+1) = stack.base_ip[1];
    *(ip_addr+2) = stack.base_ip[2];
    *(ip_addr+3) = stack.base_ip[3];	
}

/**************************************************************************/
/*!
    Get the Subnet mask
*/
/**************************************************************************/
void eth_GetSubnetMask(uint8_t *submask)
{
    *(submask+0) = stack.subnetmask[0];
    *(submask+1) = stack.subnetmask[1];
    *(submask+2) = stack.subnetmask[2];
    *(submask+3) = stack.subnetmask[3];		
}

/**************************************************************************/
/*!
    Get the Gateway
*/
/**************************************************************************/
void eth_GetGateway(uint8_t *gateway)
{
    *(gateway+0) = stack.gateway[0];
    *(gateway+1) = stack.gateway[1];
    *(gateway+2) = stack.gateway[2];
    *(gateway+3) = stack.gateway[3];	
}


