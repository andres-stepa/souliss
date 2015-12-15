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
#ifndef VNET_ETHRASPI_H
#define VNET_ETHRASPI_H

#include "Arduino.h"
#include "GetConfig.h"				// need : ethUsrCfg.h

#include "frame/vNet/tools/UserMode.h"

void eth_GetIP(uint8_t *ip_addr);
void eth_SetBaseIP(uint8_t *ip_addr);
void eth_SetSubnetMask(uint8_t *submask);
void eth_SetGateway(uint8_t *gateway);
void vNet_Init_M1();
uint8_t vNet_Send_M1(uint16_t addr, oFrame *frame, uint8_t len);
uint8_t vNet_DataAvailable_M1();
uint8_t vNet_RetrieveData_M1(uint8_t *data);
uint16_t vNet_GetSourceAddress_M1();
void vNet_SetAddress_M1(uint16_t addr);


#endif
