/* 
 * File:   RaspiBoard.h
 * Author: astepaniuk
 *
 * Created on September 1, 2015, 1:33 AM
 */

#ifndef RASPIBOARD_H
#define	RASPIBOARD_H


#define MCU_TYPE_INSKETCH
#define	MCU_TYPE			0x03

#define BOARD_MODEL_INSKETCH
#define	BOARD_MODEL			0x10

//#define	COMMS_MODEL_INSKETCH
#define VNET_MEDIA_INSKETCH
#define ETH_INSKETCH	

//#define	COMMS_MODEL					7
#define ETH_W5100  					0
#define ETH_W5200  					0
#define ETH_W5500					0
#define ETH_ENC28J60                                    0
#define WIFI_MRF24					0
#define WIFI_ESP8266                                    0
#define ETH_RASPI                                       1

#define VNET_MEDIA1_ENABLE                          1

#define nRF24SPI_INSKETCH
#define nRF24SPI                                    SPI

#endif	/* RASPIBOARD_H */

