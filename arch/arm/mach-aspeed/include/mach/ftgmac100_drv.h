/*
 *    
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 */

/* store this information for the driver.. */

struct ftgmac100_eth_data
{
	unsigned char	dev_addr[6];	//MAC address
	unsigned char	phy_addr;		//Phy Address
	unsigned char	phy_id;			//Phy ID
	unsigned char 	DF_support;		//Defragment support
	unsigned long	NCSI_support;
	unsigned long	INTEL_NCSI_EVA_support;	
};
