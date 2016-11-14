#ifndef SETTINGS_H
#define SETTINGS_H

typedef struct
{
	uint8_t ip_addr_0;
	uint8_t ip_addr_1;
	uint8_t ip_addr_2;
	uint8_t ip_addr_3;

	uint16_t port;

}stIPAddress;

typedef struct
{
	stIPAddress IPAdress_Client;
	stIPAddress IPAdress_Server;
}stSettings;

void Settings_Init(stSettings *Settings);
void Settings_Load(stSettings *Settings);
void Settings_Save(stSettings *Settings);

#endif
