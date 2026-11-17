#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

#define COM_PREFIX "BR"
#define COM_MAX_DATA 64U

#define COM_CMD_STATUS 'S'
#define COM_CMD_READ 'R'
#define COM_CMD_MODE 'M'
#define COM_CMD_ACTUATOR 'A'
#define COM_CMD_EMERGENCY 'E'
#define COM_CMD_ALARMS 'L'

#define COM_RESP_STATUS 's'
#define COM_RESP_READ 'r'
#define COM_RESP_ALARM 'l'
#define COM_RESP_EVENT '!'
#define COM_RESP_ACK 'K'
#define COM_RESP_NAK 'N'

typedef struct
{
	uint8_t command;
	char data[COM_MAX_DATA + 1U];
} com_command_t;

void protocol_init(void);
void protocol_poll(uint32_t now_ms);
uint8_t protocol_get_command(com_command_t *command);
uint8_t protocol_send_frame(uint8_t command, const char *data);

#endif
