#include "protocol.h"

#include <stdio.h>
#include <string.h>

#include "hc05.h"

#define COM_COMMAND_QUEUE_SIZE 8U
#define COM_COMMAND_MASK (COM_COMMAND_QUEUE_SIZE - 1U)
#define COM_LINE_SIZE (COM_MAX_DATA + 16U)
#define COM_FRAME_TIMEOUT_MS 100UL

static com_command_t command_queue[COM_COMMAND_QUEUE_SIZE];
static uint8_t command_head;
static uint8_t command_tail;

static char parser_line[COM_LINE_SIZE];
static uint8_t parser_length;
static uint8_t parser_active;
static uint8_t parser_overflow;
static uint32_t parser_started_ms;

static uint8_t hex_value(char value)
{
	if (value >= '0' && value <= '9')
	{
		return (uint8_t)(value - '0');
	}

	if (value >= 'A' && value <= 'F')
	{
		return (uint8_t)(value - 'A' + 10);
	}

	if (value >= 'a' && value <= 'f')
	{
		return (uint8_t)(value - 'a' + 10);
	}

	return 0xFFU;
}

static void parser_reset(void)
{
	parser_length = 0;
	parser_active = 0;
	parser_overflow = 0;
}

static void send_nak(const char *command, const char *reason)
{
	char data[COM_MAX_DATA + 1U];
	(void)snprintf(data, sizeof(data), "%s,%s", command, reason);
	(void)protocol_send_frame(COM_RESP_NAK, data);
}

static void report_uart_errors(void)
{
	uint8_t errors = hc05_take_rx_errors();

	if (errors & HC05_RX_ERROR_FRAME)
	{
		send_nak("?", "UART_FRAME");
	}

	if (errors & HC05_RX_ERROR_OVERRUN)
	{
		send_nak("?", "UART_OVERRUN");
	}

	if (errors & HC05_RX_ERROR_PARITY)
	{
		send_nak("?", "UART_PARITY");
	}

	if (errors & HC05_RX_ERROR_OVERFLOW)
	{
		send_nak("?", "RX_OVERFLOW");
	}
}

uint8_t protocol_send_frame(uint8_t command, const char *data)
{
	char body[COM_MAX_DATA + 8U];
	char frame[COM_MAX_DATA + 16U];
	size_t data_length = data ? strlen(data) : 0U;

	if (data_length > COM_MAX_DATA || (data && (
		strchr(data, '|') || strchr(data, '\n') || strchr(data, '\r')
	)))
	{
		return 0;
	}

	int body_length = snprintf(
		body,
		sizeof(body),
		"%s|%c|%s",
		COM_PREFIX,
		(char)command,
		data ? data : ""
	);

	if (body_length < 0 || (size_t)body_length >= sizeof(body))
	{
		return 0;
	}

	uint8_t checksum = 0U;
	for (int i = 0; i < body_length; i++)
	{
		checksum ^= (uint8_t)body[i];
	}

	int frame_length = snprintf(
		frame,
		sizeof(frame),
		"@%s|%c|%s|%02X\n",
		COM_PREFIX,
		(char)command,
		data ? data : "",
		checksum
	);

	if (frame_length < 0 || (size_t)frame_length >= sizeof(frame))
	{
		return 0;
	}

	return hc05_write_bytes((const uint8_t *)frame, (uint8_t)frame_length);
}

static void parser_accept(void)
{
	uint8_t next = (uint8_t)(
	(command_head + 1U) & COM_COMMAND_MASK
	);

	if (next == command_tail)
	{
		send_nak("?", "BUSY");
		parser_reset();
		return;
	}

	com_command_t *entry = &command_queue[command_head];
	entry->command = (uint8_t)parser_line[3];

	uint8_t data_length = (uint8_t)(parser_length - 8U);
	for (uint8_t i = 0; i < data_length; i++)
	{
		entry->data[i] = parser_line[5U + i];
	}
	entry->data[data_length] = '\0';

	command_head = next;
	parser_reset();
}

static void parser_reject(const char *reason)
{
	send_nak("?", reason);
	parser_reset();
}

static void parser_finish(void)
{
	if (parser_overflow)
	{
		parser_reject("LONG");
		return;
	}

	if (parser_length < 8U ||
		parser_line[0] != 'B' ||
		parser_line[1] != 'R' ||
		parser_line[2] != '|' ||
		parser_line[4] != '|')
	{
		parser_reject("FORMAT");
		return;
	}

	uint8_t separator = (uint8_t)(parser_length - 3U);
	if (parser_line[separator] != '|')
	{
		parser_reject("FORMAT");
		return;
	}

	uint8_t data_length = (uint8_t)(separator - 5U);
	if (data_length > COM_MAX_DATA)
	{
		parser_reject("LONG");
		return;
	}

	uint8_t high = hex_value(parser_line[separator + 1U]);
	uint8_t low = hex_value(parser_line[separator + 2U]);
	if (high == 0xFFU || low == 0xFFU)
	{
		parser_reject("CHECK");
		return;
	}

	uint8_t checksum = 0U;
	for (uint8_t i = 0; i < separator; i++)
	{
		checksum ^= (uint8_t)parser_line[i];
	}

	if (checksum != (uint8_t)((high << 4) | low))
	{
		parser_reject("CHECK");
		return;
	}

	parser_accept();
}

static void parser_consume(uint8_t value, uint32_t now_ms)
{
	if (parser_active &&
		(uint32_t)(now_ms - parser_started_ms) > COM_FRAME_TIMEOUT_MS)
	{
		parser_reject("TIMEOUT");
	}

	if (value == '@')
	{
		parser_reset();
		parser_active = 1U;
		parser_started_ms = now_ms;
		return;
	}

	if (!parser_active)
	{
		return;
	}

	if (value == '\n')
	{
		if (parser_length > 0U &&
			parser_line[parser_length - 1U] == '\r')
		{
			parser_length--;
		}

		parser_finish();
		return;
	}

	if (parser_overflow)
	{
		return;
	}

	if (parser_length >= COM_LINE_SIZE)
	{
		parser_overflow = 1U;
		return;
	}

	parser_line[parser_length++] = (char)value;
}

void protocol_init(void)
{
	command_head = 0;
	command_tail = 0;
	parser_reset();
}

void protocol_poll(uint32_t now_ms)
{
	uint8_t value;
	uint8_t processed = 0;

	while (processed < 32U && hc05_read_byte(&value))
	{
		parser_consume(value, now_ms);
		processed++;
	}

	report_uart_errors();

	if (parser_active &&
		(uint32_t)(now_ms - parser_started_ms) > COM_FRAME_TIMEOUT_MS)
	{
		parser_reject("TIMEOUT");
	}
}

uint8_t protocol_get_command(com_command_t *command)
{
	if (!command || command_tail == command_head)
	{
		return 0;
	}

	*command = command_queue[command_tail];
	command_tail = (uint8_t)(
	(command_tail + 1U) & COM_COMMAND_MASK
	);

	return 1;
}
