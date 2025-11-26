#include "common.h"

//
// https://github.com/rygorous/gaffer_net/blob/master/main.cpp
// https://go-compression.github.io/algorithms/arithmetic/
// https://github.com/WangXuan95/TinyZZZ/blob/main/src/lz4C.c
// https://github.com/rygorous/ryg_rans - PD, bugs
// https://github.com/pcodec/pcodec - APACHE
// https://github.com/Cyan4973/FiniteStateEntropy - BSD
// https://github.com/samtools/htscodecs
// https://encode.su/threads/2078-List-of-Asymmetric-Numeral-Systems-implementations
// https://www.reddit.com/r/compression/comments/12v7mkt/worries_about_tans/
// https://cbloomrants.blogspot.com/2023/07/float-to-int-casts-for-data-compression.html
// https://cbloomrants.blogspot.com/2023/07/notes-on-float-and-multi-byte-delta.html
// https://aras-p.info/blog/2023/01/29/Float-Compression-0-Intro/
//

//
// TODO:
// + compression, easiest would be just using a hardcoded huffman table, also try rans, delta
// massaging
// + buffer channels to create larger packets, reducing header overhead and improving compression
// + for extra packet security we could store a 16 bit sequence number and ensure as we drop, we drop every packet
// + could also do a simple checksum on packets
//

LOG_MODULE_REGISTER(channel);
// LOG_MODULE_REGISTER(channel, LOG_LEVEL_DBG);

struct packet_header {
	uint16_t timestamp;
	enum channel channel;
	enum quantize quant;
	uint16_t rate;
	uint16_t len;
	uint8_t data[];
};

#define PACKET_BUFFER_SIZE 32768
uint8_t packet_buffer[PACKET_BUFFER_SIZE];
uint32_t buffer_size = PACKET_BUFFER_SIZE;
uint32_t write_pos;
uint32_t read_pos;
uint32_t wrap_pos = PACKET_BUFFER_SIZE;
int32_t last_sample;

uint64_t first_timestamp;
uint64_t last_timestamp;

K_MUTEX_DEFINE(packet_mutex);

const char *channel_names[] = {"null", "accel.x", "accel.y", "accel.z", "temperature", "pressure"};
const char *quantize_names[] = {"1.0", "0.1", "0.01", "0.001", "0.0001"};
const float quantize_factors[] = {1.0f, 0.1f, 0.01f, 0.001f, 0.0001f};

static bool is_valid_packet(struct packet_header *packet)
{
	if (packet->channel >= CHANNEL_COUNT) {
		return false;
	}
	if (packet->quant >= QUANTIZE_COUNT) {
		return false;
	}
	if (packet->rate == 0) {
		return false;
	}
	if (packet->len == 0 || packet->len > 1024) {
		return false;
	}
	return true;
}

static void check_packet(struct packet_header *packet)
{
	if (is_valid_packet(packet)) {
		return;
	}

	uint32_t pos = (uint8_t *)packet - packet_buffer;

	LOG_ERR("bad packet at %u: channel=%#x quant=%#x rate=%#x len=%#x", pos, packet->channel,
		packet->quant, packet->rate, packet->len);

	k_oops();
}

static struct packet_header *packet_at(uint32_t pos)
{
	if (pos >= wrap_pos) {
		LOG_ERR("bad packet position: %u (wrap at %u)", pos, wrap_pos);
		k_oops();
	}
	struct packet_header *packet = (struct packet_header *)(packet_buffer + pos);
	check_packet(packet);
	return packet;
}

static struct packet_header *first_packet()
{
	if (read_pos == write_pos) {
		return NULL;
	}
	return packet_at(read_pos);
}

static struct packet_header *next_packet(struct packet_header *packet)
{
	check_packet(packet);
	uint32_t pos = (uint8_t *)packet - packet_buffer;
	pos += sizeof(struct packet_header) + packet->len;
	pos = (pos + 3) & ~3; // align to 4 bytes
	if (pos == wrap_pos) {
		pos = 0;
	}
	if (pos == write_pos) {
		return NULL;
	}
	return packet_at(pos);
}

static void drop_packet()
{
	struct packet_header *packet = packet_at(read_pos);

	LOG_DBG("dropping packet %s len=%d at %u", channel_names[packet->channel], packet->len,
		read_pos);

	first_timestamp += packet->timestamp;

	read_pos += sizeof(struct packet_header) + packet->len;
	read_pos = (read_pos + 3) & ~3; // align to 4 bytes

	if (read_pos == wrap_pos) {
		read_pos = 0;
	}
}

bool channel_start_packet(enum channel ch, enum quantize quant, uint64_t ts, uint16_t rate,
			  uint16_t sample_count)
{
	k_mutex_lock(&packet_mutex, K_FOREVER);

	uint32_t reserve_size = sizeof(struct packet_header) + sample_count * 4; // worst case size
	reserve_size = (reserve_size + 3) & ~3;                                  // align to 4 bytes

	LOG_DBG("reserve packet channel=%s samples=%d reserve_size=%u write_pos=%u read_pos=%u "
		"wrap_pos=%u",
		channel_names[ch], sample_count, reserve_size, write_pos, read_pos, wrap_pos);

	if (write_pos + reserve_size > buffer_size) {
		LOG_DBG("wrapping packet buffer at %u", write_pos);

		if (read_pos > write_pos) {
			while (read_pos != 0) {
				drop_packet();
			}
            drop_packet();
        }

		wrap_pos = write_pos;
		write_pos = 0;
	}

    while (read_pos > write_pos && read_pos < write_pos + reserve_size) {
        drop_packet();
    }

	LOG_DBG("new packet channel=%s samples=%d at %u", channel_names[ch], sample_count,
		write_pos);

	memset(packet_buffer + write_pos, 0, reserve_size);

	struct packet_header *packet = (struct packet_header *)(packet_buffer + write_pos);

	packet->timestamp = ts - last_timestamp;
	packet->channel = ch;
	packet->quant = quant;
	packet->rate = rate;
	packet->len = 0;

	last_timestamp = ts;
	last_sample = 0;

	return true;
}

void channel_add_packet_sample(float s)
{
	struct packet_header *packet = (struct packet_header *)(packet_buffer + write_pos);

	float sq = s * quantize_factors[packet->quant];
	float t = (sq >= 0.0f) ? 0.5f : -0.5f;
	int32_t si = (int32_t)(sq + t);
	int32_t d = si - last_sample;

	if (d >= 127 || d < -128) {
		uint32_t us = (uint32_t)(si + 32768); // make unsigned for transmission
		packet->data[packet->len++] = 0xff;
		packet->data[packet->len++] = (us >> 8) & 0xff;
		packet->data[packet->len++] = us & 0xff;
	} else {
		packet->data[packet->len++] = (uint8_t)(d + 128);
	}

	last_sample = si;
}

void channel_finish_packet()
{
	struct packet_header *packet = (struct packet_header *)(packet_buffer + write_pos);

	LOG_DBG("finishing packet channel=%s len=%d at %u", channel_names[packet->channel],
		packet->len, write_pos);

	write_pos += sizeof(struct packet_header) + packet->len;
	write_pos = (write_pos + 3) & ~3; // align to 4 bytes

	LOG_DBG("new write_packet at %u", write_pos);

	k_mutex_unlock(&packet_mutex);
}

uint64_t channel_timestamp()
{
	return k_uptime_get_32();
}

static int cmd_channel_buffer_size(const struct shell *shell, size_t argc, char *argv[])
{
	uint32_t new_size = strtoul(argv[1], NULL, 0);
	if (new_size < 256 || new_size > PACKET_BUFFER_SIZE) {
		shell_fprintf(shell, SHELL_ERROR, "invalid buffer size: %u (min 256 max %u)\n",
			      new_size, PACKET_BUFFER_SIZE);
		return -1;
	}

	k_mutex_lock(&packet_mutex, K_FOREVER);
	buffer_size = new_size;
	write_pos = 0;
	read_pos = 0;
	wrap_pos = buffer_size;
	k_mutex_unlock(&packet_mutex);

	shell_fprintf(shell, SHELL_NORMAL, "channel buffer size is now %u\n", buffer_size);
	return 0;
}

static int cmd_channel_log(const struct shell *shell, size_t argc, char *argv[])
{
	k_mutex_lock(&packet_mutex, K_FOREVER);

	uint64_t timestamp = first_timestamp;

	for (struct packet_header *packet = first_packet(); packet != NULL;
	     packet = next_packet(packet)) {
		timestamp += packet->timestamp;
		shell_fprintf(shell, SHELL_NORMAL, "ts=%llu ch=%s quant=%s rate=%u len=%u\n",
			      timestamp, channel_names[packet->channel],
			      quantize_names[packet->quant], packet->rate, packet->len);
	}

	k_mutex_unlock(&packet_mutex);
	return 0;
}

static int cmd_channel_status(const struct shell *shell, size_t argc, char *argv[])
{
	k_mutex_lock(&packet_mutex, K_FOREVER);

	uint32_t time_window = channel_timestamp() - first_timestamp;

	shell_fprintf(shell, SHELL_NORMAL, "buffer status:\n");
	shell_fprintf(shell, SHELL_NORMAL, " buffer_size: %u\n", buffer_size);
	shell_fprintf(shell, SHELL_NORMAL, " time_window: %u\n", time_window);

	shell_fprintf(shell, SHELL_NORMAL, "channel status:\n");
	for (int ch = 0; ch < CHANNEL_COUNT; ch++) {
		uint32_t packet_count = 0;
		uint32_t byte_count = 0;
		for (struct packet_header *packet = first_packet(); packet != NULL;
		     packet = next_packet(packet)) {
			if (packet->channel == ch) {
				packet_count++;
				byte_count += packet->len;
			}
		}
		float packets_per_sec =
			(time_window > 0) ? (packet_count * 1000.0f / time_window) : 0.0f;
		float bytes_per_sec =
			(time_window > 0) ? (byte_count * 1000.0f / time_window) : 0.0f;
		shell_fprintf(shell, SHELL_NORMAL,
			      " %s: packets=%u bytes=%u packets/sec=%.2f bytes/sec=%.2f\n",
			      channel_names[ch], packet_count, byte_count, (double)packets_per_sec,
			      (double)bytes_per_sec);
	}

	k_mutex_unlock(&packet_mutex);
	return 0;
}

static void channel_test_thread_main(void *, void *, void *)
{
	for (;;) {
        uint64_t timestamp = channel_timestamp();
        uint32_t count = sys_rand32_get() % 100 + 1;
        uint32_t samples_per_sec = 999;
        if (channel_start_packet(CHANNEL_NULL, QUANTIZE_1_0, timestamp,
                samples_per_sec, count)) {
            for (int i = 0; i < count; i++) {
                channel_add_packet_sample(i);
            }
            channel_finish_packet();
        }

        k_msleep(sys_rand32_get() % 100 + 1);
	}
}

K_THREAD_STACK_DEFINE(channel_test_thread_stack, 256);
struct k_thread channel_test_thread;

static int cmd_channel_start_test(const struct shell *shell, size_t argc, char *argv[])
{
    k_thread_create(&channel_test_thread, channel_test_thread_stack,
        K_THREAD_STACK_SIZEOF(channel_test_thread_stack),
        channel_test_thread_main,
        NULL, NULL, NULL,
        7, 0, K_NO_WAIT);

    shell_fprintf(shell, SHELL_NORMAL, "channel test started\n");
    return 0;
}

static int cmd_channel_stop_test(const struct shell *shell, size_t argc, char *argv[])
{
    k_thread_abort(&channel_test_thread);

    shell_fprintf(shell, SHELL_NORMAL, "channel test stopped\n");
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(
	channel_cmds,
	SHELL_CMD_ARG(buffer_size, NULL, "set buffer size", cmd_channel_buffer_size, 2, 0),
	SHELL_CMD_ARG(log, NULL, "print packets in buffer", cmd_channel_log, 1, 0),
	SHELL_CMD_ARG(status, NULL, "print channels status", cmd_channel_status, 1, 0),
    SHELL_CMD_ARG(start_test, NULL, "start channel test", cmd_channel_start_test, 1, 0),
    SHELL_CMD_ARG(stop_test, NULL, "stop channel test", cmd_channel_stop_test, 1, 0),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(channel, &channel_cmds, "Sensor channel commands", NULL);
