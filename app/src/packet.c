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

// likely just using a hardcoded huffman table

LOG_MODULE_REGISTER(packet);

#define PACKET_SIZE 8192
uint8_t packet[PACKET_SIZE];

const char *packet_type;
uint64_t packet_timestamp;
uint32_t packet_rate;
float packet_step;
int32_t last_v = 0;
uint32_t packet_len = 0;

bool start_packet(const char *type, uint64_t ts, uint32_t rate, float step)
{
    packet_type = type;
    packet_timestamp = ts;
    packet_rate = rate;
    packet_step = step;
    last_v = 0;
    packet_len = 0;
    return true;
}

int32_t quantf(float v, float step)
{
    v = v / step;
    float t = (v >= 0.0f) ? 0.5f : -0.5f;
    return (int32_t)(v + t);
}

void add_packet_sample(float s)
{
    int32_t v = quantf(s, packet_step);
    int32_t d = v - last_v;

    if (d >= 127 || d < -128)
    {
        uint32_t us = (uint32_t)(v + 32768); // make unsigned for transmission
        packet[packet_len++] = 0xff;
        packet[packet_len++] = (us >> 8) & 0xff;
        packet[packet_len++] = us & 0xff;
    }
    else
    {
        packet[packet_len++] = (uint8_t)(d + 128);
    }

    last_v = v;
}

char str[2 * PACKET_SIZE + 1];

void finish_packet()
{
    for (uint32_t i = 0; i < packet_len; i++)
        snprintf(&str[i * 2], 3, "%02X", packet[i]);
    LOG_INF("%s %lld %d %f %s", packet_type, packet_timestamp, packet_rate, (double)packet_step, str);
}

uint64_t timestamp()
{
    return k_uptime_get_32();
}
