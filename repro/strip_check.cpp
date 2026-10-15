// strip_check.cpp: get_statistics_message_pos() on hand-built RTPS messages, against
// the stock and the patched RTPSStatisticsMessages.hpp (patch 0004). Messages are an
// RTPS header, INFO_TS and a DATA, with or without the 60-byte statistics submessage,
// and with 0x80 at size - 60 when they have none. A message with the submessage must
// lose exactly it, one without must keep its size. Bodies of 65440 and 65456 put a
// message WITH the submessage at 65536 and 65552 bytes, inside the 65511-65570 window
// where its length cannot tell; a DATA with length 0 is a big DATA (nothing follows it).
// Run in the pinned Jazzy base image, with the Fast DDS source tree (apt-get source
// ros-jazzy-fastrtps, then the patch for the patched run) at /src:
//   g++ -std=c++17 -I/src/src/cpp -I/opt/ros/jazzy/include/fastrtps -I/opt/ros/jazzy/include/fastcdr \
//     strip_check.cpp -L/opt/ros/jazzy/lib -lfastrtps -lfastcdr -Wl,-rpath,/opt/ros/jazzy/lib && ./a.out
// Stock 2.14.7: 7 of 13 FAIL (exit 7). With 0004: 13 of 13 ok (exit 0).
#define FASTDDS_STATISTICS 1
#include <statistics/rtps/messages/RTPSStatisticsMessages.hpp>
#include <fastdds/rtps/messages/RTPS_messages.h>
#include <cstdio>
#include <vector>
using namespace eprosima::fastrtps::rtps;
using namespace eprosima::fastdds::statistics::rtps;
static int fails = 0;
// msg: header(20) + INFO_TS + DATA(len field, body) [+ stats]
static std::vector<octet> build(uint32_t body, bool big_len0, bool stats, int marker_at_end) {
    CDRMessage_t m(body + 400);
    m.pos = m.length = RTPSMESSAGE_HEADER_SIZE;
    RTPSMessageCreator::addSubmessageHeader(&m, INFO_TS, 0, 8); m.pos += 8; m.length = m.pos;
    RTPSMessageCreator::addSubmessageHeader(&m, DATA, 0, big_len0 ? 0 : (uint16_t)body);
    memset(m.buffer + m.pos, 0, body); m.pos += body; m.length = m.pos;
    if (marker_at_end) m.buffer[m.length - 60] = 0x80;
    if (stats) add_statistics_submessage(&m);
    return std::vector<octet>(m.buffer, m.buffer + m.length);
}
static void check(const char* n, std::vector<octet> b, uint32_t want) {
    uint32_t sz = b.size(); remove_statistics_submessage(b.data(), sz);
    bool ok = sz == want; fails += !ok;
    printf("%-55s size %u -> %u (want %u) %s\n", n, (unsigned)b.size(), sz, want, ok ? "ok" : "FAIL");
}
int main() {
    for (uint32_t body : {24u, 1000u, 65400u, 65440u, 65456u}) {   // last two: totals in 65511..65570 with stats
        auto b = build(body, false, true, 0);
        char n[80]; snprintf(n, 80, "small DATA %u + stats: stripped", body);
        check(n, b, b.size() - 60);
        auto c = build(body, false, false, 1);
        snprintf(n, 80, "small DATA %u no stats, 0x80 at size-60: kept", body);
        check(n, c, c.size());
    }
    for (uint32_t body : {65500u, 69996u, 524000u}) {
        auto b = build(body, true, false, 1);
        char n[80]; snprintf(n, 80, "big DATA %u (len 0), 0x80 at size-60: kept", body);
        check(n, b, b.size());
    }
    return fails;
}
