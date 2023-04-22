#include "crc8.hpp"
#include "gseCom.hpp"

#include <stdio.h>

#define TEST_CMDID 0x31

int main()
{
    // ペイロードからパケットを作成
    uint8_t payload[] = {0x12, 0x34, 0x56, 0x77, 0x12, 0x34, 0x56, 0x77};
    uint8_t packet[20];
    GseCom::makePacket(packet, TEST_CMDID, payload, sizeof(payload));

    // パケットの中身の表示
    printf("packet:");
    for (int i = 0; i < 20; i++)
    {
        printf("%x,", packet[i]);
    }
    printf("\n");

    // パケットが正常か表示
    printf("packetcheck: %d\n", GseCom::checkPacket(packet));

    // パケットからペイロードを抽出
    uint8_t extractPacket[20];
    uint8_t payloadLength;
    GseCom::getPayload(packet, extractPacket, &payloadLength);

    // 抽出したペイロードを表示
    printf("payloadLength:%d\n", payloadLength);
    printf("extractedpayload:");
    for (int i = 0; i < payloadLength; i++)
    {
        printf("%x,", extractPacket[i]);
    }
    printf("\n");

    // パケットからcmdidを抽出
    printf("cmdid: %x\n", GseCom::getCmdId(packet));

    return 0;
}