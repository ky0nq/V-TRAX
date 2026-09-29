#ifndef RAW_UDP_H
#define RAW_UDP_H

#include "xil_types.h"

int RawUdp_Init(void);
int RawUdp_Send(u16 dst_port, const void *payload, u16 payload_len);
int RawUdp_IsReady(void);
void RawUdp_Poll(void);

#endif
