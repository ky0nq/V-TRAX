#include "raw_udp.h"
#include "ui_stream_config.h"

#include "xemacps.h"
#include "xparameters.h"
#include "xparameters_ps.h"
#include "xil_cache.h"
#include "xil_io.h"
#include "xil_mmu.h"
#include "xil_printf.h"
#include "sleep.h"
#include "xtime_l.h"
#include <string.h>

/* Zynq-7000 GEM0, polling TX only. No lwIP, no GEM interrupt connection. */
#define RAW_UDP_ETH_HDR_BYTES 14U
#define RAW_UDP_IP_HDR_BYTES  20U
#define RAW_UDP_UDP_HDR_BYTES 8U
#define RAW_UDP_MAX_PAYLOAD   1472U
#define RAW_UDP_MAX_FRAME     1518U

#define RAW_UDP_TXBD_COUNT    4U
#define RAW_UDP_RXBD_COUNT    1U
#define RAW_UDP_TX_TIMEOUT    (COUNTS_PER_SECOND / 2U)

#define PHY_REG_BMCR          0U
#define PHY_REG_BMSR          1U
#define PHY_REG_ID1           2U
#define PHY_REG_ID2           3U
#define PHY_BMCR_AN_ENABLE    0x1000U
#define PHY_BMCR_AN_RESTART   0x0200U
#define PHY_BMSR_LINK         0x0004U
#define PHY_BMSR_AN_DONE      0x0020U

#define SLCR_LOCK_ADDR        (XPS_SYS_CTRL_BASEADDR + 0x00000004U)
#define SLCR_UNLOCK_ADDR      (XPS_SYS_CTRL_BASEADDR + 0x00000008U)
#define SLCR_GEM0_CLK_CTRL    (XPS_SYS_CTRL_BASEADDR + 0x00000140U)
#define SLCR_LOCK_KEY         0x0000767BU
#define SLCR_UNLOCK_KEY       0x0000DF0DU
#define SLCR_GEM_DIV_KEEP     0xFC0FC0FFU

static XEmacPs g_emac;
static int g_ready = 0;
static u16 g_ip_id = 1U;
static u32 g_phy_addr = 32U;

#if defined(XPAR_XEMACPS_0_ENET_SLCR_1000Mbps_DIV0)

#define GEM0_1G_DIV0 XPAR_XEMACPS_0_ENET_SLCR_1000Mbps_DIV0
#define GEM0_1G_DIV1 XPAR_XEMACPS_0_ENET_SLCR_1000Mbps_DIV1

#elif defined(XPAR_PS7_ETHERNET_0_ENET_SLCR_1000MBPS_DIV0)

#define GEM0_1G_DIV0 XPAR_PS7_ETHERNET_0_ENET_SLCR_1000MBPS_DIV0
#define GEM0_1G_DIV1 XPAR_PS7_ETHERNET_0_ENET_SLCR_1000MBPS_DIV1

#else

#error "GEM0 1Gbps clock divider macros not found in xparameters.h"

#endif

/* Descriptor ring occupies one exclusively reserved, aligned DDR section. */
static u8 g_bd_space[0x100000U] __attribute__((aligned(0x100000)));
static u8 g_tx_frame[RAW_UDP_MAX_FRAME] __attribute__((aligned(64)));

static const u8 g_pc_mac[6] = {
    0xffU, 0xffU, 0xffU, 0xffU, 0xffU, 0xffU
};

static const u8 g_zybo_mac[6] = UI_MAC;

static void put_be16(u8 *p, u16 v)
{
    p[0] = (u8)(v >> 8);
    p[1] = (u8)(v & 0xFFU);
}

static u16 ip_checksum(const u8 *buf, unsigned int len)
{
    u32 sum = 0U;
    unsigned int i;

    for (i = 0U; i + 1U < len; i += 2U) {
        sum += ((u32)buf[i] << 8) | (u32)buf[i + 1U];
        while (sum >> 16)
            sum = (sum & 0xFFFFU) + (sum >> 16);
    }
    if (len & 1U) {
        sum += (u32)buf[len - 1U] << 8;
        while (sum >> 16)
            sum = (sum & 0xFFFFU) + (sum >> 16);
    }
    return (u16)(~sum);
}

static int phy_find(XEmacPs *emac, u32 *phy_addr)
{
    u32 a;
    u16 id1, id2;

    for (a = 0U; a < 32U; ++a) {
        if (XEmacPs_PhyRead(emac, a, PHY_REG_ID1, &id1) != XST_SUCCESS)
            continue;
        if (XEmacPs_PhyRead(emac, a, PHY_REG_ID2, &id2) != XST_SUCCESS)
            continue;
        if (id1 != 0x0000U && id1 != 0xFFFFU &&
            id2 != 0x0000U && id2 != 0xFFFFU) {
            *phy_addr = a;
            return XST_SUCCESS;
        }
    }
    return XST_FAILURE;
}

static int phy_restart_and_wait_link(XEmacPs *emac, u32 phy_addr)
{
    u16 bmcr = 0U;
    u16 bmsr = 0U;
    unsigned int i;

    if (XEmacPs_PhyRead(emac, phy_addr, PHY_REG_BMCR, &bmcr) != XST_SUCCESS)
        return XST_FAILURE;

    bmcr |= (PHY_BMCR_AN_ENABLE | PHY_BMCR_AN_RESTART);
    if (XEmacPs_PhyWrite(emac, phy_addr, PHY_REG_BMCR, bmcr) != XST_SUCCESS)
        return XST_FAILURE;

    /* Up to ~5 s. BMSR link bit is latch-low, so read it twice. */
    for (i = 0U; i < 100U; ++i) {
        (void)XEmacPs_PhyRead(emac, phy_addr, PHY_REG_BMSR, &bmsr);
        if (XEmacPs_PhyRead(emac, phy_addr, PHY_REG_BMSR, &bmsr) == XST_SUCCESS) {
            if ((bmsr & PHY_BMSR_LINK) != 0U &&
                (bmsr & PHY_BMSR_AN_DONE) != 0U)
                return XST_SUCCESS;
        }
        usleep(50000U);
    }
    return XST_FAILURE;
}

static void gem_clock(unsigned speed)
{
    u32 clk;
    u32 div0 = GEM0_1G_DIV0, div1 = GEM0_1G_DIV1;
    if (speed == 100U) { div0 = XPAR_PS7_ETHERNET_0_ENET_SLCR_100MBPS_DIV0; div1 = XPAR_PS7_ETHERNET_0_ENET_SLCR_100MBPS_DIV1; }
    if (speed == 10U) { div0 = XPAR_PS7_ETHERNET_0_ENET_SLCR_10MBPS_DIV0; div1 = XPAR_PS7_ETHERNET_0_ENET_SLCR_10MBPS_DIV1; }

    /* Same Zynq-7000 1G divider programming used by the XEmacPs example.
     * This is runtime PS clock setup; it does not modify Vivado/XSA/bitstream. */
    Xil_Out32(SLCR_UNLOCK_ADDR, SLCR_UNLOCK_KEY);

    clk = Xil_In32(SLCR_GEM0_CLK_CTRL);
    clk &= SLCR_GEM_DIV_KEEP;
    clk |= (div1 << 20);
    clk |= (div0 << 8);
    Xil_Out32(SLCR_GEM0_CLK_CTRL, clk);

    Xil_Out32(SLCR_LOCK_ADDR, SLCR_LOCK_KEY);
}

static int setup_bd_rings(XEmacPs *emac)
{
    XEmacPs_Bd tmpl;
    u8 *rx_space = &g_bd_space[0x00000U];
    u8 *tx_space = &g_bd_space[0x10000U];
    LONG st;

    /* Keep only this 1 MiB DDR section uncached for GEM descriptors. */
    Xil_SetTlbAttributes((INTPTR)g_bd_space, DEVICE_MEMORY);

    XEmacPs_BdClear(&tmpl);
    st = XEmacPs_BdRingCreate(&XEmacPs_GetRxRing(emac),
                              (UINTPTR)rx_space, (UINTPTR)rx_space,
                              XEMACPS_BD_ALIGNMENT, RAW_UDP_RXBD_COUNT);
    if (st != XST_SUCCESS)
        return XST_FAILURE;
    st = XEmacPs_BdRingClone(&XEmacPs_GetRxRing(emac), &tmpl, XEMACPS_RECV);
    if (st != XST_SUCCESS)
        return XST_FAILURE;

    XEmacPs_BdClear(&tmpl);
    XEmacPs_BdSetStatus(&tmpl, XEMACPS_TXBUF_USED_MASK);
    st = XEmacPs_BdRingCreate(&XEmacPs_GetTxRing(emac),
                              (UINTPTR)tx_space, (UINTPTR)tx_space,
                              XEMACPS_BD_ALIGNMENT, RAW_UDP_TXBD_COUNT);
    if (st != XST_SUCCESS)
        return XST_FAILURE;
    st = XEmacPs_BdRingClone(&XEmacPs_GetTxRing(emac), &tmpl, XEMACPS_SEND);
    return (st == XST_SUCCESS) ? XST_SUCCESS : XST_FAILURE;
}

int RawUdp_Init(void)
{
    XEmacPs_Config *cfg;
    LONG st;
    u32 nwcfg;

#ifdef SDT
    cfg = XEmacPs_LookupConfig((UINTPTR)XPAR_XEMACPS_0_BASEADDR);
#else
    cfg = XEmacPs_LookupConfig(XPAR_XEMACPS_0_DEVICE_ID);
#endif
    if (cfg == NULL) {
        xil_printf("[NET] XEmacPs config not found\r\n");
        return XST_FAILURE;
    }

    st = XEmacPs_CfgInitialize(&g_emac, cfg, cfg->BaseAddress);
    if (st != XST_SUCCESS) {
        xil_printf("[NET] XEmacPs_CfgInitialize failed\r\n");
        return XST_FAILURE;
    }

    gem_clock(1000U);
    XEmacPs_SetMdioDivisor(&g_emac, MDC_DIV_224);
    XEmacPs_SetOperatingSpeed(&g_emac, 1000U);

    /* TX only. We build IPv4/UDP checksums ourselves and do not need RX. */
    (void)XEmacPs_ClearOptions(&g_emac,
        XEMACPS_RECEIVER_ENABLE_OPTION |
        XEMACPS_RX_CHKSUM_ENABLE_OPTION |
        XEMACPS_TX_CHKSUM_ENABLE_OPTION);
    (void)XEmacPs_SetOptions(&g_emac,
        XEMACPS_TRANSMITTER_ENABLE_OPTION | XEMACPS_FCS_INSERT_OPTION);

    nwcfg = XEmacPs_ReadReg(g_emac.Config.BaseAddress, XEMACPS_NWCFG_OFFSET);
    nwcfg |= XEMACPS_NWCFG_FDEN_MASK;
    XEmacPs_WriteReg(g_emac.Config.BaseAddress, XEMACPS_NWCFG_OFFSET, nwcfg);

    st = XEmacPs_SetMacAddress(&g_emac, (void *)g_zybo_mac, 1U);
    if (st != XST_SUCCESS) {
        xil_printf("[NET] SetMacAddress failed\r\n");
        return XST_FAILURE;
    }

    if (setup_bd_rings(&g_emac) != XST_SUCCESS) {
        xil_printf("[NET] GEM BD ring setup failed\r\n");
        return XST_FAILURE;
    }

    if (phy_find(&g_emac, &g_phy_addr) != XST_SUCCESS) {
        xil_printf("[NET] PHY not found\r\n");
        return XST_FAILURE;
    }
    xil_printf("[NET] PHY addr=%lu, waiting for direct-LAN link...\r\n",
               (unsigned long)g_phy_addr);

    if (phy_restart_and_wait_link(&g_emac, g_phy_addr) != XST_SUCCESS) {
        xil_printf("[NET] PHY link/autoneg timeout. Check cable and PC Ethernet.\r\n");
        return XST_FAILURE;
    }

    /* Resolve common full-duplex ability using standard MII registers. */
    {
        u16 local, partner, gig_local, gig_partner;
        unsigned speed;
        if (XEmacPs_PhyRead(&g_emac, g_phy_addr, 4, &local) ||
            XEmacPs_PhyRead(&g_emac, g_phy_addr, 5, &partner) ||
            XEmacPs_PhyRead(&g_emac, g_phy_addr, 9, &gig_local) ||
            XEmacPs_PhyRead(&g_emac, g_phy_addr, 10, &gig_partner)) return XST_FAILURE;
        if ((gig_local & 0x200U) && (gig_partner & 0x800U)) speed = 1000U;
        else if (local & partner & 0x100U) speed = 100U;
        else if (local & partner & 0x80U) { xil_printf("[NET] Half duplex unsupported\r\n"); return XST_FAILURE; }
        else if (local & partner & 0x40U) speed = 10U;
        else return XST_FAILURE;
        gem_clock(speed);
        XEmacPs_SetOperatingSpeed(&g_emac, speed);
        xil_printf("[NET] Negotiated full duplex %u Mbps\r\n", speed);
    }

    XEmacPs_Start(&g_emac);

    /* Start() enables GEM interrupt sources internally. This module polls TX
     * descriptors and never connects GEM to the GIC, so disable them again. */
    XEmacPs_IntDisable(&g_emac, XEMACPS_IXR_ALL_MASK);

    g_ready = 1;
    xil_printf("[NET] GEM0 ready: 192.168.10.2 -> 192.168.10.255 UDP broadcast\r\n");
    return XST_SUCCESS;
}

int RawUdp_IsReady(void)
{
    return g_ready;
}

void RawUdp_Poll(void)
{
    /* TX completion is polled synchronously in RawUdp_Send(). */
}

int RawUdp_Send(u16 dst_port, const void *payload, u16 payload_len)
{
    XEmacPs_Bd *bd;
    XEmacPs_Bd *done_bd;
    XEmacPs_BdRing *tx_ring;
    u8 *eth;
    u8 *ip;
    u8 *udp;
    u16 ip_len;
    u16 udp_len;
    u16 frame_len;
    u16 csum;
    LONG st;
    u32 done;
    XTime started, now;

    if (!g_ready || payload == NULL || payload_len > RAW_UDP_MAX_PAYLOAD)
        return XST_FAILURE;

    eth = &g_tx_frame[0];
    ip  = &g_tx_frame[RAW_UDP_ETH_HDR_BYTES];
    udp = &g_tx_frame[RAW_UDP_ETH_HDR_BYTES + RAW_UDP_IP_HDR_BYTES];

    udp_len = (u16)(RAW_UDP_UDP_HDR_BYTES + payload_len);
    ip_len  = (u16)(RAW_UDP_IP_HDR_BYTES + udp_len);
    frame_len = (u16)(RAW_UDP_ETH_HDR_BYTES + ip_len);

    memcpy(&eth[0], g_pc_mac, 6U);
    memcpy(&eth[6], g_zybo_mac, 6U);
    eth[12] = 0x08U;
    eth[13] = 0x00U;

    memset(ip, 0, RAW_UDP_IP_HDR_BYTES);
    ip[0] = 0x45U;
    ip[1] = 0x00U;
    put_be16(&ip[2], ip_len);
    put_be16(&ip[4], g_ip_id++);
    put_be16(&ip[6], 0x4000U); /* Don't Fragment */
    ip[8] = 64U;
    ip[9] = 17U;               /* UDP */
    {
        const u8 local_ip[4] = { UI_BOARD_IP };
        const u8 broadcast_ip[4] = { UI_PC_IP };
        memcpy(&ip[12], local_ip, 4U);
        memcpy(&ip[16], broadcast_ip, 4U);
    }
    csum = ip_checksum(ip, RAW_UDP_IP_HDR_BYTES);
    put_be16(&ip[10], csum);

    put_be16(&udp[0], dst_port); /* source port == destination port */
    put_be16(&udp[2], dst_port);
    put_be16(&udp[4], udp_len);
    put_be16(&udp[6], 0U);       /* UDP checksum optional for IPv4 */
    memcpy(&udp[8], payload, payload_len);

    Xil_DCacheFlushRange((UINTPTR)g_tx_frame, frame_len);

    tx_ring = &XEmacPs_GetTxRing(&g_emac);
    st = XEmacPs_BdRingAlloc(tx_ring, 1U, &bd);
    if (st != XST_SUCCESS)
        return XST_FAILURE;

    XEmacPs_BdSetAddressTx(bd, (UINTPTR)g_tx_frame);
    XEmacPs_BdSetLength(bd, frame_len);
    XEmacPs_BdClearTxUsed(bd);
    XEmacPs_BdSetLast(bd);

    st = XEmacPs_BdRingToHw(tx_ring, 1U, bd);
    if (st != XST_SUCCESS) {
        (void)XEmacPs_BdRingUnAlloc(tx_ring, 1U, bd);
        return XST_FAILURE;
    }

    dsb();
    XEmacPs_Transmit(&g_emac);

    done = 0U;
    XTime_GetTime(&started);
    do {
        done = XEmacPs_BdRingFromHwTx(tx_ring, 1U, &done_bd);
        if (done != 0U)
            break;
        XTime_GetTime(&now);
    } while (now - started < RAW_UDP_TX_TIMEOUT);
    if (done == 0U) {
        g_ready = 0; /* Never reuse a buffer still owned by DMA. */
        XEmacPs_Stop(&g_emac);
        xil_printf("[NET] TX timeout; stopped. Restart test.\r\n");
        return XST_FAILURE;
    }

    if (XEmacPs_BdGetStatus(done_bd) &
        (XEMACPS_TXBUF_RETRY_MASK | XEMACPS_TXBUF_URUN_MASK | XEMACPS_TXBUF_EXH_MASK)) {
        g_ready = 0;
        XEmacPs_Stop(&g_emac);
        xil_printf("[NET] TX descriptor error; stopped.\r\n");
        return XST_FAILURE;
    }
    st = XEmacPs_BdRingFree(tx_ring, done, done_bd);
    if (st != XST_SUCCESS)
        return XST_FAILURE;

    return XST_SUCCESS;
}
