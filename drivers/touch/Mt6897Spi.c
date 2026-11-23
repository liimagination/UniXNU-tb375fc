/* SPDX-License-Identifier: MIT */
/*
 * MT6897 / MT6985 IPM v2 SPI Master + Clock + GPIO Driver.
 * Register sequences verified against TB375FC stock spi-mt65xx.ko,
 * clk-mt6897.ko, clk-mt6897-peri.ko, and pinctrl-mt6897.ko.
 */
#include "Mt6897Spi.h"
#include "Mt6897TouchBoard.h"

#include <string.h>

static inline uint32_t r32(const mt6897_spi_ctrl_t *c, uintptr_t base, uint32_t off) {
    return c->mmio.read32(c->mmio.ctx, base, off);
}

static inline void w32(const mt6897_spi_ctrl_t *c, uintptr_t base, uint32_t off, uint32_t val) {
    c->mmio.write32(c->mmio.ctx, base, off, val);
}

static inline void udelay_wrap(const mt6897_spi_ctrl_t *c, uint32_t us) {
    if (c->mmio.udelay) {
        c->mmio.udelay(c->mmio.ctx, us);
    }
}

void mt6897_spi_snapshot(const mt6897_spi_ctrl_t *c, mt6897_hw_snapshot_t *out) {
    if (!c || !out) return;
    memset(out, 0, sizeof(*out));
    if (c->topckgen_base)   out->topck_spi2_mux = r32(c, c->topckgen_base, MT6897_TOPCK_SPI2_MUX_REG);
    if (c->pericfg_ao_base) out->perao_cg0_sta  = r32(c, c->pericfg_ao_base, MT6897_PERAO_CG0_STA);
    if (c->gpio_base) {
        out->gpio_mode1 = r32(c, c->gpio_base, MT6897_GPIO_MODE1);
        out->gpio_mode7 = r32(c, c->gpio_base, MT6897_GPIO_MODE7);
        out->gpio_dir0  = r32(c, c->gpio_base, MT6897_GPIO_DIR0);
        out->gpio_dir1  = r32(c, c->gpio_base, MT6897_GPIO_DIR1);
        out->gpio_din0  = r32(c, c->gpio_base, MT6897_GPIO_DIN0);
        out->gpio_dout1 = r32(c, c->gpio_base, MT6897_GPIO_DOUT1);
    }
    if (c->iocfg_tm_base) {
        out->iocfg_ies0 = r32(c, c->iocfg_tm_base, MT6897_IOCFG_TM_IES0);
        out->iocfg_ies1 = r32(c, c->iocfg_tm_base, MT6897_IOCFG_TM_IES1);
    }
    if (c->spi2_base && ((out->perao_cg0_sta & MT6897_PERAO_SPI2_BCLK_BIT) == 0u)) {
        out->spi_cfg0    = r32(c, c->spi2_base, MTK_SPI_CFG0_REG);
        out->spi_cfg1    = r32(c, c->spi2_base, MTK_SPI_CFG1_REG);
        out->spi_cfg2    = r32(c, c->spi2_base, MTK_SPI_CFG2_REG);
        out->spi_cfg3    = r32(c, c->spi2_base, MTK_SPI_CFG3_IPM_REG);
        out->spi_cmd     = r32(c, c->spi2_base, MTK_SPI_CMD_REG);
        out->spi_pad_sel = r32(c, c->spi2_base, MTK_SPI_PAD_SEL_REG);
        out->spi_status1 = r32(c, c->spi2_base, MTK_SPI_STATUS1_REG);
    }
}

int mt6897_spi_enable_clocks(mt6897_spi_ctrl_t *c) {
    if (!c || !c->topckgen_base || !c->pericfg_ao_base) return -1;

    uint32_t parent_idx = c->use_26m_parent ? MT6897_SPI2_PARENT_26M : MT6897_SPI2_PARENT_MAINPLL;
    c->src_clk_hz = c->use_26m_parent ? MT6897_SPI2_CLK_26M_HZ : MT6897_SPI2_CLK_MAINPLL_HZ;

    /* 1. Clear spi2_sel mux bits [26:24] and power-down gate bit [31]. */
    w32(c, c->topckgen_base, MT6897_TOPCK_SPI2_MUX_CLR,
        MT6897_TOPCK_SPI2_MUX_MASK | MT6897_TOPCK_SPI2_PDN_BIT);
    /* 2. Select parent clock index. */
    if (parent_idx != 0u) {
        w32(c, c->topckgen_base, MT6897_TOPCK_SPI2_MUX_SET,
            (parent_idx << MT6897_TOPCK_SPI2_MUX_SHIFT) & MT6897_TOPCK_SPI2_MUX_MASK);
    }
    /* 3. Latch mux update bit 13 in CLK_CFG_UPDATE2 (+0x0c). */
    w32(c, c->topckgen_base, MT6897_TOPCK_CLK_CFG_UPD2, MT6897_TOPCK_SPI2_UPD_BIT);
    udelay_wrap(c, 50);

    /* 4. Un-gate peraop_spi2_bclk (bit 14 in pericfg_ao CG0 CLR=+0x28; 0 = enabled). */
    w32(c, c->pericfg_ao_base, MT6897_PERAO_CG0_CLR, MT6897_PERAO_SPI2_BCLK_BIT);
    udelay_wrap(c, 50);

    uint32_t sta = r32(c, c->pericfg_ao_base, MT6897_PERAO_CG0_STA);
    return ((sta & MT6897_PERAO_SPI2_BCLK_BIT) == 0u) ? 0 : -2;
}

void mt6897_spi_configure_pins(const mt6897_spi_ctrl_t *c) {
    if (!c || !c->gpio_base) return;

    /* GPIO 9 (IRQ): mode 0 (GPIO), input (DIR0 bit 9 = 0), IES0 bit 19 = 1. */
    w32(c, c->gpio_base, MT6897_REG_CLR(MT6897_GPIO_MODE1), (0xFu << 4));
    w32(c, c->gpio_base, MT6897_REG_CLR(MT6897_GPIO_DIR0), (1u << TB375FC_GPIO_TP_IRQ));

    /* GPIO 11, 13, 14: mode 1 (SPI2); GPIO 12 (CSB): mode 1 if HW CS, mode 0 high if manual CS. */
    uint32_t clr_mode1 = (0xFu << 12) | (0xFu << 16) | (0xFu << 20) | (0xFu << 24);
    uint32_t set_mode1 = (1u << 12) | (1u << 20) | (1u << 24);
    if (!c->manual_cs) {
        set_mode1 |= (1u << 16);
    }
    w32(c, c->gpio_base, MT6897_REG_CLR(MT6897_GPIO_MODE1), clr_mode1);
    w32(c, c->gpio_base, MT6897_REG_SET(MT6897_GPIO_MODE1), set_mode1);

    if (c->manual_cs) {
        /* Deassert CS (high) before enabling output driver on GPIO 12. */
        w32(c, c->gpio_base, MT6897_REG_SET(MT6897_GPIO_DOUT0), (1u << TB375FC_GPIO_SPI2_CSB));
        w32(c, c->gpio_base, MT6897_REG_SET(MT6897_GPIO_DIR0), (1u << TB375FC_GPIO_SPI2_CSB));
    }

    /* GPIO 60 (RESET): mode 0 (GPIO), output high (active-low reset released). */
    w32(c, c->gpio_base, MT6897_REG_CLR(MT6897_GPIO_MODE7), (0xFu << 16));
    w32(c, c->gpio_base, MT6897_REG_SET(MT6897_GPIO_DOUT1), (1u << 28));
    w32(c, c->gpio_base, MT6897_REG_SET(MT6897_GPIO_DIR1), (1u << 28));

    /* Enable input buffers (IES) on IRQ (GPIO 9) and SPI2 pins (GPIO 11..14) in iocfg_tm. */
    if (c->iocfg_tm_base) {
        w32(c, c->iocfg_tm_base, MT6897_REG_SET(MT6897_IOCFG_TM_IES0), (1u << 19));
        w32(c, c->iocfg_tm_base, MT6897_REG_SET(MT6897_IOCFG_TM_IES1),
            (1u << 12) | (1u << 13) | (1u << 14) | (1u << 15));
    }
}

void mt6897_tp_set_reset(const mt6897_spi_ctrl_t *c, int high) {
    if (!c || !c->gpio_base) return;
    w32(c, c->gpio_base,
        high ? MT6897_REG_SET(MT6897_GPIO_DOUT1) : MT6897_REG_CLR(MT6897_GPIO_DOUT1),
        (1u << 28));
}

int mt6897_tp_read_irq(const mt6897_spi_ctrl_t *c) {
    if (!c || !c->gpio_base) return 0;
    return (r32(c, c->gpio_base, MT6897_GPIO_DIN0) >> TB375FC_GPIO_TP_IRQ) & 1;
}

static void spi_hw_reset_pulse(const mt6897_spi_ctrl_t *c) {
    uint32_t cmd = r32(c, c->spi2_base, MTK_SPI_CMD_REG);
    w32(c, c->spi2_base, MTK_SPI_CMD_REG, cmd | MTK_SPI_CMD_RST);
    udelay_wrap(c, 1);
    /* Matches mtk_spi_set_cs() ipm_v2 path: clear RST (bit 2) and IPM_NONIDLE (bit 18). */
    cmd = r32(c, c->spi2_base, MTK_SPI_CMD_REG);
    w32(c, c->spi2_base, MTK_SPI_CMD_REG, cmd & ~(MTK_SPI_CMD_RST | MTK_SPI_CMD_IPM_NONIDLE));
}

void mt6897_spi_init_controller(mt6897_spi_ctrl_t *c) {
    if (!c || !c->spi2_base) return;
    if (c->src_clk_hz == 0u) {
        c->src_clk_hz = c->use_26m_parent ? MT6897_SPI2_CLK_26M_HZ : MT6897_SPI2_CLK_MAINPLL_HZ;
    }
    if (c->bus_hz == 0u) {
        c->bus_hz = TB375FC_NVT_SPI_MAX_HZ;
    }
    if (c->tick_dly == 0u) {
        c->tick_dly = 3u; /* live-fdt.dts: mediatek,tickdly = <0x03 0x00> */
    }

    /* Reset controller to clean idle state. */
    spi_hw_reset_pulse(c);

    /* Configure CMD register matching mtk_spi_prepare_message() for mt6985_compat (SPI mode 0, MSB first). */
    uint32_t cmd = 0u;
    cmd |= MTK_SPI_CMD_TXMSBF | MTK_SPI_CMD_RXMSBF;
    cmd |= MTK_SPI_CMD_IPM_GET_TICKDLY | MTK_SPI_CMD_IPM_NONIDLE; /* 0xC0000 for ipm_v2 */
    cmd |= ((c->tick_dly & 0x7u) << MTK_SPI_CMD_TICK_DLY_SHIFT);
    /* Little-endian FIFO packing (RX_ENDIAN=0, TX_ENDIAN=0), mode 0 (CPOL=0, CPHA=0),
     * active-low CS (CS_POL=0), FIFO mode (TX_DMA=0, RX_DMA=0), polling (IE=0). */
    w32(c, c->spi2_base, MTK_SPI_CMD_REG, cmd);

    /* mediatek,pad-select = <0x00> */
    w32(c, c->spi2_base, MTK_SPI_PAD_SEL_REG, 0u);

    /* Default CS setup/hold timing in CFG0 (enhance_timing=1: 16-bit hold [31:16], 16-bit setup [15:0]). */
    w32(c, c->spi2_base, MTK_SPI_CFG0_REG, (3u << 16) | 3u);

    /* Default CS idle time [7:0] = 3 in CFG1. */
    w32(c, c->spi2_base, MTK_SPI_CFG1_REG, 3u);

    /* Set SCK divider in CFG2 (+0x28). */
    uint32_t half_div = mt6897_spi_calc_half_div(c->src_clk_hz, c->bus_hz);
    w32(c, c->spi2_base, MTK_SPI_CFG2_REG, mt6897_spi_cfg2_for_half_div(half_div));

    /* Drain any stale status. */
    (void)r32(c, c->spi2_base, MTK_SPI_STATUS0_REG);
}

static void spi_set_manual_cs(const mt6897_spi_ctrl_t *c, int assert_low) {
    w32(c, c->gpio_base,
        assert_low ? MT6897_REG_CLR(MT6897_GPIO_DOUT0) : MT6897_REG_SET(MT6897_GPIO_DOUT0),
        (1u << TB375FC_GPIO_SPI2_CSB));
}

int mt6897_spi_xfer(mt6897_spi_ctrl_t *c, const uint8_t *tx, uint8_t *rx, size_t len) {
    if (!c || !c->spi2_base || len == 0 || (!tx && !rx)) return -1;

    /* 1. Configure CFG3_IPM (+0x40) duplex mode matching mtk_spi_transfer_one(). */
    uint32_t cfg3 = MTK_SPI_CFG3_FULL_DUPLEX;
    if (tx && !rx)      cfg3 = MTK_SPI_CFG3_HALF_TX_1BIT;
    else if (!tx && rx) cfg3 = MTK_SPI_CFG3_HALF_RX_1BIT;
    w32(c, c->spi2_base, MTK_SPI_CFG3_IPM_REG, cfg3);

    /* 2. Program SCK divider in CFG2 (+0x28). */
    uint32_t half_div = mt6897_spi_calc_half_div(c->src_clk_hz, c->bus_hz);
    w32(c, c->spi2_base, MTK_SPI_CFG2_REG, mt6897_spi_cfg2_for_half_div(half_div));

    /* 3. Prepare CMD: ensure IPMv2 bits set, DMA/IRQ bits clear. */
    uint32_t cmd = r32(c, c->spi2_base, MTK_SPI_CMD_REG);
    cmd &= ~(MTK_SPI_CMD_TX_DMA | MTK_SPI_CMD_RX_DMA | MTK_SPI_CMD_FINISH_IE | MTK_SPI_CMD_PAUSE_IE | MTK_SPI_CMD_PAUSE_EN_IPMV2);
    cmd |= MTK_SPI_CMD_IPM_GET_TICKDLY | MTK_SPI_CMD_IPM_NONIDLE;
    int multi_chunk = (len > MTK_SPI_FIFO_SIZE);

    if (c->manual_cs) {
        spi_set_manual_cs(c, 1);
    } else if (multi_chunk) {
        /* Hold HW CS asserted across 32-byte FIFO chunks using IPM v2 PAUSE_EN (BIT 7). */
        cmd |= MTK_SPI_CMD_PAUSE_EN_IPMV2;
    }
    w32(c, c->spi2_base, MTK_SPI_CMD_REG, cmd);

    int paused = 0;
    size_t done = 0;
    int err = 0;

    while (done < len) {
        size_t rem = len - done;
        uint32_t chunk = (rem > MTK_SPI_FIFO_SIZE) ? MTK_SPI_FIFO_SIZE : (uint32_t)rem;

        /* Program chunk length in CFG1 (+0x04). */
        uint32_t cfg1 = r32(c, c->spi2_base, MTK_SPI_CFG1_REG);
        w32(c, c->spi2_base, MTK_SPI_CFG1_REG, mt6897_spi_cfg1_for_chunk(cfg1, chunk));

        /* Load TX FIFO (+0x10), 4 bytes/word + trailing zero-padded word. */
        if (tx) {
            uint32_t words = chunk / 4u;
            for (uint32_t i = 0; i < words; ++i) {
                uint32_t w = 0;
                memcpy(&w, tx + done + i * 4u, 4);
                w32(c, c->spi2_base, MTK_SPI_TX_DATA_REG, w);
            }
            uint32_t tail = chunk & 3u;
            if (tail) {
                uint32_t w = 0;
                memcpy(&w, tx + done + words * 4u, tail);
                w32(c, c->spi2_base, MTK_SPI_TX_DATA_REG, w);
            }
        }

        /* Drain any stale STATUS0 flags before starting first chunk. */
        if (!paused) {
            (void)r32(c, c->spi2_base, MTK_SPI_STATUS0_REG);
        }

        /* Trigger ACT (first chunk) or RESUME (subsequent paused chunks). */
        uint32_t trig = r32(c, c->spi2_base, MTK_SPI_CMD_REG);
        trig |= paused ? MTK_SPI_CMD_RESUME : MTK_SPI_CMD_ACT;
        w32(c, c->spi2_base, MTK_SPI_CMD_REG, trig);

        /* Poll read-to-clear STATUS0 (+0x1c) for FINISH_IF (bit 0) or PAUSE_IF (bit 1).
         * Wait 50us per poll (up to 4000 * 50us = 200ms): avoids hammering read-to-clear STATUS0
         * over APB (which broke Boot 54) while cutting 140s IODelay(1) overhead down to ~2s! */
        uint32_t st0 = 0;
        for (uint32_t spin = 0; spin < 4000u; ++spin) {
            udelay_wrap(c, 50);
            st0 = r32(c, c->spi2_base, MTK_SPI_STATUS0_REG);
            if ((st0 & (MTK_SPI_STATUS0_FINISH_IF | MTK_SPI_STATUS0_PAUSE_IF)) != 0u) break;
        }
        if ((st0 & (MTK_SPI_STATUS0_FINISH_IF | MTK_SPI_STATUS0_PAUSE_IF)) == 0u) {
            err = -110; /* ETIMEDOUT */
            break;
        }
        paused = (st0 & MTK_SPI_STATUS0_PAUSE_IF) ? 1 : 0;

        /* Drain RX FIFO (+0x14). */
        if (rx) {
            uint32_t words = chunk / 4u;
            for (uint32_t i = 0; i < words; ++i) {
                uint32_t w = r32(c, c->spi2_base, MTK_SPI_RX_DATA_REG);
                memcpy(rx + done + i * 4u, &w, 4);
            }
            uint32_t tail = chunk & 3u;
            if (tail) {
                uint32_t w = r32(c, c->spi2_base, MTK_SPI_RX_DATA_REG);
                memcpy(rx + done + words * 4u, &w, tail);
            }
        }

        done += chunk;
    }

    /* Deassert CS & reset paused state (matches mtk_spi_set_cs(false) for ipm_v2). */
    if (c->manual_cs) {
        spi_set_manual_cs(c, 0);
        if (err != 0) spi_hw_reset_pulse(c);
    } else if (multi_chunk || paused || err != 0) {
        uint32_t c2 = r32(c, c->spi2_base, MTK_SPI_CMD_REG);
        w32(c, c->spi2_base, MTK_SPI_CMD_REG, c2 & ~MTK_SPI_CMD_PAUSE_EN_IPMV2);
        spi_hw_reset_pulse(c);
    }

    return err;
}
