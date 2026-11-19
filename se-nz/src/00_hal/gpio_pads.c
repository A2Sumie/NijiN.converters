#include "../../include/hc32f460_regs.h"

static const uint32_t PIN_PA0_E_RXD     = 0UL;
static const uint32_t PIN_PA1_E_TXD     = 1UL;
static const uint32_t PIN_PA2_E_BODY_CS = 2UL;
static const uint32_t PIN_PA3_E_XDETECT = 3UL;
static const uint32_t PIN_PA9_Z_RDY     = 9UL;
static const uint32_t PIN_PA11_Z_HCLK   = 11UL;
static const uint32_t PIN_PA12_Z_HDATA  = 12UL;

static const uint32_t PIN_PB13_Z_MISO   = 13UL;
static const uint32_t PIN_PB14_Z_SCK    = 14UL;
static const uint32_t PIN_PB15_Z_MOSI   = 15UL;

static const uint32_t PIN_PC13_E4_VD    = 13UL;
static const uint32_t PIN_PH2_E_LENS_CS = 2UL;

BORYOZA_FORCE_INLINE uint32_t pin_mask32(uint32_t pin) {
    return 1UL << pin;
}

BORYOZA_FORCE_INLINE uint16_t pin_mask16(uint32_t pin) {
    return (uint16_t)(1UL << pin);
}

void gpio_pads_cold_init(void) {
    mmio_w16(GPIO_PWPR_ADDR, GPIO_PWPR_UNLOCK);

    /* 1. Pre-latch PA9 = 0 (LOW), PA1 = 0, PA2 = 0 BEFORE enabling POUTE */
    mmio_w16(gpio_porr_addr(GPIO_PORT_A),
             (uint16_t)(pin_mask32(PIN_PA9_Z_RDY) |
                        pin_mask32(PIN_PA1_E_TXD) |
                        pin_mask32(PIN_PA2_E_BODY_CS)));

    mmio_w16(gpio_pfsr_addr(GPIO_PORT_A, PIN_PA9_Z_RDY), GPIO_FSEL_GPIO);
    mmio_w16(gpio_pcr_addr(GPIO_PORT_A, PIN_PA9_Z_RDY),
             (uint16_t)(GPIO_PCR_POUTE_BIT | GPIO_PCR_DRV_LOW));

    mmio_w16(gpio_pfsr_addr(GPIO_PORT_A, PIN_PA1_E_TXD), GPIO_FSEL_GPIO);
    mmio_w16(gpio_pcr_addr(GPIO_PORT_A, PIN_PA1_E_TXD),
             (uint16_t)(GPIO_PCR_POUTE_BIT | GPIO_PCR_DRV_LOW));

    mmio_w16(gpio_pfsr_addr(GPIO_PORT_A, PIN_PA2_E_BODY_CS), GPIO_FSEL_GPIO);
    mmio_w16(gpio_pcr_addr(GPIO_PORT_A, PIN_PA2_E_BODY_CS),
             (uint16_t)(GPIO_PCR_POUTE_BIT | GPIO_PCR_DRV_LOW));

    /* 2. PA0 = input (E RXD), PA3 = LENS_XDETECT input (PUU=1), PH2 = LENS_CS input */
    mmio_w16(gpio_pfsr_addr(GPIO_PORT_A, PIN_PA0_E_RXD), GPIO_FSEL_GPIO);
    mmio_w16(gpio_pcr_addr(GPIO_PORT_A, PIN_PA0_E_RXD), 0x0000U);

    mmio_w16(gpio_pfsr_addr(GPIO_PORT_A, PIN_PA3_E_XDETECT), GPIO_FSEL_GPIO);
    mmio_w16(gpio_pcr_addr(GPIO_PORT_A, PIN_PA3_E_XDETECT), (uint16_t)GPIO_PCR_PUU_BIT);

    mmio_w16(gpio_pfsr_addr(GPIO_PORT_H, PIN_PH2_E_LENS_CS), GPIO_FSEL_GPIO);
    mmio_w16(gpio_pcr_addr(GPIO_PORT_H, PIN_PH2_E_LENS_CS), 0x0000U);

    /* 3. Keep Z-mount SPI1 (PA11/12) & SPI3 (PB13/14/15) Hi-Z input until PB14 SCK is sensed high */
    mmio_w16(gpio_pfsr_addr(GPIO_PORT_A, PIN_PA11_Z_HCLK), GPIO_FSEL_GPIO);
    mmio_w16(gpio_pcr_addr(GPIO_PORT_A, PIN_PA11_Z_HCLK), 0x0000U);
    mmio_w16(gpio_pfsr_addr(GPIO_PORT_A, PIN_PA12_Z_HDATA), GPIO_FSEL_GPIO);
    mmio_w16(gpio_pcr_addr(GPIO_PORT_A, PIN_PA12_Z_HDATA), 0x0000U);

    mmio_w16(gpio_pfsr_addr(GPIO_PORT_B, PIN_PB13_Z_MISO), GPIO_FSEL_GPIO);
    mmio_w16(gpio_pcr_addr(GPIO_PORT_B, PIN_PB13_Z_MISO), 0x0000U);
    mmio_w16(gpio_pfsr_addr(GPIO_PORT_B, PIN_PB14_Z_SCK), GPIO_FSEL_GPIO);
    mmio_w16(gpio_pcr_addr(GPIO_PORT_B, PIN_PB14_Z_SCK), 0x0000U);
    mmio_w16(gpio_pfsr_addr(GPIO_PORT_B, PIN_PB15_Z_MOSI), GPIO_FSEL_GPIO);
    mmio_w16(gpio_pcr_addr(GPIO_PORT_B, PIN_PB15_Z_MOSI), 0x0000U);

    /* 4. Keep PC13 Hi-Z input until E-mount TMRA4_PWM8 starts */
    mmio_w16(gpio_pfsr_addr(GPIO_PORT_C, PIN_PC13_E4_VD), GPIO_FSEL_GPIO);
    mmio_w16(gpio_pcr_addr(GPIO_PORT_C, PIN_PC13_E4_VD), 0x0000U);

    mmio_w16(GPIO_PWPR_ADDR, GPIO_PWPR_LOCK);
}

void gpio_pads_bind_z_bus(void) {
    mmio_w16(GPIO_PWPR_ADDR, GPIO_PWPR_UNLOCK);

    /* Pre-latch PA11/PA12 HIGH (SPI Mode 3 CPOL=1 idle) before FSEL so frameless SPI1 DMA sees zero edge glitch */
    mmio_w16(gpio_posr_addr(GPIO_PORT_A),
             (uint16_t)(pin_mask32(PIN_PA11_Z_HCLK) | pin_mask32(PIN_PA12_Z_HDATA)));
    mmio_w16(gpio_pcr_addr(GPIO_PORT_A, PIN_PA11_Z_HCLK),
             (uint16_t)(GPIO_PCR_POUT_BIT | GPIO_PCR_DRV_LOW));
    mmio_w16(gpio_pfsr_addr(GPIO_PORT_A, PIN_PA11_Z_HCLK), GPIO_FSEL_SPI_SCK);

    mmio_w16(gpio_pcr_addr(GPIO_PORT_A, PIN_PA12_Z_HDATA),
             (uint16_t)(GPIO_PCR_POUT_BIT | GPIO_PCR_DRV_LOW));
    mmio_w16(gpio_pfsr_addr(GPIO_PORT_A, PIN_PA12_Z_HDATA), GPIO_FSEL_SPI_MOSI);

    /* Bind PB13/14/15 to SPI3 Slave (FSEL=41 MISO, FSEL=43 SCK, FSEL=40 MOSI) */
    mmio_w16(gpio_pcr_addr(GPIO_PORT_B, PIN_PB13_Z_MISO), (uint16_t)GPIO_PCR_DRV_LOW);
    mmio_w16(gpio_pfsr_addr(GPIO_PORT_B, PIN_PB13_Z_MISO), GPIO_FSEL_SPI_MISO);

    mmio_w16(gpio_pcr_addr(GPIO_PORT_B, PIN_PB14_Z_SCK), 0x0000U);
    mmio_w16(gpio_pfsr_addr(GPIO_PORT_B, PIN_PB14_Z_SCK), GPIO_FSEL_SPI_SCK);

    mmio_w16(gpio_pcr_addr(GPIO_PORT_B, PIN_PB15_Z_MOSI), 0x0000U);
    mmio_w16(gpio_pfsr_addr(GPIO_PORT_B, PIN_PB15_Z_MOSI), GPIO_FSEL_SPI_MOSI);

    mmio_w16(GPIO_PWPR_ADDR, GPIO_PWPR_LOCK);
}

static void gpio_pads_z_pins_hiz_unlocked(void) {
    mmio_w16(gpio_pfsr_addr(GPIO_PORT_A, PIN_PA11_Z_HCLK), GPIO_FSEL_GPIO);
    mmio_w16(gpio_pcr_addr(GPIO_PORT_A, PIN_PA11_Z_HCLK), 0x0000U);
    mmio_w16(gpio_pfsr_addr(GPIO_PORT_A, PIN_PA12_Z_HDATA), GPIO_FSEL_GPIO);
    mmio_w16(gpio_pcr_addr(GPIO_PORT_A, PIN_PA12_Z_HDATA), 0x0000U);
    mmio_w16(gpio_pfsr_addr(GPIO_PORT_B, PIN_PB13_Z_MISO), GPIO_FSEL_GPIO);
    mmio_w16(gpio_pcr_addr(GPIO_PORT_B, PIN_PB13_Z_MISO), 0x0000U);
    mmio_w16(gpio_pfsr_addr(GPIO_PORT_B, PIN_PB14_Z_SCK), GPIO_FSEL_GPIO);
    mmio_w16(gpio_pcr_addr(GPIO_PORT_B, PIN_PB14_Z_SCK), 0x0000U);
    mmio_w16(gpio_pfsr_addr(GPIO_PORT_B, PIN_PB15_Z_MOSI), GPIO_FSEL_GPIO);
    mmio_w16(gpio_pcr_addr(GPIO_PORT_B, PIN_PB15_Z_MOSI), 0x0000U);
}

void gpio_pads_release_z_bus(void) {
    mmio_w16(GPIO_PWPR_ADDR, GPIO_PWPR_UNLOCK);
    mmio_w16(gpio_posr_addr(GPIO_PORT_A), pin_mask16(PIN_PA9_Z_RDY));
    gpio_pads_z_pins_hiz_unlocked();
    mmio_w16(GPIO_PWPR_ADDR, GPIO_PWPR_LOCK);
}

void gpio_pads_quiesce_z_powerdown(void) {
    mmio_w16(GPIO_PWPR_ADDR, GPIO_PWPR_UNLOCK);
    mmio_w16(gpio_porr_addr(GPIO_PORT_A), pin_mask16(PIN_PA9_Z_RDY));
    mmio_w16(gpio_pfsr_addr(GPIO_PORT_A, PIN_PA9_Z_RDY), GPIO_FSEL_GPIO);
    mmio_w16(gpio_pcr_addr(GPIO_PORT_A, PIN_PA9_Z_RDY),
             (uint16_t)(GPIO_PCR_POUTE_BIT | GPIO_PCR_DRV_LOW));
    gpio_pads_z_pins_hiz_unlocked();
    mmio_w16(GPIO_PWPR_ADDR, GPIO_PWPR_LOCK);
}

void gpio_pads_bind_pc13_vsync(void) {
    mmio_w16(GPIO_PWPR_ADDR, GPIO_PWPR_UNLOCK);
    mmio_w16(gpio_posr_addr(GPIO_PORT_C), pin_mask16(PIN_PC13_E4_VD));
    mmio_w16(gpio_pcr_addr(GPIO_PORT_C, PIN_PC13_E4_VD),
             (uint16_t)(GPIO_PCR_POUT_BIT | GPIO_PCR_POUTE_BIT | GPIO_PCR_DRV_LOW));
    mmio_w16(gpio_pfsr_addr(GPIO_PORT_C, PIN_PC13_E4_VD), GPIO_FSEL_TMRA4_PWM8);
    mmio_w16(GPIO_PWPR_ADDR, GPIO_PWPR_LOCK);
}

void gpio_pads_release_pc13_vsync(void) {
    mmio_w16(GPIO_PWPR_ADDR, GPIO_PWPR_UNLOCK);
    mmio_w16(gpio_porr_addr(GPIO_PORT_C), pin_mask16(PIN_PC13_E4_VD));
    mmio_w16(gpio_pfsr_addr(GPIO_PORT_C, PIN_PC13_E4_VD), GPIO_FSEL_GPIO);
    mmio_w16(gpio_pcr_addr(GPIO_PORT_C, PIN_PC13_E4_VD), 0x0000U);
    mmio_w16(GPIO_PWPR_ADDR, GPIO_PWPR_LOCK);
}

BORYOZA_RAMFUNC void gpio_pa9_set_busy(void) {
    mmio_w16(gpio_posr_addr(GPIO_PORT_A), pin_mask16(PIN_PA9_Z_RDY));
}

BORYOZA_RAMFUNC void gpio_pa9_set_ready(void) {
    mmio_w16(gpio_porr_addr(GPIO_PORT_A), pin_mask16(PIN_PA9_Z_RDY));
}

BORYOZA_RAMFUNC uint8_t gpio_pa9_is_busy(void) {
    uint32_t podr = mmio_r16(gpio_podr_addr(GPIO_PORT_A));
    return (uint8_t)((podr >> PIN_PA9_Z_RDY) & 1UL);
}

BORYOZA_RAMFUNC uint8_t gpio_pa0_rxd_high(void) {
    uint32_t pidr = mmio_r16(gpio_pidr_addr(GPIO_PORT_A));
    return (uint8_t)((pidr >> PIN_PA0_E_RXD) & 1UL);
}

BORYOZA_RAMFUNC uint8_t gpio_pa3_lens_mounted(void) {
    uint32_t pidr = mmio_r16(gpio_pidr_addr(GPIO_PORT_A));
    return (uint8_t)(((pidr >> PIN_PA3_E_XDETECT) & 1UL) == 0UL ? 1U : 0U);
}

BORYOZA_RAMFUNC uint8_t gpio_pb14_sck_high(void) {
    uint32_t pidr = mmio_r16(gpio_pidr_addr(GPIO_PORT_B));
    return (uint8_t)((pidr >> PIN_PB14_Z_SCK) & 1UL);
}

BORYOZA_RAMFUNC uint8_t gpio_ph2_high(void) {
    uint32_t pidr = mmio_r16(gpio_pidr_addr(GPIO_PORT_H));
    return (uint8_t)((pidr >> PIN_PH2_E_LENS_CS) & 1UL);
}

BORYOZA_RAMFUNC void gpio_pa2_set_high(void) {
    mmio_w16(gpio_posr_addr(GPIO_PORT_A), pin_mask16(PIN_PA2_E_BODY_CS));
}

BORYOZA_RAMFUNC void gpio_pa2_set_low(void) {
    mmio_w16(gpio_porr_addr(GPIO_PORT_A), pin_mask16(PIN_PA2_E_BODY_CS));
}

void gpio_pa1_enable_uart_tx(void) {
    mmio_w16(GPIO_PWPR_ADDR, GPIO_PWPR_UNLOCK);
    mmio_w16(gpio_pcr_addr(GPIO_PORT_A, PIN_PA0_E_RXD), (uint16_t)GPIO_PCR_PUU_BIT);
    mmio_w16(gpio_pfsr_addr(GPIO_PORT_A, PIN_PA0_E_RXD), GPIO_FSEL_USART1_RX);
    mmio_w16(gpio_posr_addr(GPIO_PORT_A), pin_mask16(PIN_PA1_E_TXD));
    mmio_w16(gpio_pcr_addr(GPIO_PORT_A, PIN_PA1_E_TXD),
             (uint16_t)(GPIO_PCR_POUT_BIT | GPIO_PCR_POUTE_BIT | GPIO_PCR_DRV_LOW));
    mmio_w16(gpio_pfsr_addr(GPIO_PORT_A, PIN_PA1_E_TXD), GPIO_FSEL_USART1_TX);
    mmio_w16(GPIO_PWPR_ADDR, GPIO_PWPR_LOCK);
}

void gpio_pa1_drive_low_gpio(void) {
    mmio_w16(GPIO_PWPR_ADDR, GPIO_PWPR_UNLOCK);
    mmio_w16(gpio_pfsr_addr(GPIO_PORT_A, PIN_PA0_E_RXD), GPIO_FSEL_GPIO);
    mmio_w16(gpio_pcr_addr(GPIO_PORT_A, PIN_PA0_E_RXD), 0x0000U);
    mmio_w16(gpio_porr_addr(GPIO_PORT_A), pin_mask16(PIN_PA1_E_TXD));
    mmio_w16(gpio_pfsr_addr(GPIO_PORT_A, PIN_PA1_E_TXD), GPIO_FSEL_GPIO);
    mmio_w16(gpio_pcr_addr(GPIO_PORT_A, PIN_PA1_E_TXD),
             (uint16_t)(GPIO_PCR_POUTE_BIT | GPIO_PCR_DRV_LOW));
    mmio_w16(GPIO_PWPR_ADDR, GPIO_PWPR_LOCK);
}

void gpio_pa1_latch_high_for_baud_switch(uint8_t latch_en) {
    mmio_w16(GPIO_PWPR_ADDR, GPIO_PWPR_UNLOCK);
    uint32_t pcr = mmio_r16(gpio_pcr_addr(GPIO_PORT_A, PIN_PA1_E_TXD));
    if (latch_en != 0U) {
        pcr |= GPIO_PCR_LTE_BIT;
    } else {
        pcr &= ~GPIO_PCR_LTE_BIT;
    }
    mmio_w16(gpio_pcr_addr(GPIO_PORT_A, PIN_PA1_E_TXD), (uint16_t)pcr);
    mmio_w16(GPIO_PWPR_ADDR, GPIO_PWPR_LOCK);
}
