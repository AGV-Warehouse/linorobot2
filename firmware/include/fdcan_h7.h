#ifndef FDCAN_H7_H
#define FDCAN_H7_H

// Minimal classic-CAN (2.0B) driver for the STM32H7 FDCAN peripheral, with a
// FlexCAN_T4-style API (CAN_message_t + begin/setBaudRate/write/read) so the
// AK10 driver's call sites are identical to the original Teensy firmware.
//
// Written against the STM32duino HAL directly instead of a third-party CAN
// library: the popular Arduino CAN libraries (pazi88/STM32_CAN etc.) only
// support the older bxCAN peripheral, which the H7 does not have.
//
// Fixed configuration: FDCAN1 on PD0 (CAN_RX, AF9) / PD1 (CAN_TX, AF9).
// PA11/PA12 (the alternate FDCAN1 pins) are deliberately avoided -- they are
// the USB OTG FS pins. The FDCAN kernel clock is switched to HSE (8 MHz on
// the Nucleo-H753ZI, fed by the ST-LINK MCO; the variant's clock config runs
// it in bypass mode), which divides exactly to the 1 Mbps bus rate.
//
// Requires -DHAL_FDCAN_MODULE_ENABLED (set in platformio.ini).

#include <Arduino.h>

#if !defined(HAL_FDCAN_MODULE_ENABLED)
#error "fdcan_h7.h needs -DHAL_FDCAN_MODULE_ENABLED in platformio.ini build_flags"
#endif

// Same field layout as FlexCAN_T4's / STM32_CAN's message struct, reduced to
// the fields this firmware uses.
typedef struct CAN_message_t
{
    uint32_t id = 0;            // arbitration id
    struct
    {
        bool extended = false;  // 29-bit id
    } flags;
    uint8_t len = 8;
    uint8_t buf[8] = {0};
} CAN_message_t;

class FdCanH7
{
public:
    // Kept for FlexCAN_T4 call-order compatibility (begin() then
    // setBaudRate()); all hardware init happens in setBaudRate().
    void begin() {}

    // Full peripheral bring-up: kernel clock, GPIO, bit timing, accept-all
    // filter into RX FIFO0, start.
    bool setBaudRate(uint32_t bitrate)
    {
        // FDCAN kernel clock <- HSE (8 MHz): small, exact divisor of 1 Mbps.
        RCC_PeriphCLKInitTypeDef pclk = {};
        pclk.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
        pclk.FdcanClockSelection = RCC_FDCANCLKSOURCE_HSE;
        if (HAL_RCCEx_PeriphCLKConfig(&pclk) != HAL_OK)
            return false;

        __HAL_RCC_FDCAN_CLK_ENABLE();
        __HAL_RCC_GPIOD_CLK_ENABLE();

        GPIO_InitTypeDef gpio = {};
        gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1;   // PD0 = RX, PD1 = TX
        gpio.Mode = GPIO_MODE_AF_PP;
        gpio.Pull = GPIO_NOPULL;
        gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        gpio.Alternate = GPIO_AF9_FDCAN1;
        HAL_GPIO_Init(GPIOD, &gpio);

        // Bit timing: find a time-quanta count (8..25 per bit) that divides
        // the kernel clock exactly, sample point ~87.5% (CiA recommendation).
        // 8 MHz @ 1 Mbps -> prescaler 1, 8 tq: sync(1) + tseg1(6) + tseg2(1).
        uint32_t kernel_hz = HSE_VALUE;
        uint32_t total_tq = 0, prescaler = 0;
        for (uint32_t tq = 8; tq <= 25; tq++)
        {
            if (kernel_hz % (bitrate * tq) != 0)
                continue;
            uint32_t p = kernel_hz / (bitrate * tq);
            if (p >= 1 && p <= 512)
            {
                total_tq = tq;
                prescaler = p;
                break;
            }
        }
        if (total_tq == 0)
            return false;   // bitrate not exactly reachable from HSE
        uint32_t tseg1 = (total_tq * 7) / 8 - 1;         // ~87.5% sample point
        uint32_t tseg2 = total_tq - 1 - tseg1;

        hfdcan_.Instance = FDCAN1;
        hfdcan_.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
        hfdcan_.Init.Mode = FDCAN_MODE_NORMAL;
        hfdcan_.Init.AutoRetransmission = ENABLE;
        hfdcan_.Init.TransmitPause = DISABLE;
        hfdcan_.Init.ProtocolException = DISABLE;
        hfdcan_.Init.NominalPrescaler = prescaler;
        hfdcan_.Init.NominalSyncJumpWidth = 1;
        hfdcan_.Init.NominalTimeSeg1 = tseg1;
        hfdcan_.Init.NominalTimeSeg2 = tseg2;
        // Data phase is unused in classic mode but the fields must be valid
        // (DataTimeSeg1 max is 32, so the nominal values fit).
        hfdcan_.Init.DataPrescaler = prescaler;
        hfdcan_.Init.DataSyncJumpWidth = 1;
        hfdcan_.Init.DataTimeSeg1 = tseg1;
        hfdcan_.Init.DataTimeSeg2 = tseg2;
        // H7 message RAM layout (FDCAN1 owns offset 0; FDCAN2 is unused).
        hfdcan_.Init.MessageRAMOffset = 0;
        hfdcan_.Init.StdFiltersNbr = 0;      // no match filters: global
        hfdcan_.Init.ExtFiltersNbr = 0;      //   accept-all does the routing
        hfdcan_.Init.RxFifo0ElmtsNbr = 32;
        hfdcan_.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
        hfdcan_.Init.RxFifo1ElmtsNbr = 0;
        hfdcan_.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
        hfdcan_.Init.RxBuffersNbr = 0;
        hfdcan_.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
        hfdcan_.Init.TxEventsNbr = 0;
        hfdcan_.Init.TxBuffersNbr = 0;
        hfdcan_.Init.TxFifoQueueElmtsNbr = 8;
        hfdcan_.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
        hfdcan_.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
        if (HAL_FDCAN_Init(&hfdcan_) != HAL_OK)
            return false;

        // Route every frame (std + ext) to RX FIFO0; the AK10 status frames
        // use extended ids (e.g. 0x2968) and ak10Poll() sorts them by id.
        if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan_,
                                         FDCAN_ACCEPT_IN_RX_FIFO0,
                                         FDCAN_ACCEPT_IN_RX_FIFO0,
                                         FDCAN_REJECT_REMOTE,
                                         FDCAN_REJECT_REMOTE) != HAL_OK)
            return false;

        return HAL_FDCAN_Start(&hfdcan_) == HAL_OK;
    }

    bool write(const CAN_message_t &msg)
    {
        // DLC macros encode the byte count directly for <= 8 bytes in current
        // HAL versions; index a table anyway so an older (<<16-encoded) HAL
        // still produces the right register value.
        static const uint32_t dlc[9] = {
            FDCAN_DLC_BYTES_0, FDCAN_DLC_BYTES_1, FDCAN_DLC_BYTES_2,
            FDCAN_DLC_BYTES_3, FDCAN_DLC_BYTES_4, FDCAN_DLC_BYTES_5,
            FDCAN_DLC_BYTES_6, FDCAN_DLC_BYTES_7, FDCAN_DLC_BYTES_8};

        FDCAN_TxHeaderTypeDef tx = {};
        tx.Identifier = msg.id;
        tx.IdType = msg.flags.extended ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
        tx.TxFrameType = FDCAN_DATA_FRAME;
        tx.DataLength = dlc[msg.len <= 8 ? msg.len : 8];
        tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
        tx.BitRateSwitch = FDCAN_BRS_OFF;
        tx.FDFormat = FDCAN_CLASSIC_CAN;
        tx.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
        tx.MessageMarker = 0;
        return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan_, &tx, msg.buf) == HAL_OK;
    }

    bool read(CAN_message_t &msg)
    {
        if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan_, FDCAN_RX_FIFO0) == 0)
            return false;
        FDCAN_RxHeaderTypeDef rx = {};
        if (HAL_FDCAN_GetRxMessage(&hfdcan_, FDCAN_RX_FIFO0, &rx, msg.buf) != HAL_OK)
            return false;
        msg.id = rx.Identifier;
        msg.flags.extended = (rx.IdType == FDCAN_EXTENDED_ID);
        // Handle both HAL DLC encodings (plain byte count vs value << 16).
        uint32_t d = rx.DataLength;
        msg.len = (d > 0xFFFFu) ? (uint8_t)(d >> 16) : (uint8_t)d;
        if (msg.len > 8)
            msg.len = 8;
        return true;
    }

private:
    FDCAN_HandleTypeDef hfdcan_ = {};
};

#endif
