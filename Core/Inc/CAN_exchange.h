/**
 ******************************************************************************
 * @file    CAN_exchange.h
 * @brief   Bulletproof CAN health monitor — header
 ******************************************************************************
 *
 * ACK validation strategy (3-layer):
 *   Layer 1 — AutoRetransmission DISABLED: TxErrorCnt grows monotonically
 *             on failure and is never silently reset by hardware retries.
 *   Layer 2 — PSR.LastErrorCode: direct protocol-level error classification
 *             read from the FDCAN peripheral status register after each TX.
 *   Layer 3 — TX Event FIFO: a frame is only stored here after a real
 *             on-bus ACK from another node. This is the definitive proof.
 *
 * A node with no other device on the bus can NEVER reach CAN_STATUS_OK:
 *   - Every TX attempt will produce LEC = ACK_ERROR (code 3)
 *   - TxErrorCnt will grow and never decrease
 *   - TX Event FIFO will remain empty
 *   - s_ack_seen will remain 0
 *
 * Usage:
 *   1. CAN_Exchange_Init()           — once at startup
 *   2. CAN_Exchange_Poll()           — regularly (main loop or timer)
 *   3. CAN_Exchange_Transmit_Data()  — whenever you want to send telemetry
 *   4. CAN_Exchange_Get_Status()     — read cached status, zero cost
 *
 ******************************************************************************
 */

#ifndef __CAN_EXCHANGE_H
#define __CAN_EXCHANGE_H

#include "stm32g4xx_hal.h"
#include <stdint.h>

#define ACTIVATE_CAN_DEBUG_SCREEN 0 // Set to 1 to enable CAN debug screen rendering on the I2C display

/* ============================================================================
 * Tuning constants
 * ========================================================================== */

/**
 * @brief Number of consecutive Poll() calls returning the same raw status
 *        before the visible (debounced) status actually changes.
 *
 *        With CAN_POLL_PERIOD_MS = 100 ms:
 *          3  → 300 ms stabilisation window  (recommended)
 *          5  → 500 ms stabilisation window
 *
 *        BUS_OFF always bypasses this and is applied immediately.
 */
#define CAN_DEBOUNCE_THRESHOLD   3U

/**
 * @brief Milliseconds after the last confirmed ACK before the monitor
 *        considers the peer node silent/lost and downgrades status.
 *        Set to 0 to disable heartbeat supervision entirely.
 *
 *        Must be > CAN_TX_PERIOD_MS + a margin, e.g. if TX every 1000 ms
 *        set this to at least 2500 ms.
 */
#define CAN_ACK_TIMEOUT_MS       3000U

/* ============================================================================
 * Diagnostic status enum
 * ========================================================================== */

/**
 * CAN_STATUS_PENDING   — No TX has been attempted yet. Bus health is unknown.
 * CAN_STATUS_OK        — At least one frame was hardware-ACKed by a peer node.
 * CAN_STATUS_ACK_ERR   — TX attempted but no node acknowledged (LEC = 3,
 *                         TxErrorCnt growing, TX Event FIFO empty).
 * CAN_STATUS_STUFF_ERR — Bit-stuffing violation detected on the bus.
 * CAN_STATUS_FORM_ERR  — Frame delimiter / framing bit corrupted.
 * CAN_STATUS_BIT1_ERR  — Node sent recessive, read back dominant.
 * CAN_STATUS_BIT0_ERR  — Node sent dominant, read back recessive (short).
 * CAN_STATUS_CRC_ERR   — CRC checksum mismatch.
 * CAN_STATUS_BUS_OFF   — TEC exceeded 255; node isolated from bus.
 *                         Recovery is automatic; status returns to PENDING
 *                         then must re-earn OK through a new ACKed TX.
 * CAN_STATUS_DISABLED  — Peripheral not initialised or deliberately offline.
 */
typedef enum
{
  CAN_STATUS_OK        = 0x00U,
  CAN_STATUS_DISABLED  = 0x01U,
  CAN_STATUS_ACK_ERR   = 0x02U,
  CAN_STATUS_BIT0_ERR  = 0x03U,
  CAN_STATUS_BIT1_ERR  = 0x04U,
  CAN_STATUS_BUS_OFF   = 0x05U,
  CAN_STATUS_STUFF_ERR = 0x06U,
  CAN_STATUS_FORM_ERR  = 0x07U,
  CAN_STATUS_CRC_ERR   = 0x08U,
  CAN_STATUS_PENDING   = 0x09U
} CAN_Status_t;

/* ============================================================================
 * Diagnostic snapshot — returned by CAN_Exchange_Get_Status()
 * ========================================================================== */
typedef struct
{
  CAN_Status_t status;          /* Debounced bus health state                 */
  uint8_t      tx_error_cnt;    /* Raw TEC from hardware (0–255)              */
  uint8_t      rx_error_cnt;    /* Raw REC from hardware (0–255)              */
  uint8_t      lec;             /* Last Error Code from PSR register          */
  uint8_t      ack_seen;        /* 1 if at least one TX Event confirmed       */
  uint32_t     last_ack_tick;   /* HAL_GetTick() of last hardware-ACKed TX    */
  uint32_t     last_poll_tick;  /* HAL_GetTick() of last Poll() call          */
} CAN_Diag_t;

/* ============================================================================
 * Public API
 * ========================================================================== */

/**
 * @brief  Initialise FDCAN peripheral and reset all health-monitor state.
 *         Must be called once before any other CAN_Exchange function.
 * @retval HAL_OK on success, HAL_ERROR if peripheral init or start failed.
 */
HAL_StatusTypeDef CAN_Exchange_Init(void);

/**
 * @brief  Poll hardware counters and update cached diagnostic snapshot.
 *         Call this regularly — recommended every CAN_POLL_PERIOD_MS (100 ms).
 *         This is the only function that changes the visible status.
 * @retval None
 */
void CAN_Exchange_Poll(void);

/**
 * @brief  Queue a telemetry frame onto the TX FIFO.
 *         Sets s_tx_attempted only on successful queue.
 *         TX Event FIFO confirmation (real ACK) is checked inside Poll().
 * @param  voltage     Pack voltage in volts (e.g. 11.4f for a 3S pack)
 * @param  percentage  State of charge 0.0–100.0
 * @retval HAL_OK if frame was queued, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef CAN_Exchange_Transmit_Data(float voltage, float percentage);

/**
 * @brief  Force a minimal probe frame onto the bus immediately.
 *         Use once at startup to kick the health monitor out of PENDING
 *         before the first regular telemetry TX.
 * @retval HAL_OK if frame was queued, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef CAN_Exchange_Trigger_Startup_Probe(void);

/**
 * @brief  Return the last debounced diagnostic snapshot (zero cost, no I/O).
 * @retval Copy of the internal CAN_Diag_t structure.
 */
CAN_Diag_t CAN_Exchange_Get_Status(void);

#endif /* __CAN_EXCHANGE_H */
