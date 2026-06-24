/**
 ******************************************************************************
 * @file    CAN_exchange.c
 * @brief   CAN health monitor — TX Event FIFO ACK detection + extended errors
 ******************************************************************************
 */

#include "CAN_exchange.h"
#include "stm32g4xx_hal.h"
#include <stdio.h>

/* ============================================================================
 * External peripheral handle — defined in main.c
 * ========================================================================== */
extern FDCAN_HandleTypeDef hfdcan1;

/* ============================================================================
 * Private constants
 * ========================================================================== */
#define LEC_ACK_ERROR    3U
#define LEC_NO_ERROR     0U
#define LEC_NO_CHANGE    7U

/* ============================================================================
 * Private state
 * ========================================================================== */
static CAN_Diag_t   s_diag            = {0};
static uint8_t      s_tx_attempted    = 0U;
static uint8_t      s_ack_seen        = 0U;
static uint32_t     s_last_ack_tick   = 0U;
static uint8_t      s_prev_tx_err     = 0U;
static CAN_Status_t s_raw_status      = CAN_STATUS_PENDING;
static CAN_Status_t s_filtered_status = CAN_STATUS_PENDING;
static uint8_t      s_stable_counter  = 0U;
static uint8_t      s_busoff_count    = 0U;

/* ============================================================================
 * Debug helpers
 * ========================================================================== */
static const char* CAN_StatusName(CAN_Status_t s)
{
  switch (s)
  {
    case CAN_STATUS_PENDING:    return "PENDING";
    case CAN_STATUS_OK:         return "OK";
    case CAN_STATUS_ACK_ERR:    return "ACK_ERR";
    case CAN_STATUS_BUS_OFF:    return "BUS_OFF";
    case CAN_STATUS_DISABLED:   return "DISABLED";
    case CAN_STATUS_STUFF_ERR:  return "STUFF_ERR";
    case CAN_STATUS_FORM_ERR:   return "FORM_ERR";
    case CAN_STATUS_BIT1_ERR:   return "BIT1_ERR";
    case CAN_STATUS_BIT0_ERR:   return "BIT0_ERR";
    case CAN_STATUS_CRC_ERR:    return "CRC_ERR";
    default:                    return "UNKNOWN";
  }
}

static const char* CAN_LECName(uint8_t lec)
{
  switch (lec)
  {
    case 0U: return "NO_ERROR";
    case 1U: return "STUFF_ERR";
    case 2U: return "FORM_ERR";
    case 3U: return "ACK_ERR";
    case 4U: return "BIT1_ERR";
    case 5U: return "BIT0_ERR";
    case 6U: return "CRC_ERR";
    case 7U: return "NO_CHANGE";
    default: return "UNKNOWN";
  }
}

/* ============================================================================
 * Extended error diagnostics
 * Print a human-readable explanation for every LEC code encountered.
 * ========================================================================== */
static void CAN_PrintExtendedErrorDiag(uint8_t lec,
                                       uint32_t tec,
                                       uint32_t rec,
                                       uint32_t busoff,
                                       uint32_t err_passive,
                                       uint32_t warning)
{
  switch (lec)
  {
    case 1U:
      printf("[CAN DIAG] STUFF_ERR — More than 5 consecutive identical bits detected.\n"
             "           Likely cause: bitrate mismatch, bad termination, or noisy cable.\n"
             "           TEC=%lu REC=%lu\n", tec, rec);
      break;

    case 2U:
      printf("[CAN DIAG] FORM_ERR — Fixed-format field (EOF, ACK delim, CRC delim) violated.\n"
             "           Likely cause: bitrate mismatch or signal integrity issue.\n"
             "           TEC=%lu REC=%lu\n", tec, rec);
      break;

    case 3U:
      printf("[CAN DIAG] ACK_ERR — No node acknowledged the transmitted frame.\n"
             "           Likely cause: no listener on bus, GND missing, or bitrate mismatch.\n"
             "           TEC=%lu REC=%lu\n", tec, rec);
      break;

    case 4U:
      printf("[CAN DIAG] BIT1_ERR — Sent recessive (1) but read dominant (0) outside arbitration.\n"
             "           Likely cause: short circuit on CANH/CANL, or two nodes transmitting.\n"
             "           TEC=%lu REC=%lu\n", tec, rec);
      break;

    case 5U:
      printf("[CAN DIAG] BIT0_ERR — Sent dominant (0) but read recessive (1).\n"
             "           Likely cause: open circuit, broken transceiver TX pin, or no GND.\n"
             "           TEC=%lu REC=%lu\n", tec, rec);
      break;

    case 6U:
      printf("[CAN DIAG] CRC_ERR — CRC mismatch between transmitter and receiver.\n"
             "           Likely cause: bitrate mismatch, sample point mismatch, or EMI.\n"
             "           TEC=%lu REC=%lu\n", tec, rec);
      break;

    case 0U:
    case 7U:
    default:
      break;
  }

  if (busoff)
  {
    printf("[CAN DIAG] BUS-OFF — TEC exceeded 255. Node is off the bus.\n"
           "           Bus will recover after 128 × 11 recessive bits.\n");
  }
  else if (err_passive)
  {
    printf("[CAN DIAG] ERROR-PASSIVE — TEC or REC >= 128. TX uses passive error frames.\n"
           "           TEC=%lu REC=%lu — reduce errors to return to ERROR-ACTIVE.\n",
           tec, rec);
  }
  else if (warning)
  {
    printf("[CAN DIAG] ERROR-WARNING — TEC or REC >= 96. Bus health degrading.\n"
           "           TEC=%lu REC=%lu\n", tec, rec);
  }
}

/* ============================================================================
 * Private function prototypes
 * ========================================================================== */
static void              CAN_ApplyDebounce(CAN_Status_t raw);
static void              CAN_Recover_BusOff(void);
static CAN_Status_t      CAN_LEC_To_Status(uint8_t lec);
static HAL_StatusTypeDef CAN_Queue_Frame(uint16_t id, const uint8_t *data, uint8_t len);

/* ============================================================================
 * Private helpers
 * ========================================================================== */
static void CAN_ApplyDebounce(CAN_Status_t raw)
{
  if (raw == s_raw_status)
  {
    if (s_stable_counter < CAN_DEBOUNCE_THRESHOLD)
    {
      s_stable_counter++;
    }
  }
  else
  {
    s_raw_status     = raw;
    s_stable_counter = 1U;
  }

  if (s_stable_counter >= CAN_DEBOUNCE_THRESHOLD)
  {
    s_filtered_status = s_raw_status;
  }
}

// Recover from Bus-Off state
static void CAN_Recover_BusOff(void)
{
  s_busoff_count++;
  printf("[CAN BUSOFF] Recovery attempt %u/3\n", s_busoff_count);

  if (s_busoff_count > 3U)
  {
    printf("[CAN BUSOFF] Too many retries — disabling CAN permanently\n");
    s_diag.status = CAN_STATUS_BUS_OFF;
    HAL_FDCAN_Stop(&hfdcan1);
    return;
  }

  HAL_FDCAN_Stop(&hfdcan1);
  HAL_Delay(128U);
  HAL_FDCAN_Start(&hfdcan1);

  s_ack_seen       = 0U;
  s_tx_attempted   = 0U;
  s_prev_tx_err    = 0U;
  s_stable_counter = 0U;

  printf("[CAN BUSOFF] Recovery done — restarted\n");
}

// Convert LEC code to CAN_Status_t to display on the I2C screen
static CAN_Status_t CAN_LEC_To_Status(uint8_t lec)
{
  switch (lec)
  {
    case 0U: return CAN_STATUS_OK;
    case 1U: return CAN_STATUS_STUFF_ERR;
    case 2U: return CAN_STATUS_FORM_ERR;
    case 3U: return CAN_STATUS_ACK_ERR;
    case 4U: return CAN_STATUS_BIT1_ERR;
    case 5U: return CAN_STATUS_BIT0_ERR;
    case 6U: return CAN_STATUS_CRC_ERR;
    case 7U: return CAN_STATUS_PENDING;
    default: return CAN_STATUS_ACK_ERR;
  }
}

// Queue a CAN frame for transmission
static HAL_StatusTypeDef CAN_Queue_Frame(uint16_t id, const uint8_t *data, uint8_t len)
{
  if (hfdcan1.State == HAL_FDCAN_STATE_RESET)
  {
    printf("[CAN TX] ERROR: FDCAN in RESET state — cannot send\n");
    return HAL_ERROR;
  }

  uint32_t dlc_map[9] = {
    FDCAN_DLC_BYTES_0, FDCAN_DLC_BYTES_1, FDCAN_DLC_BYTES_2,
    FDCAN_DLC_BYTES_3, FDCAN_DLC_BYTES_4, FDCAN_DLC_BYTES_5,
    FDCAN_DLC_BYTES_6, FDCAN_DLC_BYTES_7, FDCAN_DLC_BYTES_8
  };
  if (len > 8U) { len = 8U; }

  FDCAN_TxHeaderTypeDef hdr    = {0};
  hdr.Identifier               = (uint32_t)id;
  hdr.IdType                   = FDCAN_STANDARD_ID;
  hdr.TxFrameType              = FDCAN_DATA_FRAME;
  hdr.DataLength               = dlc_map[len];
  hdr.ErrorStateIndicator      = FDCAN_ESI_ACTIVE;
  hdr.BitRateSwitch            = FDCAN_BRS_OFF;
  hdr.FDFormat                 = FDCAN_CLASSIC_CAN;
  hdr.TxEventFifoControl       = FDCAN_STORE_TX_EVENTS; // required for ACK detection
  hdr.MessageMarker            = 0x01U;

  printf("[CAN TX] Queuing frame: ID=0x%03X, len=%u, data[0]=0x%02X\n",
         id, len, (len > 0U) ? data[0] : 0U);

  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &hdr, (uint8_t*)data) != HAL_OK)
  {
    printf("[CAN TX] ERROR: AddMessageToTxFifoQ failed (HAL error=0x%08lX)\n",
           HAL_FDCAN_GetError(&hfdcan1));
    return HAL_ERROR;
  }

  printf("[CAN TX] Frame queued OK\n");
  s_tx_attempted = 1U;
  return HAL_OK;
}

/* ============================================================================
 * Public API
 * ========================================================================== */
HAL_StatusTypeDef CAN_Exchange_Init(void)
{
  printf("[CAN INIT] Starting FDCAN1 init...\n");

  hfdcan1.Instance                  = FDCAN1;
  hfdcan1.Init.FrameFormat          = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode                 = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission   = ENABLE;
  hfdcan1.Init.TransmitPause        = DISABLE;
  hfdcan1.Init.ProtocolException    = DISABLE;

  // CAN bus speed
  // Clock speed = 170 MHz, Prescaler = 85
  // TimeQuanta = 1 (SyncJump) + 13 (TimeSEG1) + 2 (TimeSEG2) = 16
  // Sample point = (SyncJump + TimeSEG1) / TimeQuanta = 14 / 16 = 0.875 = 87.5% (within 1% of target 87.5%)
  // Calcul for Baudrate: Clock / Prescaler / TimeQuanta = 170 MHz / 85 / 16 = 125 kbit/s

  // Nominal bit timing configuration
  hfdcan1.Init.NominalPrescaler     = 85U;
  hfdcan1.Init.NominalTimeSeg1      = 13U;
  hfdcan1.Init.NominalTimeSeg2      = 2U;
  hfdcan1.Init.NominalSyncJumpWidth = 1U; // 1–4 time quanta

  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    printf("[CAN INIT] ERROR: HAL_FDCAN_Init failed\n");
    return HAL_ERROR;
  }
  printf("[CAN INIT] HAL_FDCAN_Init OK\n");

  // Accept all standard IDs 0x000–0x7FF into FIFO0
  FDCAN_FilterTypeDef filter = {0};
  filter.IdType       = FDCAN_STANDARD_ID;
  filter.FilterIndex  = 0U;
  filter.FilterType   = FDCAN_FILTER_RANGE;
  filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  filter.FilterID1    = 0x000U;
  filter.FilterID2    = 0x7FFU;

  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK)
  {
    printf("[CAN INIT] ERROR: HAL_FDCAN_ConfigFilter failed\n");
    return HAL_ERROR;
  }
  printf("[CAN INIT] Filter: RANGE 0x000–0x7FF → FIFO0 OK\n");

  // Global filter: non-matching frames go to FIFO0 as safety net
  if (HAL_FDCAN_ConfigGlobalFilter(
        &hfdcan1,
        FDCAN_ACCEPT_IN_RX_FIFO0,
        FDCAN_ACCEPT_IN_RX_FIFO0,
        FDCAN_FILTER_REMOTE,
        FDCAN_FILTER_REMOTE) != HAL_OK)
  {
    printf("[CAN INIT] ERROR: HAL_FDCAN_ConfigGlobalFilter failed\n");
    return HAL_ERROR;
  }
  printf("[CAN INIT] Global filter OK\n");

  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  {
    printf("[CAN INIT] ERROR: HAL_FDCAN_Start failed\n");
    return HAL_ERROR;
  }
  printf("[CAN INIT] FDCAN1 started OK\n");

  // Enable TX Event FIFO notification — fires when a frame is acknowledged 
  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_TX_EVT_FIFO_NEW_DATA, 0U);
  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0U);

  // Reset state
  s_tx_attempted    = 0U;
  s_ack_seen        = 0U;
  s_last_ack_tick   = 0U;
  s_prev_tx_err     = 0U;
  s_raw_status      = CAN_STATUS_PENDING;
  s_filtered_status = CAN_STATUS_PENDING;
  s_stable_counter  = 0U;
  s_busoff_count    = 0U;

  s_diag.status         = CAN_STATUS_PENDING;
  s_diag.tx_error_cnt   = 0U;
  s_diag.rx_error_cnt   = 0U;
  s_diag.lec            = 0U;
  s_diag.ack_seen       = 0U;
  s_diag.last_ack_tick  = 0U;
  s_diag.last_poll_tick = HAL_GetTick();

  printf("[CAN INIT] Init complete — status=PENDING\n");
  return HAL_OK;
}

// Poll the FDCAN peripheral for errors, ACKs, and RX frames
void CAN_Exchange_Poll(void)
{
  if (hfdcan1.State == HAL_FDCAN_STATE_RESET)
  {
    printf("[CAN POLL] FDCAN in RESET state\n");
    s_diag.status = CAN_STATUS_DISABLED;
    return;
  }

  FDCAN_ErrorCountersTypeDef  err   = {0};
  FDCAN_ProtocolStatusTypeDef proto = {0};

  HAL_FDCAN_GetErrorCounters(&hfdcan1, &err);
  HAL_FDCAN_GetProtocolStatus(&hfdcan1, &proto);

  uint8_t lec      = (uint8_t)proto.LastErrorCode;
  uint8_t tec_grew = ((uint8_t)err.TxErrorCnt > s_prev_tx_err) ? 1U : 0U;

  printf("[CAN POLL] TEC=%lu REC=%lu LEC=%u(%s) BusOff=%lu ErrPassive=%lu ErrWarn=%lu\n",
         err.TxErrorCnt,
         err.RxErrorCnt,
         lec,
         CAN_LECName(lec),
         proto.BusOff,
         proto.ErrorPassive,
         proto.Warning);

  // Print extended diagnostics on any error condition
  if (lec != LEC_NO_ERROR && lec != LEC_NO_CHANGE)
  {
    CAN_PrintExtendedErrorDiag(lec,
                               err.TxErrorCnt,
                               err.RxErrorCnt,
                               proto.BusOff,
                               proto.ErrorPassive,
                               proto.Warning);
  }
  else if (proto.BusOff || proto.ErrorPassive || proto.Warning)
  {
    CAN_PrintExtendedErrorDiag(lec,
                               err.TxErrorCnt,
                               err.RxErrorCnt,
                               proto.BusOff,
                               proto.ErrorPassive,
                               proto.Warning);
  }

  s_diag.tx_error_cnt   = (uint8_t)err.TxErrorCnt;
  s_diag.rx_error_cnt   = (uint8_t)err.RxErrorCnt;
  s_diag.lec            = lec;
  s_diag.last_poll_tick = HAL_GetTick();

  /* -----------------------------------------------------------------------
   * Drain TX Event FIFO — each entry = one successfully acknowledged frame
   * This is the correct way to detect ACK on FDCAN without needing an RX
   * --------------------------------------------------------------------- */
  FDCAN_TxEventFifoTypeDef txEvent = {0};
  while (HAL_FDCAN_GetTxEvent(&hfdcan1, &txEvent) == HAL_OK)
  {
    printf("[CAN TX ACK] Frame acknowledged by bus: ID=0x%03X marker=0x%02X\n",
           (unsigned)txEvent.Identifier,
           (unsigned)txEvent.MessageMarker);
    s_ack_seen      = 1U;
    s_last_ack_tick = HAL_GetTick();
    s_busoff_count  = 0U;
  }

  /* -----------------------------------------------------------------------
   * Drain RX FIFO0 — frames sent by other nodes on the bus
   * --------------------------------------------------------------------- */
  uint32_t fifo_level = HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0);
  if (fifo_level > 0U)
  {
    printf("[CAN POLL] RX FIFO0 fill level = %lu\n", fifo_level);
  }

  // Drain RX FIFO0 — each entry = one frame received from another node
  while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0U)
  {
    FDCAN_RxHeaderTypeDef rxHdr = {0};
    uint8_t rxData[8]           = {0};

    HAL_StatusTypeDef rx_res = HAL_FDCAN_GetRxMessage(
                                 &hfdcan1, FDCAN_RX_FIFO0, &rxHdr, rxData);

    if (rx_res == HAL_OK)
    {
      printf("[CAN RX] ID=0x%03X DLC=%lu Data:",
             (unsigned)rxHdr.Identifier,
             rxHdr.DataLength);
      for (uint8_t i = 0U; i < 8U && i < (uint8_t)rxHdr.DataLength; i++)
      {
        printf(" %02X", rxData[i]);
      }
      printf("\n");

      // RX also confirms bus is alive
      s_ack_seen      = 1U;
      s_last_ack_tick = HAL_GetTick();
      s_busoff_count  = 0U;
      printf("[CAN POLL] ACK seen via RX — bus is alive\n");
    }
    else
    {
      printf("[CAN RX] GetRxMessage FAILED (result=%d, HAL err=0x%08lX)\n",
             rx_res, HAL_FDCAN_GetError(&hfdcan1));
      break;
    }
  }

  /* -----------------------------------------------------------------------
   * Clear ack_seen if errors confirm nobody is listening
   * --------------------------------------------------------------------- */
  s_prev_tx_err = (uint8_t)err.TxErrorCnt;

  if (!s_ack_seen)
  {
    if (lec == LEC_ACK_ERROR || tec_grew)
    {
      printf("[CAN POLL] Clearing ack_seen (lec=%s, tec_grew=%u)\n",
             CAN_LECName(lec), tec_grew);
      s_ack_seen = 0U;
    }
  }

  /* -----------------------------------------------------------------------
   * Bus-Off handling
   * --------------------------------------------------------------------- */
  if (proto.BusOff)
  {
    printf("[CAN POLL] BUS-OFF detected — triggering recovery\n");
    s_diag.status     = CAN_STATUS_BUS_OFF;
    s_filtered_status = CAN_STATUS_BUS_OFF;
    s_raw_status      = CAN_STATUS_BUS_OFF;
    CAN_Recover_BusOff();
    s_diag.ack_seen      = s_ack_seen;
    s_diag.last_ack_tick = s_last_ack_tick;
    return;
  }

  /* -----------------------------------------------------------------------
   * Status decision
   * --------------------------------------------------------------------- */
  CAN_Status_t raw;

  if (!s_tx_attempted)
  {
    raw = CAN_STATUS_PENDING;
  }
  else if (!s_ack_seen)
  {
    uint8_t lec_is_error = ((lec != LEC_NO_ERROR) && (lec != LEC_NO_CHANGE));
    raw = lec_is_error ? CAN_LEC_To_Status(lec) : CAN_STATUS_ACK_ERR;
  }
  else
  {
#if CAN_ACK_TIMEOUT_MS > 0U
    uint32_t now = HAL_GetTick();
    if ((now - s_last_ack_tick) > CAN_ACK_TIMEOUT_MS)
    {
      printf("[CAN POLL] ACK timeout (%lu ms since last ACK)\n",
             now - s_last_ack_tick);
      s_ack_seen = 0U;
      raw        = CAN_STATUS_ACK_ERR;
    }
    else
    {
      raw = CAN_STATUS_OK;
    }
#else
    raw = CAN_STATUS_OK;
#endif
  }

  // Log status transitions only 
  if (raw != s_raw_status)
  {
    printf("[CAN POLL] Status transition: %s → %s\n",
           CAN_StatusName(s_raw_status), CAN_StatusName(raw));
  }

  CAN_ApplyDebounce(raw);

  s_diag.status        = s_filtered_status;
  s_diag.ack_seen      = s_ack_seen;
  s_diag.last_ack_tick = s_last_ack_tick;

  printf("[CAN POLL] raw=%s filtered=%s debounce=%u\n",
         CAN_StatusName(s_raw_status),
         CAN_StatusName(s_filtered_status),
         s_stable_counter);
}

// Transmit data via CAN
HAL_StatusTypeDef CAN_Exchange_Transmit_Data(float voltage, float percentage)
{
  uint8_t payload[3] = {0};

  float voltage_mv = voltage * 1000.0f;
  if (voltage_mv > 65535.0f) { voltage_mv = 65535.0f; }
  if (voltage_mv < 0.0f)     { voltage_mv = 0.0f; }
  uint16_t mv = (uint16_t)voltage_mv;

  payload[0] = (uint8_t)(mv & 0xFFU);
  payload[1] = (uint8_t)((mv >> 8U) & 0xFFU);

  uint8_t pct = (uint8_t)(percentage + 0.5f);
  if (pct > 100U) { pct = 100U; }
  payload[2] = pct;

  return CAN_Queue_Frame(0x123U, payload, 3U);
}

// Send a startup probe frame to detect if any node is present on the bus
HAL_StatusTypeDef CAN_Exchange_Trigger_Startup_Probe(void)
{
  printf("[CAN TX] Sending startup probe (ID=0x7FF)\n");
  uint8_t payload[1] = {0x00U};
  return CAN_Queue_Frame(0x7FFU, payload, 1U);
}

// Get the current diagnostic status of the CAN exchange
CAN_Diag_t CAN_Exchange_Get_Status(void)
{
  return s_diag;
}