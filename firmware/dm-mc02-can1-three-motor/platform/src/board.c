#include "board.h"

#include "main.h"

static FDCAN_HandleTypeDef can1;

void board_clock_init(void)
{
    RCC_OscInitTypeDef oscillator = {0};
    RCC_ClkInitTypeDef clocks = {0};

    HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
    while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
    }

    oscillator.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_HSI48;
    oscillator.HSEState = RCC_HSE_ON;
    oscillator.HSI48State = RCC_HSI48_ON;
    oscillator.PLL.PLLState = RCC_PLL_ON;
    oscillator.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    oscillator.PLL.PLLM = 2;
    oscillator.PLL.PLLN = 40;
    oscillator.PLL.PLLP = 1;
    oscillator.PLL.PLLQ = 4;
    oscillator.PLL.PLLR = 2;
    oscillator.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
    oscillator.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
    oscillator.PLL.PLLFRACN = 0;
    if (HAL_RCC_OscConfig(&oscillator) != HAL_OK) {
        Error_Handler();
    }

    clocks.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                       RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 |
                       RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
    clocks.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clocks.SYSCLKDivider = RCC_SYSCLK_DIV1;
    clocks.AHBCLKDivider = RCC_HCLK_DIV2;
    clocks.APB3CLKDivider = RCC_APB3_DIV2;
    clocks.APB1CLKDivider = RCC_APB1_DIV2;
    clocks.APB2CLKDivider = RCC_APB2_DIV2;
    clocks.APB4CLKDivider = RCC_APB4_DIV2;
    if (HAL_RCC_ClockConfig(&clocks, FLASH_LATENCY_4) != HAL_OK) {
        Error_Handler();
    }
}

bool board_can1_init(void)
{
    GPIO_InitTypeDef gpio = {0};
    RCC_PeriphCLKInitTypeDef peripheral_clock = {0};
    FDCAN_FilterTypeDef filter = {0};

    peripheral_clock.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    peripheral_clock.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL;
    if (HAL_RCCEx_PeriphCLKConfig(&peripheral_clock) != HAL_OK) {
        return false;
    }

    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_FDCAN_CLK_ENABLE();

    gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOD, &gpio);

    can1.Instance = FDCAN1;
    can1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
    can1.Init.Mode = FDCAN_MODE_NORMAL;
    can1.Init.AutoRetransmission = ENABLE;
    can1.Init.TransmitPause = DISABLE;
    can1.Init.ProtocolException = ENABLE;
    can1.Init.NominalPrescaler = 3;
    can1.Init.NominalSyncJumpWidth = 8;
    can1.Init.NominalTimeSeg1 = 31;
    can1.Init.NominalTimeSeg2 = 8;
    can1.Init.DataPrescaler = 3;
    can1.Init.DataSyncJumpWidth = 8;
    can1.Init.DataTimeSeg1 = 31;
    can1.Init.DataTimeSeg2 = 8;
    can1.Init.MessageRAMOffset = 0;
    can1.Init.StdFiltersNbr = 1;
    can1.Init.ExtFiltersNbr = 0;
    can1.Init.RxFifo0ElmtsNbr = 8;
    can1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
    can1.Init.RxFifo1ElmtsNbr = 0;
    can1.Init.RxBuffersNbr = 0;
    can1.Init.TxEventsNbr = 0;
    can1.Init.TxBuffersNbr = 0;
    can1.Init.TxFifoQueueElmtsNbr = 8;
    can1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
    can1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
    if (HAL_FDCAN_Init(&can1) != HAL_OK) {
        return false;
    }

    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = 0;
    filter.FilterID2 = 0;
    if (HAL_FDCAN_ConfigFilter(&can1, &filter) != HAL_OK) {
        return false;
    }
    if (HAL_FDCAN_ConfigGlobalFilter(&can1, FDCAN_ACCEPT_IN_RX_FIFO0,
                                     FDCAN_ACCEPT_IN_RX_FIFO0,
                                     FDCAN_REJECT_REMOTE,
                                     FDCAN_REJECT_REMOTE) != HAL_OK) {
        return false;
    }

    return HAL_FDCAN_Start(&can1) == HAL_OK;
}

bool board_can1_get_status(Can1Status *status)
{
    FDCAN_ErrorCountersTypeDef counters;
    FDCAN_ProtocolStatusTypeDef protocol;

    if (HAL_FDCAN_GetErrorCounters(&can1, &counters) != HAL_OK ||
        HAL_FDCAN_GetProtocolStatus(&can1, &protocol) != HAL_OK) {
        return false;
    }

    status->tx_error_count = counters.TxErrorCnt;
    status->rx_error_count = counters.RxErrorCnt;
    status->last_error_code = protocol.LastErrorCode;
    status->error_passive = protocol.ErrorPassive != 0u;
    status->warning = protocol.Warning != 0u;
    status->bus_off = protocol.BusOff != 0u;
    return true;
}

bool board_can1_recover(void)
{
    (void)HAL_FDCAN_AbortTxRequest(&can1, 0xffffffffu);
    if (HAL_FDCAN_Stop(&can1) != HAL_OK) {
        return false;
    }
    return HAL_FDCAN_Start(&can1) == HAL_OK;
}

bool board_can1_transmit(uint32_t standard_id, const uint8_t data[8], uint8_t dlc)
{
    FDCAN_TxHeaderTypeDef header = {0};

    if (data == NULL || standard_id > 0x7ffu ||
        HAL_FDCAN_GetTxFifoFreeLevel(&can1) == 0u) {
        return false;
    }

    header.Identifier = standard_id;
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    if (dlc == 4u) {
        header.DataLength = FDCAN_DLC_BYTES_4;
    } else if (dlc == 8u) {
        header.DataLength = FDCAN_DLC_BYTES_8;
    } else {
        return false;
    }
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    header.MessageMarker = 0u;
    return HAL_FDCAN_AddMessageToTxFifoQ(&can1, &header, data) == HAL_OK;
}

bool board_can1_receive(uint32_t *standard_id, uint8_t data[8], uint8_t *dlc)
{
    FDCAN_RxHeaderTypeDef header = {0};

    if (standard_id == NULL || data == NULL || dlc == NULL ||
        HAL_FDCAN_GetRxFifoFillLevel(&can1, FDCAN_RX_FIFO0) == 0u) {
        return false;
    }
    if (HAL_FDCAN_GetRxMessage(&can1, FDCAN_RX_FIFO0, &header, data) != HAL_OK ||
        header.IdType != FDCAN_STANDARD_ID ||
        header.RxFrameType != FDCAN_DATA_FRAME || header.DataLength > 8u) {
        return false;
    }

    *standard_id = header.Identifier;
    *dlc = (uint8_t)header.DataLength;
    return true;
}
