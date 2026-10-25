/*
 * SmartMush STM32F103C8T6 node main.c - READ ONLY (no sensor address write)
 *
 * USART1 PA9/PA10 + PA8 DE/RE:
 *   RS485 communication with ESP32-S3 master.
 *
 * USART2 PA2/PA3 + PA4 DE/RE:
 *   RS485 communication with Hshop SHT40-58-RS485 sensors.
 *
 * DIP bits:
 *   PB9, PB8, PB15, PB14, PB13, PB12, PB11, PB10
 *   ON = logic 1 in node_id.
 *
 * Master protocol:
 *   FC03, start register 0, quantity 9.
 *   Each sensor occupies 3 registers: temperature x10,
 *   humidity x10, sensor status. Sensor IDs are 1, 2, 3.
 *
 * Sensor protocol assumptions:
 *   IDs 1, 2, 3, FC03, start 0x0000, 2 registers, x10.
 *   Verify these settings against the sensor manual.
 *
 * CHANGES vs previous version:
 *   [FIX 1] USART1 RX uses a SLIDING WINDOW + CRC check, so a node
 *           ignores responses of other nodes on the shared RS485 bus
 *           and never loses frame alignment.
 *   [FIX 2] Sensors are polled ONE per step (round-robin) and the
 *           sensor timeout is shorter, so the node never blocks longer
 *           than ~120 ms and always answers the master within its timeout.
 */

#include "main.h"

#include <stdint.h>
#include <string.h>

/* UART handles */
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* Function prototypes */
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);

/* RS485 DE/RE pins */
#define NODE_DE_PIN       GPIO_PIN_8
#define SENSOR_DE_PIN     GPIO_PIN_4

/* Sensor Modbus settings */
#define SENSOR_COUNT      3U
#define SENSOR_START      0x0000U
#define SENSOR_STEP_MS    1500U   /* read one sensor every 1.5 s -> each sensor every 4.5 s */
#define SENSOR_TIMEOUT_MS 100U    /* 9 bytes @9600 only take ~10 ms */

/* USART1 master request frame size */
#define MASTER_FRAME_SIZE 8U

/* Node and sensor data */
static uint8_t node_id = 1U;
static uint8_t sensor_ok[SENSOR_COUNT] = {0U, 0U, 0U};
static uint16_t temp_x10[SENSOR_COUNT] = {0U, 0U, 0U};
static uint16_t hum_x10[SENSOR_COUNT] = {0U, 0U, 0U};
static volatile uint8_t dip_raw = 0U;

/*
 * USART1 interrupt receive state.
 * [FIX 1] A sliding window of the last 8 received bytes is checked
 * for a valid CRC after every byte.
 */
static uint8_t uart1_rx_byte = 0U;
static uint8_t uart1_win[MASTER_FRAME_SIZE];
static volatile uint8_t uart1_win_count = 0U;
static uint8_t uart1_rx_frame[MASTER_FRAME_SIZE];
static volatile uint8_t uart1_frame_ready = 0U;
static volatile uint8_t uart1_rx_error = 0U;

/* Modbus RTU CRC16 */
static uint16_t crc16(const uint8_t *p, uint16_t n)
{
    uint16_t c = 0xFFFFU;

    for (uint16_t j = 0; j < n; j++)
    {
        c ^= p[j];

        for (uint8_t i = 0; i < 8; i++)
        {
            if (c & 1U)
            {
                c = (c >> 1) ^ 0xA001U;
            }
            else
            {
                c >>= 1;
            }
        }
    }

    return c;
}

/* Read DIP switches: active LOW means ON */
static uint8_t readDip(void)
{
    const uint16_t pins[8] =
    {
        GPIO_PIN_9,
        GPIO_PIN_8,
        GPIO_PIN_15,
        GPIO_PIN_14,
        GPIO_PIN_13,
        GPIO_PIN_12,
        GPIO_PIN_11,
        GPIO_PIN_10
    };

    uint8_t value = 0U;

    for (uint8_t i = 0; i < 8; i++)
    {
        if (HAL_GPIO_ReadPin(GPIOB, pins[i]) == GPIO_PIN_RESET)
        {
            value |= (uint8_t)(1U << i);
        }
    }

    dip_raw = value;

    value &= (uint8_t)~(1U << 3); /* Ignore PB14 / DIP bit 3 per hardware fix */

    return value;
}

/*
 * Send an RS485 frame.
 * DE/RE HIGH = transmit, LOW = receive.
 */
static HAL_StatusTypeDef sendRs485(
    UART_HandleTypeDef *uart,
    uint16_t de_pin,
    const uint8_t *data,
    uint16_t length)
{
    HAL_StatusTypeDef status;

    HAL_GPIO_WritePin(GPIOA, de_pin, GPIO_PIN_SET);
    HAL_Delay(1);

    status = HAL_UART_Transmit(uart, (uint8_t *)data, length, 200);

    if (status == HAL_OK)
    {
        while (__HAL_UART_GET_FLAG(uart, UART_FLAG_TC) == RESET)
        {
        }
    }

    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOA, de_pin, GPIO_PIN_RESET);

    return status;
}

/*
 * Start one-byte interrupt reception on USART1.
 */
static void startUart1ReceiveIT(void)
{
    if (HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1U) != HAL_OK)
    {
        Error_Handler();
    }
}

/*
 * HAL callback: called when one USART1 byte is received.
 * USART1_IRQHandler must call HAL_UART_IRQHandler(&huart1).
 *
 * [FIX 1] Sliding window: shift in each new byte, and accept a request
 * only when the last 8 bytes form a frame with a valid CRC. Responses
 * from other nodes (23 bytes) are skipped automatically.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        /* Drop the oldest byte, append the newest */
        for (uint8_t i = 0; i < (MASTER_FRAME_SIZE - 1U); i++)
        {
            uart1_win[i] = uart1_win[i + 1U];
        }
        uart1_win[MASTER_FRAME_SIZE - 1U] = uart1_rx_byte;

        if (uart1_win_count < MASTER_FRAME_SIZE)
        {
            uart1_win_count++;
        }

        /* Do not overwrite a frame still waiting for processing */
        if ((uart1_win_count >= MASTER_FRAME_SIZE) && (uart1_frame_ready == 0U))
        {
            uint16_t c = crc16(uart1_win, 6U);

            if ((uart1_win[6] == (uint8_t)c) &&
                (uart1_win[7] == (uint8_t)(c >> 8)))
            {
                memcpy(uart1_rx_frame, uart1_win, MASTER_FRAME_SIZE);
                uart1_frame_ready = 1U;
                uart1_win_count = 0U;
            }
        }

        /* Rearm reception for the next byte */
        if (HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1U) != HAL_OK)
        {
            uart1_rx_error = 1U;
        }
    }
}

/*
 * Recover USART1 reception after a UART error.
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        uart1_win_count = 0U;
        uart1_rx_error = 1U;
    }
}

/*
 * Read Hshop SHT40 through its RS485/Modbus interface.
 * Blocking on USART2 for up to SENSOR_TIMEOUT_MS.
 * USART1 bytes keep being received by interrupt meanwhile.
 */
static uint8_t readSensor(uint8_t sensor_id, uint16_t *temperature, uint16_t *humidity)
{
    uint8_t q[8] = {sensor_id, 0x03, (uint8_t)(SENSOR_START >> 8), (uint8_t)SENSOR_START, 0, 2, 0, 0};
    uint16_t c = crc16(q, 6);
    q[6] = (uint8_t)c;
    q[7] = (uint8_t)(c >> 8);

    uint8_t r[9];

    __HAL_UART_FLUSH_DRREGISTER(&huart2);

    if (sendRs485(&huart2, SENSOR_DE_PIN, q, 8) != HAL_OK) return 0;

    if (HAL_UART_Receive(&huart2, r, 9, SENSOR_TIMEOUT_MS) != HAL_OK) return 0;

    if (r[0] != sensor_id || r[1] != 0x03 || r[2] != 4) return 0;

    c = crc16(r, 7);

    if (r[7] != (uint8_t)c || r[8] != (uint8_t)(c >> 8)) return 0;

    /* Hshop SHT40: register 0 = humidity, register 1 = temperature. */
    *humidity = ((uint16_t)r[3] << 8) | r[4];

    /* Preserve negative temperatures in signed 16-bit two's complement. */
    *temperature = ((uint16_t)r[5] << 8) | r[6];

    return 1;
}

/*
 * [FIX 2] Read ONE sensor per call, rotating 1 -> 2 -> 3.
 * Worst-case blocking time is ~120 ms instead of ~1.1 s.
 */
static uint8_t next_sensor = 0U;

static void pollOneSensor(void)
{
    uint16_t t = 0U;
    uint16_t h = 0U;
    uint8_t idx = next_sensor;

    sensor_ok[idx] = readSensor((uint8_t)(idx + 1U), &t, &h);

    if (sensor_ok[idx])
    {
        temp_x10[idx] = t;
        hum_x10[idx] = h;
    }

    next_sensor = (uint8_t)((next_sensor + 1U) % SENSOR_COUNT);
}

static void handleRequest(const uint8_t *request)
{
    uint8_t q[8];
    uint8_t r[23];

    memcpy(q, request, 8);

    /* Kiểm tra CRC của request */
    uint16_t c = crc16(q, 6);

    if (q[6] != (uint8_t)c ||
        q[7] != (uint8_t)(c >> 8))
    {
        return;
    }

    /* Chỉ xử lý request gửi đến đúng Node ID */
    if (q[0] != node_id)
    {
        return;
    }

    /* Chỉ hỗ trợ Modbus FC03 */
    if (q[1] != 0x03U)
    {
        return;
    }

    uint16_t start = ((uint16_t)q[2] << 8) | q[3];
    uint16_t quantity = ((uint16_t)q[4] << 8) | q[5];

    /* Đọc 9 thanh ghi: 3 cảm biến x 3 thanh ghi */
    if (start != 0U || quantity != 9U)
    {
        return;
    }

    r[0] = node_id;
    r[1] = 0x03;
    r[2] = 18;

    for (uint8_t i = 0; i < SENSOR_COUNT; i++)
    {
        uint16_t base = 3U + (uint16_t)i * 6U;

        uint16_t temp = (uint16_t)temp_x10[i];
        uint16_t hum  = (uint16_t)hum_x10[i];
        uint16_t ok   = sensor_ok[i] ? 1U : 0U;

        r[base + 0] = (uint8_t)(temp >> 8);
        r[base + 1] = (uint8_t)temp;
        r[base + 2] = (uint8_t)(hum >> 8);
        r[base + 3] = (uint8_t)hum;
        r[base + 4] = (uint8_t)(ok >> 8);
        r[base + 5] = (uint8_t)ok;
    }

    c = crc16(r, 21);

    r[21] = (uint8_t)c;
    r[22] = (uint8_t)(c >> 8);

    (void)sendRs485(&huart1, NODE_DE_PIN, r, sizeof(r));
}

/* Main */
int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();

    HAL_GPIO_WritePin(
        GPIOA,
        NODE_DE_PIN | SENSOR_DE_PIN,
        GPIO_PIN_RESET
    );

    node_id = readDip();

    /* Start USART1 reception before polling the sensors. */
    startUart1ReceiveIT();

    /* Initial read of all sensors (short timeout, ~120 ms each at worst). */
    for (uint8_t i = 0; i < SENSOR_COUNT; i++)
    {
        pollOneSensor();
        HAL_Delay(20);
    }

    uint32_t last_sensor_step = HAL_GetTick();

    uint8_t request[MASTER_FRAME_SIZE];

    while (1)
    {
        node_id = readDip();

        /*
         * Process a complete request captured by the ISR.
         * Briefly disable interrupts while copying shared data.
         */
        if (uart1_frame_ready != 0U)
        {
            __disable_irq();

            for (uint8_t i = 0; i < MASTER_FRAME_SIZE; i++)
            {
                request[i] = uart1_rx_frame[i];
            }

            uart1_frame_ready = 0U;

            __enable_irq();

            handleRequest(request);
        }

        /* Recover reception if HAL reported an error. */
        if (uart1_rx_error != 0U)
        {
            __disable_irq();
            uart1_rx_error = 0U;
            uart1_win_count = 0U;
            __enable_irq();

            if (HAL_UART_GetState(&huart1) == HAL_UART_STATE_READY)
            {
                (void)HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1U);
            }
        }

        /* [FIX 2] Read one sensor every SENSOR_STEP_MS. */
        if ((HAL_GetTick() - last_sensor_step) >= SENSOR_STEP_MS)
        {
            last_sensor_step = HAL_GetTick();
            pollOneSensor();
        }
    }
}

/* Clock configuration */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef o = {0};
    RCC_ClkInitTypeDef c = {0};

    o.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    o.HSIState = RCC_HSI_ON;
    o.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    o.PLL.PLLState = RCC_PLL_NONE;

    if (HAL_RCC_OscConfig(&o) != HAL_OK)
    {
        Error_Handler();
    }

    c.ClockType = RCC_CLOCKTYPE_HCLK |
                  RCC_CLOCKTYPE_SYSCLK |
                  RCC_CLOCKTYPE_PCLK1 |
                  RCC_CLOCKTYPE_PCLK2;

    c.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    c.AHBCLKDivider = RCC_SYSCLK_DIV1;
    c.APB1CLKDivider = RCC_HCLK_DIV1;
    c.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&c, FLASH_LATENCY_0) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USART1: ESP32 master RS485 */
static void MX_USART1_UART_Init(void)
{
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 9600;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USART2: sensor RS485 */
static void MX_USART2_UART_Init(void)
{
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 9600;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        Error_Handler();
    }
}

/* GPIO configuration */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    HAL_GPIO_WritePin(
        GPIOA,
        GPIO_PIN_4 | GPIO_PIN_8,
        GPIO_PIN_RESET
    );

    /* RS485 DE/RE pins */
    g.Pin = GPIO_PIN_4 | GPIO_PIN_8;
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &g);

    /* DIP inputs with pull-ups */
    g.Pin = GPIO_PIN_8 |
            GPIO_PIN_9 |
            GPIO_PIN_10 |
            GPIO_PIN_11 |
            GPIO_PIN_12 |
            GPIO_PIN_13 |
            GPIO_PIN_14 |
            GPIO_PIN_15;

    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &g);
}

/* Error handler */
void Error_Handler(void)
{
    __disable_irq();

    while (1)
    {
    }
}
