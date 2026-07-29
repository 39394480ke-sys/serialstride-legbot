typedef unsigned int uint32_t;
typedef signed int int32_t;
typedef unsigned char uint8_t;
typedef signed char int8_t;

_Static_assert(sizeof(uint32_t) == 4u, "uint32_t must be 32 bits");
_Static_assert(sizeof(int32_t) == 4u, "int32_t must be 32 bits");
_Static_assert(sizeof(uint8_t) == 1u, "uint8_t must be 8 bits");
_Static_assert(sizeof(int8_t) == 1u, "int8_t must be 8 bits");

#define REG32(address) (*(volatile uint32_t *)(address))

#define RCC_CR REG32(0x40021000u)
#define RCC_CFGR REG32(0x40021004u)
#define RCC_APB2ENR REG32(0x40021018u)
#define RCC_APB1ENR REG32(0x4002101Cu)
#define FLASH_ACR REG32(0x40022000u)

#define GPIOA_CRH REG32(0x40010804u)
#define GPIOC_CRH REG32(0x40011004u)
#define GPIOC_BSRR REG32(0x40011010u)
#define GPIOC_BRR REG32(0x40011014u)

#define USART1_SR REG32(0x40013800u)
#define USART1_DR REG32(0x40013804u)
#define USART1_BRR REG32(0x40013808u)
#define USART1_CR1 REG32(0x4001380Cu)

#define CAN1_BASE 0x40006400u
#define CAN_MCR REG32(CAN1_BASE + 0x00u)
#define CAN_MSR REG32(CAN1_BASE + 0x04u)
#define CAN_TSR REG32(CAN1_BASE + 0x08u)
#define CAN_RF0R REG32(CAN1_BASE + 0x0Cu)
#define CAN_ESR REG32(CAN1_BASE + 0x18u)
#define CAN_BTR REG32(CAN1_BASE + 0x1Cu)
#define CAN_TI0R REG32(CAN1_BASE + 0x180u)
#define CAN_TDT0R REG32(CAN1_BASE + 0x184u)
#define CAN_TDL0R REG32(CAN1_BASE + 0x188u)
#define CAN_TDH0R REG32(CAN1_BASE + 0x18Cu)
#define CAN_RI0R REG32(CAN1_BASE + 0x1B0u)
#define CAN_RDT0R REG32(CAN1_BASE + 0x1B4u)
#define CAN_RDL0R REG32(CAN1_BASE + 0x1B8u)
#define CAN_RDH0R REG32(CAN1_BASE + 0x1BCu)

#define CAN_FILTER_BASE 0x40006600u
#define CAN_FMR REG32(CAN_FILTER_BASE + 0x00u)
#define CAN_FM1R REG32(CAN_FILTER_BASE + 0x04u)
#define CAN_FS1R REG32(CAN_FILTER_BASE + 0x0Cu)
#define CAN_FFA1R REG32(CAN_FILTER_BASE + 0x14u)
#define CAN_FA1R REG32(CAN_FILTER_BASE + 0x1Cu)
#define CAN_F0R1 REG32(CAN_FILTER_BASE + 0x40u)
#define CAN_F0R2 REG32(CAN_FILTER_BASE + 0x44u)

#define SYSTICK_CTRL REG32(0xE000E010u)
#define SYSTICK_LOAD REG32(0xE000E014u)
#define SYSTICK_VAL REG32(0xE000E018u)

#define RCC_CR_HSEON (1u << 16)
#define RCC_CR_HSERDY (1u << 17)
#define RCC_CR_PLLON (1u << 24)
#define RCC_CR_PLLRDY (1u << 25)
#define RCC_CFGR_SW_MASK (3u << 0)
#define RCC_CFGR_SWS_MASK (3u << 2)
#define RCC_CFGR_HPRE_MASK (15u << 4)
#define RCC_CFGR_PPRE1_MASK (7u << 8)
#define RCC_CFGR_PPRE2_MASK (7u << 11)
#define RCC_CFGR_PLLSRC (1u << 16)
#define RCC_CFGR_PLLXTPRE (1u << 17)
#define RCC_CFGR_PLLMUL_MASK (15u << 18)
#define RCC_CFGR_PPRE1_DIV2 (4u << 8)
#define RCC_CFGR_PLLMUL9 (7u << 18)
#define RCC_CFGR_SW_PLL (2u << 0)
#define RCC_CFGR_SWS_PLL (2u << 2)

#define FLASH_ACR_LATENCY_2 (2u << 0)
#define FLASH_ACR_PRFTBE (1u << 4)

#define RCC_APB2ENR_AFIOEN (1u << 0)
#define RCC_APB2ENR_IOPAEN (1u << 2)
#define RCC_APB2ENR_IOPCEN (1u << 4)
#define RCC_APB2ENR_USART1EN (1u << 14)
#define RCC_APB1ENR_CAN1EN (1u << 25)

#define USART_SR_RXNE (1u << 5)
#define USART_SR_TXE (1u << 7)
#define USART_CR1_RE (1u << 2)
#define USART_CR1_TE (1u << 3)
#define USART_CR1_UE (1u << 13)

#define CAN_MCR_INRQ (1u << 0)
#define CAN_MCR_NART (1u << 4)
#define CAN_MCR_ABOM (1u << 6)
#define CAN_MSR_INAK (1u << 0)
#define CAN_TSR_RQCP0 (1u << 0)
#define CAN_TSR_TXOK0 (1u << 1)
#define CAN_TSR_TME0 (1u << 26)
#define CAN_RF0R_FMP0_MASK (3u << 0)
#define CAN_RF0R_RFOM0 (1u << 5)
#define CAN_TI0R_TXRQ (1u << 0)
#define CAN_TI0R_IDE (1u << 2)
#define CAN_BTR_LBKM (1u << 30)
#define CAN_BTR_SILM (1u << 31)
#define CAN_FMR_FINIT (1u << 0)

/*
 * APB1 is 36 MHz. 1 Mbps uses 18 time quanta:
 * prescaler=2, BS1=15 tq, BS2=2 tq, SJW=1 tq, sample point=88.9%.
 * bxCAN register fields store each value minus one.
 */
#define CAN_BTR_1MBPS ((1u << 20) | (14u << 16) | 1u)

#define LED_PIN 13u
#define H6215_CAN_ID 1u
#define H6215_MASTER_ID 0u
#define H6215_RID_CONTROL_MODE 10u
#define H6215_RID_SOFTWARE_VERSION 14u
#define H6215_RID_P_MAX 21u
#define H6215_RID_V_MAX 22u
#define H6215_RID_T_MAX 23u

typedef struct {
    uint32_t id;
    uint8_t dlc;
    uint8_t data[8];
} can_frame_t;

typedef enum {
    MOTION_IDLE = 0,
    MOTION_ENABLE_WAIT,
    MOTION_RUNNING,
    MOTION_STOPPING,
    MOTION_CONTINUOUS,
    MOTION_WATCHDOG_STOPPING
} motion_state_t;

static volatile uint32_t system_millis;
static uint8_t h6215_control_mode;
static uint8_t h6215_control_mode_valid;
static uint8_t motion_armed;
static uint32_t motion_armed_at;
static motion_state_t motion_state;
static uint8_t motion_direction;
static uint32_t motion_phase_started_at;
static uint32_t motion_last_command_at;
static uint32_t motion_last_ramp_at;
static uint32_t motion_last_user_command_at;
static int8_t current_speed_step;
static int8_t target_speed_step;
static can_frame_t last_feedback_frame;
static uint8_t last_feedback_valid;
static uint32_t last_feedback_print_at;

void SysTick_Handler(void)
{
    ++system_millis;
}

static void led_init(void)
{
    RCC_APB2ENR |= RCC_APB2ENR_IOPCEN;
    GPIOC_CRH = (GPIOC_CRH & ~(0xFu << 20)) | (0x2u << 20);
    GPIOC_BSRR = 1u << LED_PIN;
}

static void led_toggle(void)
{
    static uint8_t is_on;

    is_on ^= 1u;
    if (is_on != 0u) {
        GPIOC_BRR = 1u << LED_PIN;
    } else {
        GPIOC_BSRR = 1u << LED_PIN;
    }
}

static uint8_t clock_init_72mhz(void)
{
    uint32_t timeout = 2000000u;
    uint32_t cfgr;

    RCC_CR |= RCC_CR_HSEON;
    while (((RCC_CR & RCC_CR_HSERDY) == 0u) && (timeout != 0u)) {
        --timeout;
    }
    if (timeout == 0u) {
        return 0u;
    }

    FLASH_ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY_2;

    cfgr = RCC_CFGR;
    cfgr &= ~(RCC_CFGR_SW_MASK
              | RCC_CFGR_HPRE_MASK
              | RCC_CFGR_PPRE1_MASK
              | RCC_CFGR_PPRE2_MASK
              | RCC_CFGR_PLLSRC
              | RCC_CFGR_PLLXTPRE
              | RCC_CFGR_PLLMUL_MASK);
    cfgr |= RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_PLLSRC | RCC_CFGR_PLLMUL9;
    RCC_CFGR = cfgr;

    RCC_CR |= RCC_CR_PLLON;
    timeout = 2000000u;
    while (((RCC_CR & RCC_CR_PLLRDY) == 0u) && (timeout != 0u)) {
        --timeout;
    }
    if (timeout == 0u) {
        return 0u;
    }

    RCC_CFGR = (RCC_CFGR & ~RCC_CFGR_SW_MASK) | RCC_CFGR_SW_PLL;
    timeout = 2000000u;
    while (((RCC_CFGR & RCC_CFGR_SWS_MASK) != RCC_CFGR_SWS_PLL)
           && (timeout != 0u)) {
        --timeout;
    }

    return (timeout != 0u) ? 1u : 0u;
}

static void usart1_init(uint32_t brr)
{
    RCC_APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_USART1EN;

    /* PA9: AF push-pull TX. PA10: floating-input RX. */
    GPIOA_CRH = (GPIOA_CRH & ~((0xFu << 4) | (0xFu << 8)))
                | (0xBu << 4)
                | (0x4u << 8);

    USART1_BRR = brr;
    USART1_CR1 = USART_CR1_RE | USART_CR1_TE | USART_CR1_UE;
}

static void usart1_write(const char *text)
{
    while (*text != '\0') {
        while ((USART1_SR & USART_SR_TXE) == 0u) {
        }
        USART1_DR = (uint8_t)*text;
        ++text;
    }
}

static uint8_t usart1_read_byte(uint8_t *value)
{
    if ((USART1_SR & USART_SR_RXNE) == 0u) {
        return 0u;
    }
    *value = (uint8_t)USART1_DR;
    return 1u;
}

static void usart1_write_hex(uint32_t value, uint8_t digits)
{
    static const char hex_digits[] = "0123456789ABCDEF";
    uint8_t shift = (uint8_t)((digits - 1u) * 4u);

    while (digits != 0u) {
        const uint8_t nibble = (uint8_t)((value >> shift) & 0xFu);
        while ((USART1_SR & USART_SR_TXE) == 0u) {
        }
        USART1_DR = (uint8_t)hex_digits[nibble];
        --digits;
        if (digits != 0u) {
            shift = (uint8_t)(shift - 4u);
        }
    }
}

static void usart1_write_u8(uint8_t value)
{
    char buffer[3];
    uint8_t length = 0u;
    uint8_t i;

    if (value >= 100u) {
        buffer[length++] = (char)('0' + (value / 100u));
        value = (uint8_t)(value % 100u);
        buffer[length++] = (char)('0' + (value / 10u));
    } else if (value >= 10u) {
        buffer[length++] = (char)('0' + (value / 10u));
    }
    buffer[length++] = (char)('0' + (value % 10u));

    for (i = 0u; i < length; ++i) {
        while ((USART1_SR & USART_SR_TXE) == 0u) {
        }
        USART1_DR = (uint8_t)buffer[i];
    }
}

static void usart1_write_u32(uint32_t value)
{
    char buffer[10];
    uint8_t length = 0u;

    do {
        buffer[length++] = (char)('0' + (value % 10u));
        value /= 10u;
    } while (value != 0u);

    while (length != 0u) {
        --length;
        while ((USART1_SR & USART_SR_TXE) == 0u) {
        }
        USART1_DR = (uint8_t)buffer[length];
    }
}

static void usart1_write_milli(int32_t value)
{
    uint32_t magnitude;

    if (value < 0) {
        usart1_write("-");
        magnitude = (uint32_t)(-value);
    } else {
        magnitude = (uint32_t)value;
    }

    usart1_write_u32(magnitude / 1000u);
    usart1_write(".");
    usart1_write_hex((magnitude % 1000u) / 100u, 1u);
    usart1_write_hex((magnitude % 100u) / 10u, 1u);
    usart1_write_hex(magnitude % 10u, 1u);
}

static void systick_init(void)
{
    SYSTICK_LOAD = 72000u - 1u;
    SYSTICK_VAL = 0u;
    SYSTICK_CTRL = 0x7u;
}

static uint8_t wait_register_mask(
    volatile uint32_t *reg,
    uint32_t mask,
    uint32_t expected,
    uint32_t timeout
)
{
    while (((*reg & mask) != expected) && (timeout != 0u)) {
        --timeout;
    }
    return (timeout != 0u) ? 1u : 0u;
}

static uint8_t can_enter_init(void)
{
    CAN_MCR |= CAN_MCR_INRQ;
    return wait_register_mask(
        (volatile uint32_t *)(CAN1_BASE + 0x04u),
        CAN_MSR_INAK,
        CAN_MSR_INAK,
        1000000u
    );
}

static uint8_t can_leave_init(void)
{
    CAN_MCR &= ~CAN_MCR_INRQ;
    return wait_register_mask(
        (volatile uint32_t *)(CAN1_BASE + 0x04u),
        CAN_MSR_INAK,
        0u,
        1000000u
    );
}

static void can_filter_accept_all(void)
{
    CAN_FMR |= CAN_FMR_FINIT;
    CAN_FA1R &= ~1u;
    CAN_FM1R &= ~1u;
    CAN_FS1R |= 1u;
    CAN_FFA1R &= ~1u;
    CAN_F0R1 = 0u;
    CAN_F0R2 = 0u;
    CAN_FA1R |= 1u;
    CAN_FMR &= ~CAN_FMR_FINIT;
}

static uint8_t can_init(uint8_t loopback)
{
    RCC_APB2ENR |= RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPAEN;
    RCC_APB1ENR |= RCC_APB1ENR_CAN1EN;

    /* PA11: floating CAN_RX. PA12: AF push-pull CAN_TX. */
    GPIOA_CRH = (GPIOA_CRH & ~((0xFu << 12) | (0xFu << 16)))
                | (0x4u << 12)
                | (0xBu << 16);

    if (can_enter_init() == 0u) {
        return 0u;
    }

    /* One-shot mode prevents endless retries on a disconnected bus. */
    CAN_MCR = CAN_MCR_INRQ | CAN_MCR_NART | CAN_MCR_ABOM;
    CAN_BTR = CAN_BTR_1MBPS;
    if (loopback != 0u) {
        CAN_BTR |= CAN_BTR_LBKM | CAN_BTR_SILM;
    }

    can_filter_accept_all();
    return can_leave_init();
}

static uint8_t can_set_normal_mode(void)
{
    if (can_enter_init() == 0u) {
        return 0u;
    }
    CAN_BTR = CAN_BTR_1MBPS;
    return can_leave_init();
}

static uint32_t pack_four_bytes(const uint8_t *data)
{
    return ((uint32_t)data[0])
           | ((uint32_t)data[1] << 8)
           | ((uint32_t)data[2] << 16)
           | ((uint32_t)data[3] << 24);
}

static uint8_t can_transmit(
    uint32_t standard_id,
    const uint8_t data[8],
    uint8_t dlc,
    uint32_t timeout_ms
)
{
    uint32_t start;

    if ((standard_id > 0x7FFu) || (dlc > 8u)) {
        return 0u;
    }
    if ((CAN_TSR & CAN_TSR_TME0) == 0u) {
        return 0u;
    }

    CAN_TSR = CAN_TSR_RQCP0;
    CAN_TDT0R = dlc;
    CAN_TDL0R = pack_four_bytes(&data[0]);
    CAN_TDH0R = pack_four_bytes(&data[4]);
    CAN_TI0R = (standard_id << 21) | CAN_TI0R_TXRQ;

    start = system_millis;
    while ((CAN_TSR & CAN_TSR_RQCP0) == 0u) {
        if ((uint32_t)(system_millis - start) >= timeout_ms) {
            return 0u;
        }
    }

    return ((CAN_TSR & CAN_TSR_TXOK0) != 0u) ? 1u : 0u;
}

static uint8_t can_receive(can_frame_t *frame)
{
    uint32_t rir;
    uint32_t low;
    uint32_t high;

    if ((CAN_RF0R & CAN_RF0R_FMP0_MASK) == 0u) {
        return 0u;
    }

    rir = CAN_RI0R;
    frame->dlc = (uint8_t)(CAN_RDT0R & 0xFu);
    low = CAN_RDL0R;
    high = CAN_RDH0R;

    if ((rir & CAN_TI0R_IDE) != 0u) {
        frame->id = (rir >> 3) & 0x1FFFFFFFu;
    } else {
        frame->id = (rir >> 21) & 0x7FFu;
    }

    frame->data[0] = (uint8_t)(low >> 0);
    frame->data[1] = (uint8_t)(low >> 8);
    frame->data[2] = (uint8_t)(low >> 16);
    frame->data[3] = (uint8_t)(low >> 24);
    frame->data[4] = (uint8_t)(high >> 0);
    frame->data[5] = (uint8_t)(high >> 8);
    frame->data[6] = (uint8_t)(high >> 16);
    frame->data[7] = (uint8_t)(high >> 24);

    CAN_RF0R = CAN_RF0R_RFOM0;
    return 1u;
}

static void can_print_status(void)
{
    const uint32_t esr = CAN_ESR;
    const uint8_t tec = (uint8_t)(esr >> 16);
    const uint8_t rec = (uint8_t)(esr >> 24);

    usart1_write("CAN_STATUS ESR=0x");
    usart1_write_hex(esr, 8u);
    usart1_write(" TEC=");
    usart1_write_u8(tec);
    usart1_write(" REC=");
    usart1_write_u8(rec);
    usart1_write(" LEC=");
    usart1_write_u8((uint8_t)((esr >> 4) & 0x7u));
    if ((esr & (1u << 2)) != 0u) {
        usart1_write(" BUS_OFF");
    } else if ((esr & (1u << 1)) != 0u) {
        usart1_write(" ERROR_PASSIVE");
    } else if ((esr & 1u) != 0u) {
        usart1_write(" ERROR_WARNING");
    } else {
        usart1_write(" ERROR_ACTIVE");
    }
    usart1_write("\r\n");
}

static uint8_t can_loopback_test(void)
{
    static const uint8_t test_data[8] = {
        0xDEu, 0xADu, 0xBEu, 0xEFu, 0x12u, 0x34u, 0x56u, 0x78u
    };
    can_frame_t frame;
    uint32_t start;
    uint8_t i;

    if (can_transmit(0x123u, test_data, 8u, 100u) == 0u) {
        return 0u;
    }

    start = system_millis;
    while (can_receive(&frame) == 0u) {
        if ((uint32_t)(system_millis - start) >= 100u) {
            return 0u;
        }
    }

    if ((frame.id != 0x123u) || (frame.dlc != 8u)) {
        return 0u;
    }
    for (i = 0u; i < 8u; ++i) {
        if (frame.data[i] != test_data[i]) {
            return 0u;
        }
    }
    return 1u;
}

static void send_h6215_read_request(uint8_t register_id)
{
    uint8_t request[8] = {
        (uint8_t)(H6215_CAN_ID & 0xFFu),
        (uint8_t)(H6215_CAN_ID >> 8),
        0x33u,
        register_id,
        0x00u,
        0x00u,
        0x00u,
        0x00u
    };

    usart1_write("READ_REQUEST ID=0x7FF RID=0x");
    usart1_write_hex(register_id, 2u);
    usart1_write("\r\n");
    if (can_transmit(0x7FFu, request, 8u, 100u) != 0u) {
        usart1_write("READ_REQUEST_TX_OK\r\n");
    } else {
        usart1_write("READ_REQUEST_TX_ERROR\r\n");
    }
    can_print_status();
}

static uint8_t send_h6215_motor_command(uint8_t command)
{
    uint8_t data[8] = {
        0xFFu, 0xFFu, 0xFFu, 0xFFu,
        0xFFu, 0xFFu, 0xFFu, command
    };

    return can_transmit(H6215_CAN_ID, data, 8u, 100u);
}

static uint8_t send_h6215_velocity_step(int8_t step)
{
    static const uint8_t velocity_bytes[11][4] = {
        {0x00u, 0x00u, 0x00u, 0xBFu}, /* -0.5 */
        {0xCDu, 0xCCu, 0xCCu, 0xBEu}, /* -0.4 */
        {0x9Au, 0x99u, 0x99u, 0xBEu}, /* -0.3 */
        {0xCDu, 0xCCu, 0x4Cu, 0xBEu}, /* -0.2 */
        {0xCDu, 0xCCu, 0xCCu, 0xBDu}, /* -0.1 */
        {0x00u, 0x00u, 0x00u, 0x00u}, /*  0.0 */
        {0xCDu, 0xCCu, 0xCCu, 0x3Du}, /* +0.1 */
        {0xCDu, 0xCCu, 0x4Cu, 0x3Eu}, /* +0.2 */
        {0x9Au, 0x99u, 0x99u, 0x3Eu}, /* +0.3 */
        {0xCDu, 0xCCu, 0xCCu, 0x3Eu}, /* +0.4 */
        {0x00u, 0x00u, 0x00u, 0x3Fu}  /* +0.5 */
    };
    uint8_t data[8] = {
        0x00u, 0x00u, 0x00u, 0x00u,
        0x00u, 0x00u, 0x00u, 0x00u
    };
    uint8_t index;

    if (step < -5) {
        step = -5;
    } else if (step > 5) {
        step = 5;
    }
    index = (uint8_t)(step + 5);
    data[0] = velocity_bytes[index][0];
    data[1] = velocity_bytes[index][1];
    data[2] = velocity_bytes[index][2];
    data[3] = velocity_bytes[index][3];
    return can_transmit(0x200u + H6215_CAN_ID, data, 4u, 100u);
}

static void stop_and_disable_motion(const char *reason)
{
    motion_state = MOTION_IDLE;
    motion_armed = 0u;
    current_speed_step = 0;
    target_speed_step = 0;
    (void)send_h6215_velocity_step(0);
    (void)send_h6215_motor_command(0xFDu);
    usart1_write(reason);
    usart1_write("\r\n");
}

static void start_guarded_motion_test(uint32_t now, uint8_t direction)
{
    if ((h6215_control_mode_valid == 0u) || (h6215_control_mode != 3u)) {
        usart1_write("START_REJECTED: SEND m AND REQUIRE VELOCITY MODE 3\r\n");
        return;
    }
    if ((motion_armed == 0u)
        || ((uint32_t)(now - motion_armed_at) > 10000u)) {
        motion_armed = 0u;
        usart1_write("START_REJECTED: SEND a FIRST\r\n");
        return;
    }
    if (motion_state != MOTION_IDLE) {
        usart1_write("START_REJECTED: TEST ALREADY ACTIVE\r\n");
        return;
    }

    motion_armed = 0u;
    motion_direction = direction;
    if (send_h6215_motor_command(0xFCu) == 0u) {
        stop_and_disable_motion("ENABLE_TX_ERROR: DISABLE ATTEMPTED");
        return;
    }

    motion_state = MOTION_ENABLE_WAIT;
    motion_phase_started_at = now;
    motion_last_command_at = now;
    motion_last_ramp_at = now;
    motion_last_user_command_at = now;
    current_speed_step = 0;
    target_speed_step = 0;
    usart1_write("MOTOR_ENABLE_TX_OK; WAITING 100ms\r\n");
}

static void print_speed_target(void)
{
    usart1_write("TARGET_SPEED=");
    if (target_speed_step < 0) {
        usart1_write("-");
        usart1_write_u8((uint8_t)(-target_speed_step));
    } else {
        usart1_write("+");
        usart1_write_u8((uint8_t)target_speed_step);
    }
    usart1_write("00mrad/s\r\n");
}

static void update_guarded_motion_test(uint32_t now)
{
    if ((motion_armed != 0u)
        && ((uint32_t)(now - motion_armed_at) > 10000u)) {
        motion_armed = 0u;
        usart1_write("ARM_TIMEOUT\r\n");
    }

    if (motion_state == MOTION_ENABLE_WAIT) {
        if ((uint32_t)(now - motion_phase_started_at) >= 100u) {
            if (motion_direction == 0u) {
                if (send_h6215_velocity_step(0) == 0u) {
                    stop_and_disable_motion("ZERO_SPEED_TX_ERROR: MOTOR DISABLED");
                    return;
                }
                motion_state = MOTION_CONTINUOUS;
                motion_last_command_at = now;
                motion_last_ramp_at = now;
                motion_last_user_command_at = now;
                usart1_write("CONTINUOUS_READY TARGET=0; USE + - 0 k; WATCHDOG=5s\r\n");
                return;
            }
            if (send_h6215_velocity_step(
                    (motion_direction == 1u) ? 2 : -2
                ) == 0u) {
                stop_and_disable_motion("SPEED_TX_ERROR: MOTOR DISABLED");
                return;
            }
            motion_state = MOTION_RUNNING;
            motion_phase_started_at = now;
            motion_last_command_at = now;
            if (motion_direction == 1u) {
                usart1_write("TEST_RUNNING SPEED=+0.2rad/s DURATION=1000ms\r\n");
            } else {
                usart1_write("TEST_RUNNING SPEED=-0.2rad/s DURATION=1000ms\r\n");
            }
        }
    } else if (motion_state == MOTION_RUNNING) {
        if ((uint32_t)(now - motion_phase_started_at) >= 1000u) {
            if (send_h6215_velocity_step(0) == 0u) {
                stop_and_disable_motion("ZERO_SPEED_TX_ERROR: DISABLE ATTEMPTED");
                return;
            }
            motion_state = MOTION_STOPPING;
            motion_phase_started_at = now;
            motion_last_command_at = now;
            usart1_write("ZERO_SPEED_HOLD 200ms\r\n");
        } else if ((uint32_t)(now - motion_last_command_at) >= 10u) {
            motion_last_command_at = now;
            if (send_h6215_velocity_step(
                    (motion_direction == 1u) ? 2 : -2
                ) == 0u) {
                stop_and_disable_motion("SPEED_TX_ERROR: MOTOR DISABLED");
            }
        }
    } else if (motion_state == MOTION_STOPPING) {
        if ((uint32_t)(now - motion_phase_started_at) >= 200u) {
            (void)send_h6215_velocity_step(0);
            (void)send_h6215_motor_command(0xFDu);
            motion_state = MOTION_IDLE;
            usart1_write("TEST_COMPLETE MOTOR_DISABLED\r\n");
        } else if ((uint32_t)(now - motion_last_command_at) >= 10u) {
            motion_last_command_at = now;
            if (send_h6215_velocity_step(0) == 0u) {
                stop_and_disable_motion("ZERO_SPEED_TX_ERROR: MOTOR DISABLED");
            }
        }
    } else if ((motion_state == MOTION_CONTINUOUS)
               || (motion_state == MOTION_WATCHDOG_STOPPING)) {
        if ((motion_state == MOTION_CONTINUOUS)
            && ((uint32_t)(now - motion_last_user_command_at) >= 5000u)) {
            target_speed_step = 0;
            motion_state = MOTION_WATCHDOG_STOPPING;
            usart1_write("HOST_WATCHDOG_TIMEOUT: RAMPING_TO_ZERO\r\n");
        }

        if ((uint32_t)(now - motion_last_ramp_at) >= 100u) {
            motion_last_ramp_at = now;
            if (current_speed_step < target_speed_step) {
                ++current_speed_step;
            } else if (current_speed_step > target_speed_step) {
                --current_speed_step;
            }
        }

        if ((uint32_t)(now - motion_last_command_at) >= 10u) {
            motion_last_command_at = now;
            if (send_h6215_velocity_step(current_speed_step) == 0u) {
                stop_and_disable_motion("SPEED_TX_ERROR: MOTOR DISABLED");
                return;
            }
        }

        if ((motion_state == MOTION_WATCHDOG_STOPPING)
            && (current_speed_step == 0)) {
            (void)send_h6215_velocity_step(0);
            (void)send_h6215_motor_command(0xFDu);
            motion_state = MOTION_IDLE;
            usart1_write("WATCHDOG_STOP_COMPLETE MOTOR_DISABLED\r\n");
        }
    }
}

static const char *h6215_state_name(uint8_t state)
{
    if (state == 0u) {
        return "DISABLED";
    }
    if (state == 1u) {
        return "ENABLED";
    }
    if (state == 8u) {
        return "OVERVOLTAGE";
    }
    if (state == 9u) {
        return "UNDERVOLTAGE";
    }
    if (state == 10u) {
        return "OVERCURRENT";
    }
    if (state == 11u) {
        return "MOS_OVERTEMP";
    }
    if (state == 12u) {
        return "MOTOR_OVERTEMP";
    }
    if (state == 13u) {
        return "COMM_LOST";
    }
    if (state == 14u) {
        return "OVERLOAD";
    }
    return "UNKNOWN";
}

static void print_h6215_feedback(const can_frame_t *frame)
{
    const uint8_t motor_id = frame->data[0] & 0x0Fu;
    const uint8_t state = frame->data[0] >> 4;
    const uint32_t position_raw =
        ((uint32_t)frame->data[1] << 8) | frame->data[2];
    const uint32_t velocity_raw =
        ((uint32_t)frame->data[3] << 4) | (frame->data[4] >> 4);
    const uint32_t torque_raw =
        ((uint32_t)(frame->data[4] & 0x0Fu) << 8) | frame->data[5];
    /*
     * The motor was just verified as P_MAX=12.5, V_MAX=45 and T_MAX=10.
     * Values below are expressed in milli-units to avoid a floating-point
     * printf dependency in this minimal bare-metal firmware.
     */
    const int32_t position_millirad =
        (int32_t)((position_raw * 25000u) / 65535u) - 12500;
    const int32_t velocity_millirad_s =
        (int32_t)((velocity_raw * 90000u) / 4095u) - 45000;
    const int32_t torque_millinewton_m =
        (int32_t)((torque_raw * 20000u) / 4095u) - 10000;

    usart1_write("FB ID=");
    usart1_write_u8(motor_id);
    usart1_write(" STATE=");
    usart1_write(h6215_state_name(state));
    usart1_write(" P=");
    usart1_write_milli(position_millirad);
    usart1_write("rad V=");
    usart1_write_milli(velocity_millirad_s);
    usart1_write("rad/s T=");
    usart1_write_milli(torque_millinewton_m);
    usart1_write("Nm TMOS=");
    usart1_write_u8(frame->data[6]);
    usart1_write("C TROTOR=");
    usart1_write_u8(frame->data[7]);
    usart1_write("C\r\n");
}

static uint8_t h6215_feedback_is_safe(const can_frame_t *frame)
{
    const uint8_t state = frame->data[0] >> 4;
    const uint32_t velocity_raw =
        ((uint32_t)frame->data[3] << 4) | (frame->data[4] >> 4);
    const int32_t velocity_millirad_s =
        (int32_t)((velocity_raw * 90000u) / 4095u) - 45000;

    if ((motion_state != MOTION_IDLE) && (state != 1u)) {
        return 0u;
    }
    if ((motion_state == MOTION_RUNNING)
        && ((velocity_millirad_s > 800)
            || (velocity_millirad_s < -800))) {
        return 0u;
    }
    if (((motion_state == MOTION_CONTINUOUS)
         || (motion_state == MOTION_WATCHDOG_STOPPING))
        && ((velocity_millirad_s > 800)
            || (velocity_millirad_s < -800))) {
        return 0u;
    }
    if ((frame->data[6] >= 60u) || (frame->data[7] >= 60u)) {
        return 0u;
    }
    return 1u;
}

static void print_received_frame(const can_frame_t *frame)
{
    uint8_t i;
    const uint8_t is_parameter_response =
        (frame->id == H6215_MASTER_ID)
        && (frame->dlc == 8u)
        && (frame->data[0] == (uint8_t)(H6215_CAN_ID & 0xFFu))
        && (frame->data[1] == (uint8_t)(H6215_CAN_ID >> 8))
        && (frame->data[2] == 0x33u)
        && ((frame->data[3] == H6215_RID_CONTROL_MODE)
            || (frame->data[3] == H6215_RID_SOFTWARE_VERSION)
            || (frame->data[3] == H6215_RID_P_MAX)
            || (frame->data[3] == H6215_RID_V_MAX)
            || (frame->data[3] == H6215_RID_T_MAX));

    if ((is_parameter_response == 0u)
        && (frame->id == H6215_MASTER_ID)
        && (frame->dlc == 8u)
        && ((frame->data[0] & 0x0Fu)
            == (uint8_t)(H6215_CAN_ID & 0x0Fu))) {
        last_feedback_frame = *frame;
        last_feedback_valid = 1u;
        if ((uint32_t)(system_millis - last_feedback_print_at) >= 100u) {
            last_feedback_print_at = system_millis;
            print_h6215_feedback(frame);
        }
        if (h6215_feedback_is_safe(frame) == 0u) {
            print_h6215_feedback(frame);
            stop_and_disable_motion("FEEDBACK_SAFETY_TRIP MOTOR_DISABLED");
        }
        return;
    }

    usart1_write("CAN_RX ID=0x");
    usart1_write_hex(frame->id, 3u);
    usart1_write(" DLC=");
    usart1_write_u8(frame->dlc);
    usart1_write(" DATA=");
    for (i = 0u; i < frame->dlc; ++i) {
        usart1_write_hex(frame->data[i], 2u);
        if ((uint8_t)(i + 1u) != frame->dlc) {
            usart1_write(" ");
        }
    }
    usart1_write("\r\n");

    if (is_parameter_response != 0u) {
        const uint8_t register_id = frame->data[3];
        const uint32_t value = pack_four_bytes(&frame->data[4]);

        if (register_id == H6215_RID_SOFTWARE_VERSION) {
            usart1_write("H6215_SW_VERSION_RAW=0x");
            usart1_write_hex(value, 8u);
            usart1_write("\r\n");
        } else if (register_id == H6215_RID_CONTROL_MODE) {
            h6215_control_mode = (uint8_t)value;
            h6215_control_mode_valid = 1u;
            usart1_write("H6215_CONTROL_MODE=");
            usart1_write_u8((uint8_t)value);
            if (value == 1u) {
                usart1_write(" MIT");
            } else if (value == 2u) {
                usart1_write(" POSITION_VELOCITY");
            } else if (value == 3u) {
                usart1_write(" VELOCITY");
            } else {
                usart1_write(" UNKNOWN");
            }
            usart1_write("\r\n");
        } else if ((register_id == H6215_RID_P_MAX)
                   || (register_id == H6215_RID_V_MAX)
                   || (register_id == H6215_RID_T_MAX)) {
            if (register_id == H6215_RID_P_MAX) {
                usart1_write("H6215_P_MAX");
            } else if (register_id == H6215_RID_V_MAX) {
                usart1_write("H6215_V_MAX");
            } else {
                usart1_write("H6215_T_MAX");
            }
            usart1_write("_FLOAT_RAW=0x");
            usart1_write_hex(value, 8u);
            usart1_write("\r\n");
        }
    }
}

static void clock_failure_loop(void)
{
    volatile uint32_t delay;

    usart1_init(0x45u);
    usart1_write("CLOCK_HSE_OR_PLL_FAIL\r\n");
    for (;;) {
        led_toggle();
        for (delay = 0u; delay < 300000u; ++delay) {
        }
    }
}

int main(void)
{
    uint32_t last_led_toggle = 0u;
    can_frame_t frame;
    uint8_t command;

    led_init();
    if (clock_init_72mhz() == 0u) {
        clock_failure_loop();
    }

    /* 72 MHz / 115200 = 625, so BRR is 0x271. */
    usart1_init(0x271u);
    systick_init();
    usart1_write("\r\nH6215_VELOCITY_TEST_START\r\n");
    usart1_write("CLOCK_OK SYSCLK=72MHz APB1=36MHz\r\n");

    if (can_init(1u) == 0u) {
        usart1_write("CAN_INIT_FAIL\r\n");
        for (;;) {
        }
    }
    if (can_loopback_test() == 0u) {
        usart1_write("CAN_LOOPBACK_FAIL\r\n");
        for (;;) {
        }
    }
    usart1_write("CAN_LOOPBACK_PASS\r\n");

    if (can_set_normal_mode() == 0u) {
        usart1_write("CAN_NORMAL_MODE_FAIL\r\n");
        for (;;) {
        }
    }
    usart1_write("CAN_NORMAL_MODE_READY 1Mbps\r\n");
    usart1_write("READ: s=status, r=version, m=mode, p=P_MAX, v=V_MAX, t=T_MAX, f=last feedback\r\n");
    usart1_write("PULSE: a then g=+0.2rad/s or b=-0.2rad/s; x=stop\r\n");
    usart1_write("LIVE: a then c=start; +=faster, -=slower, 0=zero, k=keepalive\r\n");

    for (;;) {
        const uint32_t now = system_millis;

        if ((uint32_t)(now - last_led_toggle) >= 500u) {
            last_led_toggle = now;
            led_toggle();
        }

        update_guarded_motion_test(now);

        if (usart1_read_byte(&command) != 0u) {
            if (command == (uint8_t)'s') {
                can_print_status();
            } else if (command == (uint8_t)'r') {
                send_h6215_read_request(H6215_RID_SOFTWARE_VERSION);
            } else if (command == (uint8_t)'m') {
                send_h6215_read_request(H6215_RID_CONTROL_MODE);
            } else if (command == (uint8_t)'p') {
                send_h6215_read_request(H6215_RID_P_MAX);
            } else if (command == (uint8_t)'v') {
                send_h6215_read_request(H6215_RID_V_MAX);
            } else if (command == (uint8_t)'t') {
                send_h6215_read_request(H6215_RID_T_MAX);
            } else if (command == (uint8_t)'f') {
                if (last_feedback_valid != 0u) {
                    print_h6215_feedback(&last_feedback_frame);
                } else {
                    usart1_write("NO_MOTOR_FEEDBACK_YET\r\n");
                }
            } else if (command == (uint8_t)'a') {
                if ((h6215_control_mode_valid != 0u)
                    && (h6215_control_mode == 3u)
                    && (motion_state == MOTION_IDLE)) {
                    motion_armed = 1u;
                    motion_armed_at = now;
                    usart1_write("ARMED_FOR_10_SECONDS; SEND g TO START OR x TO CANCEL\r\n");
                } else {
                    usart1_write("ARM_REJECTED: SEND m AND REQUIRE VELOCITY MODE 3\r\n");
                }
            } else if (command == (uint8_t)'g') {
                start_guarded_motion_test(now, 1u);
            } else if (command == (uint8_t)'b') {
                start_guarded_motion_test(now, 2u);
            } else if (command == (uint8_t)'c') {
                start_guarded_motion_test(now, 0u);
            } else if ((command == (uint8_t)'+')
                       && (motion_state == MOTION_CONTINUOUS)) {
                if (target_speed_step < 5) {
                    ++target_speed_step;
                }
                motion_last_user_command_at = now;
                print_speed_target();
            } else if ((command == (uint8_t)'-')
                       && (motion_state == MOTION_CONTINUOUS)) {
                if (target_speed_step > -5) {
                    --target_speed_step;
                }
                motion_last_user_command_at = now;
                print_speed_target();
            } else if ((command == (uint8_t)'0')
                       && (motion_state == MOTION_CONTINUOUS)) {
                target_speed_step = 0;
                motion_last_user_command_at = now;
                print_speed_target();
            } else if ((command == (uint8_t)'k')
                       && (motion_state == MOTION_CONTINUOUS)) {
                motion_last_user_command_at = now;
                usart1_write("KEEPALIVE_OK\r\n");
            } else if (command == (uint8_t)'x') {
                stop_and_disable_motion("EMERGENCY_STOP MOTOR_DISABLED");
            }
        }

        while (can_receive(&frame) != 0u) {
            print_received_frame(&frame);
        }
    }
}
