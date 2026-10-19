/*
 * servo4.c
 *
 * 四路舵机驱动，占用 TIM2 的 CH1~CH4（PA0~PA3）。
 * 每个舵机由一个 PLUS 键和一个 MINUS 键控制，分别让脉宽变大 / 变小。
 *
 * 设计要点：
 *   - 以"脉宽"为中心，HAL 的通道状态机由本模块统一照看，
 *     避免 HAL_TIM_PWM_Start/Stop 与直接写寄存器混用导致状态打架。
 *   - 四路 PWM 启动一次后由 TIM2 硬件自由运行，主循环卡顿也不会丢脉冲。
 *   - 按键用电平判定（按住就一直转），到 1000/2000 端点自动停住。
 *   - 只有 8 个舵机键参与"上一帧状态"比对，START/十字键不受影响。
 *
 *  Created on: Oct 13, 2026
 *      Author: ch'r
 */

#include "servo4.h"
#include "tim.h"

/*---------------------------------------------------------------
 * 私有数据
 *-------------------------------------------------------------*/

/* 通道号，下标与舵机编号一致 */
static const uint32_t servo_ch[SERVO_COUNT] =
{
    TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4
};

/* 引脚编号（0~3，对应 PA0~PA3），下标与舵机编号一致 */
static const uint8_t servo_pin_num[SERVO_COUNT] =
{
    0U, 1U, 2U, 3U
};

/* 四路舵机的按键分配（全部取自 ps2.btn2，与 main.c 用的 START / 十字键
 * 所在的 btn1 不冲突）。servo_step() 里已经逐路显式写死，想换键直接改
 * 那几处的 PS2B_xxx 即可：
 *   舵机0 PA0 : + = L1(0x04)   - = L2(0x01)
 *   舵机1 PA1 : + = R1(0x08)   - = R2(0x02)
 *   舵机2 PA2 : + = △ (0x10)   - = × (0x40)
 *   舵机3 PA3 : + = □ (0x80)   - = ○ (0x20)
 */

/* 8 个舵机键的掩码：只有这些位参与状态比对，
 * 万一 btn2 里有别的杂位也不会误触发。 */
#define SERVO_KEY_MASK  (uint8_t)(PS2B_L1 | PS2B_L2 | PS2B_R1 | PS2B_R2 | \
                                  PS2B_TRIANGLE | PS2B_CROSS | PS2B_CIRCLE | PS2B_SQUARE)

/* 当前目标脉宽（微秒）。初值 0 = 还没设定过，不输出脉冲。 */
static uint16_t servo_pulse[SERVO_COUNT];

/* 上一次采样的 8 个按键状态 */
static uint8_t  servo_key_last;

/* 节流窗口内出现过、但还没处理的按键变化 */
static uint8_t  servo_pending;

/* 状态 */
static uint8_t  servo_inited;    /* Servo_Init() 校验是否通过 */
static uint8_t  servo_running;   /* 四路 PWM 是否已启动 */
static uint8_t  btn_seeded;      /* 是否已用第一帧按键状态播种 */
static uint32_t servo_tick;      /* 上一次走步的时间戳 */

/*---------------------------------------------------------------
 * 私有函数
 *-------------------------------------------------------------*/

/**
  * @brief  把 servo_pulse[] 写进四个 CCR。
  */
static void servo_load_ccr(void)
{
    /* 四路逐条显式写出，不再走循环 —— 每一路都看得见，谁也影响不到谁。
     * 通道与引脚一一对应：CH1=PA0、CH2=PA1、CH3=PA2、CH4=PA3。 */
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, servo_pulse[0]);   /* 舵机0  PA0 */
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, servo_pulse[1]);   /* 舵机1  PA1 */
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, servo_pulse[2]);   /* 舵机2  PA2 */
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, servo_pulse[3]);   /* 舵机3  PA3 */
}

/**
  * @brief  停掉四路 PWM，并把 HAL 的通道状态同步回 READY。
  */
static void servo_pwm_stop_all(void)
{
    for (uint8_t i = 0; i < SERVO_COUNT; i++)
        (void)HAL_TIM_PWM_Stop(&htim2, servo_ch[i]);

    servo_running = 0;
}

/**
  * @brief  检查四个引脚是否真的是复用推挽输出（AF_PP）。
  *         PA0~PA3 在 GPIOA 的 CRL 里，每个脚占 4 位：
  *             bit[1:0] 00=输入, 11=输出50MHz
  *             bit[3:2] 10=复用推挽 AF_PP
  */
static HAL_StatusTypeDef servo_check_pins(void)
{
    for (uint8_t i = 0; i < SERVO_COUNT; i++)
    {
        uint32_t shift = (uint32_t)servo_pin_num[i] * 4U;
        uint32_t mode  = (GPIOA->CRL >> shift) & 0x0FU;

        if ((mode & 0x03U) == 0x00U || (mode & 0x0CU) != 0x08U)
            return HAL_ERROR;
    }
    return HAL_OK;
}

/**
  * @brief  校验 TIM2 的时基：能整除出 1MHz，且 ARR = 19999。
  */
static HAL_StatusTypeDef servo_check_timebase(void)
{
    uint32_t timclk = HAL_RCC_GetPCLK1Freq();

    /* APB1 分频 != 1 时，定时器时钟是 PCLK1 的两倍 */
    if (RCC->CFGR & RCC_CFGR_PPRE1_2)
        timclk *= 2U;

    if (!timclk || (timclk % 1000000U))
        return HAL_ERROR;                       /* 除不出整 1MHz */

    if (htim2.Init.Prescaler != (timclk / 1000000U - 1U))
        return HAL_ERROR;                       /* 计数频率不是 1MHz */

    if (htim2.Init.Period != 19999U)
        return HAL_ERROR;                       /* 周期不是 20ms */

    return HAL_OK;
}

/**
  * @brief  往四个通道写 CCR，再生成一次更新事件让影子寄存器立即装载；
  *         若尚未运行，则一并把四路 PWM 启起来。
  */
static HAL_StatusTypeDef servo_apply(void)
{
    servo_load_ccr();

    if (servo_running)
        return HAL_OK;              /* 已在跑：新脉宽下一个周期生效 */

    /* 还没跑：从相位 0 干净地起来，保证第一个脉冲完整 */
    __HAL_TIM_SET_COUNTER(&htim2, 0);

    if (HAL_TIM_GenerateEvent(&htim2, TIM_EVENTSOURCE_UPDATE) != HAL_OK)
        return HAL_ERROR;
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);

    /* 四路逐条显式启动，不再走循环：CH1=PA0、CH2=PA1、CH3=PA2、CH4=PA3。
     * 任何一路起不来就整体停掉并报错，不留半开状态。 */
    if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK)   /* 舵机0  PA0 */
    {
        servo_pwm_stop_all();
        return HAL_ERROR;
    }
    if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2) != HAL_OK)   /* 舵机1  PA1 */
    {
        servo_pwm_stop_all();
        return HAL_ERROR;
    }
    if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3) != HAL_OK)   /* 舵机2  PA2 */
    {
        servo_pwm_stop_all();
        return HAL_ERROR;
    }
    if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4) != HAL_OK)   /* 舵机3  PA3 */
    {
        servo_pwm_stop_all();
        return HAL_ERROR;
    }

    servo_running = 1;
    return HAL_OK;
}

/**
  * @brief  脉宽限幅到 1000~2000 微秒。
  */
static uint16_t servo_clamp(uint16_t pulse_us)
{
    if (pulse_us > SERVO_MAX_PULSE_US) return SERVO_MAX_PULSE_US;
    if (pulse_us < SERVO_MIN_PULSE_US) return SERVO_MIN_PULSE_US;
    return pulse_us;
}

/**
  * @brief  走一步：按住的键决定方向，到端点自动停住。
  */
static void servo_step(uint8_t keys)
{
    /* 四路逐条显式写出，不再走循环。键位与引脚一一对应：
     *   舵机0 PA0 : + = L1(0x04)   - = L2(0x01)
     *   舵机1 PA1 : + = R1(0x08)   - = R2(0x02)
     *   舵机2 PA2 : + = △ (0x10)   - = × (0x40)
     *   舵机3 PA3 : + = □ (0x80)   - = ○ (0x20)
     * 每路只认自己那两个键，四路互不干扰。 */

    /* ---------- 舵机0 : PA0 / TIM2_CH1 ---------- */
    if (keys & PS2B_L1)
    {
        if (servo_pulse[0] + SERVO_STEP_US <= SERVO_MAX_PULSE_US)
            servo_pulse[0] = (uint16_t)(servo_pulse[0] + SERVO_STEP_US);
    }
    else if (keys & PS2B_L2)
    {
        if (servo_pulse[0] >= SERVO_MIN_PULSE_US + SERVO_STEP_US)
            servo_pulse[0] = (uint16_t)(servo_pulse[0] - SERVO_STEP_US);
    }

    /* ---------- 舵机1 : PA1 / TIM2_CH2 ---------- */
    if (keys & PS2B_R1)
    {
        if (servo_pulse[1] + SERVO_STEP_US <= SERVO_MAX_PULSE_US)
            servo_pulse[1] = (uint16_t)(servo_pulse[1] + SERVO_STEP_US);
    }
    else if (keys & PS2B_R2)
    {
        if (servo_pulse[1] >= SERVO_MIN_PULSE_US + SERVO_STEP_US)
            servo_pulse[1] = (uint16_t)(servo_pulse[1] - SERVO_STEP_US);
    }

    /* ---------- 舵机2 : PA2 / TIM2_CH3 ---------- */
    if (keys & PS2B_TRIANGLE)
    {
        if (servo_pulse[2] + SERVO_STEP_US <= SERVO_MAX_PULSE_US)
            servo_pulse[2] = (uint16_t)(servo_pulse[2] + SERVO_STEP_US);
    }
    else if (keys & PS2B_CROSS)
    {
        if (servo_pulse[2] >= SERVO_MIN_PULSE_US + SERVO_STEP_US)
            servo_pulse[2] = (uint16_t)(servo_pulse[2] - SERVO_STEP_US);
    }

    /* ---------- 舵机3 : PA3 / TIM2_CH4 ---------- */
    if (keys & PS2B_SQUARE)
    {
        if (servo_pulse[3] + SERVO_STEP_US <= SERVO_MAX_PULSE_US)
            servo_pulse[3] = (uint16_t)(servo_pulse[3] + SERVO_STEP_US);
    }
    else if (keys & PS2B_CIRCLE)
    {
        if (servo_pulse[3] >= SERVO_MIN_PULSE_US + SERVO_STEP_US)
            servo_pulse[3] = (uint16_t)(servo_pulse[3] - SERVO_STEP_US);
    }
}

/*---------------------------------------------------------------
 * 对外接口
 *-------------------------------------------------------------*/

HAL_StatusTypeDef Servo_Init(void)
{
    HAL_StatusTypeDef st;

    /* 重复调用：已经在跑就当作成功直接返回 */
    if (servo_running)
        return HAL_OK;

    servo_inited   = 0;
    btn_seeded     = 0;
    servo_key_last = 0;
    servo_pending  = 0;
    servo_tick     = 0;

    /* 1) CubeMX 的句柄必须真的指向 TIM2 */
    if (htim2.Instance != TIM2)
        return HAL_ERROR;

    /* 2) 不能勾 TIM2 重映射：一重映射 CH1 就跑到 PA15，PA0 就废了 */
    __HAL_RCC_AFIO_CLK_ENABLE();
    if (AFIO->MAPR & AFIO_MAPR_TIM2_REMAP)
        return HAL_ERROR;

    /* 3) 时基必须是 1MHz / 20ms */
    st = servo_check_timebase();
    if (st != HAL_OK)
        return st;

    /* 4) 引脚必须是 AF_PP（由 HAL_TIM_MspPostInit 配好） */
    st = servo_check_pins();
    if (st != HAL_OK)
        return st;

    /* 5) TIM2 不能被从模式 / 中断 / 单脉冲占用 */
    if (TIM2->SMCR || TIM2->DIER || (TIM2->CR1 & (TIM_CR1_OPM | TIM_CR1_UDIS)))
        return HAL_ERROR;

    servo_inited = 1;

    /* 6) 上电即锁中位：四个舵机都有确定位置，按键才有东西可调 */
    for (uint8_t i = 0; i < SERVO_COUNT; i++)
        servo_pulse[i] = SERVO_CENTER_PULSE_US;

    return servo_apply();
}

HAL_StatusTypeDef Servo_SetPulseUs(uint8_t id, uint16_t pulse_us)
{
    if (!servo_inited || id >= SERVO_COUNT)
        return HAL_ERROR;

    servo_pulse[id] = servo_clamp(pulse_us);

    if (!servo_running)
        return HAL_OK;               /* 输出没起来：先记着 */

    __HAL_TIM_SET_COMPARE(&htim2, servo_ch[id], servo_pulse[id]);
    return HAL_OK;
}

uint16_t Servo_GetPulseUs(uint8_t id)
{
    if (id >= SERVO_COUNT)
        return 0U;
    return servo_pulse[id];
}

HAL_StatusTypeDef Servo_SetAngle(uint8_t id, uint8_t deg)
{
    uint32_t pulse;

    if (!servo_inited || id >= SERVO_COUNT || deg > 180U)
        return HAL_ERROR;

    /* 0度 -> 1000us, 180度 -> 2000us */
    pulse = SERVO_MIN_PULSE_US +
            ((uint32_t)deg * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US)) / 180U;

    return Servo_SetPulseUs(id, (uint16_t)pulse);
}

HAL_StatusTypeDef Servo_Stop(void)
{
    if (!servo_inited)
        return HAL_ERROR;

    servo_pwm_stop_all();
    return HAL_OK;
}

void Servo_Task(volatile const uint8_t *ps2_btn2)
{
    uint8_t now_keys;
    uint8_t changed;
    uint8_t step_now;

    if (!servo_inited || ps2_btn2 == NULL)
        return;

    now_keys = (uint8_t)(*ps2_btn2 & SERVO_KEY_MASK);

    /*---------------------------------------------------------------
     * 无效帧保护（★ 必须放在播种判断之前）
     *
     *   SERVO_KEY_MASK 用满了 btn2 的 8 个位，所以
     *     "8 个键同时按下"        -> now_keys == 0xFF
     *     "接收器没拿到手柄数据"    -> PS2_data[4] == 0x00，
     *                                取反后 btn2 == 0xFF，也是 0xFF
     *   两种情况在数值上完全一样，靠 btn2 本身分不开。
     *
     *   正常操作不可能同时按住 8 个键并持续几十毫秒，所以这里
     *   一律当"全部松开"处理，绝不产生任何动作。否则 step_now
     *   会是 0xFF，servo_step() 里每一路的 if(plus) 都命中，
     *   四路脉宽每走一步一起 +SERVO_STEP_US，很快就会全部顶到
     *   MAX 的限位上，舵机一直嗡嗡响不转。
     *
     *   清零 btn_seeded：等数据恢复后第一帧只重新播种、不动作。
     *-------------------------------------------------------------*/
    if (now_keys == SERVO_KEY_MASK)
    {
        servo_key_last = 0U;
        servo_pending  = 0U;
        btn_seeded     = 0U;
        return;
    }

    /* 第一帧只记录状态，不产生动作。
     * 否则开机时手柄还在自检、btn2 可能是 0xFF，四个舵机会一起飞走。 */
    if (!btn_seeded)
    {
        servo_key_last = now_keys;
        btn_seeded     = 1;
        servo_tick     = HAL_GetTick() - SERVO_PERIOD_MS;   /* 第一次立刻响应 */
        return;
    }

    changed = (uint8_t)(now_keys ^ servo_key_last);

    /* 变化了就纳入"还没处理"，等节流窗口允许时再真正走步。
     * 注意这里不立刻更新 last，否则窗口内的按键会被吞掉。 */
    if (changed)
        servo_pending |= changed;

    if (HAL_GetTick() - servo_tick < SERVO_PERIOD_MS)
        return;

    servo_tick = HAL_GetTick();

    /* 窗口内出现过的变化（pending），或者当前仍按住的键（now_keys），
     * 都要处理。前者保证短按不丢，后者保证按住能连续转。
     *
     * ★ 必须原样保留 now_keys 的位，绝不能按舵机做
     *   "plus | minus 一起置位"的转换。
     *
     *   servo_step() 用 if(plus) / else if(minus) 区分方向，一旦某个
     *   舵机的 plus 和 minus 同时出现在 keys 里，else if 就永远执行
     *   不到 —— 减方向整体失效，四路舵机只能单向往 2000us 走到底，
     *   之后无论按什么键都卡在端点顶住不动（持续嗡嗡响）。
     *   这正是"舵机只能转极小角度 / 回不去"的直接原因。
     *
     *   now_keys 已经 & 过 SERVO_KEY_MASK，只含这 8 个舵机键，
     *   直接并进来就对了：pending 保证短按不丢，now_keys 保证按住连续转。 */
    step_now = (uint8_t)(servo_pending | now_keys);

    servo_key_last = now_keys;

    if (step_now == 0U)
        return;                      /* 没有要动的：pending 留着下个窗口再说 */

    servo_pending = 0U;

    servo_step(step_now);

    if (servo_running)
        servo_load_ccr();
}

uint8_t Servo_KeyToBit(volatile const uint8_t *ps2_btn2)
{
    if (ps2_btn2 == NULL)
        return 0U;

    return (uint8_t)(*ps2_btn2 & SERVO_KEY_MASK);
}