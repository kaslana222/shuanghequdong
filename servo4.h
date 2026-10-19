/*
 * servo4.h
 *
 * 四路舵机驱动（TIM2 的 CH1~CH4）
 *    CH1 -> PA0      CH2 -> PA1      CH3 -> PA2      CH4 -> PA3
 *  这四路就是 CubeMX 默认的 TIM2 引脚，不需要任何重映射。
 *
 *  TIM2 由 MX_TIM2_Init() 拥有和配置：
 *      Prescaler = 71, Counter Period = 19999   （72MHz -> 1MHz -> 20ms = 50Hz）
 *      四个通道均为 PWM Generation，Pulse 初值 0
 *  本模块只负责"启动 / 改脉宽 / 停止"，不配置 TIM2、不碰 PA0~PA3，
 *  因此必须在 MX_TIM2_Init() 之后调用 Servo_Init()。
 *
 *  ★ 按键约定：每个舵机一对按键，一个正转、一个反转 ★
 *      舵机0 (PA0):  + = L1          - = L2
 *      舵机1 (PA1):  + = R1          - = R2
 *      舵机2 (PA2):  + = TRIANGLE △  - = CROSS ×
 *      舵机3 (PA3):  + = SQUARE □    - = CIRCLE ○
 *    "+"   -> 脉宽变大 -> 180° 舵机朝一个方向转
 *    "-"   -> 脉宽变小 -> 朝反方向转
 *    到 SERVO_MIN_PULSE_US / SERVO_MAX_PULSE_US 端点自动停住，不会绕回。
 *
 *    全部 8 个键都在 ps2.btn2 里，和 main.c 用的
 *    START / 十字键 / 摇杆（都在 btn1）完全不冲突，
 *    所以 START 的电机解锁功能保持原样，不用动。
 *
 *    想换键：servo4.c 的 servo_step() 里四路是逐条写死的，
 *    直接改对应那一行的 PS2B_xxx 即可。
 *
 *  信号脚只出脉冲，不能当电源：舵机请用独立电源，并与本板共地。
 */

#ifndef INC_SERVO4_H_
#define INC_SERVO4_H_

#include "stm32f1xx_hal.h"

/*---------------------------------------------------------------
 * 1) 数量与脉宽范围
 *    1MHz 计数 -> CCR 的数值直接等于脉宽微秒数
 *
 *    SG90（180°）的行业口径是 0.5ms=0°、2.5ms=180°，
 *    Arduino Servo 库用的也是 544~2400us 这一档。
 *    所以 1000~2000us 只能覆盖大约一半行程，这里改成 500~2500。
 *
 *    ★ 标定提示（只改这三个宏，其余逻辑不用动）★
 *      总行程偏小 / 转不到底 -> 保持或加大 MIN~MAX 的跨度
 *      某一端顶住不动、持续嗡嗡响（撞机械限位）
 *                            -> 把那一端往中间收，例如 600 / 2400
 *      中位偏了              -> 调 CENTER
 *    舵机个体差异很大，务必以实测为准。
 *-------------------------------------------------------------*/
#define SERVO_COUNT            4U
#define SERVO_MIN_PULSE_US     500U
#define SERVO_MAX_PULSE_US     2500U
#define SERVO_CENTER_PULSE_US  1500U

/*---------------------------------------------------------------
 * 2) 旋转速度
 *    SERVO_STEP_US   : 每走一步脉宽变化多少微秒
 *                      全程 2000us 跨度下，100us 约等于 9°，
 *                      即约 20 步走完 0~180°
 *    SERVO_PERIOD_MS : 多少毫秒走一步（越小转得越快，
 *                      但太小会让主循环变卡）
 *
 *    注意主循环本身约 12ms 一圈（HAL_Delay(10) + PS/2 读取 2.3ms），
 *    所以 PERIOD_MS=20 时实际约 25ms 才走一步。想让手感更利落，
 *    优先调大 STEP_US，不要把 PERIOD_MS 压到 20 以下。
 *-------------------------------------------------------------*/
#define SERVO_STEP_US          100U
#define SERVO_PERIOD_MS        20U

/*---------------------------------------------------------------
 * 3) 八个按键（全部取自 ps2.btn2，不是 btn1）
 *
 *    标准 PS/2 手柄的 btn2 字节排布：
 *        bit0 L2        bit1 R2        bit2 L1        bit3 R1
 *        bit4 TRIANGLE△ bit5 CIRCLE○   bit6 CROSS×    bit7 SQUARE□
 *
 *    如果你的手柄实际落位不同，只改这 8 个宏、保证互不相同
 *    即可，驱动逻辑不关心具体是哪一位。
 *    拿不准哪一位对应哪个键时，可以用最后的 Servo_KeyToBit()
 *    在调试器里现场测一下。
 *-------------------------------------------------------------*/
#define PS2B_L2        0x01U
#define PS2B_R2        0x02U
#define PS2B_L1        0x04U
#define PS2B_R1        0x08U

#define PS2B_TRIANGLE  0x10U    /* △ */
#define PS2B_CIRCLE    0x20U    /* ○ */
#define PS2B_CROSS     0x40U    /* × */
#define PS2B_SQUARE    0x80U    /* □ */

/* 舵机编号，方便上层代码引用 */
#define SERVO_0     0U
#define SERVO_1     1U
#define SERVO_2     2U
#define SERVO_3     3U

/*---------------------------------------------------------------
 * 4) 接口
 *-------------------------------------------------------------*/

/* 校验 CubeMX 配置（时钟、PSC/ARR、重映射、引脚复用），
 * 并把四路 PWM 一起启动，脉宽先给中位 1500us。
 * 必须在 MX_TIM2_Init() 之后调用，返回 HAL_OK 才算成功。
 * 重复调用是安全的（已在运行时直接返回 HAL_OK）。 */
HAL_StatusTypeDef Servo_Init(void);

/* 设置某一路舵机的脉宽（微秒），立即生效，自动限幅到 1000~2000。
 * 输出没起来时只记录，不报错。id 越界返回 HAL_ERROR。 */
HAL_StatusTypeDef Servo_SetPulseUs(uint8_t id, uint16_t pulse_us);

/* 读取某一路当前脉宽（微秒）。id 越界返回 0。 */
uint16_t Servo_GetPulseUs(uint8_t id);

/* 直接用"角度"设置，0~180 度线性映射到 1000~2000us。
 * 这是粗略标定，要精确请用 Servo_SetPulseUs()。 */
HAL_StatusTypeDef Servo_SetAngle(uint8_t id, uint8_t deg);

/* 关掉四路 PWM 输出并锁存为"停止"，直到再次 Servo_Init()。
 * 注意：这不是断电隔离，舵机此时一般无力矩，不会保持位置。 */
HAL_StatusTypeDef Servo_Stop(void);

/* 周期任务：在主循环里每次调用即可。
 *   - 用 HAL_GetTick() 做 SERVO_PERIOD_MS 节流
 *   - 按住 "+" 键脉宽递增，按住 "-" 键递减，到端点停住
 *   - 输出没启动时只更新目标值，不报错
 * 用法：Servo_Task(&ps2.btn2);  */
void Servo_Task(volatile const uint8_t *ps2_btn2);

/* 调试小工具：返回 8 个舵机键里当前按下的那些位。
 * 用法：在 Servo_Task 上面下断点，逐次按下某个键，
 * 看返回值是哪一位，就能确认这个键对应哪个宏。
 * 只是读一下，不做任何动作，想留就留、想删就删。 */
uint8_t Servo_KeyToBit(volatile const uint8_t *ps2_btn2);

#endif /* INC_SERVO4_H_ */
