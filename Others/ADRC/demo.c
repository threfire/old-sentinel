#include "adrc.h"

#include <stdint.h>
#include <stdio.h>

/**
 * @file demo.c
 * @brief 按 PID 替换思路编写的 ADRC 最小示例
 *
 * 如果你现在工程里原本是这样用 PID：
 *
 *   PID_init(&pid, ...);
 *   out = PID_Calc(&pid, ref, set);
 *   PID_clear(&pid);
 *
 * 那么替换成 ADRC 后，推荐先按下面这个最小形式接入：
 *
 *   ADRC_init(&adrc, &param);
 *   out = ADRC_Calc(&adrc, ref, set);
 *   ADRC_clear(&adrc);
 *
 * 这个文件就专门按这种方式来写。
 */

/**
 * @brief 准备一组 ADRC 参数
 *
 * 这一步就对应 PID 里准备 Kp / Ki / Kd 和限幅参数。
 */
static void demo_prepare_adrc_param(adrc_param_t *param)
{
    if(param == 0)
    {
        return;
    }

    param->sample_time_s = 0.001f;
    param->b0 = 46867.05571923f;
    param->controller_bandwidth = 209.42624735f;
    param->observer_bandwidth_ratio = 2.5f;
    param->tracking_gain = 0.0f;
    param->max_out = 4.0f;
    param->output_rate_limit = 300.0f;
    param->error_linear_zone = 12.0f;
    param->alpha1 = 0.5f;
    param->alpha2 = 0.25f;
}

/**
 * @brief 模拟把控制量送给电机执行器
 *
 * 真实工程里把这里替换成：
 * 1. CAN 发送电流
 * 2. FOC 的 iq 给定
 * 3. PWM 或电压输出
 */
static void demo_send_motor_current(float current_a)
{
    printf("send current = %.4f A\n", current_a);
}

/**
 * @brief 打印调试信息
 */
static void demo_print_state(uint32_t index, const adrc_type_def *adrc)
{
    if(adrc == 0)
    {
        return;
    }

    printf("idx=%02lu, set=%.1f rpm, fdb=%.1f rpm, out=%.4f A, delta=%.4f A, disturb=%.2f\n",
           (unsigned long)index,
           adrc->set,
           adrc->fdb,
           adrc->out,
           adrc->delta_out,
           adrc->disturbance);
}

int main(void)
{
    uint32_t i;
    int status;
    float out;
    adrc_type_def adrc;
    adrc_param_t param;

    /*
     * 这组数据模拟的是：
     * 1. 摩擦轮已经基本稳速
     * 2. 中间有几次发射导致的小幅掉速
     *
     * 在真实工程里，这里的 ref 应来自：
     * - 编码器测速
     * - 电机回传速度
     * - CAN 总线反馈值
     */
    const float measured_speed_samples[] =
    {
        3661.0f, 3660.5f, 3652.0f, 3664.0f, 3650.5f,
        3663.0f, 3649.0f, 3662.5f, 3651.0f, 3664.5f
    };

    /* 第一步：准备参数。 */
    demo_prepare_adrc_param(&param);

    /* 第二步：初始化 ADRC。 */
    status = ADRC_init(&adrc, &param);
    if(status != ADRC_OK)
    {
        printf("ADRC_init failed: %d\n", status);
        return status;
    }

    /*
     * 如果对象已经在运行，例如摩擦轮一直开着，
     * 建议先做一次热启动复位，使切入控制时更平顺。
     * 这一步不是必须的，但在常开工况下通常更合适。
     */
    ADRC_hot_reset(&adrc, 3660.0f, 3660.6f, 1.67f);

    for(i = 0; i < (uint32_t)(sizeof(measured_speed_samples) / sizeof(measured_speed_samples[0])); ++i)
    {
        /*
         * 第三步：每个控制周期直接调用 ADRC_Calc。
         *
         * 参数顺序和 PID_Calc 保持一致：
         * ref = 当前反馈值
         * set = 当前设定值
         */
        out = ADRC_Calc(&adrc, measured_speed_samples[i], 3660.6f);

        /* 第四步：把计算结果送给执行器。 */
        demo_send_motor_current(out);

        /*
         * 如果你还想看内部量，ADRC 对象里直接保留了：
         * adrc.set
         * adrc.fdb
         * adrc.out
         * adrc.delta_out
         * adrc.disturbance
         */
        demo_print_state(i, &adrc);
    }

    /* 第五步：如果需要停机或清空内部状态，调用 ADRC_clear。 */
    ADRC_clear(&adrc);

    /*
     * 所以最小替换用法就是：
     *
     *   adrc_type_def adrc;
     *   adrc_param_t param;
     *   demo_prepare_adrc_param(&param);
     *   ADRC_init(&adrc, &param);
     *   out = ADRC_Calc(&adrc, ref, set);
     *   ADRC_clear(&adrc);
     *
     * 如果后面你还想在线改参数，可以再加：
     *   ADRC_set_param(&adrc, &param);
     */

    return 0;
}
