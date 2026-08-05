#include "gw_gray.h"

// ==================== 8路灰度引脚物理映射 ====================
// 根据实际硬件连线，从左到右的8颗传感器引脚依次为：
// 左1: A28 (Huidu_IN10)
// 左2: A27 (Huidu_IN9)
// 左3: B25 (Huidu_IN8)
// 左4: B24 (Huidu_IN7)
// 右1: B23 (Huidu_IN6)
// 右2: B20 (Huidu_IN5)
// 右3: A14 (Huidu_IN4)
// 右4: B15 (Huidu_IN3)

// 有效8路，极性反转（新传感器：高电平白地，低电平黑线）
#define Read_Huidu_Left1  (DL_GPIO_readPins(Huidu_IN10_PORT, Huidu_IN10_PIN) == Huidu_IN10_PIN)
#define Read_Huidu_Left2  (DL_GPIO_readPins(Huidu_IN9_PORT, Huidu_IN9_PIN) == Huidu_IN9_PIN)
#define Read_Huidu_Left3  (DL_GPIO_readPins(Huidu_IN8_PORT, Huidu_IN8_PIN) == Huidu_IN8_PIN)
#define Read_Huidu_Left4  (DL_GPIO_readPins(Huidu_IN7_PORT, Huidu_IN7_PIN) == Huidu_IN7_PIN)
#define Read_Huidu_Right1 (DL_GPIO_readPins(Huidu_IN6_PORT, Huidu_IN6_PIN) == Huidu_IN6_PIN)
#define Read_Huidu_Right2 (DL_GPIO_readPins(Huidu_IN5_PORT, Huidu_IN5_PIN) == Huidu_IN5_PIN)
#define Read_Huidu_Right3 (DL_GPIO_readPins(Huidu_IN4_PORT, Huidu_IN4_PIN) == Huidu_IN4_PIN)
#define Read_Huidu_Right4 (DL_GPIO_readPins(Huidu_IN3_PORT, Huidu_IN3_PIN) == Huidu_IN3_PIN)

uint16_t Huidu_Datas;

uint16_t Huidu_Read(void) {
  // 8路真实探头数据映射到 8 位 (bit7 ~ bit0)
  Huidu_Datas = (Read_Huidu_Left1  << 7) | 
                (Read_Huidu_Left2  << 6) |
                (Read_Huidu_Left3  << 5) | 
                (Read_Huidu_Left4  << 4) |
                (Read_Huidu_Right1 << 3) | 
                (Read_Huidu_Right2 << 2) |
                (Read_Huidu_Right3 << 1) | 
                (Read_Huidu_Right4 << 0);

  return Huidu_Datas;
}

float Huidu_Error;
float huidu_error;
float huidu_lasterror;
int Huidu_Sum;

// 8路灰度探头对应的权重 (从左至右)
// 权重可根据实际传感器间距进行微调
// 有效8路权重以0为中心严格等距对称: 
const float Huidu_Weights[8] = {-2.0f, -1.8f, -1.6f, -0.3f, 0.3,1.6f, 1.8f, 2.0};

float Huidu_Proc(uint16_t huidu_data) {
  static uint8_t White_Blind_Count = 0; //  静态变量：记录连续遇到全白的周期数
  float total_weight = 0;

  Huidu_Sum = 0;
  for (int i = 0; i < 8; i++) {
    // 经过读取宏的反相映射后：变量中的 bit 为 0 代表检测到黑线，1 代表白地
    // 因此我们检测 0 来累加压中黑线的探头数量
    // bit7 对应 i=0(权重-3.5)，bit0 对应 i=7(权重3.5)
    if (((huidu_data >> (7 - i)) & 0x01) == 0) {
      // 【终极抗噪防线：动态追踪窗 (Dynamic Tracking Window)】
      // 因为赛道是最平缓的0.5m圆弧，黑线在10ms内的物理移动距离绝不可能超过 2.5 个权重单位。
      // 如果发现某探头离上一帧的位置超过了 2.5，那它100%是场地的阴影或脏污噪点！直接抛弃！
      float dist = fabs(Huidu_Weights[i] - huidu_lasterror);
      
      // 条件：如果上次车还在正常循迹 (误差<4.0，包含最外侧的3.5)，且噪点离得太远 (>2.5)，则屏蔽。
      // 如果上次已经处于彻底脱轨状态 (强制设为6.0)，则放开视野，允许任何探头将车拉回。
      if (fabs(huidu_lasterror) >= 4.0f || dist <= 2.5f) {
        Huidu_Sum++;
        total_weight += Huidu_Weights[i];
      }
    }
  }

  // 基础误差计算
  if (Huidu_Sum >= 1 && Huidu_Sum <= 7) // 正常循迹，探测到1~7个黑点都算有效线宽
  {
    White_Blind_Count = 0; // 检测到有效信号，清零干扰计数器

    // 加权平均计算误差
    Huidu_Error = total_weight / Huidu_Sum;

    // 核心算法优化：前驱车(FWD)极度敏捷，改用完全线性的映射，保证每两个探头间的差值绝对相等。
    // 放弃激进的三次曲线，防止在最外侧(2.5 -> 3.5)产生过大的数学跃变。
    Huidu_Error = Huidu_Error; 

  } else if (Huidu_Sum > 7) // 探头触发过多，可能压到十字路口或者大面积黑块
  {
    White_Blind_Count = 0;         // 检测到路口特征，清零计数器
    Huidu_Error = huidu_lasterror; // 冲过路口时维持原有误差直行
  } else if (Huidu_Sum == 0)       // 丢线（全白）
  {
    White_Blind_Count++; // 累计全白周期

    // 抗局部干扰逻辑 (盲区补偿)
    // 灰度灯之间有物理间隔，线经常卡在两个灯中间导致全白(Huidu_Sum=0)
    // 放大容忍度到 30 个周期 (300ms)
    if (White_Blind_Count <= 10) {
      // 【致命 Bug 修复】：如果小车处于急转弯边缘（上一次误差绝对值
      // > 2.0），此时全白绝不是因为卡在灯缝里！而是完全飞出了探头！
      // 解决办法：如果是从边缘飞出的，必须瞬间拉爆误差，强行激发极大的
      // D 项差速把车头拽回来！因为前驱车转向灵活，最大拉回力从 8.0 降到 6.0，防过充。
      if (huidu_lasterror > 2.0f) {
        Huidu_Error = 2.7f; // 瞬间拉高，激发 Kd
      } else if (huidu_lasterror < -2.0f) {
        Huidu_Error = -2.7f;
      } else {
        // 只有误差很小时全白，才说明是卡在中间缝隙了，此时平滑过渡
        Huidu_Error = huidu_lasterror;
      }
    } else {
      // 只有当连续超过 300ms 还没碰到任何黑线，才真正判定为物理脱轨
      if (huidu_lasterror > 0)
        Huidu_Error = 2.7f; // 施加补偿力矩强制纠偏
      else if (huidu_lasterror < 0)
        Huidu_Error = -2.7f;
      else
        Huidu_Error = 0;
    }
  }

  // 【全局指数移动平均 (EMA) 滤波】
  // 恢复轻量级滤波 (0.6新 / 0.4旧)，保留敏捷性的同时防止突刺
  Huidu_Error = Huidu_Error * 0.4f + huidu_lasterror * 0.6f;

  huidu_error = Huidu_Error;
  huidu_lasterror = huidu_error;

  return huidu_error;
}
