# ESP32-S3 交互层（esp-claw）

阶段 4 实现。基于乐鑫官方 [esp-claw](https://github.com/espressif/esp-claw)。

## 硬件要求（esp-claw 硬性要求）

- ESP32-S3，**≥8MB Flash + 8MB PSRAM**（下单/验货时确认模组型号，如 N16R8）
- 外设：I2S 麦克风、功放+扬声器、TFT 液晶屏、舵机 ×N（接口经 pcb/ 载板引出）

## 安装流程（待老师参考资料到位后补全 ⚠️）

1. 安装 ESP-IDF **v5.5**
2. 克隆 espressif/esp-claw，配置 Wi-Fi 与 LLM API key（主头文件中）
3. `idf.py build flash monitor`
4. 通过聊天下发/热加载 Lua skills

## skills/ 规划

| skill | 功能 | 硬件 |
|---|---|---|
| `voice` | 拾音 → 对话 → TTS 播报 | 麦克风、扬声器 |
| `expression` | 表情动画（待机/聆听/说话/开心/困惑） | TFT |
| `motion` | 肢体动作（挥手、点头、转头） | 舵机 |

三者联动：对话状态机驱动表情与动作（说话时嘴部动画 + 手势）。

## 待确认

- [ ] 与 RDK X5 侧 OpenClaw 的对接方式：Wi-Fi（网络层对接）还是 UART
- [ ] TFT 驱动 IC 型号与 esp-claw 显示支持情况
