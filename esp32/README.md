# ESP32-S3 交互层（esp-claw）

阶段 4 实现。基于乐鑫官方 [esp-claw](https://github.com/espressif/esp-claw)。

## 硬件要求（esp-claw 硬性要求）

- ESP32-S3，**≥8MB Flash + 8MB PSRAM**（下单/验货时确认模组型号，如 N16R8）
- 外设：I2S 麦克风、功放+扬声器、TFT 液晶屏、舵机 ×N（接口经 pcb/ 载板引出）

## 安装流程（官方中文教程：https://esp-claw.com/zh-cn/tutorial/ ）

**推荐路径（先跑通再定制）：**
1. 按教程 BOM 组装（面包板阶段；后续换成 pcb/ 的载板）
2. **浏览器一键烧录**：esp-claw.com 在线烧录工具，无需本地编译
3. Web 控制台完成 Wi-Fi 与设备配置
4. Skills Lab 里开发/热加载 Lua skills

**定制路径（需要改固件时）：**
1. 安装 ESP-IDF **v5.5**，克隆 espressif/esp-claw
2. 主头文件配置 Wi-Fi 与 LLM API key
3. `idf.py build flash monitor`

## skills/ 规划

| skill | 功能 | 硬件 |
|---|---|---|
| `voice` | 拾音 → 对话 → TTS 播报 | 麦克风、扬声器 |
| `expression` | 表情动画（待机/聆听/说话/开心/困惑） | TFT |
| `motion` | 肢体动作（挥手、点头、转头） | 舵机 |

三者联动：对话状态机驱动表情与动作（说话时嘴部动画 + 手势）。

## 待确认

- [ ] 与 RDK X5 侧 OpenClaw 的对接方式：Wi-Fi（网络层对接）还是 UART
- [ ] TFT 驱动 IC 型号与 esp-claw 显示支持情况（对照 esp-claw.com 支持列表）
- [ ] 本车 ESP32-S3 模组的 Flash/PSRAM 规格验货（需 ≥8MB+8MB）
