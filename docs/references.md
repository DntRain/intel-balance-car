# 参考资料

> 老师提供的官方参考链接待补充（任务书中"参考:"为空白）。以下为初步调研结果。

## OpenClaw（RDK X5 / 导航层）

- rosclaw —— ROS2 与 OpenClaw 的桥接插件层：https://github.com/PlaiPin/rosclaw
- OpenClaw 在 RDK X5 上运行演示：https://www.youtube.com/watch?v=gwTokGHI5t0
- OpenClaw 与 ROS2 集成（自然语言 → 机器人动作）：https://blog.csdn.net/RadyGo/article/details/159284374
- 环境组合（社区实践）：Ubuntu 22.04（RDK 官方适配版）+ ROS2 Humble + OpenClaw

## esp-claw（ESP32-S3 / 交互层）

- 官方仓库（中文 README）：https://github.com/espressif/esp-claw
- 支持芯片列表（ESP32-S3/P4/C5，≥8MB Flash + 8MB PSRAM）：https://esp-claw.com/en/tutorial/supported-list/
- 部署实践（阿里云+本地部署 OpenClaw 及 ESP32-S3 部署）：https://developer.aliyun.com/article/1714174
- 关键信息：基于 ESP-IDF v5.5；skills 用 Lua 编写、运行时热加载；Wi-Fi 与 API key 配置在主头文件

## RDK X5

- 上手配置与测试（社区）：https://blog.csdn.net/qq_42978535/article/details/144671463

## 待补充

- [ ] 老师提供的 OpenClaw 安装参考
- [ ] 老师提供的 esp-claw 安装参考
- [ ] 雷达型号对应的 ROS2 驱动包
- [ ] TFT 屏驱动 IC 数据手册
- [ ] 立创 EDA 中 ESP32-S3 模组 / TFT 排针封装库链接
