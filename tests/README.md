# Tests

本目录存放各模块的**独立验证程序**，而不是单元测试。

每个子目录是一个可单独构建烧录的 ESP-IDF 小工程，或者一份"把 `app_main` 替换成测试代码即可运行"的片段。

| 目录 | 目的 | 怎么用 |
|---|---|---|
| `esp_core/` | TWAI loopback、GPIO、基本外设自测 | 把 `test_twai_loopback.cpp` 内容粘到 `main/app_main.cpp` 即可烧录 |
| `motor_rso2/` | RS02 使能 + 反馈读回测试 | 同上，把 `test_motor_enable.cpp` 当作 app_main |
| `imu_witmotion/` | IMU 帧解析自测 | 同上 |

> 后续接入 Unity / ESP-IDF test framework 后，再把这些迁移成正式 unit test。
