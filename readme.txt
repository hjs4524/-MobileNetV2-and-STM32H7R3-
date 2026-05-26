STM32H7R3 OpenMV 机械臂抓取项目接力说明
========================================

一、项目目标
------------

本工程基于正点原子 H7R3 开发板，从原始“通用定时器 PWM 输出实验”改造为：

1. OpenMV 摄像头识别桌面目标色块，输出目标中心像素坐标。
2. STM32H7R3 通过串口接收 OpenMV 坐标。
3. STM32 将像素坐标通过 2D 手眼标定转换为桌面坐标。
4. STM32 对 4 自由度机械臂做逆运动学求解。
5. 6 路舵机执行抓取、搬运、投放、回 Home 的完整流程。

当前工程定位：代码层面的闭环抓取 demo 已搭好并编译通过，后续重点是实物联调、手眼标定和舵机参数微调。


二、工程结构
------------

主要文件：

1. User/main.c
   - STM32 主流程。
   - 初始化 MPU/cache/HAL/时钟/延时/串口/LED/舵机。
   - 接收 OpenMV 行数据。
   - 解析 `cx,cy` 或 `X=cx,Y=cy`。
   - 对目标像素做稳定判定。
   - 使用仿射标定将像素点转换为桌面坐标。
   - 调用机械臂逆解。
   - 执行抓取、投放、回 Home。

2. User/openmv_single_blob_uart.py
   - OpenMV 端脚本。
   - 使用 RGB565 + QVGA。
   - 在 ROI 内找单个色块。
   - 有目标时发送 `cx,cy\r\n`。
   - 无目标时周期发送 `NONE\r\n`。

3. Drivers/SYSTEM/usart/usart.c / usart.h
   - USART1：调试打印口，PB14/PB15。
   - USART2：OpenMV 通信口，PD5/PD6。
   - USART2 使用中断逐字节接收，按 `\r\n` 拼成一行。

4. Drivers/BSP/TIMER/gtim.c / gtim.h
   - 6 路舵机 PWM 底层。
   - TIM2 输出 4 路 PWM：PA0、PA1、PB10、PB11。
   - TIM5 输出 2 路 PWM：PA2、PA3。
   - PWM 周期 20ms，计数基准 1MHz，比较值单位可按 us 理解。

5. Drivers/BSP/SERVO/servo.c / servo.h
   - 0 到 180 度舵机角度转换为 500 到 2500us 脉宽。
   - 单舵机和多舵机缓动控制。
   - 舵机联调测试接口。
   - 4DOF 机械臂逆运动学。
   - 默认机械臂尺寸参数：
     base_height = 120mm
     upper_arm   = 105mm
     forearm     = 105mm
     wrist       = 80mm

6. Projects/MDK-ARM/atk_h7r3.uvprojx
   - Keil MDK 工程文件。
   - 已包含 main.c、usart.c、gtim.c、servo.c 等当前项目核心文件。

7. Output/
   - Keil 构建输出目录。
   - 最近一次构建记录显示 0 Error(s), 0 Warning(s)。


三、硬件与通信约定
------------------

开发板：正点原子 H7R3，MCU 为 STM32H7R3 系列。

LED：
1. LED0 - PD14
2. LED1 - PC0

调试串口：
1. USART1_TX - PB14
2. USART1_RX - PB15
3. 波特率：115200

OpenMV 串口：
1. STM32 USART2_TX - PD5
2. STM32 USART2_RX - PD6
3. OpenMV 脚本使用 UART_ID = 3
4. 波特率：115200
5. 数据格式：
   - 有目标：`cx,cy\r\n`
   - 无目标：`NONE\r\n`

舵机 PWM：
1. Servo 0 - TIM2_CH1 - PA0
2. Servo 1 - TIM2_CH2 - PA1
3. Servo 2 - TIM2_CH3 - PB10
4. Servo 3 - TIM2_CH4 - PB11
5. Servo 4 - TIM5_CH3 - PA2
6. Servo 5 - TIM5_CH4 - PA3

注意：Servo 5 当前作为夹爪控制使用，main.c 中开爪约 110 度，闭爪约 20 度。


四、当前主流程
--------------

1. 系统初始化。
2. 舵机全部回 90 度。
3. 夹爪打开。
4. 机械臂移动到 Home 位：
   - x = 220mm
   - y = 0mm
   - z = 140mm
   - pitch = -20deg
5. 循环读取 OpenMV 数据。
6. 收到 `NONE` 时清空目标状态，等待下一个目标。
7. 收到像素坐标后，要求目标连续稳定若干帧：
   - TARGET_STABLE_COUNT_REQUIRED = 5
   - TARGET_PIXEL_JITTER_TOLERANCE = 2 像素
8. 目标锁定后，像素坐标转换为桌面坐标。
9. 执行动作：
   - 回 Home
   - 打开夹爪
   - 移动到目标上方
   - 下探到抓取高度
   - 闭合夹爪
   - 抬起目标
   - 移动到投放区上方
   - 到投放高度
   - 打开夹爪
   - 抬起
   - 回 Home
10. 一轮完成后等待目标离开视野，避免重复抓同一个目标。


五、当前标定与动作参数
----------------------

像素到桌面坐标使用 2D 仿射标定，参数在 User/main.c 中：

desk_x = CALIB_A11 * px + CALIB_A12 * py + CALIB_A13
desk_y = CALIB_A21 * px + CALIB_A22 * py + CALIB_A23

当前参数来自 2026-04-22 的 4 个标定点：

1. pixel(156,102) -> desk(240,80)
2. pixel(156,133) -> desk(240,40)
3. pixel(215,95)  -> desk(220,80)
4. pixel(217,127) -> desk(240,45)

注意：代码注释中已说明当前 4 个点存在明显非线性或测量误差，标定只适合 bring-up，后续应增加更多标定点重新拟合。

当前抓取/投放参数：

1. 目标上方高度 TARGET_Z_ABOVE = 135mm
2. 抓取接近/抓取后抬起高度 TARGET_Z_VERTICAL_ABOVE = 60mm
3. 抓取高度 TARGET_Z_GRAB = 10mm
4. 抓取上方 pitch = -30deg
5. 优先抓取 pitch = -90deg，即夹爪垂直向下
6. 如果 -90deg 不可达，程序会从 -85deg、-80deg 逐步尝试到 -35deg，选择最接近垂直且可达的抓取姿态
7. 投放区 DROP_X = 200mm
8. 投放区 DROP_Y = -120mm
9. 投放上方 DROP_Z_ABOVE = 145mm
10. 投放高度 DROP_Z_RELEASE = 130mm
11. 投放 pitch = -25deg

垂直抓取可达性说明：

1. 当前逆解中 pitch = -90deg 表示末端/夹爪垂直向下。
2. 在当前默认连杆参数 upper_arm = 105mm、forearm = 105mm、wrist = 80mm、base_height = 120mm 下，低位垂直抓取时，肩关节到腕点最大可达约 210mm。
3. 因为垂直向下时腕长主要占用 Z 方向，不再帮忙减少水平距离，所以低位垂直抓取的桌面水平半径约不能超过 208mm。
4. 用户 2026-04-24 提供的串口日志中，识别出的目标常被标定到 x=236~253mm、y=7~78mm，水平半径约 247~256mm，因此严格 -90deg 垂直抓取会全部失败。
5. 为先恢复可抓取能力，程序已改为优先垂直抓取；若严格垂直不可达，则自动选择最接近垂直的可达 pitch，并打印 `vertical pick unreachable, use nearest reachable pitch=...`。
6. 若实物机械臂连杆长度不是当前默认值，应优先修正 servo.c 中 base_height / upper_arm / forearm / wrist，再评估垂直抓取工作区。


六、当前完成情况
----------------

已完成：

1. STM32 基础工程保留并能构建。
2. 6 路舵机 PWM 驱动完成。
3. 舵机角度设置、缓动、测试接口完成。
4. 4DOF 逆运动学接口完成。
5. USART1 调试打印完成。
6. USART2 OpenMV 接收完成。
7. OpenMV 单色块识别和串口发送脚本完成。
8. STM32 主流程抓取 demo 完成。
9. 手眼标定仿射参数已接入。
10. Keil 最近一次构建：0 Error(s), 0 Warning(s)。

待验证/待优化：

1. 实物串口线序和电平是否正确。
2. OpenMV UART3 与 STM32 USART2 是否正确互连。
3. OpenMV 阈值 thresholds 是否适合当前目标颜色和光照。
4. ROI 是否覆盖实际抓取区域。
5. 6 路舵机编号与真实机械臂关节是否一一对应。
6. 每个舵机方向 angle_dir 是否需要调整。
7. 每个舵机零偏 angle_offset_deg 是否需要实测修正。
8. 每个关节限位 angle_min_deg / angle_max_deg 是否符合实际机械结构。
9. 机械臂连杆参数是否与实物一致。
10. 抓取高度 TARGET_Z_GRAB 是否安全且能夹到目标。
11. 投放点 DROP_X / DROP_Y / DROP_Z_RELEASE 是否合适。
12. 当前 4 点手眼标定精度不足，建议采集更多点重新拟合。
13. readme.txt 原始内容已替换为项目接力文档；以后工程改动都应在本文末尾追加记录。


七、接力建议
------------

新对话接手时建议先读：

1. readme.txt
2. User/main.c
3. Drivers/BSP/SERVO/servo.c
4. Drivers/BSP/TIMER/gtim.c
5. Drivers/SYSTEM/usart/usart.c
6. User/openmv_single_blob_uart.py

推荐下一步调试顺序：

1. 只开 OPENMV_UART_DEBUG_ONLY，确认 STM32 能稳定收到 OpenMV 坐标。
2. 单独测试 6 路舵机编号、方向、角度范围。
3. 修正 servo.c 中默认机械臂参数、方向、零偏。
4. 测试 Home 位、垂直上安全位、投放位是否安全。
5. 采集更多像素点和桌面坐标点，重算仿射标定。
6. 再启用完整抓取流程。


八、编译约定
------------

每次修改工程代码或 Keil 工程配置后，都使用以下 Keil 路径编译：

D:\keil5\UV4\UV4.exe

当前工程文件：

Projects\MDK-ARM\atk_h7r3.uvprojx

建议命令，工作目录为 `Projects\MDK-ARM`：

`D:\keil5\UV4\UV4.exe -b atk_h7r3.uvprojx -t atk_h7r3 -j0 -o ..\..\Output\codex_build.log`

编译完成后，应在本文“修改记录 / 接力日志”中追加编译结果。


九、修改记录 / 接力日志
-----------------------

2026-04-24

1. 读取并梳理当前工程。
2. 确认工程已由原始 PWM 实验改造成 OpenMV + STM32H7R3 + 6 路舵机机械臂抓取 demo。
3. 确认主流程、OpenMV 脚本、USART、PWM、舵机和 4DOF 逆解均已接入工程。
4. 确认 Output/atk_h7r3.build_log.htm 中最近一次 Keil 构建为 0 Error(s), 0 Warning(s)。
5. 将原始 readme.txt 替换为当前项目接力说明。
6. 后续每次修改工程，应在本节继续追加日期、改动文件、改动原因、验证结果和遗留问题。
7. 按用户要求新增编译约定：后续每次修改工程代码或 Keil 工程配置后，使用 D:\keil5\UV4\UV4.exe 编译，并记录结果。
8. 已用 D:\keil5\UV4\UV4.exe 命令行编译验证当前工程，Output/atk_h7r3.build_log.htm 显示 0 Error(s), 0 Warning(s)。本次只修改 readme.txt，hex/axf 未因工程源码变化而更新。

2026-04-24 垂直抓取姿态调整

1. 修改 User/main.c。
2. 新增 GRIPPER_VERTICAL_DOWN_PITCH_DEG = -90deg，用于表示夹爪垂直向下。
3. 将 TARGET_PITCH_GRAB_DEG 改为 -90deg。
4. 新增 TARGET_Z_VERTICAL_ABOVE = 60mm，作为垂直接近和夹紧后垂直抬起高度。
5. 新增 arm_pose_is_reachable()，在抓取前预检查垂直接近位和垂直抓取位是否可达。
6. arm_pick_at_desk_point() 调整为：高位接近 -> 垂直接近 -> 垂直下探抓取 -> 闭爪 -> 垂直抬起。
7. 如果目标点在 -90deg 垂直抓取姿态下不可达，程序打印 `vertical pick unreachable` 并取消本次抓取，避免倾斜硬抓。
8. 已按编译约定使用 D:\keil5\UV4\UV4.exe 编译，Output/atk_h7r3.build_log.htm 显示 0 Error(s), 0 Warning(s)。hex/axf 已更新。

2026-04-24 恢复远处目标可抓取能力

1. 用户提供实机串口日志，目标识别稳定，但所有目标都被标定到 x=236~253mm、y=7~78mm 附近。
2. 日志中每次都在 `vertical pick unreachable: ... pitch=-90.0` 后失败，说明识别链路正常，失败原因是严格垂直抓取姿态不可达。
3. 修改 User/main.c，新增 TARGET_PITCH_FALLBACK_MAX_DEG = -35deg 和 TARGET_PITCH_SEARCH_STEP_DEG = 5deg。
4. 新增 arm_select_pick_pitch()：优先尝试 -90deg；若不可达，则依次尝试 -85deg、-80deg、...、-35deg，选择第一个可达姿态。
5. arm_pick_at_desk_point() 改为使用选出的 pick_pitch_deg 执行接近、下探、夹取、抬起。
6. 严格垂直不可达但找到替代姿态时，串口打印 `vertical pick unreachable, use nearest reachable pitch=...`。
7. 若 -90deg 到 -35deg 全部不可达，才打印 `pick unreachable` 并放弃本次抓取。
8. 注意：这是为了先恢复可抓取能力。若最终要求所有点都严格垂直抓取，需要缩小抓取区域、移动相机/物体到机械臂近处，或修正真实连杆参数和机械安装。
9. 已按编译约定使用 D:\keil5\UV4\UV4.exe 编译，Output/atk_h7r3.build_log.htm 显示 0 Error(s), 0 Warning(s)。hex/axf 已更新。

2026-04-24 姿态解算参数检查与腕部修正

1. 用户反馈：姿态解算后的实际位置偏差很大，夹爪斜向上夹取。
2. 检查 Drivers/BSP/SERVO/servo.c：
   - base_height = 120mm、upper_arm = 105mm、forearm = 105mm、wrist = 80mm 都是默认估算值，必须按实物轴心重新测量。
   - angle_offset_deg[] 默认全是 90deg，angle_dir[] 默认全是 +1，这只是占位配置，不可能天然匹配真实机械臂。
   - 逆解 pitch 约定：0deg 表示末端水平向前，-90deg 表示末端垂直向下。
3. 代入用户日志中的目标点，例如 x=241.8、y=43.7、z=50、pitch=-60，当前数学解会得到腕部 J4 raw 约 -71.4deg。
4. 原配置 J4 舵机输出 = 90 + rawJ4，得到约 18.6deg；Home 位 J4 也只有约 8.4deg。腕部长期贴近 0deg 下限，非常容易造成实物夹爪斜向上或翻腕方向不对。
5. 在 User/main.c 的 arm_apply_default_limits() 中临时修正 J4 腕部俯仰零偏：angle_offset_deg[3] = 180deg。这样同样目标点的 J4 会落在约 90~120deg 的合理中间区。
6. 同时发现 arm_move_to_pose() 每次移动 6 路舵机，会把第 6 路夹爪重置到 IK 固定角度 90deg；这会导致闭爪后执行抬起动作时夹爪被重新打开。
7. 修改 User/main.c：新增 g_gripper_angle_deg，gripper_open()/gripper_close() 维护当前夹爪角度，arm_move_to_pose() 下发 IK 结果前保留第 6 路夹爪角度。
8. 仍需实物标定：
   - J1~J4 每个舵机的机械零点 angle_offset_deg。
   - 每个舵机方向 angle_dir。
   - 每个关节真实安全限位 angle_min_deg / angle_max_deg。
   - base_height、upper_arm、forearm、wrist 的真实轴心尺寸。
9. 若修正后夹爪仍然越调越向上，优先检查 J4 方向；可能需要把 angle_dir[3] 从 +1 改为 -1，并重新设置 J4 零偏。
10. 已按编译约定使用 D:\keil5\UV4\UV4.exe 编译，Output/atk_h7r3.build_log.htm 显示 0 Error(s), 0 Warning(s)。hex/axf 已更新。

2026-04-24 根据工作区图片继续修正 J4 腕部方向

1. 用户提供 OpenMV 视野图和实际工作区照片：机械臂在目标纸张左侧，OpenMV 从上方斜拍，目标在纸张标定区域中部。
2. 用户反馈上一版仍然斜向上抓取，说明只把 J4 零偏改到 180deg 还不够，J4 物理方向大概率与数学模型相反。
3. 修改 User/main.c 的 arm_apply_default_limits()：
   - J4 angle_offset_deg[3] 从 180deg 改回 90deg。
   - J4 angle_dir[3] 改为 -1。
   - J4 angle_max_deg[3] 放宽到 180deg，避免 Home/投放姿态因为腕部接近 170~180deg 被误判不可达。
4. 当前 J4 映射变为：servo_j4 = 90 - raw_j4。
5. 对用户日志中的典型目标点，rawJ4 约 -60 到 -72deg，新映射后 J4 输出约 150~162deg，和上一版的 90~120deg 相比会向相反方向补偿。
6. 启动时新增串口打印：
   - `arm model: base_h=... upper=... forearm=... wrist=...`
   - `J4 map: servo=90.0 -1*raw, limit 0.0..180.0`
7. 图片也暴露出位置误差的另一来源：当前 OpenMV 标定只用了 4 个点，且目标在斜拍画面中，2D 仿射标定很容易产生明显误差。后续必须用纸上黑色方块/网格采集更多点，按机械臂底座坐标系重新标定。
8. 如果这一版 J4 方向变正确但位置仍偏，需要先修手眼标定；如果 J4 仍朝错误方向，需要提供启动日志中的 J4 map 和一次抓取时的 IK J=... 输出继续判断。
9. 已按编译约定使用 D:\keil5\UV4\UV4.exe 编译，Output/codex_build.log 和 Output/atk_h7r3.build_log.htm 均显示 0 Error(s), 0 Warning(s)。hex/axf 已更新。

2026-04-24 根据实机日志修正投放点

1. 用户提供新日志：抓取流程已经能执行到 target grab、gripper close、target lift，说明识别、抓取姿态选择和夹爪保持逻辑已生效。
2. 新失败点在投放阶段：
   - drop above: x=180 y=-120 z=145 pitch=-25 成功，但 J3=174.5、J4=179.5，已经贴近关节极限。
   - drop release: x=180 y=-120 z=120 pitch=-25 逆解失败。
3. 代入当前模型计算，drop release 失败原因是 J3 理论输出约 180.6deg，超过 angle_max_deg[2] = 180deg。
4. 修改 User/main.c：
   - DROP_X 从 180mm 改为 200mm。
   - DROP_Z_RELEASE 从 120mm 改为 130mm。
   - DROP_Y 保持 -120mm，DROP_Z_ABOVE 保持 145mm，DROP_PITCH_DEG 保持 -25deg。
5. 修改后的理论输出约为：
   - drop above: J=[59.0, 74.7, 160.8, 170.5]
   - drop release: J=[59.0, 67.7, 165.0, 167.7]
6. 新投放点比旧投放点远离 J3/J4 极限，应该能避免 `drop release IK failed`。
7. 已按编译约定使用 D:\keil5\UV4\UV4.exe 编译，Output/codex_build.log 显示 0 Error(s), 0 Warning(s)。hex/axf 已更新。

2026-04-24 抓取阶段将抓夹朝向轴直接固定角度

1. 用户进一步澄清：要控制的是末端抓夹整体朝上/朝下摆动的那个轴，并要求在抓取阶段直接固定该轴角度，由其他关节去配合抓取。
2. 初版实现误将抓取固定角度施加到了 `solution.joint_deg[3]`，用户上板后反馈“末端这个轴并没有变化”，说明固定的不是目标物理轴。
3. 因此本次修正为对更末端的工具朝向轴施加固定角度：
   - 保留 `GRAB_WRIST_SERVO_DEG = 90.0f`
   - 在 `arm_move_to_pose_ex(...)` 中，当 `prefer_wrist_perpendicular != 0` 时，改为执行 `solution.joint_deg[4] = GRAB_WRIST_SERVO_DEG`
4. 当前启用该固定角度的阶段仍然只有：
   - `target pick approach`
   - `target grab`
   - `target lift`
   其余动作如 `home / above / drop` 不受影响。
5. 这样做的目标，是先验证“你圈出来的那个更末端轴”是否确实由当前第 5 路舵机控制；如果这次上板后该轴开始变化，就说明之前是改错轴了，后续只需继续调整 `GRAB_WRIST_SERVO_DEG` 即可。

2026-04-24 根据实物现象确认上一版改到了错误的轴

1. 用户上板后确认：上一版改动使现在图片中重新圈出的那个轴发生了转动。
2. 这说明：
   - `solution.joint_deg[4]` 确实对应这次新圈出的中间轴
   - 但它仍然不是用户真正想固定的“末端抓夹朝向轴”
3. 因此本次撤销上一版把抓取固定角度施加到 `joint_deg[4]` 的修改，恢复为施加到 `joint_deg[3]`：
   - `solution.joint_deg[3] = GRAB_WRIST_SERVO_DEG`
4. 当前结论是：
   - `joint_deg[4]` 对应的是更靠前一层中间腕部轴
   - 用户真正想控制的抓夹朝向，仍更接近 `joint_deg[3]` 这一层
5. 这次回退的目的，是先回到上一个更接近目标轴的控制对象，避免在错误的物理轴上继续调角度。

2026-04-24 增加启动逐轴点动识别模式

1. 经过多轮实物观察，已确认仅凭肉眼描述仍难以稳定对应 `joint_deg[3] / [4] / [5]` 与真实末端物理轴的关系。
2. 为避免继续在错误轴上调试，本次增加一个临时的启动逐轴识别模式。
3. 修改 [User/main.c](e:\STM32H7机械臂控制\机械臂抓取代码1\User\main.c)：
   - 新增 `AXIS_IDENTIFY_STARTUP_DEMO = 1`
   - 新增 `arm_identify_end_axes()`
4. 启动后会按顺序执行：
   - `identify joint_deg[3]`：将第 4 路舵机从 90 打到 60，再打到 120，再回 90
   - `identify joint_deg[4]`：将第 5 路舵机从 90 打到 60，再打到 120，再回 90
   - `identify joint_deg[5]`：执行夹爪开合测试
5. 串口会输出：
   - `axis identify demo start`
   - `identify joint_deg[3]`
   - `identify joint_deg[4]`
   - `identify joint_deg[5]`
   - `axis identify demo end`
6. 这样做的目的，是在进入正常抓取流程前，先把每一路末端舵机和真实物理轴一一对应清楚，后续才能准确固定用户真正想控制的那个轴。

2026-04-24 确认抓夹朝向轴为 joint_deg[4]

1. 用户完成启动逐轴识别后确认：真正想控制的末端抓夹朝向轴，对应 `identify joint_deg[4]`。
2. 因此本次将抓取阶段固定角度的作用对象正式切换到 `joint_deg[4]`。
3. 修改 [User/main.c](e:\STM32H7机械臂控制\机械臂抓取代码1\User\main.c)：
   - 在 `arm_move_to_pose_ex(...)` 中，当抓取阶段标志有效时，改为执行：
     - `solution.joint_deg[4] = GRAB_WRIST_SERVO_DEG`
   - 不再对 `joint_deg[3]` 施加该固定角度。
4. 同时将 `AXIS_IDENTIFY_STARTUP_DEMO` 从 `1` 改回 `0`，避免后续每次启动都先跑逐轴识别动作，恢复正常工作流程。
5. 这样后续若要继续调整抓夹朝向，只需要直接修改：
   - `GRAB_WRIST_SERVO_DEG`
   即可。

2026-04-24 修正抓取阶段 joint_deg[4]“看起来没动”的问题

1. 用户反馈：虽然代码已经切换为在抓取阶段固定 `joint_deg[4]`，但实物看起来仍像是 `joint_deg[3]` 在动，而 `joint_deg[4]` 没有变化。
2. 分析后确认，原因并不是又改错了轴，而是：
   - `joint_deg[4]` 在当前工程里平时本来就常接近 `90deg`
   - 上一版 `GRAB_WRIST_SERVO_DEG` 也正好设置为 `90deg`
   - 因此抓取阶段虽然确实执行了 `solution.joint_deg[4] = 90`，但视觉上等于“没变化”
3. 修改 [User/main.c](e:\STM32H7机械臂控制\机械臂抓取代码1\User\main.c)：
   - 将 `GRAB_WRIST_SERVO_DEG` 从 `90.0f` 改为 `110.0f`
   - 在抓取阶段固定该轴时新增串口打印：
     - `grab wrist fixed: joint_deg[4]=110.0`
4. 这样本次上板验证时，可以同时通过：
   - 实物动作
   - 串口打印
   两方面确认当前真正被固定的是否就是 `joint_deg[4]`

2026-04-24 抓取阶段冻结 joint_deg[3] 仅驱动 joint_deg[4]

1. 用户进一步明确要求：
   - 抓取时先停止 `identify joint_deg[3]` 这一路的转动
   - 抓取时只控制 `identify joint_deg[4]` 转动
2. 为了最小改动验证，本次直接在抓取阶段同时覆盖这两路输出：
   - `joint_deg[3]` 固定为常量，不再跟随 IK 改变
   - `joint_deg[4]` 固定为抓取朝向角
3. 修改 [User/main.c](e:\STM32H7机械臂控制\机械臂抓取代码1\User\main.c)：
   - 新增 `GRAB_PREWRIST_SERVO_DEG = 90.0f`
   - 保留 `GRAB_WRIST_SERVO_DEG = 110.0f`
   - 在 `arm_move_to_pose_ex(...)` 中，当抓取阶段标志有效时执行：
     - `solution.joint_deg[3] = GRAB_PREWRIST_SERVO_DEG`
     - `solution.joint_deg[4] = GRAB_WRIST_SERVO_DEG`
4. 串口打印改为：
   - `grab axes fixed: joint_deg[3]=90.0 joint_deg[4]=110.0`
5. 这样本次上板时，若仍看到 `identify joint_deg[3]` 在抓取阶段明显变化，就说明问题不在这层覆盖逻辑；否则就能直接开始只调 `joint_deg[4]` 的抓夹朝向角。

2026-04-24 下调抓取阶段 joint_deg[4] 固定角

1. 用户要求继续试调抓取阶段 `joint_deg[4]` 的固定角，将：
   - `GRAB_WRIST_SERVO_DEG`
   从 `110.0f` 改为 `70.0f`
2. 这样当前抓取阶段固定值变为：
   - `joint_deg[3] = 90.0`
   - `joint_deg[4] = 70.0`
3. 本次改动属于单参数微调，目的就是继续寻找“抓夹真正垂直桌面”的实物角度。

2026-04-24 继续下调抓取阶段 joint_deg[4] 固定角

1. 用户继续要求下调抓取阶段 `joint_deg[4]` 的固定角，将：
   - `GRAB_WRIST_SERVO_DEG`
   从 `70.0f` 改为 `40.0f`
2. 这样当前抓取阶段固定值变为：
   - `joint_deg[3] = 90.0`
   - `joint_deg[4] = 40.0`
3. 本次仍属于同一条调参链路中的单参数微调，目标是继续逼近抓夹垂直桌面的实物角度。

2026-04-24 按用户要求回退姿态解算改动

1. 用户明确要求：把近期为了“抓取阶段更下压”而加入的姿态解算修改原封不动改回原来的实现。
2. 本次仅回退“姿态解算/逆解分支选择”相关内容，不回退与此无关的其他参数调整和抓爪保持修正。
3. 修改 [Drivers/BSP/SERVO/servo.h](e:\STM32H7机械臂控制\机械臂抓取代码1\Drivers\BSP\SERVO\servo.h)：
   - 删除 `arm4dof_config_t` 中新增的 `elbow_preference` 字段。
4. 修改 [Drivers/BSP/SERVO/servo.c](e:\STM32H7机械臂控制\机械臂抓取代码1\Drivers\BSP\SERVO\servo.c)：
   - 删除 `arm4dof_try_solve_branch()` 双分支求解辅助函数。
   - `arm4dof_load_default_config()` 删除 `config->elbow_preference = 0`。
   - `arm4dof_solve()` 恢复为原来的单分支逆解：固定使用 `sin_j3 = +sqrt(1-cos_j3^2)` 这一支，不再根据“下压偏好”在两支解之间选择。
5. 修改 [User/main.c](e:\STM32H7机械臂控制\机械臂抓取代码1\User\main.c)：
   - 删除 `PICK_WRIST_SERVO_OVERRIDE_EN`、`PICK_WRIST_DOWN_SERVO_DEG`、`PICK_WRIST_OVERRIDE_PITCH_DEG`、`PICK_ELBOW_DOWNWARD_PREFERENCE` 等为姿态调试加入的宏。
   - `arm_move_to_pose()` 恢复为直接使用 `arm4dof_solve(config, ...)`，不再复制 `work_cfg` 并动态设置肘部偏好。
   - 删除启动日志中的 `pick elbow preference...` 和抓取阶段腕部覆盖相关打印。
6. 回退后，当前工程的姿态解算重新回到最初的“单支逆解 + 由关节映射参数决定舵机方向”的状态，便于在统一基线上继续排查。

2026-04-24 更新机械臂实测几何参数

1. 用户补充了机械臂的实测尺寸：
   - `base_height = 100mm`
   - `upper_arm = 130mm`
   - `forearm = 95mm`
   - `wrist = 85mm`
2. 其中 `wrist` 按当前 IK 模型的定义，表示“控制夹爪上下摆动的腕部俯仰轴”到“夹爪实际抓取点”的距离，不是夹爪开合轴，也不是末端自转轴。
3. 修改 [Drivers/BSP/SERVO/servo.c](e:\STM32H7机械臂控制\机械臂抓取代码1\Drivers\BSP\SERVO\servo.c) 中 `arm4dof_load_default_config()`：
   - `config->base_height` 从 `120.0f` 改为 `100.0f`
   - `config->upper_arm` 从 `105.0f` 改为 `130.0f`
   - `config->forearm` 从 `105.0f` 改为 `95.0f`
   - `config->wrist` 从 `80.0f` 改为 `85.0f`
4. 这次改动只更新几何模型参数，不修改抓取流程和姿态选择逻辑；目的是让 IK 先建立在更接近实物的尺寸上。

2026-04-24 按姿态解算演示程序改写逆解

1. 用户提供了参考文件 [姿态解算演示程序.c](e:\STM32H7机械臂控制\【算法】C语言演示程序\姿态解算演示程序.c)，要求把当前工程的姿态解算改成与该演示程序一致的思路。
2. 参考演示程序后确认，其核心逻辑不是“给定末端 pitch 直接反解”，而是：
   - 枚举总姿态角 `j_all = 0..180deg`
   - 对每个 `j_all` 计算对应的 `j2/j3/j4`
   - 筛掉越界解
   - 从全部可行解里取中间那一支
3. 修改 [Drivers/BSP/SERVO/servo.c](e:\STM32H7机械臂控制\机械臂抓取代码1\Drivers\BSP\SERVO\servo.c) 中 `arm4dof_solve()`：
   - 删除原来“由 `target.pitch_deg` 直接参与腕部投影”的逆解公式。
   - 改为两遍扫描 `posture_index = 0..180` 的方式，按演示程序同样的几何关系计算：
     - `L = len - wrist * sin(posture)`
     - `H = z - wrist * cos(posture) - base_height`
     - `j3` 由余弦定理求出
     - `j2` 由 `K1/K2` 关系求出
     - `j4 = posture - j2 - j3`
   - 第一遍统计全部可行解数量，第二遍取中位可行解。
   - 继续保留现有舵机零偏、方向和限位检查。
4. 修改 [Drivers/BSP/SERVO/servo.h](e:\STM32H7机械臂控制\机械臂抓取代码1\Drivers\BSP\SERVO\servo.h)：
   - 在 `arm4dof_solution_t` 中新增 `posture_deg`，用于记录最终选中的总姿态角。
5. 修改 [User/main.c](e:\STM32H7机械臂控制\机械臂抓取代码1\User\main.c)：
   - `arm_move_to_pose()` 的串口日志改为同时打印：
     - 请求值 `req_pitch`
     - 实际扫描选中的 `solve_posture`
6. 当前兼容性说明：
   - `target.pitch_deg` 仍然保留在接口中，方便现有调用流程不改。
   - 但按演示程序的求解思路，当前真正决定姿态的是扫描得到的 `solve_posture`，不再是原先直接指定的 `pitch_deg`。
7. 这样改的直接目的，是把工程里的逆解切换到和参考演示程序同一套构型选择方式上，便于后续按同一算法继续标定机械参数和关节映射。
8. 已按编译约定使用 D:\keil5\UV4\UV4.exe 编译，Output/codex_build.log 显示 0 Error(s), 0 Warning(s)。hex/axf 已更新。

2026-04-24 抓取阶段改为搜索最低可达高度

1. 用户上板日志显示：按演示程序同款扫描式逆解后，`HOME` 位已经可以求解，但桌面抓取阶段会直接失败：
   - `pick unreachable: x=247.7 y=35.0 z=10.0 pitch -90.0..-35.0`
2. 进一步核算当前模型后确认：对这类较远目标点，当前机械尺寸和关节限位下，`z=10mm` 到 `z=60mm` 区间没有任何可行解，最低大约要到 `z≈80mm` 才开始出现可行姿态。
3. 由于当前 `arm4dof_solve()` 已经改成扫描 `solve_posture` 的方式，原 `main.c` 中“按 pitch 搜索抓取姿态”的逻辑已经不再准确，会误导调试。
4. 修改 [User/main.c](e:\STM32H7机械臂控制\机械臂抓取代码1\User\main.c)：
   - 新增：
     - `PICK_Z_SEARCH_STEP_MM = 5mm`
     - `PICK_APPROACH_MARGIN_MM = 20mm`
   - 删除旧的 `arm_select_pick_pitch()` 逻辑，改为 `arm_select_pick_height()`：
     - 从 `TARGET_Z_GRAB` 开始，按 5mm 递增搜索
     - 一直搜索到 `TARGET_Z_ABOVE`
     - 找到第一个可行的抓取高度 `pick_grab_z`
     - 同时给出对应接近高度 `pick_approach_z = min(grab_z + 20, TARGET_Z_ABOVE)`
   - `arm_pick_at_desk_point()` 改为使用搜索出的 `pick_grab_z / pick_approach_z`
5. 新日志行为：
   - 若 `z=10` 可达：
     - `target lowest grab reachable: z=10.0 approach_z=30.0`
   - 若目标点过远，低位不可达但较高位置可达：
     - `target low unreachable, use lowest reachable grab_z=80.0 approach_z=100.0`
   - 若整个搜索区间都无解：
     - `pick unreachable: x=... y=... no reachable z in 10.0..135.0`
6. 这样改的目的不是直接解决桌面抓取，而是先把“为什么抓不到”变成可量化的串口诊断，方便继续判断问题是工作区太远、关节限位过紧，还是手眼标定把点映射得过远。

2026-04-24 更新机构重测参数

1. 用户根据机械臂实物结构重新测量并给出新的机构参数：
   - `base_height = 130mm`
   - `upper_arm = 130mm`
   - `forearm = 130mm`
   - `wrist = 80mm`
   - `P = 0`
2. 当前工程中的扫描式姿态解算模型还没有单独引入 `P` 参数，因此本次 `P = 0` 与现有实现一致，即继续按“底座旋转中心与俯仰平面无额外水平偏移”处理。
3. 修改 [Drivers/BSP/SERVO/servo.c](e:\STM32H7机械臂控制\机械臂抓取代码1\Drivers\BSP\SERVO\servo.c) 中 `arm4dof_load_default_config()`：
   - `config->base_height` 从 `100.0f` 改为 `130.0f`
   - `config->upper_arm` 保持 / 更新为 `130.0f`
   - `config->forearm` 从 `95.0f` 改为 `130.0f`
   - `config->wrist` 从 `85.0f` 改为 `80.0f`
4. 这一步的目的是先把代码里的默认机构模型和用户当前重新确认的实物尺寸对齐，避免在错误的几何基础上继续调试抓取姿态。

2026-04-24 扫描式逆解改为优先下压构型

1. 用户提供新的实物照片后确认：当前机械臂在抓取时总是走成“悬着过去”的图一姿态，而不是“朝桌面压下去”的图二姿态。
2. 分析后确认根因不在几何公式本身，而在当前扫描式逆解的“选解策略”：
   - 之前按参考演示程序，做法是扫描全部可行姿态后取“中位解”。
   - 这种策略适合演示“存在可行解”，但不适合真实桌面抓取，因为它不会优先选择朝下压的构型。
3. 修改 [Drivers/BSP/SERVO/servo.c](e:\STM32H7机械臂控制\机械臂抓取代码1\Drivers\BSP\SERVO\servo.c) 中 `arm4dof_solve()`：
   - 保留当前“扫描 `posture_index=0..180` 并筛选可行解”的整体框架。
   - 删除“统计可行解总数并取中位解”的逻辑。
   - 改为在所有可行解中，按下列优先级选择更适合抓取的构型：
     1. 更大的 `posture_index`（整条手臂更向下压）
     2. 更大的 `raw_j2_deg`（肩部更下压）
     3. 更小的 `raw_j4_deg`（腕部更朝下）
4. 这样改的目标很明确：在同一个目标点有多组可行姿态时，不再随机落到“数学可达但抓不到”的悬空姿态，而是优先选更接近图二那种下压抓取构型。
5. 这次改动只修改“可行解里的选择标准”，不改几何参数、串口协议和主流程，属于针对构型偏好的最小修改。

2026-04-24 修正底座旋转轴 J1 方向

1. 用户根据实物现象反馈：目标物在机械臂右侧时，机械臂底座却朝左侧旋转，说明不是俯仰构型本身先错，而是底座旋转轴方向与代码中的正方向定义相反。
2. 分析当前逆解可知，`J1` 来自 `atan2(y, x)`；若目标在右侧，理论上 `J1` 应该朝对应方向偏转。
3. 结合实物现象判断，问题更可能出在舵机映射方向，而不是手眼标定立刻整体翻转，因此先做最小修正验证。
4. 修改 [User/main.c](e:\STM32H7机械臂控制\机械臂抓取代码1\User\main.c) 中 `arm_apply_default_limits()`：
   - 新增 `arm_cfg->angle_dir[0] = -1;`
5. 这表示底座旋转轴 J1 由原来的：
   - `servo_j1 = 90 + raw_j1`
   改为：
   - `servo_j1 = 90 - raw_j1`
6. 这样改的目的是先让“右侧目标 -> 底座向右转”这个最基本的方向关系正确，再继续观察俯仰构型是否仍有偏差。

2026-04-24 改判为大臂俯仰轴 J2 方向问题

1. 用户进一步根据实物现象确认：问题并不是底座旋转轴 J1，而是大臂俯仰时朝向反了。
2. 因此撤销上一版“仅翻转 J1”的判断，改为按最小改动验证 J2 方向是否与模型相反。
3. 修改 [User/main.c](e:\STM32H7机械臂控制\机械臂抓取代码1\User\main.c) 中 `arm_apply_default_limits()`：
   - `arm_cfg->angle_dir[0]` 设回 `1`
   - 新增 `arm_cfg->angle_dir[1] = -1`
4. 这样映射后：
   - J1 恢复为 `servo_j1 = 90 + raw_j1`
   - J2 改为 `servo_j2 = 90 - raw_j2`
5. 这次改动的目标是验证“模型里肩部向下压”的解，在实物上是否会从原来的反方向动作，变成正确的大臂下压趋势。

2026-04-24 让扫描式逆解重新跟随请求的末端姿态

1. 用户进一步反馈：当前机械臂的大臂方向已经更接近机械臂动作了，但在接近目标后，末端夹爪抓取时仍然不是垂直于桌面/物体的方向。
2. 复查当前实现发现：虽然 `main.c` 在抓取阶段仍然传入 `TARGET_PITCH_GRAB_DEG = -90deg`，但扫描式逆解 `arm4dof_solve()` 实际上只是在全部可行姿态里按“构型偏好”选解，并没有真正优先匹配这个请求姿态。
3. 修改 [Drivers/BSP/SERVO/servo.c](e:\STM32H7机械臂控制\机械臂抓取代码1\Drivers\BSP\SERVO\servo.c) 中 `arm4dof_solve()` 的候选评分：
   - 新增 `preferred_posture_deg = 90 - target->pitch_deg`
   - 并将其限制在 `0..180deg`
4. 评分规则改为：
   1. 优先选择 `posture_index` 最接近 `preferred_posture_deg` 的可行解
   2. 在接近程度相同时，再优先更下压的肩部和更向下的腕部
5. 这样映射后：
   - `req_pitch = -90deg` 时，优先姿态约为 `posture = 180deg`
     - 对应末端尽量垂直向下抓取
   - `req_pitch = -30deg` 时，优先姿态约为 `posture = 120deg`
     - 适合作为目标上方接近姿态
6. 这次改动不是硬改 J4 单个关节，而是让整条手臂在“所有可行解”里，优先挑选最接近目标末端姿态的那一支，因此 J2/J3/J4 会一起协调变化，更有利于在可达范围内实现垂直抓取。

2026-04-24 根据新确认点重算手眼仿射标定

1. 用户现场确认：`pixel=(161,143)` 对应的桌面坐标应为 `(240,50)`，而旧标定映射结果约为 `(247.6,29.3)`，说明当前仿射系数在该区域存在明显偏差，尤其是 Y 方向误差较大。
2. 复查现有标定点后发现，旧拟合中使用的 `pixel(217,127)->desk(240,45)` 与新确认点一起参与拟合时相互冲突，说明旧点中至少有一组当前不再可信。
3. 因此本次不采用“简单整体平移”补偿，而是把新确认点作为可信点，替换掉冲突的旧点后重新拟合一版仿射。
4. 修改 [User/main.c](e:\STM32H7机械臂控制\机械臂抓取代码1\User\main.c) 中标定注释和系数：
   - 采用 4 个当前可信点：
     - `pixel(156,102) -> desk(240,80)`
     - `pixel(156,133) -> desk(240,40)`
     - `pixel(215,95)  -> desk(220,80)`
     - `pixel(161,143) -> desk(240,50)`
   - 暂时将 `pixel(217,127) -> desk(240,45)` 视为离群点，不参与拟合。
5. 新仿射系数为：
   - `desk_x = -0.334148329 * px + 0.040749796 * py + 287.970660`
   - `desk_y = -0.085574572 * px - 0.721271394 * py + 166.919315`
6. 这是一版围绕当前工作区的临时 bring-up 标定，目标是先让你刚确认的区域落点更接近实测。后续仍建议继续补充更多点，重新做多点拟合。

2026-04-24 增加桌面 Y 方向偏置补偿

1. 用户继续反馈：当前映射结果在 Y 方向仍普遍存在约 `20mm` 的误差。
2. 在没有更多新标定点之前，不重新改动整套仿射系数，而是先做最小修正，避免再次把 X/Y 耦合关系整体带偏。
3. 修改 [User/main.c](e:\STM32H7机械臂控制\机械臂抓取代码1\User\main.c)：
   - 新增：
     - `CALIB_DESK_X_BIAS = 0.0f`
     - `CALIB_DESK_Y_BIAS = 20.0f`
   - 在 `handeye_pixel_to_desk()` 中，将该偏置直接叠加到最终桌面坐标。
4. 当前采用的假设是：`desk_y` 映射结果整体偏小约 `20mm`，因此先统一加上 `20mm` 做现场校正。
5. 这样做的好处是：
   - 改动极小
   - 不破坏当前仿射系数的整体趋势
   - 后续若用户继续确认误差，例如还差 `5mm`，只需要调整 `CALIB_DESK_Y_BIAS` 即可

2026-04-24 按 5 个新点重新标定手眼仿射

1. 用户提供了新的 5 组标定数据，左边为像素坐标，右边为桌面坐标：
   - `pixel(140,120) -> desk(240,80)`
   - `pixel(149,151) -> desk(240,40)`
   - `pixel(211,121) -> desk(320,80)`
   - `pixel(210,152) -> desk(320,40)`
   - `pixel(180,132) -> desk(280,65)`
2. 基于这 5 个点重新计算二维仿射映射，得到：
   - `desk_x = 1.205366269 * px - 0.142979751 * py + 84.775666`
   - `desk_y = 0.019345480 * px - 1.292081327 * py + 232.245900`
3. 修改 [User/main.c](e:\STM32H7机械臂控制\机械臂抓取代码1\User\main.c)：
   - 用上述 5 点重写标定注释和仿射系数
   - 将此前用于临时修正的：
     - `CALIB_DESK_X_BIAS`
     - `CALIB_DESK_Y_BIAS`
     都恢复为 `0.0f`
4. 这样做的原因是：既然已经有了覆盖当前工作区的新 5 点标定，就不再叠加旧的人工偏置，避免“仿射重算 + 手工补偿”双重修正把坐标再次带偏。

2026-04-24 手眼映射从仿射升级为平面单应性

1. 用户继续反馈：当前机械臂对靠近本体的目标抓取还可以，但对更远的目标，抓夹高度会偏高，说明远处点的桌面坐标仍存在系统性误差。
2. 结合场景分析，当前相机是斜拍桌面，而之前一直使用的是二维仿射映射。仿射对局部区域可用，但对整个桌面范围不能正确表达透视变化，因此会出现“近处还行、远处变差”的典型现象。
3. 基于用户给出的同一平面上的 5 个标定点：
   - `pixel(140,120) -> desk(240,80)`
   - `pixel(149,151) -> desk(240,40)`
   - `pixel(211,121) -> desk(320,80)`
   - `pixel(210,152) -> desk(320,40)`
   - `pixel(180,132) -> desk(280,65)`
   重新计算平面单应性（homography）矩阵。
4. 修改 [User/main.c](e:\STM32H7机械臂控制\机械臂抓取代码1\User\main.c)：
   - 将 `handeye_calib_2d_t` 从 6 个仿射系数字段改为 9 个单应性矩阵字段 `h11..h33`
   - 将标定常量替换为：
     - `H = [[0.435619205, -0.793218732, 176.196344],`
     - `     [-0.035721682, -0.829167962, 151.216568],`
     - `     [-0.000715835, -0.002563239, 1.000000000]]`
   - `handeye_pixel_to_desk()` 改为齐次坐标除法：
     - `desk_x = (h11*px + h12*py + h13) / (h31*px + h32*py + h33)`
     - `desk_y = (h21*px + h22*py + h23) / (h31*px + h32*py + h33)`
   - 启动日志由 `2D calib affine` 改为 `2D calib homography`
5. 这样做的目的，是让远处和近处都在同一个桌面平面模型下进行映射，减少因为透视误差导致的“远处目标被映得更远，从而抓取高度偏高”的问题。

2026-04-24 抓取阶段新增 wrist 与小臂垂直偏好

1. 用户提出：抓取物体时，希望末端 wrist 尽量与小臂保持垂直，但同时又不能只生硬改手腕角，必须让其他关节一起协调，避免变成不可达。
2. 为此，本次不做 J4 的硬性覆盖，而是在逆解选解阶段增加“抓取阶段 wrist 垂直偏好”。
3. 修改 [Drivers/BSP/SERVO/servo.h](e:\STM32H7机械臂控制\机械臂抓取代码1\Drivers\BSP\SERVO\servo.h)：
   - 在 `arm4dof_target_t` 中新增 `prefer_wrist_perpendicular`
4. 修改 [Drivers/BSP/SERVO/servo.c](e:\STM32H7机械臂控制\机械臂抓取代码1\Drivers\BSP\SERVO\servo.c)：
   - 在扫描全部可行解时，若 `target->prefer_wrist_perpendicular != 0`
   - 则将 `raw_j4` 偏向 `±90deg`
   - 具体通过 `fabs(fabs(raw_j4_deg) - 90.0f)` 作为罚项，加入候选评分
5. 修改 [User/main.c](e:\STM32H7机械臂控制\机械臂抓取代码1\User\main.c)：
   - 新增 `arm_move_to_pose_ex(...)`
   - 普通动作仍走原来的 `arm_move_to_pose(...)`
   - 仅在以下抓取相关阶段启用 `prefer_wrist_perpendicular = 1`
     - `target pick approach`
     - `target grab`
     - `target lift`
6. 这样做的效果是：
   - 对 `home / above / drop` 等普通动作不施加额外 wrist 约束
   - 对抓取阶段，在所有可行解里优先挑选 wrist 更接近与小臂垂直的那一支
   - 若完全严格垂直做不到，其他关节仍可协同调整，选择“最接近垂直且可达”的姿态

2026-04-24 抓取阶段将 wrist 直接固定为垂直于小臂

1. 用户进一步要求：不要只做“偏好”，而是希望在抓取物体时直接把 wrist 固定成垂直于小臂，其余关节去调整到合理抓取姿态。
2. 因此本次在保留“抓取阶段优先搜索接近垂直构型”的基础上，增加最终强制固定逻辑。
3. 修改 [Drivers/BSP/SERVO/servo.c](e:\STM32H7机械臂控制\机械臂抓取代码1\Drivers\BSP\SERVO\servo.c)：
   - 当 `target->prefer_wrist_perpendicular != 0` 且已找到可行解后：
     - 将 `selected_j4_deg` 直接钳到 `+90deg` 或 `-90deg`
     - 符号沿用所选候选解原本的符号方向
   - 之后重新走一次最终舵机映射与限位检查，若固定后越界则判定该目标不可达
4. 这样当前抓取阶段的实际行为是：
   - 先搜索一组整体上可达、且 wrist 尽量接近垂直的关节组合
   - 最终下发前把 wrist 明确固定成与小臂垂直
   - J2/J3 等其他关节仍使用已选出的合理抓取构型
5. 这样做的目标，就是让抓取姿态的末端形态更稳定、更符合用户想要的“手腕直接垂直小臂”的抓取方式，而不是只在候选解里做软偏好。

2026-04-24 高度相关参数检查与抓取高度下调

1. 用户反馈：机械臂完全不会运动到桌面高度。
2. 检查 User/main.c 中与抓取/投放高度相关的参数：
   - HOME_Z = 140mm：回 Home 时的末端高度。
   - TARGET_Z_ABOVE = 135mm：目标上方高位接近高度。
   - TARGET_Z_VERTICAL_ABOVE = 60mm：抓取前接近高度、抓取后抬起高度。
   - TARGET_Z_GRAB = 50mm：真正执行夹爪闭合时的抓取高度。
   - DROP_Z_ABOVE = 145mm：投放点上方高度。
   - DROP_Z_RELEASE = 130mm：投放松爪高度。
3. 检查 Drivers/BSP/SERVO/servo.c 中影响高度解算的机械参数：
   - base_height = 120mm：底座平面/桌面坐标到肩关节轴心的高度。
   - wrist = 80mm：腕部到末端参考点的长度，会参与 `wrist_z = (target_z - base_height) - wrist * sin(pitch)`。
   - upper_arm / forearm：影响目标高度和半径是否可达。
4. 当前“不下到桌面”的直接原因是 TARGET_Z_GRAB 仍为 50mm，程序闭爪时目标末端高度就是 50mm，不是桌面高度。
5. 修改 User/main.c：TARGET_Z_GRAB 从 50mm 下调到 10mm。
6. 按用户日志中的典型目标 x=251.7、y=2.5 代入当前模型，z=10mm 仍可逆解，预计会选择 pitch 约 -50deg，理论关节约 J=[90.6, 65.3, 112.0, 137.3]。
7. 如果实物仍不到桌面，需要继续确认：
   - 机械坐标系里桌面到底是不是 z=0。
   - base_height 是否真的是肩关节轴心到桌面的垂直高度。
   - wrist=80mm 是否包含了夹爪到实际接触点的长度；如果没有，实际夹取点会比模型末端更高或更低。
   - TARGET_Z_GRAB 可继续小步调整，例如 10mm、5mm、0mm，但要防止撞桌。
8. 已按编译约定使用 D:\keil5\UV4\UV4.exe 编译，Output/codex_build.log 显示 0 Error(s), 0 Warning(s)。hex/axf 已更新。

2026-04-24 根据手绘图增加抓取阶段腕部强制向下测试

1. 用户提供手绘图：机械臂抓取后末端不是朝桌面向下，而是斜向上夹取。
2. 由此判断当前问题不是单纯 TARGET_Z_GRAB 高度不够，而是抓取阶段腕部 J4 的实物映射仍未标定好；依靠 IK 的 pitch 结果间接控制 J4 会继续把夹爪带偏。
3. 修改 User/main.c，新增抓取阶段腕部覆盖参数：
   - PICK_WRIST_SERVO_OVERRIDE_EN = 1
   - PICK_WRIST_DOWN_SERVO_DEG = 60deg
   - PICK_WRIST_OVERRIDE_PITCH_DEG = -35deg
4. arm_move_to_pose() 中，当 pitch <= -35deg 时，认为这是抓取接近/抓取/抓取后抬起阶段，强制 solution.joint_deg[3] = 60deg。
5. target above 使用 pitch=-30deg，不触发覆盖；drop 使用 pitch=-25deg，也不触发覆盖。
6. 启动时新增打印：`pick wrist override enabled: J4=60.0 when pitch<=-35.0`。
7. 抓取移动时新增打印：`pick wrist override: J4=60.0`。
8. 这是一版实物调试固件：如果 J4=60 后夹爪从斜向上明显变为向下，说明方向判断正确，后续只需要微调 PICK_WRIST_DOWN_SERVO_DEG；如果仍然斜向上，则继续降低该值，例如 45deg、30deg，或反向提高到 120deg 对比。
9. 该覆盖会牺牲一部分 IK 姿态精度，但能快速确定真实 J4 舵机角度与夹爪朝向的关系。完成 J4 标定后，应把该覆盖替换为正确的 angle_offset_deg[3] / angle_dir[3] 参数。
10. 已按编译约定使用 D:\keil5\UV4\UV4.exe 编译，Output/codex_build.log 显示 0 Error(s), 0 Warning(s)。hex/axf 已更新。

2026-04-24 抓取阶段改为优先选择下压构型

1. 用户继续反馈：问题不只是末端抓夹，大臂和小臂本身也没有朝向桌面下压的趋势。
2. 复查后确认根因之一是逆解一直只取单一肘部构型分支，因此机械臂会优先走“侧着够过去”的那条解，而不是“往下压”的那条解。
3. 修改 [Drivers/BSP/SERVO/servo.h](e:\STM32H7机械臂控制\机械臂抓取代码1\Drivers\BSP\SERVO\servo.h)：
   - 在 `arm4dof_config_t` 中新增 `elbow_preference`。
4. 重写 [Drivers/BSP/SERVO/servo.c](e:\STM32H7机械臂控制\机械臂抓取代码1\Drivers\BSP\SERVO\servo.c) 的逆解实现：
   - 新增 `arm4dof_try_solve_branch()`，分别尝试 `sin(j3)` 正分支和负分支。
   - `arm4dof_solve()` 现在会同时求两支解。
   - 当两支都可行时，若 `elbow_preference > 0`，优先选择肩关节舵机角更大的那支，也就是更有“下压趋势”的构型。
5. 修改 [User/main.c](e:\STM32H7机械臂控制\机械臂抓取代码1\User\main.c)：
   - 关闭上一版的 `PICK_WRIST_SERVO_OVERRIDE_EN`，避免 J4 强制覆盖继续干扰判断。
   - 新增 `PICK_ELBOW_DOWNWARD_PREFERENCE = 1`。
   - 在 `arm_move_to_pose()` 中，当 `pitch <= -35deg` 时，抓取相关动作会复制一份配置并设置 `work_cfg.elbow_preference = 1`，优先选“下压构型”。
   - 启动时新增串口打印：`pick elbow preference: 1 when pitch<=-35.0`。
6. 这次修改针对的是整条手臂构型，不只是末端夹爪角度；预期现象是大臂、小臂在抓取接近和下探阶段更明显地朝桌面方向折下来。
7. 已按编译约定使用 D:\keil5\UV4\UV4.exe 编译，Output/codex_build.log 显示 0 Error(s), 0 Warning(s)。hex/axf 已更新。

2026-04-25 小幅提升整轮抓取动作速度

1. 用户要求：在不大改当前抓取流程和逆解逻辑的前提下，让机械臂整体完成一次目标抓取更快一些。
2. 评估当前代码后确认，主要限速点不在 PWM 频率，而在 [User/main.c](e:\STM32H7机械臂控制\机械臂抓取代码1\User\main.c) 顶部的统一缓动步进延时和动作后的保守等待时间。
3. 本次只做最小速度参数调整，不修改逆解、舵机映射、抓取顺序和标定参数：
   - `ARM_MOVE_STEP_DELAY_MS` 从 `35ms` 下调到 `22ms`
   - `GRIPPER_STEP_DELAY_MS` 从 `30ms` 下调到 `20ms`
   - `ARM_SETTLE_DELAY_MS` 从 `500ms` 下调到 `250ms`
   - `TARGET_RETRY_DELAY_MS` 从 `80ms` 下调到 `50ms`
4. 这样做的目的，是优先缩短每次关节缓动和动作间停顿时间，让整轮抓取明显更紧凑，同时尽量不破坏当前已经联调出的动作路径。
5. 风险说明：
   - 若实物舵机扭矩偏弱、负载较重或机构摩擦较大，提速后可能出现抖动、未到位就进入下一步、夹取成功率下降。
   - 如果上板后发现接近目标或投放阶段开始不稳定，优先把 `ARM_SETTLE_DELAY_MS` 单独加回，例如先调回 `300ms` 或 `350ms`，不必立刻把全部速度参数回退。
6. 已按编译约定使用 D:\keil5\UV4\UV4.exe 编译，Output/atk_h7r3.build_log.htm 显示 0 Error(s), 0 Warning(s)。hex/axf 已更新。

2026-04-25 增加直线插补第一版以改善抓取平滑度

1. 用户反馈当前已经能基本准确抓取，下一步希望考虑机械臂运动轨迹规划，使抓取动作更平滑，优先从直线插补开始。
2. 结合当前工程实现确认：
   - 之前的 `arm_move_to_pose_ex()` 属于“单目标点 -> 一次 IK -> 一次整段关节缓动”的点到点运动。
   - 这种方式关节角变化是连续的，但末端在笛卡尔空间并不会严格沿直线运动。
3. 本次在 [User/main.c](e:\STM32H7机械臂控制\机械臂抓取代码1\User\main.c) 中做最小实现：
   - 新增 `arm_pose_t`、`g_arm_last_pose`、`g_arm_last_pose_valid`，记录最近一次成功下发的末端目标位姿。
   - 新增 `arm_prepare_pose_solution()`，把“构造目标、求 IK、抓取阶段固定末端两轴、保留当前夹爪角度”收敛成统一步骤。
   - 新增 `arm_execute_solution()`，统一执行关节缓动，并允许中间插补点不额外等待。
   - 新增 `ARM_LINEAR_STEP_MM = 5mm`、`ARM_LINEAR_PITCH_STEP_DEG = 4deg`，作为直线插补步长。
   - 新增 `arm_move_linear_pose_ex()` / `arm_move_linear_pose()`：
     - 若当前没有有效起点位姿，则退回原来的点到点运动；
     - 若有有效起点位姿，则把 `x/y/z/pitch` 从起点到终点按固定步长分段；
     - 对每个插补点逐点做 IK 并执行；
     - 中间点不做 `ARM_SETTLE_DELAY_MS` 停顿，只在最后一个点保留落点等待。
4. 当前第一版直线插补已接入这些动作段：
   - `target pick approach`
   - `target grab`
   - `target lift`
   - `drop release`
   - `drop lift`
5. 当前仍保持原来的点到点动作段：
   - 回 Home
   - 移动到目标上方
   - 移动到投放区上方
   这样做是为了先把最需要“末端路径更规整”的下探/抬起阶段变平滑，避免一次性扩大改动范围。
6. 这次实现的目标不是完整工业级轨迹规划，而是先在现有阻塞式舵机控制架构上，提供一版低风险、可上板验证的笛卡尔直线插补基础能力。
7. 后续若这版上板效果稳定，推荐下一步顺序：
   - 观察抓取接近、下探、抬起是否明显更平滑；
   - 再决定是否把 `home -> target above`、`drop above -> home` 也切到直线插补；
   - 最后再考虑梯形速度/S 曲线速度规划和圆弧插补。
8. 风险说明：
   - 直线插补会增加单段路径上的 IK 求解次数；若某些中间点不可达，会在 `linear waypoint failed` 处提前终止。
   - 由于底层仍是普通 PWM 舵机 + 阻塞式缓动，平滑度会明显改善，但仍不同于工业伺服连续轨迹控制。
9. 已按编译约定使用 D:\keil5\UV4\UV4.exe 编译，Output/codex_build.log 与 Output/atk_h7r3.build_log.htm 显示 0 Error(s), 0 Warning(s)。hex/axf 已更新。

2026-04-25 直线插补扩展到更多过渡动作段

1. 在第一版直线插补上板后，用户允许继续按既定方向推进，因此本次继续扩大笛卡尔直线插补的覆盖范围。
2. 本次仍坚持最小演进原则：
   - 继续使用现有 `arm_move_linear_pose()` / `arm_move_linear_pose_ex()`；
   - 不引入圆弧插补；
   - 不引入梯形速度或 S 曲线；
   - 不修改底层舵机驱动和逆解公式。
3. 修改 [User/main.c](e:\STM32H7机械臂控制\机械臂抓取代码1\User\main.c)：
   - `target above`：由原来的点到点 `arm_move_to_pose()` 改为 `arm_move_linear_pose()`
   - `drop above`：由原来的点到点 `arm_move_to_pose()` 改为 `arm_move_linear_pose()`
   - 启动后的首次回 Home：统一改为 `arm_move_linear_pose()`，若当前还没有有效起点位姿，函数内部会自动退回原来的点到点运动
   - 正常抓取循环中的“先回 Home 再开始本轮抓取”：改为 `arm_move_linear_pose()`
   - 抓取完成后的“回 Home”：改为 `arm_move_linear_pose()`
4. 这样扩展后，当前整轮抓取中仍保留点到点方式的只剩：
   - 失败兜底时的 `arm_move_to_vertical_up_pose()` 临时关节安全姿态
   - 个别没有笛卡尔目标语义、仅用于应急恢复的动作
5. 当前整体路径特性变为：
   - 接近目标上方、下探、抬起、投放下降、投放抬起、回 Home 等主要动作段，都会优先尝试沿笛卡尔直线插补执行；
   - 若当前没有已知起点位姿，则自动退回旧的点到点实现，避免因为状态不足导致启动阶段异常。
6. 这一步的目标，是先把“整轮抓取动作看起来更顺、更像一条连续路径”这件事做出来，再决定是否有必要继续引入更复杂的速度曲线或圆弧过渡。
7. 风险说明：
   - 覆盖段增多后，若某些较长路径中间经过局部不可达区，出现 `linear waypoint failed` 的概率会高于第一版。
   - 如果上板后发现某一整段大范围移动更容易失败，建议优先把那一段单独退回点到点，而不是立刻否定整套直线插补。
8. 已按编译约定使用 D:\keil5\UV4\UV4.exe 编译，Output/codex_build.log 与 Output/atk_h7r3.build_log.htm 显示 0 Error(s), 0 Warning(s)。hex/axf 已更新。

2026-04-25 在线性轨迹上增加简化缓起缓停规划

1. 在直线插补覆盖到整轮主要动作段后，用户继续允许推进，因此本次开始做最小版本的“速度规划”。
2. 考虑到当前底层仍是普通 PWM 舵机 + 阻塞式关节缓动，本次不直接上复杂的梯形速度或真正时间参数化轨迹，而是先在现有直线插补上增加一个低风险的缓起缓停分布。
3. 修改 [User/main.c](e:\STM32H7机械臂控制\机械臂抓取代码1\User\main.c)：
   - 新增 `ARM_LINEAR_EASE_MIN_SEGMENTS = 4`
   - 新增 `arm_smoothstep01()`，使用 `t*t*(3-2*t)` 对 `0..1` 插值参数做 smoothstep 平滑
   - 在 `arm_move_linear_pose_ex()` 中：
     - 当当前轨迹段的插补段数小于 4 时，仍使用原来的线性 `t`
     - 当插补段数不少于 4 时，先把线性 `t` 通过 smoothstep 变换后，再计算轨迹点
4. 这样改动后，轨迹几何形状仍然保持为同一条直线，但轨迹点沿直线的分布变为：
   - 起点附近更密
   - 中段更疏
   - 终点附近更密
   对应的直观效果，就是起步和收尾更柔和，中间段更利落。
5. 这次实现的定位，是“简化版 S 曲线节奏”，不是严格意义上的工业级时间最优速度规划；它的优点是：
   - 改动极小
   - 不改变现有 IK、舵机映射和抓取流程
   - 能先验证当前机械臂是否适合更柔和的起停风格
6. 若上板后这版效果稳定，后续可选方向有两个：
   - 继续保守演进：保留 smoothstep，只微调 `ARM_LINEAR_STEP_MM` / `ARM_LINEAR_PITCH_STEP_DEG`
   - 再进一步：把“插补点位置分布平滑”升级为“按时间控制的梯形速度或 S 曲线速度”
7. 已按编译约定使用 D:\keil5\UV4\UV4.exe 编译，Output/codex_build.log 与 Output/atk_h7r3.build_log.htm 显示 0 Error(s), 0 Warning(s)。hex/axf 已更新。

2026-04-25 将主流程拆分到独立 .c/.h 模块，精简 main.c

1. 用户提出希望把 `main.c` 中同类型函数封装到独立的 `.c/.h` 文件里，让主函数本身更短、更像应用入口。
2. 本次采用的方案是“整块业务流程模块化”，而不是只零散抽几个函数：
   - 新增 [User/app_pick_demo.h](e:\STM32H7机械臂控制\机械臂抓取代码1\User\app_pick_demo.h)
   - 新增 [User/app_pick_demo.c](e:\STM32H7机械臂控制\机械臂抓取代码1\User\app_pick_demo.c)
   - 将原来 `main.c` 中的抓取 demo 业务逻辑、轨迹插补、OpenMV 解析、手眼映射、抓取/投放流程等整体迁移到 `app_pick_demo.c`
3. 调整后的职责划分：
   - `main.c`：只保留一个非常薄的入口，包含 `main()` 并调用 `app_pick_demo_run()`
   - `app_pick_demo.c`：负责系统初始化、串口数据处理、目标稳定判定、运动规划、抓取流程与投放流程
   - `app_pick_demo.h`：只暴露 `app_pick_demo_run()` 入口
4. 这样做的原因是：
   - 直接把整套 demo 逻辑归到单独模块里，比“主函数里只抽出一两个辅助函数”更彻底
   - 后续如果继续扩展轨迹规划、状态机或调试模式，都可以集中在 `app_pick_demo.c` 中维护
   - `main.c` 现在已经恢复成标准的“只含入口和调用”的形式，可读性明显更强
5. 同步修改 [Projects/MDK-ARM/atk_h7r3.uvprojx](e:\STM32H7机械臂控制\机械臂抓取代码1\Projects\MDK-ARM\atk_h7r3.uvprojx)，把新增的 `app_pick_demo.c` 纳入 Keil 工程编译列表。
6. 本次重构属于结构整理，不应改变原有抓取行为；后续若再做功能迭代，优先继续改 `app_pick_demo.c`，避免把 `main.c` 再次堆大。
7. 已按编译约定使用 D:\keil5\UV4\UV4.exe 编译，Output/codex_build.log 与 Output/atk_h7r3.build_log.htm 显示 0 Error(s), 0 Warning(s)。hex/axf 已更新。

2026-04-25 继续整理 app_pick_demo.c 内部流程函数

1. 在将业务逻辑整体迁移到 `app_pick_demo.c` 后，用户继续要求提升可读性，因此本次不再新增文件，而是在该模块内部继续按职责分区提炼函数。
2. 修改 [User/app_pick_demo.c](e:\STM32H7机械臂控制\机械臂抓取代码1\User\app_pick_demo.c)：
   - 新增 `app_system_init()`：集中系统/外设初始化和机械臂默认配置加载
   - 新增 `app_print_startup_banner()`：集中启动日志打印
   - 新增 `app_prepare_startup_motion()`：集中开机回中、开爪、可选轴识别、回 Home
   - 新增 `app_handle_none_target()`：集中处理 `NONE` 输入和“等待目标离开视野”状态恢复
   - 新增 `app_try_lock_target()`：集中处理目标稳定判定和锁定逻辑
   - 新增 `app_execute_pick_cycle()`：集中处理“开爪 -> 回 Home -> 抓取 -> 投放 -> 回 Home”的单轮动作
   - 新增 `app_reset_for_wait_target_leave()`：统一失败后切到“等待目标离开”状态
3. 调整后 `app_pick_demo_run()` 的职责进一步收缩为：
   - 调用初始化与启动准备函数
   - 读取串口一行 OpenMV 数据
   - 做少量分支判断
   - 调用更具体的流程函数
4. 这样整理的目的，是让 `app_pick_demo_run()` 更像“顶层控制流”，而不是继续堆放大量细节判断；后续如果继续做状态机或调试模式，也更容易找到对应入口。
5. 本次仍然是结构整理，不改变抓取算法、轨迹规划参数、手眼标定和舵机映射。
6. 已按编译约定使用 D:\keil5\UV4\UV4.exe 编译，Output/codex_build.log 与 Output/atk_h7r3.build_log.htm 显示 0 Error(s), 0 Warning(s)。hex/axf 已更新。

2026-04-25 为 app_pick_demo.c 增加分区注释，进一步提升可读性

1. 在 `app_pick_demo.c` 已完成函数级整理后，继续做一层更轻量的可读性优化。
2. 本次不改函数逻辑、不改调用关系，只增加模块内部分区注释：
   - `Motion helpers`
   - `App flow helpers`
   - `OpenMV input helpers`
3. 这样做的目的，是让后续阅读者一眼就能区分：
   - 哪些函数属于机械臂运动/插补/姿态求解辅助
   - 哪些函数属于应用启动和单轮抓取流程控制
   - 哪些函数属于 OpenMV 输入解析和目标锁定
4. 这次改动纯属代码组织层面的轻量整理，不改变任何功能行为。
5. 已按编译约定使用 D:\keil5\UV4\UV4.exe 编译，Output/codex_build.log 与 Output/atk_h7r3.build_log.htm 显示 0 Error(s), 0 Warning(s)。hex/axf 已更新。

2026-04-25 将新增英文注释统一改为中文

1. 用户要求把近期新增的英文注释统一改成中文，以便与当前工程整体注释风格保持一致。
2. 修改 [User/app_pick_demo.c](e:\STM32H7机械臂控制\机械臂抓取代码1\User\app_pick_demo.c)：
   - 文件头 `@brief` 改为中文
   - 参数分区注释改为中文
   - 标定说明注释改为中文
   - 分区注释 `Motion helpers / App flow helpers / OpenMV input helpers` 改为中文
3. 修改 [User/main.c](e:\STM32H7机械臂控制\机械臂抓取代码1\User\main.c)：
   - 文件头 `@brief` 改为中文
4. 本次只改注释文本，不改任何程序逻辑或调用关系。
5. 已按编译约定使用 D:\keil5\UV4\UV4.exe 编译，Output/codex_build.log 与 Output/atk_h7r3.build_log.htm 显示 0 Error(s), 0 Warning(s)。hex/axf 已更新。

2026-04-25 修复模块拆分后程序上电无串口、机械臂不动作的问题

1. 用户反馈：在将抓取 demo 从 `main.c` 拆分到 `app_pick_demo.c` 后，实物现象变为“串口不打印、机械臂也完全不动”。
2. 排查后确认，问题不在抓取流程逻辑本身，而在链接地址：
   - 当前工程使用 [User/Script/ATK-DNH7R3_flash_ROMxspi1.sct](e:\STM32H7机械臂控制\机械臂抓取代码1\User\Script\ATK-DNH7R3_flash_ROMxspi1.sct) 分散加载脚本
   - 脚本中 `ER_IROM1` 片内 Flash 执行区显式列出了 `main.o`
   - 但新加的 `app_pick_demo.o` 没有被显式列入 `ER_IROM1`
   - 因此链接器把 `app_pick_demo.o` 的只读代码默认放进了 `ER_ROM1 @ 0x90000000`
3. 用 `fromelf` 检查修复前产物时可以看到：
   - `main` 位于 `0x0800...`
   - `app_pick_demo_run` 位于 `0x9000...`
   这会导致启动后 `main()` 很早就跳去外部执行区，和当前实机“完全不打印、完全不动作”的现象一致。
4. 修改 [User/Script/ATK-DNH7R3_flash_ROMxspi1.sct](e:\STM32H7机械臂控制\机械臂抓取代码1\User\Script\ATK-DNH7R3_flash_ROMxspi1.sct)：
   - 在 `ER_IROM1` 中把 `app_pick_demo.o` 显式加入 `main.o` 后面
5. 修复后再次用 `fromelf` 验证：
   - `app_pick_demo_run` 已回到 `0x0800...`
   - `main` 直接调用片内 Flash 中的 `app_pick_demo_run`
6. 这样修复的原因是：本次问题属于模块拆分后对象文件落入错误执行区，而不是业务代码逻辑错误，因此最小修复就是改链接脚本的对象归属，不回退结构整理本身。
7. 已按编译约定使用 D:\keil5\UV4\UV4.exe 编译，Output/codex_build.log 与 Output/atk_h7r3.build_log.htm 显示 0 Error(s), 0 Warning(s)。hex/axf 已更新。

2026-04-25 进一步增大末端夹爪开合范围

1. 用户要求：在当前版本基础上，让末端夹爪的开合程度进一步加大。
2. 本次采用最小改动方案，只调整 [User/app_pick_demo.c](e:\STM32H7机械臂控制\机械臂抓取代码1\User\app_pick_demo.c) 中夹爪开/合目标角，不改抓取流程、逆解或舵机映射。
3. 新增统一常量：
   - `GRIPPER_OPEN_DEG = 125deg`
   - `GRIPPER_CLOSE_DEG = 10deg`
4. 同步修改位置：
   - `gripper_open()` 使用 `GRIPPER_OPEN_DEG`
   - `gripper_close()` 使用 `GRIPPER_CLOSE_DEG`
   - `arm_move_to_vertical_up_pose()` 中第 6 路夹爪角改为 `GRIPPER_OPEN_DEG`
   - `arm_identify_end_axes()` 中第 6 路开合测试也改为使用同一组常量
5. 这样做的目的，是让夹爪张开更大、闭合更紧，同时把原来分散的 `110deg / 20deg` 硬编码统一起来，后续调参更直接。
6. 风险说明：
   - 若实物夹爪机械限位比当前更紧，`125deg / 10deg` 可能偏激进；如果上板后出现卡死、异响或夹爪顶到极限，优先把这两个角各回退 5~10 度。
7. 已按编译约定使用 D:\keil5\UV4\UV4.exe 编译，Output/codex_build.log 与 Output/atk_h7r3.build_log.htm 显示 0 Error(s), 0 Warning(s)。hex/axf 已更新。

2026-05-02 进一步提升机械臂整体动作速度

1. 用户要求：在当前能正常运行的基础上，继续加快机械臂整体速度。
2. 本次采用“小步提速”的组合方式，仍只修改 [User/app_pick_demo.c](e:\STM32H7机械臂控制\机械臂抓取代码1\User\app_pick_demo.c) 中速度相关常量，不改抓取流程、逆解和舵机映射。
3. 调整如下：
   - `ARM_MOVE_STEP_DELAY_MS`：`22ms -> 16ms`
   - `GRIPPER_STEP_DELAY_MS`：`20ms -> 15ms`
   - `ARM_SETTLE_DELAY_MS`：`250ms -> 150ms`
   - `TARGET_RETRY_DELAY_MS`：`50ms -> 40ms`
   - `ARM_LINEAR_STEP_MM`：`5mm -> 7mm`
   - `ARM_LINEAR_PITCH_STEP_DEG`：`4deg -> 5deg`
4. 这样做的效果是：
   - 关节和夹爪每一步执行更快
   - 每个动作结束后的等待更短
   - 直线插补分段数减少，因此整段轨迹的执行时间也会缩短
5. 这次改动仍然偏保守，没有一次性把速度推得很激进；目的是在明显提速的同时，尽量保持当前已调通的稳定性。
6. 风险说明：
   - 如果上板后出现抖动、过冲、到位不稳或夹取成功率下降，优先先把 `ARM_SETTLE_DELAY_MS` 加回到 `200ms` 左右，再观察是否恢复稳定。
   - 如果长段直线移动开始显得太“跳”，优先把 `ARM_LINEAR_STEP_MM` 从 `7mm` 回调到 `6mm`，不必立刻回退全部速度参数。
7. 已按编译约定使用 D:\keil5\UV4\UV4.exe 编译，Output/codex_build.log 与 Output/atk_h7r3.build_log.htm 显示 0 Error(s), 0 Warning(s)。hex/axf 已更新。

2026-05-02 在上一轮基础上再提一档整体速度

1. 用户继续要求：在上一轮提速基础上，再加快一点整体动作速度。
2. 本次仍只调整 [User/app_pick_demo.c](e:\STM32H7机械臂控制\机械臂抓取代码1\User\app_pick_demo.c) 中的速度相关常量，不改抓取流程和逆解逻辑。
3. 进一步调整如下：
   - `ARM_MOVE_STEP_DELAY_MS`：`16ms -> 12ms`
   - `GRIPPER_STEP_DELAY_MS`：`15ms -> 12ms`
   - `ARM_SETTLE_DELAY_MS`：`150ms -> 100ms`
   - `TARGET_RETRY_DELAY_MS`：`40ms -> 30ms`
   - `ARM_LINEAR_STEP_MM`：`7mm -> 8mm`
   - `ARM_LINEAR_PITCH_STEP_DEG`：`5deg -> 6deg`
4. 这一档提速相比上一轮更明显，尤其会进一步压缩：
   - 每步关节/夹爪执行时间
   - 动作完成后的停顿时间
   - 长段直线插补的总分段数
5. 风险说明：
   - 这一档已经比前一版更接近“偏激进”的提速，如果上板后抓取末端出现明显抖动、顿挫、过冲或成功率下降，建议优先回退到上一档参数。
   - 最容易先出问题的通常是 `ARM_SETTLE_DELAY_MS` 过小；若现象只是“没完全停稳就进下一步”，优先先把它从 `100ms` 回调到 `150ms`。
6. 已按编译约定使用 D:\keil5\UV4\UV4.exe 编译，Output/codex_build.log 与 Output/atk_h7r3.build_log.htm 显示 0 Error(s), 0 Warning(s)。hex/axf 已更新。

2026-05-25 针对像素 x 坐标线性调整抓夹末端俯仰角

1. 用户要求：当 OpenMV 像素 x 坐标越靠左时，抓夹末端俯仰角转得更多；越靠近 x=185 时，转得更少，其他保持不变。
2. 修改 User/app_pick_demo.c：
   - 新增 `TARGET_PIXEL_LOW_X_THRESHOLD = 185`
   - 新增 `TARGET_PIXEL_LOW_X_FULL_COMP = 120`
   - 新增 `GRAB_WRIST_LOW_X_DELTA_DEG = 30.0f`
   - 新增 `arm_get_grab_wrist_servo_deg_from_pixel()`
3. 当前逻辑为：
   - 默认抓取阶段 `joint_deg[4]` 仍固定为 `GRAB_WRIST_SERVO_DEG = 60deg`
   - 当 `pixel.x >= 185` 时，`joint_deg[4] = 60deg`
   - 当 `pixel.x <= 120` 时，`joint_deg[4] = 30deg`
   - 当 `120 < pixel.x < 185` 时，`joint_deg[4]` 在 30deg 到 60deg 之间按像素 x 线性变化
4. 串口 `pixel=(...) -> desk=(...), grab_wrist=...` 会打印当前本轮使用的抓夹末端俯仰角，便于上板确认条件是否触发。
5. 本次只影响 `prefer_wrist_perpendicular != 0` 的抓取/竖直搬运动作，不改变手眼标定、逆解、其他舵机映射、速度参数和抓取流程。
6. 已按编译约定使用 D:\keil5\UV4\UV4.exe 编译，Output/codex_build.log 显示 0 Error(s), 0 Warning(s)。hex/axf 已更新。
