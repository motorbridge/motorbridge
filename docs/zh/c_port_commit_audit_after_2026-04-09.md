# C 工程提交修复核查表（2026-04-09 之后）

本文用于检查另一个参照 motorbridge 实现的 C 工程，确认其是否覆盖 motorbridge 在 2026 年 4 月 9 日之后修复或新增的行为。

## 范围与使用方法

- 基准仓库：`motorbridge/motorbridge`，遍历 `--all` 可达历史。
- 时间边界：按 `Asia/Shanghai`（UTC+8）严格晚于 `2026-04-09 23:59:59`。
- 提交范围：`5723c20c`（2026-04-13）至 `c48ebc4b`（2026-09-05），共 152 条。
- 提交日期采用 Git author date。合并、发布、文档、CI 和回滚提交也保留在完整矩阵中。
- 本表不代表目标 C 工程已经通过；必须结合源码、单元测试、CAN/UART 抓包和实机结果填写。

状态约定：

- `[ ]`：尚未检查。
- `[x]`：已经检查并通过。
- `[!]`：已检查但存在差异、缺陷或证据不足。
- `[-]`：确认对目标工程不适用，必须在备注中写明原因。

建议证据至少包含以下一种：C 文件和函数名、测试用例编号、CAN/UART 抓包、设备型号与固件版本、复现步骤和测试日期。不能只填写“已实现”。

## 核心行为检查表

| 状态 | ID | 优先级 | 模块 | 对应 commit | C 工程检查项 | 通过准则 | 证据/备注 |
|---|---|---|---|---|---|---|---|
| [ ] | C-01 | P0 | CAN 调度 | `fee4cad9` | 检查批量 enable/disable 是否按 motor ID 稳定排序、逐个发送并留出可配置间隔；等待期间尽量排空接收队列。 | 默认间隔约 2 ms 或经实机证明的等价值；多电机连续操作不丢 ACK、不读到过期响应。 | |
| [ ] | C-02 | P0 | CAN 调度 | `7289487c` | 检查多电机控制循环的相邻 TX 是否节流，且批量操作间隔与普通 TX 间隔分别配置。 | 默认普通 TX gap 约 120 us 或有等效总线预算；压力测试无突发淹没、响应错配和明显控制抖动。 | |
| [ ] | G-01 | P1 | 网关/请求路由 | `cf87ecbe` `6d90be39` `9db93e20` `9e982aeb` `b2b67bfa` | 若 C 工程含 WebSocket/RPC 网关，检查请求携带的 `req_id` 是否原样进入响应，扫描是否复用 controller，连接、会话、路由和操作状态是否隔离。 | 并发请求不会串响应；扫描不会按每个 ID 重开总线；不同会话的目标和错误互不污染。纯协议库可标 N/A。 | |
| [ ] | G-02 | P0 | 单电机隔离 | `43c341ae` | enable、disable 和单电机命令必须作用于请求显式指定的 motor，不得误调用 enable-all/disable-all；兼容旧协议时才使用会话默认目标。 | 两台以上电机测试中，命令只改变目标电机；缺少目标时明确报错或按已记录兼容规则处理。 | |
| [ ] | G-03 | P1 | 网关鉴权 | `13c64142` | 若支持浏览器 WebSocket，检查 token 查询参数与既有鉴权方式的一致性，且 token 不被日志泄漏。 | 合法 token 可连接，缺失/错误 token 被拒绝；日志和错误响应不回显密钥。纯协议库可标 N/A。 | |
| [ ] | R-01 | P0 | RobStride 模式切换 | `822dffe9` `f2d57943` | `ensure_control_mode` 先读 `0x7005`；必要时先 disable 并等待约 60 ms；最多 3 次写入/回读，重试间隔约 30 ms；最终以寄存器回读为准。 | 已使能状态下切模式仍可成功；写入失败或回读不一致必须报错，不得仅更新本地缓存并伪成功。 | |
| [ ] | R-02 | P0 | RobStride PP pos_vel | `822dffe9` `295c5398` | PP `pos_vel` 的速度写入 `0x7024 vel_max`，可选加速度写 `0x7025 acc_set`，不得错写 CSP 的 `0x7017 limit_spd`。 | 抓包中索引正确；位置和速度限制均生效；PP/CSP 模式互不污染。 | |
| [ ] | R-03 | P0 | RobStride 扫描 | `99ecc65a` `895e0a28` | 默认 host/feedback ID 为 `0xFD`，兼容 `0xFD/0xFF/0xFE`；同一 feedback ID 的扫描复用 controller，特别是 Windows PCAN 不得高频 Initialize/Uninitialize。 | 扫描多个 motor ID 时每个 feedback ID 只初始化一次总线；未知参数回复可保留 raw U32，不导致整次轮询失败。 | |
| [ ] | R-04 | P0 | RobStride ID | `a258ec5c` `28929bde` | motor ID 限制为 `1..255`，host ID 为 `0..255`；ping/扫描回复必须匹配目标 device ID；协议中的 motor、host、feedback ID 角色保持一致。 | 边界值和非法值测试通过；其他设备回复不会被误判为目标设备。 | |
| [ ] | R-05 | P0 | RobStride 改 ID 持久化 | `e9ec70e4` | 修改 motor ID 后，使用新 ID 对应的 handle/controller 执行 store，而不是继续用旧 ID。 | 断电重启后新 ID 保持；旧 ID 不再响应；保存失败时给出明确错误。 | |
| [ ] | R-06 | P0 | RobStride 参数表 | `faba15fa` `bd0d2cdc` `4289a22b` `c074439d` `a2b50edb` | `rs-00` 至 `rs-06` 使用型号专属参数表；runtime 表只含协议可访问参数；Section 4 参数完整到 `0x702E`，类型、范围及 little-endian 编解码正确。 | 对每个支持型号执行参数存在性、类型、边界、越界拒绝和 golden bytes 测试；不得用 auto/错误型号参数代替。 | |
| [ ] | R-07 | P0 | RobStride 零点范围 | `40ffc379` `0db9ec1c` | set-zero 时写 `zero_sta(0x7029)`；默认值 1 对应 `-pi..pi`，并支持值 0 的 `0..2pi` 范围；写后回读。 | 两种范围均有实机或模拟测试；回读不一致失败；零点后位置范围符合配置。 | |
| [ ] | R-08 | P0 | RobStride 故障帧 | `9908d0fa` | `FAULT_REPORT` 只更新 fault cache，不按运动状态解码，不覆盖最后一次 `OPERATION_STATUS`，也不递增控制 ACK/status sequence；clear-error ACK 后清缓存。 | 注入故障帧不会产生伪 `-720 deg`、`-50 rad/s`、`0 C` 遥测；C API 可读取 fault/warning raw；清故障后缓存一致。 | |
| [ ] | R-09 | P0 | RobStride MIT 编码 | `b93b83d9` | 固定 MIT 参数的字段顺序、位宽、归一化和打包顺序，并建立 golden-vector 测试。 | C 编码字节与 motorbridge/厂商向量逐字节一致；反馈反解后在量化误差范围内。 | |
| [ ] | R-10 | P0 | RobStride 型号限值 | `4216ccb5` `bb5134ed` `082cbb8c` | 命令编码和反馈解码必须使用同一型号的 `vmax/tmax`；核对当前型号表，特别是 rs-05 `vmax=50`、`tmax=5.5`，以及 rs-00/03/06 的修正速度范围。 | 所有型号表与当前 motorbridge 及厂商手册一致；正负满量程和零点编码测试通过。 | |
| [ ] | R-11 | P0 | RobStride 状态位 | `29305d54` | `mode_state` 从扩展 ID 的 extra_data bit 14..15（原 CAN ID bit 22..23）读取；0=DISABLED、1=CALI、2=ENABLED；不得与故障位混淆。 | 构造 0/1/2 状态及叠加故障位的帧，解析结果均正确。 | |
| [ ] | R-12 | P0 | RobStride 固件版本 | `5e54fe15` | 版本请求 marker 为 `0xC4`；回复签名为 `[0x00,0xC4,0x56]`；由于版本回复与状态回复 comm type 都为 2，必须先匹配签名。 | 版本帧正确返回版本文本/字节；不会被状态解析器写成垃圾 telemetry；普通状态帧不误判为版本。 | |
| [ ] | R-13 | P0 | RobStride set-zero | `3a8f7259` | `SET_ZERO_POSITION`（comm type 6）发送 8 字节 `01 00 00 00 00 00 00 00`，不是 DLC=1；模式判断直接读取寄存器而非只信缓存。 | CAN 抓包 DLC=8 且 payload 正确；缓存与设备模式不一致时仍按设备实际状态处理。 | |
| [ ] | R-14 | P0 | RobStride 受控停止 | `4690e3f5` | stop 与 disable 分离；收到 stop 后先无条件取消周期命令。Velocity 写 0；PP 写 `PpVelocityMax=0`；CSP 读当前位置后写回；MIT 保持当前位置并复用 kp/kd，无缓存时用约上限 10% 的保守值。 | 即使 vendor stop 后续失败，也不再重发旧命令；四种模式分别有测试；C API 提供独立 stop，不以 disable 冒充。 | |
| [ ] | R-15 | P1 | RobStride 扩展供应商 | `a79aa296` | 若目标支持实验性 RobStride CiA402/MIT vendor，核对 vendor 选择、模式能力和“不支持”返回；不支持时明确 N/A。 | 不会把实验 vendor 当作标准 RobStride 自动混用；能力查询与实际行为一致。 | |
| [ ] | O-01 | P1 | 状态参数流 | `503fb153` `96dddf3d` | 若有遥测流，RobStride/Damiao observation 参数必须按请求周期读取、按 motor 隔离，并区分无值、超时和通信错误。 | 连续状态流不阻塞控制；不同 motor 参数不串线；停止订阅后不再轮询。 | |
| [ ] | D-01 | P0 | Damiao 模式切换 | `aa33be2d` `40194bc5` `f4adda09` `bbcc2da4` | 在统一 deadline 内执行模式切换；每次验证失败都重新写 mode register；区分 read reply 与 write ACK；ACK 值必须匹配；最多 3 次、ACK timeout 约 50 ms、retry gap 约 20 ms。 | 延迟回复、错误 ACK、丢包和回读不一致均不会伪成功；总耗时不突破预算。 | |
| [ ] | D-02 | P0 | Damiao 安全切换/保存 | `bbcc2da4` | 切模式前清零各命令通道，必要时保持当前位置；store_parameters 前停止 active ticker，确认或执行 disable，等待后再保存。 | 模式切换和保存期间不出现旧目标突跳；保存后断电重启参数保留；失败路径不会恢复发送旧命令。 | |
| [ ] | D-03 | P1 | Damiao 寄存器快照 | `cc1117e5` | 刷新时读取设备最新寄存器；成功的读写结果同步本地 snapshot，失败不得用陈旧值伪装成功。 | 外部修改寄存器后刷新可见；写后读一致；通信失败有明确状态。 | |
| [ ] | D-04 | P0 | Damiao handle 复用 | `55ed1fde` `ace6be0a` | dm-serial 遥测和状态轮询复用已注册 motor handle，不重复注册相同 ID，不为每轮轮询重建设备。 | 多电机长时间遥测无 duplicate-ID、句柄泄漏、周期性断流或控制阻塞。 | |
| [ ] | D-05 | P1 | dm-serial 超时 | `dadfe8dc` | 串口回复预算不能使用过窄的 1 ms；基准修正为约 10 ms，并允许在总 deadline 内配置。 | 在目标波特率和负载下统计超时率；10 ms 或经证据确认的预算可稳定收帧，错误仍能及时返回。 | |
| [ ] | D-06 | P1 | Damiao 型号别名 | `b4a839a9` | 支持 `4340_v20` 别名并显示为 `4340X`，或在目标工程的型号映射中提供等价兼容。 | 两个名称解析到同一正确参数规格；未知型号明确拒绝。 | |
| [ ] | A-01 | P0 | C ABI 线程安全 | `252568a6` | controller/motor 调用序列化；NULL、锁错误和内部错误返回明确错误码；`last_error` 为线程局部且调用方立即复制；free 与其他调用不可并发。 | 并发压力、NULL、重复/并发 free 和错误注入测试通过；无 motor 的 unbound controller 操作不得静默成功。 | |
| [ ] | A-02 | P0 | 生命周期 | `9dec7c26` | controller drop/free 时停止 polling 线程并 join，释放后后台任务不得访问 controller、bus 或 callback。 | ASan/TSan 或等价压力测试无 use-after-free、竞态、死锁和遗留线程。 | |
| [ ] | A-03 | P1 | ABI 能力契约 | `17f437b2` | C/C++/上层绑定的 API surface、能力位、transport/vendor 支持保持一致；不支持的能力不得返回成功。 | 自动比对头文件、导出符号和能力声明；版本不匹配时可诊断。 | |
| [ ] | A-04 | P0 | 回滚后的 ABI | `24c891a2` `70d3f403` | 确认最终实现没有移植已被回滚的 RobStride state-sequence 暴露行为，除非目标工程另有正式契约。 | API 与当前最终 motorbridge 契约一致；若保留扩展，名称空间和兼容性说明清楚。 | |
| [ ] | T-01 | P1 | macOS PCAN | `5723c20c` `f34fcf47` | 若目标支持 macOS PCBUSB，动态库候选至少覆盖 `libPCBUSB.dylib`、`$HOME/.local/lib/libPCBUSB.dylib` 和 `PCBUSB`，并汇总加载失败原因。 | 系统安装和用户本地安装都可加载；全部候选失败时错误包含逐项原因。嵌入式固件可标 N/A。 | |
| [ ] | T-02 | P1 | dm-device transport | `3f3d1302` `15fccaee` `437043d7` `cc888ed8` `77b40e50` `c5b17210` `0250fef8` `17ada053` | 若支持 Damiao DM_Device SDK，核对设备类型、双通道扫描、raw CAN、runtime ABI、平台/架构 gate 和缺 runtime 的诊断。 | 支持矩阵与运行时一致；双通道不会漏扫/重复；不支持平台在构建或初始化时明确失败。 | |
| [ ] | T-03 | P0 | mcu-serial transport | `0b8c19e7` | 实现帧 `[A5][LEN][can_ID LE4][DATA][CRC8][5A]`；CAN ID bit31 表示 extended；CRC8 poly `0x07` 覆盖 LEN、ID、DATA；只支持 classic CAN 8 字节；首个 motor 添加时 lazy open。 | golden frame、CRC 错误、截断/粘包、标准/扩展 ID 和最大 DLC 测试通过；CAN-FD/Hexfellow 明确拒绝；必须完成实机验证。 | |
| [ ] | T-04 | P1 | Windows PCAN | `895e0a28` | controller 扫描和正常操作不得高频反复 Initialize/Uninitialize PCAN；断线恢复采用受控重连。 | Windows PCAN 实机连续扫描/轮询稳定，无初始化风暴和驱动锁死。 | |
| [ ] | CBR-01 | P1 | CyberBeast | `53fcddb2` `18acca8b` | 若支持 CyberBeast，实现 29-bit J1939 风格扩展 ID、2-bit sequence、big-endian float32、8-byte MIT、heartbeat/query/SDO、控制与状态路由。 | 编解码 golden vectors 与实机测试通过；start/stop/zero/clear/estop 正确；verify/set-id 等未支持操作明确返回 unsupported。 | |
| [ ] | S-01 | P1 | 上层控制安全 | `7b1bd809` `faae4ed9` `701fbf3c` `ef686e02` `18827d03` `777c5f59` `3581ded8` | 若 C 工程含控制应用，检查实时滑杆限频、MIT 参数限幅、危险动作确认、安全动作序列、zero-all 后状态刷新和非阻塞警告。 | 快速输入不会淹没总线；危险操作不能误触；异常中断进入可预测安全状态。纯协议库可标 N/A。 | |
| [ ] | S-02 | P0 | 夹爪零点门禁 | `c48ebc4b` | 批量 enable 前先检查 motor ID 7：RobStride 读 `0x7019 mechPos`，容差 `+/-5 deg`；Damiao 读寄存器 `0x50`（80），容差 `+/-3 deg`；非有限值或超范围拒绝整个批次。 | 检查发生在任何电机上电之前；无 ID 7 时跳过；真实读取错误向上传播；超限时所有电机均保持未使能。 | |

## 推荐验证组合

| 状态 | 测试层级 | 最低要求 | 结果/报告链接 |
|---|---|---|---|
| [ ] | 静态对照 | 按核心检查表定位目标 C 工程文件、函数、常量和错误路径。 | |
| [ ] | 单元测试 | 协议编解码、CRC、位域、端序、型号限值、寄存器索引和错误码使用 golden vectors。 | |
| [ ] | 总线仿真 | 注入超时、乱序、错误 ACK、故障帧、未知参数、截断帧和并发回复。 | |
| [ ] | 并发与生命周期 | 多线程调用、销毁时轮询、重复初始化、断线重连，建议使用 ASan/TSan。 | |
| [ ] | 实机回归 | 至少覆盖实际使用的每种 transport、vendor、motor 型号和固件版本。 | |
| [ ] | 安全回归 | 验证 stop、disable、set-zero、改 ID、参数保存、批量 enable 和断电重启。 | |

## 完整 Commit 追踪矩阵

“映射”列中的 ID 指向上方核心行为检查表。`审阅`表示需对照目标工程的对应上层/构建/文档范围决定是否适用；`追踪`表示提交本身不形成新的 C 运行时行为，但仍保留历史记录。

### 2026-04

| 状态 | 日期 | Commit | 原始主题 | 分类 | C 工程结论/映射 |
|---|---|---|---|---|---|
| [ ] | 04-13 | `5723c20c` | feat(macos): add PCBUSB PCAN backend support and docs | CI/构建 | T-01；嵌入式 C 工程可 N/A |
| [ ] | 04-13 | `5c4b2ecc` | chore(macos): bundle PCBUSB archive and add one-click setup script | 发布 | T-01；检查分发工程时审阅 |
| [ ] | 04-13 | `86f26c00` | docs(readme): add manual PCBUSB download steps and fix zh-cn encoding | 文档 | T-01 的安装说明；否则 N/A |
| [ ] | 04-13 | `509397c0` | release(python): prepare v0.1.8 with bundled ws_gateway | 发布 | 追踪；无直接 C 行为 |
| [ ] | 04-13 | `fcda29f7` | feat(python): add standalone motorbridge-gateway entrypoint | 网关/上层 | G-01/G-03；无网关则 N/A |
| [ ] | 04-13 | `663ae5bf` | docs(python-mintlify): document standalone motorbridge-gateway usage | 文档 | G-01；无网关则 N/A |
| [ ] | 04-13 | `541e34c2` | release(python): bump to v0.1.9 and sync docs | 发布 | 追踪；无直接 C 行为 |
| [ ] | 04-13 | `70ab6a0a` | docs(gateway): add portable macOS ws_gateway startup instructions | 文档 | G-01/T-01；无网关则 N/A |
| [ ] | 04-13 | `4aa30504` | docs(mintlify): add macOS DYLD fallback for gateway launch | 文档 | T-01；无 macOS 则 N/A |
| [ ] | 04-13 | `7b1bd809` | feat(factory-ui): add live slider move mode for robot arm joints | 网关/上层 | S-01 |
| [ ] | 04-13 | `b022d5be` | Improve gateway UX across platforms and refine factory calib UI | 网关/上层 | S-01；其余审阅 |
| [ ] | 04-13 | `836700db` | docs: full audit and align CLI/gateway capability docs | 文档 | A-03/G-01；无对应上层则 N/A |
| [ ] | 04-13 | `34743234` | docs+ui: rename assistant and expand cross-platform install/transport guidance | 文档 | T-01/T-02；主要为追踪 |
| [ ] | 04-13 | `cf87ecbe` | refactor: add req_id WS matching and reuse controllers in python scan | 网关/上层 | G-01 |
| [ ] | 04-13 | `6d90be39` | refactor(ws_gateway): split shared model and session layers | 网关/上层 | G-01 |
| [ ] | 04-13 | `9db93e20` | refactor(ws_gateway): decouple websocket routing and op handlers | 网关/上层 | G-01 |
| [ ] | 04-13 | `9e982aeb` | refactor(ws_gateway): split ops into focused modules under 300 lines | 网关/上层 | G-01；检查功能隔离 |
| [ ] | 04-13 | `b2b67bfa` | refactor(ws_gateway): unify vendor naming and command module layout | 网关/上层 | G-01/A-03 |
| [ ] | 04-14 | `900d7fcf` | docs: fix mojibake and align RobStride CLI guidance | 文档 | R-01 至 R-13 的说明一致性 |
| [ ] | 04-14 | `97e531ff` | python-cli: add robstride zero/set-zero and sync docs | 网关/上层 | R-07/R-13；无 CLI 则审阅 |
| [ ] | 04-14 | `5347c465` | docs: clarify robstride socketcan-only channel note | 文档 | R-03/T-04；检查限制是否明确 |
| [ ] | 04-14 | `96c88f55` | docs(robstride): unify core/python usage mapping and parameter validity | 文档 | R-06；检查能力声明 |
| [ ] | 04-14 | `e30af00b` | docs(mintlify): sync zh/en chapters and align latest sdk behavior | 文档 | A-03；追踪 |
| [ ] | 04-14 | `c87ba3be` | docs(mintlify): refine cli/state docs and tutorial consistency | 文档 | R-11/O-01；追踪 |
| [ ] | 04-14 | `fee4cad9` | core: pace enable/disable bursts with configurable bulk op gap | C-行为 | C-01 |
| [ ] | 04-14 | `7289487c` | core: add paced tx for multi-motor loops and configurable bulk op gap | C-行为 | C-02 |
| [ ] | 04-14 | `30fa4d38` | python: bump package version to 0.2.2 | 发布 | 追踪；无直接 C 行为 |
| [ ] | 04-14 | `f95de96e` | ci(release): add macos arm64 wheel job to ABI release workflow | CI/构建 | T-01；目标无 wheel 则 N/A |
| [ ] | 04-14 | `406795a0` | docs: sync release/versioning and setup guidance across bindings and examples | 文档 | A-03；追踪 |
| [ ] | 04-15 | `faae4ed9` | feat(ui): upgrade reBot arm safety flow, parameter tooling, and help center docs | 网关/上层 | S-01 |
| [ ] | 04-15 | `701fbf3c` | ui(ws): improve robot-arm safety flow and expand help setup docs | 网关/上层 | S-01/G-01 |
| [ ] | 04-16 | `2fd913c6` | release: v0.2.3 | 发布 | 追踪；无直接 C 行为 |
| [ ] | 04-16 | `6e69efd8` | docs/release: align ws bind security and python packaging flow | 文档 | G-03；无网关则 N/A |
| [ ] | 04-16 | `d2177781` | chore: checkpoint current bindings and factory ui todo | 维护 | 追踪；无直接 C 行为 |
| [ ] | 04-16 | `0241dc75` | chore(factory-ui): remove duplicated urdf asset tree | 网关/上层 | N/A；仅资源整理 |
| [ ] | 04-16 | `2f727343` | refactor(factory-ui): complete P0 shared hooks/components hardening | 网关/上层 | S-01；无 UI 则 N/A |
| [ ] | 04-16 | `ba36354e` | refactor(factory-ui): complete remaining TODO with modular hooks and infra | 网关/上层 | S-01；无 UI 则 N/A |
| [ ] | 04-16 | `ec29e47b` | fix(ci): add can_debugging.md reference in factory-ui TODO | CI/构建 | 文档追踪；无 UI 则 N/A |
| [ ] | 04-17 | `ef686e02` | feat(robot-arm): safe demo sequence and non-blocking beta warning | 网关/上层 | S-01 |
| [ ] | 04-17 | `18827d03` | feat(factory-ui): enhance robot-arm replay workflow and sequence library | 网关/上层 | S-01；检查回放安全 |
| [ ] | 04-17 | `822dffe9` | fix(ws+ui): stabilize robstride mode switching and restore pos_vel flow | C-行为 | R-01/R-02 |
| [ ] | 04-21 | `777c5f59` | feat(ui,docs): expand motor advanced controls and sync python mintlify APIs | 网关/上层 | S-01/A-03 |
| [ ] | 04-21 | `3581ded8` | feat(ui): improve help center UX, zero-all safety refresh, and advanced risk modal | 网关/上层 | S-01 |
| [ ] | 04-22 | `cc1117e5` | fix(factory-calib-ui-ws): sync latest Damiao register values on refresh | C-行为 | D-03 |
| [ ] | 04-22 | `dd3fd855` | feat(factory-calib-ui-ws): add branded logo and forensic watermark background | 网关/上层 | N/A；仅 UI 资源 |
| [ ] | 04-23 | `73c9949f` | refactor(repo): extract studio UI out of motorbridge | 网关/上层 | S-01；确认控制安全未丢失 |
| [ ] | 04-23 | `a9667da3` | docs(readme): add motorbridge-studio repo link | 文档 | 追踪；无直接 C 行为 |
| [ ] | 04-23 | `32bc053e` | chore(deps): bump rand from 0.8.5 to 0.8.6 | 构建 | N/A；依赖版本 |
| [ ] | 04-27 | `99ecc65a` | fix(robstride): improve parameter compatibility and scan defaults | C-行为 | R-03/R-04/R-06 |
| [ ] | 04-27 | `1472242f` | chore(python): bump motorbridge package version to 0.2.4 | 发布 | 追踪；无直接 C 行为 |
| [ ] | 04-28 | `f7037b09` | chore: add linguist-detectable=false for mdx/md files | 维护 | N/A；仓库元数据 |

### 2026-05

| 状态 | 日期 | Commit | 原始主题 | 分类 | C 工程结论/映射 |
|---|---|---|---|---|---|
| [ ] | 05-09 | `21c12d7a` | Release Python package 0.2.5 | 发布 | 追踪；无直接 C 行为 |
| [ ] | 05-09 | `a258ec5c` | fix(robstride): align scan and id handling for 0.2.6 | C-行为 | R-03/R-04 |
| [ ] | 05-09 | `b9f70658` | docs: add CAN troubleshooting link to release note | 文档 | C-01/C-02/T-04；追踪 |
| [ ] | 05-12 | `faba15fa` | feat: add RobStride model-specific parameter tables | C-行为 | R-06 |
| [ ] | 05-12 | `bd0d2cdc` | fix: keep RobStride runtime table to protocol params | C-行为 | R-06 |
| [ ] | 05-12 | `4289a22b` | fix: complete RobStride runtime parameter handling | C-行为 | R-06 |
| [ ] | 05-12 | `40ffc379` | feat: add RobStride zero range option | C-行为 | R-07 |
| [ ] | 05-12 | `4ca39489` | restore: return motorbridge to v0.2.6 tree | 恢复 | 审阅最终树；不得把随后不存在的中间行为当要求 |
| [ ] | 05-12 | `c074439d` | fix: complete RobStride section 4 parameters | C-行为 | R-06 |
| [ ] | 05-12 | `0db9ec1c` | fix: set RobStride zero range during zeroing | C-行为 | R-07 |
| [ ] | 05-12 | `93114f74` | chore: bump motorbridge release to 0.2.8 | 发布 | 追踪；无直接 C 行为 |
| [ ] | 05-13 | `a2b50edb` | docs: annotate RobStride runtime parameter ranges | 文档/测试 | R-06；转为边界测试 |
| [ ] | 05-13 | `261695b3` | Merge pull request #6 from motorbridge/dependabot/cargo/rand-0.8.6 | Merge | N/A；内容由 `32bc053e` 覆盖 |
| [ ] | 05-13 | `681f021c` | docs: document PCAN CAN restart helper | 文档 | T-04；检查受控重连策略 |
| [ ] | 05-14 | `dc056084` | release: v0.2.9 | 发布 | 追踪；无直接 C 行为 |
| [ ] | 05-14 | `2719b50b` | ci: build runtime artifacts before python sdist | CI/构建 | A-03；目标无 Python sdist 则 N/A |
| [ ] | 05-14 | `b3b35ace` | fix: keep python sdist source-only | CI/构建 | N/A；Python 打包策略 |
| [ ] | 05-15 | `36c6cd24` | Update LICENSE | 维护 | N/A；许可证文件 |
| [ ] | 05-15 | `10ec02a1` | release: v0.3.0 | 发布 | 追踪；无直接 C 行为 |
| [ ] | 05-15 | `a106434b` | Release v0.3.1 python CLI timing fix | 发布/上层 | R-01/C-01；审阅 CLI 时序 |
| [ ] | 05-15 | `f2d57943` | Harden RobStride mode switching | C-行为 | R-01 |
| [ ] | 05-18 | `9908d0fa` | fix robstride fault report handling | C-行为 | R-08 |
| [ ] | 05-18 | `bcfc3c18` | release 0.3.2 | 发布 | 追踪；无直接 C 行为 |
| [ ] | 05-19 | `3449516e` | release 0.3.3 | 发布 | 追踪；无直接 C 行为 |
| [ ] | 05-20 | `c993b9d7` | release: v0.3.4 | 发布 | 追踪；无直接 C 行为 |
| [ ] | 05-20 | `252568a6` | fix: harden ABI and split Python CLI | C-行为 | A-01/A-03 |
| [ ] | 05-20 | `6520e2eb` | fix: tighten CI and CAN backend docs | CI/构建 | T-01/T-04；审阅 |
| [ ] | 05-20 | `b93b83d9` | test: lock RobStride MIT argument encoding | 测试 | R-09 |
| [ ] | 05-20 | `f7e06b3e` | chore: align package metadata and lint config | 构建 | N/A；元数据和 lint |
| [ ] | 05-20 | `9dec7c26` | fix: stop controller polling on drop | C-行为 | A-02 |
| [ ] | 05-20 | `e36cefdc` | ci: allow websocket handshake error size lint | CI/构建 | G-01；仅编译/lint 追踪 |
| [ ] | 05-20 | `bc24a7bd` | release: prepare v0.3.5 | 发布 | 追踪；无直接 C 行为 |
| [ ] | 05-20 | `24c891a2` | fix: expose robstride state sequence | C-行为 | A-04；此行为随后回滚 |
| [ ] | 05-20 | `70d3f403` | Revert "fix: expose robstride state sequence" | Revert | A-04；最终不得按上一提交移植 |
| [ ] | 05-20 | `503fb153` | feat: stream robstride observation parameters | C-行为 | O-01 |
| [ ] | 05-20 | `96dddf3d` | feat: support damiao observation parameter stream | C-行为 | O-01 |
| [ ] | 05-21 | `041e27dc` | Release v0.3.6 | 发布 | 追踪；无直接 C 行为 |
| [ ] | 05-21 | `a6c14d71` | Release v0.3.7 | 发布 | 追踪；无直接 C 行为 |
| [ ] | 05-26 | `bb341336` | Release v0.3.8 | 发布 | 追踪；无直接 C 行为 |
| [ ] | 05-26 | `23c22998` | Release v0.3.9 | 发布 | 追踪；无直接 C 行为 |

### 2026-06

| 状态 | 日期 | Commit | 原始主题 | 分类 | C 工程结论/映射 |
|---|---|---|---|---|---|
| [ ] | 06-02 | `55ed1fde` | fix(gateway): stabilize Damiao dm-serial arm telemetry | C-行为 | D-04/D-05/O-01 |
| [ ] | 06-02 | `4a25896e` | chore: release motorbridge v0.4.0 | 发布 | 追踪；无直接 C 行为 |
| [ ] | 06-02 | `a273943f` | fix(python): avoid CAN preflight for help and dm-serial | 网关/上层 | T-03；不同 transport 不应错误依赖 CAN 预检 |
| [ ] | 06-02 | `17f437b2` | feat(bindings): align C++ API and expose ABI capabilities | C-行为 | A-03 |
| [ ] | 06-02 | `ce26d4db` | chore: release motorbridge v0.4.1 | 发布 | 追踪；无直接 C 行为 |
| [ ] | 06-03 | `a0f02bd4` | chore: release motorbridge v0.4.2 | 发布 | 追踪；无直接 C 行为 |
| [ ] | 06-03 | `3f3d1302` | feat(dm-device): add Damiao SDK transport | C-行为 | T-02 |
| [ ] | 06-03 | `0fb0fb28` | docs: document dm-device v0.4.3 release | 文档 | T-02 |
| [ ] | 06-03 | `15fccaee` | docs: clarify dm-device dual-channel scan | 文档/行为 | T-02 |
| [ ] | 06-03 | `f56d10c4` | ci: install aarch64 C++ cross compiler | CI/构建 | A-03；目标支持 aarch64 时审阅 |
| [ ] | 06-03 | `437043d7` | build: gate dm-device by vendored runtime | CI/构建 | T-02 |
| [ ] | 06-03 | `cc888ed8` | build: restrict dm-device windows runtime to x86_64 | CI/构建 | T-02 |
| [ ] | 06-03 | `aa33be2d` | fix(ws): soften Damiao mode verify timeouts | C-行为 | D-01 |
| [ ] | 06-03 | `40194bc5` | fix(ws): verify Damiao register writes by readback | C-行为 | D-01/D-03 |
| [ ] | 06-03 | `77b40e50` | ci: omit dm-device runtime from manylinux wheels | CI/构建 | T-02；目标无 wheel 则 N/A |
| [ ] | 06-03 | `c5b17210` | feat(python): install dm-device runtime on demand | 网关/上层 | T-02；目标无运行时安装器则 N/A |
| [ ] | 06-03 | `8c2ab35b` | docs: add dm-device runtime support matrix | 文档 | T-02 |
| [ ] | 06-03 | `0250fef8` | fix(python): prompt for dm-device runtime setup | 网关/上层 | T-02；缺 runtime 必须可诊断 |
| [ ] | 06-03 | `17ada053` | docs: document dm-device runtime ABI requirements | 文档 | T-02/A-03 |
| [ ] | 06-03 | `02121d33` | docs: finalize v0.4.3 release notes | 文档 | T-02；追踪 |
| [ ] | 06-04 | `63222851` | release: prepare v0.4.4 | 发布 | 追踪；无直接 C 行为 |
| [ ] | 06-04 | `a79aa296` | Add experimental RobStride CiA402 and MIT vendors | C-行为 | R-15 |
| [ ] | 06-04 | `28929bde` | Document RobStride protocol ID roles | 文档/行为 | R-04 |
| [ ] | 06-04 | `852fb3f3` | Fix clippy warning in motor CLI defaults | 构建 | N/A；Rust CLI lint |
| [ ] | 06-04 | `e3452da7` | Add hardware gallery media | 文档 | N/A；媒体资源 |
| [ ] | 06-05 | `f4adda09` | fix(damiao): retry mode writes during ensure verification | C-行为 | D-01 |
| [ ] | 06-05 | `73443c70` | release: prepare v0.4.5 | 发布 | 追踪；无直接 C 行为 |
| [ ] | 06-05 | `ace6be0a` | fix(ws_gateway): reuse existing Damiao motors during state polling | C-行为 | D-04 |
| [ ] | 06-08 | `53fe6ddc` | Merge pull request #8 from KillingJacky/fix/damiao-state-reuse | Merge | N/A；内容由 `ace6be0a` 覆盖 |
| [ ] | 06-12 | `13c64142` | fix(ws_gateway): accept browser token in websocket query | 网关/上层 | G-03 |
| [ ] | 06-12 | `bb4e0c5b` | Merge branch 'feat/ws-token-query-str' | Merge | N/A；内容由 `13c64142` 覆盖 |
| [ ] | 06-12 | `b4a839a9` | feat: add Damiao 4340_v20 alias with 4340X display name | C-行为 | D-06 |
| [ ] | 06-12 | `38b8a568` | release: prepare v0.4.6 | 发布 | 追踪；无直接 C 行为 |
| [ ] | 06-16 | `bbcc2da4` | fix(damiao): harden mode switching and parameter persistence | C-行为 | D-01/D-02 |
| [ ] | 06-16 | `dadfe8dc` | fix(dm-serial): relax serial timeout budget | C-行为 | D-05 |
| [ ] | 06-16 | `2b7b3509` | release: prepare v0.4.7 | 发布 | 追踪；无直接 C 行为 |
| [ ] | 06-16 | `19e81a71` | chore(ci): limit abi release trigger and apply rustfmt | CI/构建 | N/A；CI 触发与格式化 |
| [ ] | 06-23 | `43c341ae` | fix(ws-gateway): scope single-motor commands to explicit targets | C-行为 | G-02 |
| [ ] | 06-24 | `29305d54` | feat: add RS motor status parse/display and align DM naming | C-行为 | R-11/D-06 |
| [ ] | 06-24 | `123a6ad5` | release: prepare v0.4.8 | 发布 | 追踪；无直接 C 行为 |

### 2026-07

| 状态 | 日期 | Commit | 原始主题 | 分类 | C 工程结论/映射 |
|---|---|---|---|---|---|
| [ ] | 07-03 | `4216ccb5` | 更新了tmax参数与值与电机厂商提供一致 | C-行为 | R-10 |
| [ ] | 07-06 | `b5c835cc` | Merge pull request #9 from Welt-liu/main | Merge | N/A；合并内容由相邻功能提交覆盖 |
| [ ] | 07-06 | `a3db2fbd` | release: prepare v0.4.9 | 发布 | 追踪；无直接 C 行为 |
| [ ] | 07-11 | `5e54fe15` | feat: add RobStride firmware version request and reply decoding | C-行为 | R-12 |
| [ ] | 07-21 | `1ff2ed7b` | Merge pull request #10 from infoslack/feat/robstride-firmware-version | Merge | N/A；内容由 `5e54fe15` 覆盖 |
| [ ] | 07-21 | `bb5134ed` | fix: align rs-05 velocity and torque limits with the RS05 manual | C-行为 | R-10 |
| [ ] | 07-22 | `1397a3c8` | Merge pull request #11 from infoslack/fix/robstride-rs05-limits | Merge | N/A；内容由 `bb5134ed` 覆盖 |
| [ ] | 07-22 | `082cbb8c` | fix(robstride): correct rs velocity normalization range | C-行为 | R-10 |
| [ ] | 07-22 | `bec79fb5` | chore: replace tianrking org references with motorbridge | 维护 | N/A；仓库链接 |
| [ ] | 07-22 | `14468bca` | Merge branch 'fix/urls' | Merge | N/A；内容由 `bec79fb5` 覆盖 |
| [ ] | 07-29 | `53fcddb2` | feat/cyberbeast: add CyberBeast CAN protocol vendor | C-行为 | CBR-01 |
| [ ] | 07-29 | `18acca8b` | feat/cyberbeast/ws_gateway: CyberBeast protocol WS gateway integration | C-行为 | CBR-01/G-01 |

### 2026-08

| 状态 | 日期 | Commit | 原始主题 | 分类 | C 工程结论/映射 |
|---|---|---|---|---|---|
| [ ] | 08-05 | `4690e3f5` | feat(robstride): add mode-aware controlled stop and harden handle_stop | C-行为 | R-14 |
| [ ] | 08-06 | `295c5398` | fix(robstride): route pos_vel velocity to PP vel_max register | C-行为 | R-02 |
| [ ] | 08-07 | `e9ec70e4` | fix(python-cli): store after id-set via new-id handle | C-行为 | R-05；C 工程需确认 handle 语义 |
| [ ] | 08-21 | `f34fcf47` | fix(macos): load user-local PCBUSB runtime | CI/构建 | T-01；非 macOS 可 N/A |
| [ ] | 08-21 | `1299261e` | Merge branch 'fix/pcan-native-loader' | Merge | N/A；内容由 `f34fcf47` 覆盖 |

### 2026-09

| 状态 | 日期 | Commit | 原始主题 | 分类 | C 工程结论/映射 |
|---|---|---|---|---|---|
| [ ] | 09-01 | `3a8f7259` | fix(robstride): set-zero DLC=8 and bump version to 0.5.2 | C-行为 | R-13/R-07 |
| [ ] | 09-01 | `895e0a28` | docs(zh): document Windows PCAN high-frequency re-init limitation | 文档/行为 | T-04；转为实机回归约束 |
| [ ] | 09-04 | `0b8c19e7` | feat(abi): add mcu-serial UART-to-CAN MCU bridge transport | C-行为 | T-03 |
| [ ] | 09-05 | `c48ebc4b` | fix(arm): hotfix 0.5.3 — zero-point gate for gripper travel ends | C-行为 | S-02 |

## 汇总签字

| 项目 | 数量/内容 |
|---|---|
| 提交总数 | 152 |
| 核心检查项总数 | 38 |
| 已通过 `[x]` | 待填写 |
| 有差异 `[!]` | 待填写 |
| 不适用 `[-]` | 待填写 |
| 未检查 `[ ]` | 待填写 |
| 审核人 | |
| 审核日期 | |
| 目标 C 工程版本/Commit | |
| 目标设备/固件 | |
| 总体结论 | 待填写 |

## Git 对照命令

在本仓库根目录执行以下命令，可重新生成基准列表并检查文档是否保留全部提交：

```bash
git log --all \
  --date=short \
  --pretty=format:'%H%x09%ad%x09%s' \
  --after='2026-04-09T23:59:59+08:00' \
  --reverse
```

提交总数校验：

```bash
test "$(git rev-list --count --all --after='2026-04-09T23:59:59+08:00')" -eq 152
```
