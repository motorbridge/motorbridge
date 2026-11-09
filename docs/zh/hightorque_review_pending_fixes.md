# HighTorque 代码评审待修问题清单

> 评审日期：2026-10-17
> 评审范围：HT-develop 工作区未提交改动(41 文件，约 +2969/-395 行;commit e81f0ec 之后的 hightorque v2.0.0 收敛工作)
> 评审基线:`cargo check --workspace --all-targets` 通过,hightorque / motor_abi / motor_cli 测试套件全绿。以下问题均为测试未覆盖的逻辑/契约问题。
> 处理顺序约定:**先记录(本文档)→ 同步 main → 再修复**。修复时逐条勾选。

---

## A 类:会造成错误或危险行为(优先修)

### A1. `request_firmware_version` 仍与后台收帧线程抢帧

- 位置:`motor_vendors/hightorque/src/motor.rs:341`
- 现象:真实总线 + PollingMode::Background(默认)下,电机对 `15 B5 02` 回 5 字节版本帧;CoreController 后台线程(约 200µs 周期性排空 RX 队列)先抢到该帧,`process_feedback_frame` 将其**静默丢弃**(无任何缓存版本回复的路径),导致 ABI `motor_handle_hightorque_request_firmware_version` / CLI `--mode version` / ws_gateway `version` op 随机超时。
- 背景:本次改动已为 `read_registers` 修复了同类竞态(不再与后台线程在同一总线队列抢帧),但版本查询仍留在旧路径上。
- 备注:Mock 测试中回复在 FFI recv 前微秒级推送,竞态不会出现,故 CI 无法发现。
- [x] 已解决(2026-10-17 决策:直接移除版本查询功能接口,全链路删除——vendor `request_firmware_version` / protocol `decode_version_reply`+`FirmwareVersion` / ABI FFI 符号 / capabilities 条目 / Python SDK / CLI `--mode version` / ws_gateway hightorque 分支 / demo。协议文档 §通信命令13 保留帧格式并注明移除原因)

### A2. ws_gateway `tqe` 传非数字 JSON 类型时静默编码为力矩无限制

- 位置:`integrations/ws_gateway/src/router/handlers/control_aux.rs:62`
- 现象:`{"op":"pos_classic","pos":1.0,"tqe":"1.5"}`(字符串)→ `Value::as_f64()` 返回 `None` → `send_cmd_pos_classic(pos, None)` → 帧 `07 07` 字节 6-7 填 `00 80`(0x8000 无限制力矩哨兵)。操作者以为设了 1.5 Nm 限幅,实际电机无限力矩运行。
- 期望:安全相关的限幅字段类型错误应返回错误,而不是降级为无限制。
- [x] 已解决(2026-10-19:control_aux.rs handle_pos_classic 的 tqe 解析改为区分"字段缺席=故意不限(None)"与"字段存在但非数字=报错拒绝";回归测试验证四种非数字类型均报错且控制帧零发出;连接层 ops 列表的 version 残留一并清理;协议文档 §14.2 同步)

### A3. 寄存器缓存键只含地址,不含类型

- 位置:`motor_vendors/hightorque/src/motor.rs:591`
- 现象:周期反馈流 / ws_gateway `request_feedback` 以 Int16 解码并缓存 0x01–0x03(`27 01` 回复);此时并发调用 `read_registers(&[ReadCmd{addr:0x01, ty:Int8, count:3}])`,`fresh_registers` 发现缓存 `ts >= request_at` 直接返回 Int16 解码值(如 I16(0x1234)=4660 被当作 Int8 寄存器值,实际应为 0x12=18),真正的 Int8 回复从未被查询。
- 期望:缓存键应为 `(addr, RegisterType)`;"按地址入各自的缓存键,不会误认"的说法只在地址不重叠时成立。
- [x] 已解决(2026-10-21:缓存键改为 `(addr, RegisterType)`,写入方经新增 `RegisterValue::register_type()` 取解码变体自带的类型;`fresh_registers` 按 `(addr, ty)` 认领;`cached_register` 保持"该地址最近值"语义——跨类型取入缓时刻最新的一条,FFI `hightorque_get_cached_register`(只回数值不带类型)ABI 零改动。`RegisterType` 补 `Hash` derive。回归测试 `read_registers_rejects_fresh_reply_of_different_type`:先到 Int16 后到 Int8 的 0x0E 回复,显式 Int8 读必须返回 I8 而非认领别人的 I16(旧实现在此错拿 I16)。注:寄存器默认按 Int16 解算的决策只修了默认路径的类型一致性,本项是显式指定 ty 读的独立问题)

### A4. `ABI_CAPABILITIES` 仍声明已删除的 param_* FFI 能力(未完成收尾)

- 位置:`motor_abi/src/lib.rs:47`
- 现象:本次改动删除了 `motor_abi/src/param_ffi/hightorque/` 与 `vendor_params/hightorque.rs`(即 `motor_handle_hightorque_{get,write}_param_*` FFI 符号),但能力声明 JSON 仍含 hightorque 的 `param_i8/param_u8/param_u16/param_u32/param_f32`。
- 后果:消费者查询 `motor_abi_capabilities_json`(Python `abi_capabilities()`)后按声明调用,得到 AttributeError / missing symbol,而非原先干净的"不支持"错误返回。能力声明行在同一 diff 中被编辑过,陈旧条目具有误导性。
- [x] 已解决(2026-10-17 合并 origin/main 时消除:hightorque 行已不含 param_* 条目)

---

## B 类:边界情况返回错误结果

### B1. `17 01` 状态查询帧存在 8 字节与 2 字节两种布局

- 位置:`motor_vendors/hightorque/src/motor.rs:357`
- 现象:`request_motor_feedback`(CLI `--mode read`/`ping`、ws_gateway `read` op)发 8 字节零填充帧(dlc 8,data `17 01 00…`);新 `request_registers` 路径(ws_gateway `request_feedback`、`read-reg`)发 2 字节帧(dlc 2),且更新后的协议文档 §通信命令8 记载 `17 01` 应为 dlc 2。同一命令两处布局不一致,其一必然不合规范。
- 后果:对校验 DLC 的固件,dlc-8 形式被拒收,阻塞式状态读全部超时,而同总线非阻塞路径正常——故障表现取决于调用方走了哪条 API。
- [x] 已解决(2026-10-21,用户决策"去掉阻塞式方法,将 DM 对齐命名归还非阻塞查询":**阻塞式 `request_motor_feedback(timeout)` 整体删除**,其 8 字节零填充帧随之消失;`request_motor_feedback()` 按 DM 语义重建为非阻塞"发完即走"(内部经 `request_registers` 发 2 字节规范帧,`17 01` 从此只有一种帧型)。阻塞需求由新增的 DM 对齐 `request_fresh_state(timeout)` 承担(发帧前记 `request_at`,轮询状态缓存等 `received_at >= request_at`)。原 `wait_status` 一并移除(其 `bus.recv` 抢帧分支曾绕过后台线程,27 01 回帧被它吃掉时寄存器缓存漏喂)。调用方迁移:ABI(500ms,保持 record 脚本依赖的阻塞+超时语义)、CLI ping/read、ws_gateway read op / 会话刷新循环 / vendor_ops 探测均改 `request_fresh_state`;回归测试 `request_motor_feedback_sends_canonical_two_byte_frame`(dlc 2)+ `request_fresh_state_ok_when_worker_consumes_reply`。**2026-10-23 追补注**:ABI `motor_handle_request_feedback` HT 分支改回非阻塞 `request_motor_feedback()`(用户决策,对齐网关 op 与 DM 语义;record_play.py 已不存在,依赖消失)——阻塞需求由 CLI/网关 read 的 `request_fresh_state` 与 ABI `hightorque_get_register(0x01,3)` 承担,见 fullchain 审计 §16)

### B2. 寄存器地址越界被静默截断

- 位置:`integrations/ws_gateway/src/router/handlers/control_aux.rs:155`(read 侧)、同文件 289 行(write 侧);CLI 侧 `motor_cli` 的 `get_u16_hex_or_dec(...)? as u8` 同病。
- 现象:`{"op":"read_register","addr":300}`(0x12C)→ `as u8` = 0x2C,读/写了错误的寄存器并把结果当作请求的那个返回。CLI `--mode read-reg --addr 0x145` 实际读 0x45。
- 期望:越界地址应返回错误,而非截断后返回一个"看起来合理"的错误结果。
- [x] 已解决(2026-10-21,用户决策"单个 addr 合法域 0x00-0xFF,越界直接拒绝":四处 `as u8` 截断改为显式校验——ws_gateway read/write 两处([control_aux.rs](integrations/ws_gateway/src/router/handlers/control_aux.rs))改为 `u8::try_from` + 报错(报错带原始值);CLI read-reg/write-reg 抽出 `reg_addr_from_args` 辅助函数统一解析+校验,顺带把缺参提示从过窄的 0x00-0x45 修正为 0x00-0xFF。回归测试:网关 `read_write_register_reject_out_of_range_addr`(读/写越界均拒绝且总线零帧;原"边界 0xFF 照常发帧"子用例在 2026-10-22 追补表校验后改用表内最高地址 0x45)、CLI `reg_addr_rejects_out_of_range`(0x100/0x145 拒绝、0x00/0xFF 放行、缺参报错)。注:连续读的地址推进回绕(0xFF+count)是 vendor 侧地址算术语义(B3 已修),不属于入参校验范围,单地址入参合法域即 0x00-0xFF)

### B3. 地址 0xFF 起连续读必然超时(fresh 与 decode 的地址推进算法不一致)

- 位置:`motor_vendors/hightorque/src/motor.rs:440`
- 现象:`fresh_registers` 用 `u8::try_from(addr + i)`(0x100 溢出返回 None,永不相等),`decode_register_reply` 用 `addr.wrapping_add(i)`(0xFF+1 → 0x00)。
- 后果:`read_registers(addr=0xFF, count=2)`:回复按 wrapping 地址(0xFF、0x00)解码并写入缓存,但查找方在等 0xFF 和 0x100,循环直到超时,报"HighTorque register read timeout"——尽管回复已到达且已缓存。
- [x] 已解决(2026-10-21:随 A3 一并修复——`fresh_registers` 的地址推进改为 `c.addr.wrapping_add(i as u8)`,与 `decode_register_reply` 统一;类型键 `(addr, c.ty)` 认领。回归测试 `read_registers_wraps_addr_advance_at_0xff`:0xFF 起连续读 2,回复含 0xFF/0x00(回绕)两值,必须全部认领(旧实现在此超时)。**2026-10-22 追补注**:同日"缺号/表外地址拒绝访问"决策落地后,0xFF 属表外、读取被整段拒绝,回绕在自家请求路径不可达(表最高地址 0x45);该测试改写为拒绝测试 `read_registers_rejects_unknown_addr_span`,回绕语义仅保留给解码侧处理外来帧,详见 fullchain 审计 §14.1)

---

## C 类:体验与可维护性

### C1. ty-code(0/1/2/3)→ RegisterType 映射重复实现 5 处

- 位置:`motor_abi/src/motor_register_ffi.rs:355`(get/request 两处)、`integrations/ws_gateway/src/router/handlers/control_aux.rs`(handle_read_register)、`motor_cli/src/hightorque_cli.rs`(`reg_ty_from_args`)、`bindings/python/src/motorbridge/abi.py`(hightorque_get_register / hightorque_request_register)。
- 后果:新增 RegisterType 或改 code 值需同步改 5 处、跨 3 种语言,漏一处则仅该层静默解码错误。
- 建议:在 vendor crate 增加 `RegisterType::from_u8()`,与既有 `cmd_bits`/`from_cmd_bits` 并列,作为唯一事实来源。
- [x] 已解决(2026-10-22,随"读取固定 Int16"重构消亡:用户决策 `ty` 参数从全部 5 处(ABI get/request、CLI `--ty`、网关 JSON `ty`、Python 形参)彻底删除,不再存在需要映射的 ty 码——映射随参数一起消失,无需 `from_u8()`。破坏性 ABI 变更,先例 A1。见 fullchain 审计 §14)

### C2. CLI `read-reg` 每行输出都打基础地址的寄存器名

- 位置:`motor_cli/src/hightorque_cli.rs:460`
- 现象:`--mode read-reg --addr 0x01 --count 3` 打出三行 `name=位置`(0x01 的名字),而第 2、3 行实际是速度/力矩寄存器的值。扫读或管道进日志时容易张冠李戴。
- [x] 已解决(2026-10-22,随"读取固定 Int16"重构顺带修:read-reg 输出循环内按每行地址 `parameter_info(a)` 查名称,并加 `unit=` 单位列;网关 read_register 响应同样改为每行 name/unit)

### C3. Python demo 文档与默认行为不符

- 位置:`bindings/python/examples/hightorque_all_interfaces_demo.py:91`
- 现象:模块 docstring 称 `PERIODIC_REPORT_MS` 控制 `[2c]` "默认 10ms",但常量初始化为 `None`(即默认关闭)。按文档预期跑 demo 的用户会看到电机正常运动而 `get_state` 读数全程冻结,以为 demo 坏了。
- [x] 非问题结案(2026-10-21 用户裁定:`PERIODIC_REPORT_MS=None` 是维护者对文件内配置开关的有意关闭,运行时 2c 的 else 分支已明示"跳过+读数将冻结",段 12 自含不依赖该开关,无误导。唯一残留是 docstring 曾写"默认 10ms"与实际不符——已同步为"默认 None=跳过(有意关闭)",常量不动)

---

## 评审时确认的非问题(勿"修复")

- `enable()` 空操作:2026-10-08 用户决策的方案 B(语义 no-op),有记录,见 fullchain 审计 §12。
- `control_reply_enabled` 默认翻回标准裸 ID 帧:candump 验证过的回退,文档已同步更新。

---

## 修复计划

1. 同步 main(`git merge origin/main`,HT-develop 落后 11 个提交,见 2026-10-17 fetch 结果;main 侧重构了 ABI feedback/state snapshot,修复时注意适配)。
2. 按 A1→A4 → B1→B3 → C1→C3 顺序修复,逐条勾选上方复选框。
3. 全部修完跑 `cargo test` + Python demo 冒烟,更新本文档状态。
