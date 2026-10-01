# 电源管理服务 —— 交接文档

状态：**软件已完成并在实机验证；真电池测试阻塞（材料未到）**
最后更新：2026-10-01

---

## 1. 这是干什么的

给 Open Duck Mini（RDK-X5 版）加一个**轻量后台服务**，管两件事：

1. **电量监测** —— 用 INA219 读电池组总电压/电流/功率，换算剩余电量（SoC），掉电后保留。
2. **温度控制** —— 读 SoC 温度，按曲线算出风扇占空比，经 UART 下发给协控制器 STM32F411。

设计取向是**轻量**：不引 numpy/pandas 之类重库，温度直接读 sysfs，状态用一个小 JSON 持久化。

### 边界（明确不做什么）

- 不参与行走/关节控制，不进控制环。按 AGENTS.md 的 RHO 模型，这块逻辑待在实时控制环之外。
- 不改 IMU、增益、偏移、action scale、相位时序。
- 不训练新策略。
- 默认**只读**，只有显式加 `--control-fan` 才会写风扇。

---

## 2. 目标机环境实况

> ⚠️ **`docs/DEPLOYMENT.md` 里写的板子地址和路径是旧的**（`192.168.1.50` / `/home/sunrise/project/...`）。
> 本 fork 的目标机不是那台。以本节为准。

| 项 | 值 |
|---|---|
| 板子 | RDK X5（D-Robotics） |
| SSH | `sunrise@192.168.31.64`（密码同用户名） |
| 工作目录 | `/home/sunrise/openduckmini` |
| 运行包路径 | `/home/sunrise/openduckmini/Open_Duck_Mini_Runtime-2_RDK_X5` |
| venv | `/home/sunrise/openduckmini/venv`（Python 3.10.12） |
| 已安装包名 | `mini-bdx-runtime 0.0.1`（`pip install -e .` 可编辑安装） |

### 硬件事实（2026-10-01 实测，不是推测）

| 项 | 值 | 怎么确认的 |
|---|---|---|
| INA219 地址 | `0x40` | i2cdetect |
| I2C 总线 | `/dev/i2c-5` | `platform_compat.get_i2c_bus_number()`，**不要硬编码** |
| 分流电阻 | 10 mΩ (0.01 Ω) | 三种独立测量互相印证 |
| PGA 量程 | ±160 mV → ±16 A 硬件钳位 | 由分流电阻和最大电流算出 |
| UART | `/dev/ttyS6` @ 115200 | 板上一份记录里找到，并实测通了 |
| 温度源 | `thermal_zone0`(ddr) / `thermal_zone1`(cpu) | sysfs |
| INA219 与 IMU | **同一条 I2C 总线** | imu 是 BNO055；内核 i2c-dev 串行化单次传输，可共存 |

**登录 shell 是 fish。** 远程跑命令必须走 bash：

```bash
ssh sunrise@192.168.31.64 'bash -s' <<'EOF'
cd /tmp
...你的命令...
EOF
```

直接 `ssh host 'VAR=x; cmd'` 会报 `fish: Unsupported use of '='`。

### 板子没有外网

`pip install` 会静默卡住（去 GitHub 拉 `pypot`）。依赖都已装好，重装用：

```bash
venv/bin/pip install -e . --no-deps
```

---

## 3. 交付文件

全部在 `runtime/` 下。

| 文件 | 说明 |
|---|---|
| [power_manager.py](../runtime/mini_bdx_runtime/mini_bdx_runtime/peripheral/power_manager.py) | 主服务：电量模型 + 温控 + UART 链路 + CLI |
| [ina219.py](../runtime/mini_bdx_runtime/mini_bdx_runtime/peripheral/ina219.py) | INA219 驱动（重写，修了 3 个 bug） |
| [power_manager_test.py](../runtime/scripts/power_manager_test.py) | 操作员诊断脚本 |
| [__init__.py](../runtime/mini_bdx_runtime/mini_bdx_runtime/peripheral/__init__.py) | 让 `peripheral` 成为真正的包 |

`peripheral/__init__.py` 不能省：没有它，`find_packages` 找不到这个子包，`from .ina219 import ...` 会炸。

**没有改动** `gc9a01a.py` / `lcd_eyes.py` / `antennas.py` / `sounds.py` 或其它任何原有文件。

---

## 4. 设计说明

### 4.1 电量

- 电池：18650 **2S2P**，单体 3500 mAh / 3.7 V 标压 / 4.2 V 满。整组 **7.0 Ah**。
- SoC 三管齐下：
  1. **库仑积分** —— 主路径，按电流对时间积分。
  2. **OCV 查表** —— 静置时（电流 < 150 mA 且持续 ≥ 60 s）用开路电压重新锚定，混合系数 0.25，修正量小于 2% 时不动，避免抖动。
  3. **掉电持久化** —— 原子写 JSON（临时文件 + `os.replace` + `fsync`），读者永远看不到半截文件。
- ⚠️ **`OCV_TABLE` 是通用 18650 曲线，不是本组电池实测值**，需要在真电池上标定。
- 低电分级（按单体电压）：**3.50 V 警告 / 3.30 V 低电 / 3.00 V 危急**（整组 7.00 / 6.60 / 6.00 V）。目前**只报警告**，不做任何动作。

### 4.2 温控

五条可切换曲线，默认 `silent`：

| 曲线 | 含义 |
|---|---|
| `off` | 恒定 0%，全程不转 |
| `silent`（默认） | 60 °C 以下完全停转，70→60%，80→85%，85→100% |
| `balanced` | 45 °C 起转，55→40%，65→70%，75→100% |
| `aggressive` | 35 °C 起转，45→50%，55→100% |
| `always30` | 恒定 30% |

叠加两个保护：

- **迟滞**：温度在迟滞带内时维持当前占空比，避免在阈值上抖。
- **斜率限制**：占空比按 `max_slew_pct_per_s` 渐变，避免风扇转速突变。
- **紧急通道**：≥ 85 °C 直接满速，**跳过迟滞和斜率限制**（这条曾是 bug，见 §6）。

### 4.3 UART 协议

STM32F411 协处理器，行式 ASCII，发一行命令、回 `ok\n` / `err\n`：

| 命令 | 作用 |
|---|---|
| `dut,<v>` | 设置 PWM 占空比 0–100 |
| `tmp,<v>` | 温度数据（MCU **仅打印**，不参与控制） |
| `get` | 查询转速，先回一行转速再回 `ok` |

实现细节：`set_duty()` 发的是**零填充三位**，即 `dut,050`。**实测 MCU 接受**（0–100 全部返回 `ok`）。纯数字 `dut,50` / `dut,0` 也实测通过，两种格式都行。

### 4.4 安全默认

- 不加 `--control-fan` → 纯只读，**绝不写风扇**。
- 退出时可以指定 `--on-exit-duty`（默认 0）。
- 串口异常会自愈重连，不会因为一次抖动就崩掉后台服务。

---

## 5. 已实测证据

### 5.1 电量读数（稳压电源供电，非真电池）

```
总线电压 : 7.268 V        分流电压 : -4.020 mV
电流     : -0.402 A       功率     : -2.921 W
单体电压 : 3.634 V        电量 ≈ 23.4% (OCV)
```

电流值与分流电压换算完全一致（`-4.020 mV / 0.01 Ω = -0.402 A`），说明标定正确。

### 5.2 风扇占空比 → 总电流（剂量-响应，实测）

用 INA219 总电流做旁证，0/25/50/75/100% 逐档测量，**完全单调**，且第二次回到 0% 时精确复现基线：

| 占空比 | 总电流 | 相对 0% |
|---|---|---|
| 0% | −0.3647 A | 基线 |
| 25% | −0.3902 A | −22.8 mA |
| 50% | −0.3962 A | −28.8 mA |
| 75% | −0.4119 A | −44.5 mA |
| 100% | −0.4227 A | −55.3 mA |
| 0%（复测） | −0.3675 A | −0.1 mA |

配对重复 3 轮（交替 0/100%）得到 58 / 57 / 60 mA 的差值，一致性远好于噪声。**结论：`dut` 命令确实生效，0% 是一个真实、可复现的低功耗状态。**

### 5.3 风扇占空比 → 温度（实测）

风扇置 0%、空闲状态下，温度从 ~53.6 °C 缓慢爬到 55.6 °C 且仍在上升。
改回 50% 后，120 秒内从 56.7 °C 降到 51.1 °C 且继续下降。

### 5.4 算法自检

`power_manager_test.py --selftest` 共 7 组，全部通过（OCV 插值/钳位、温度曲线、迟滞与斜率、库仑积分、符号约定、低电分级、指令格式）。

---

## 6. 驱动层修掉的 3 个 bug（都很隐蔽，值得知道）

1. **电流静默返回 0.0**
   没调用 `set_calibration` 时 `current_lsb = 0.0`，`current()` 会返回 0.0 而看不出任何异常。
   现在抛 `INA219NotCalibrated`。

2. **改完标定立刻读，拿到的是陈旧值**
   CURRENT/POWER 寄存器只在整轮转换结束后刷新。改完 CAL 紧接着读，拿到的是用**旧标度**算的值 —— 实测 CAL 变 5 倍时读数正好差 5 倍（−2.175 A vs 实际 −0.435 A）。
   现在 `configure_defaults()` 结尾会 `wait_for_conversion(2.0)`（本机转换周期 17.0 ms）。

3. **溢出位读错**
   原先读的是总线电压寄存器 0x02 的 **bit1**，那是 CNVR（转换完成，连续模式下一直在翻转），于是只读诊断里冒出一条假的「ADC 溢出」。
   正确的是 **bit0 = OVF**（电流/功率计算溢出）。已修正为 `math_overflow()` / `conversion_ready()`。

---

## 7. 已知问题与硬件怪癖

### 7.1 ⚠️ 协处理器的转速遥测不可信（重要）

**风扇转慢或停转时，固件似乎取不到测速脉冲，于是保持上一次读数，而不是回 0。**

实测在 `duty=0` 下连续采样 18 次（跨 36 秒）读到**完全相同的 2865** —— 真实转速计不可能这样。另一处观察到 `duty=0` 下读 3255，而那正是上一次 100% 档的真实读数。

**所以 `get_rpm()` 的值不能用来判断风扇是否在转。** 要验证风扇状态，用 INA219 总电流做旁证（见 §5.2）。这条已写进 `get_rpm()` 的 docstring 和诊断脚本输出。

> 用户已说明「后面会更新程序默认是停止的」，即 MCU 固件侧还会改。这条属于固件行为，软件侧只需要正确下发 `dut,000`。

### 7.2 INA219 CAL 寄存器回读 bit0 恒为 0

写 8389 读回 8388，写 1 读回 0，写 0xFFFF 读回 0xFFFE（偶数值精确）。CONFIG 寄存器 bit0 正常。
影响约 1/8389 ≈ **0.012%**，忽略。**不要用 CAL 回读值做断言。** 已在 `set_calibration` 注释里记下。

### 7.3 低占空比下 RPM 抖动大

≤10% 档的转速读数（75 / 165 RPM）抖动剧烈，同样不可信。结合 §7.1，低转速段整段测速都不准。

---

## 8. 未验证 / 阻塞项

| 项 | 状态 |
|---|---|
| **真电池测试** | 🚫 **阻塞** —— 材料未到，用户明确说现在做不了 |
| `DISCHARGE_IS_NEGATIVE` 符号约定 | ⚠️ **假设** —— 是在稳压电源上推出来的，不是真电池。接真电池必须重新确认 |
| `OCV_TABLE` 标定 | ⚠️ 通用曲线，未用本组电池实测校准 |
| 掉电 SoC 持久化 | ⚠️ 代码路径已写，但没有做过真实的断电重启验证 |
| systemd 后台服务 | ⬜ 未做 —— 按用户要求，等真电池验证通过后再做 |
| 电流量程 >16 A 的行为 | ⚠️ 未测（PGA 会钳位并置 OVF 标志） |

**当前电源是稳压电源，不是电池，所以电压恒定。** §5.1 里那个 23.4% 只是把 3.634 V 代进 OCV 表查出来的，在电源供电时没有实际意义 —— 不要拿它当证据。

---

## 9. 下一步（按优先级）

1. **等电池到了，做真电池验证**（这是所有后续工作的前置）：
   - 确认放电/充电时电流符号，据此校正 `DISCHARGE_IS_NEGATIVE`。
   - 满充静置，记录 OCV → 用实测点替换 `OCV_TABLE`。
   - 拉一段真实放电曲线，比对库仑积分与实测容量的偏差。
   - 做一次真实断电重启，验证 SoC 持久化。
2. **验证低电分级**在真电池上的触发点是否合理（3.50 / 3.30 / 3.00 V）。
3. **之后**再考虑 systemd 服务（默认只读模式启动，需要时再开 `--control-fan`）。
4. 按 AGENTS.md 要求同步 README / ROADMAP / EVIDENCE_MANIFEST（**本次未做**）。

---

## 10. 复现命令

所有命令都在板子上、从 `/tmp` 或任意目录执行（脚本自带 `sys.path` 处理）。

```bash
T=~/openduckmini/Open_Duck_Mini_Runtime-2_RDK_X5
V=~/openduckmini/venv/bin/python

# 纯算法自检，不需要硬件
$V $T/scripts/power_manager_test.py --selftest

# 只读诊断（默认行为，不动风扇）
$V $T/scripts/power_manager_test.py --read-only

# 持续观察
$V $T/scripts/power_manager_test.py --watch --period 2

# 改变风扇转速（必须显式 --yes）
$V $T/scripts/power_manager_test.py --duty 40 --yes
$V $T/scripts/power_manager_test.py --fan-curve-test --yes \
   --steps 0,10,20,30,40,50,70,100 --settle 3.0 --leave-at-zero
```

主服务本身：

```bash
M=$T/mini_bdx_runtime/mini_bdx_runtime/peripheral/power_manager.py

# 不带 --loop 就是只跑一次；这时的行为是只读的
$V $M

# 持续跑并控制风扇
$V $M --loop --control-fan --curve silent \
   --state-file /var/lib/duck/power_state.json

# 固定占空比（风扇会真的转，注意确认）
$V $M --loop --control-fan --duty 40
```

完整参数：`--loop --period --iterations --curve --control-fan --duty --no-uart
--on-exit-duty --status-file --state-file --state-interval --log --quiet --selftest`

### 部署到板子

本地改完 → **LF 归一化**再 scp（本地是 CRLF，直接传可能在 Linux 上出问题）：

```bash
sed 's/\r$//' runtime/.../power_manager.py > /tmp/pm.py
scp /tmp/pm.py sunrise@192.168.31.64:'~/openduckmini/Open_Duck_Mini_Runtime-2_RDK_X5/mini_bdx_runtime/mini_bdx_runtime/peripheral/power_manager.py'
```

传完清掉 `__pycache__`，然后核对 md5：

```bash
ssh sunrise@192.168.31.64 'bash -s' <<'EOF'
T=$HOME/openduckmini/Open_Duck_Mini_Runtime-2_RDK_X5
find $T -name __pycache__ -type d -exec rm -rf {} + 2>/dev/null
sed 's/\r$//' $T/mini_bdx_runtime/mini_bdx_runtime/peripheral/power_manager.py | md5sum
EOF
```

---

## 11. 安全提醒（沿用 AGENTS.md）

- 动硬件前先看证据，不要先调参。
- 只读 SSH 检查随时可做；**改变机器人状态的操作**需要明确确认。
- 目前风扇已被置为 **50%**（扫描前板子约 1770 RPM / 47.7 °C，50% 档对应该状态），板温 ~51 °C。
- 不要提交 `.duck_access/`、私钥、known_hosts、token。
- 不要提交大体积原始 JSONL / 视频。

---

## 12. 修订记录

| 日期 | 变更 |
|---|---|
| 2026-10-01 | 初版。软件完成并实机验证；记录风扇 `dut` 首次实测、RPM 遥测冻结问题、3 个驱动 bug；真电池测试标记为阻塞 |
