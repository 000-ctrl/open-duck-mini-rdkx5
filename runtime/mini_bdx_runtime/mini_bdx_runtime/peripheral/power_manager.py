"""
电源管理后台服务
================

职责
----
1. 电量监测 : 用 INA219 读整机总电压/电流/功率，估算 2S2P 18650 电池组剩余电量
2. 温度控制 : 读 SoC 温度，按可选曲线算风扇占空比，经 UART 下发给 STM32F411 协控制器
3. 状态输出 : 原子写状态文件，供其它进程（行走脚本 / 遥测）读取

设计目标：轻量。单进程单线程，sleep 到下一个 tick，无常驻线程、无轮询忙等。

与 IMU 共用 I2C
---------------
INA219(0x40) 与 BNO055(IMU) 挂在同一条总线上（RDK X5 = /dev/i2c-5）。
内核 i2c-dev 会串行化单次传输，所以两者可以共存，但必须遵守：

    - 总线号走 platform_compat.get_i2c_bus_number()，不要硬编码
    - 不做全地址扫描，只访问自己的 0x40
    - 读失败要退避重试，避免总线错误风暴影响 IMU 与伺服

协控制器协议（STM32F411, /dev/ttyS6 @ 115200, ASCII, '\\n' 结尾）
----------------------------------------------------------------
    dut,<0-100>   设置 PWM 占空比（百分比，0=停转 100=满速）  -> "ok\\n" / "err\\n"
    get           查询风扇转速                                -> "<rpm>\\nok\\n"
    tmp,<v>       回传温度（MCU 仅打印）                      -> "ok\\n"

安全默认
--------
默认**只读**：只读电量、温度、转速，不下发 dut。
必须显式加 --control-fan 才会开启温控写通道。

用法
----
    # 只读诊断（不碰风扇转速）
    python power_manager.py --once
    python power_manager.py --loop --period 1.0

    # 开启温控闭环
    python power_manager.py --loop --control-fan --curve silent

    # 固定占空比，手动测试风扇
    python power_manager.py --duty 50

    # 不连硬件，跑算法自检
    python power_manager.py --selftest
"""

import argparse
import json
import os
import signal
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG_DIR = os.path.dirname(_HERE)

if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

try:
    from .ina219 import INA219, MODE_CONTINUOUS
except ImportError:  # 作为脚本直接运行
    from ina219 import INA219, MODE_CONTINUOUS

try:
    from ..platform_compat import get_i2c_bus_number
except ImportError:
    if _PKG_DIR not in sys.path:
        sys.path.insert(0, _PKG_DIR)
    from platform_compat import get_i2c_bus_number

try:
    import serial
except ImportError:  # 纯电量模式不需要 pyserial
    serial = None


# --------------------------------------------------------------------------
# 硬件常量（2026-10-01 实机实测）
# --------------------------------------------------------------------------

INA219_ADDRESS = 0x40
SHUNT_OHM = 0.01
"""分流电阻 10mΩ。实测自洽：Vshunt=-4.4mV <-> I=0.44A <-> POWER=3.19W"""

MAX_CURRENT_A = 16.0
"""期望可测最大电流。

与 PGA 量程必须匹配，否则大电流会被硬件削顶：
    PGA ±40mV  -> 4A     PGA ±80mV  -> 8A
    PGA ±160mV -> 16A    PGA ±320mV -> 32A
这里选 ±160mV/16A：行走峰值不会削顶，分辨率 0.488mA（对 7Ah 电池足够）。
"""

ADC_AVERAGING = 16
"""ADC 平均次数。16 次 ≈ 8.5ms 转换，兼顾噪声与总线占用（与 IMU 共用总线）。"""

DISCHARGE_IS_NEGATIVE = True
"""实测：放电时 INA219 电流读数为负（分流电阻接线方向相反）。

若换板子后符号反了，改这里即可，不要改库仑积分逻辑。
"""

UART_PORT = "/dev/ttyS6"
UART_BAUD = 115200
UART_TIMEOUT_S = 0.5


# --------------------------------------------------------------------------
# 电池模型：2S2P 18650
# --------------------------------------------------------------------------

CELL_COUNT = 2
CELL_CAPACITY_AH = 3.5
CELLS_IN_PARALLEL = 2
PACK_CAPACITY_AH = CELL_CAPACITY_AH * CELLS_IN_PARALLEL  # 7.0 Ah

CV_FULL_V = 4.20
CV_EMPTY_V = 3.00

OCV_TABLE = (
    # (单体开路电压 V, 剩余电量 %)
    # 通用 18650 锂离子曲线，非本组电池实测值；后续可用实测数据校准。
    (4.20, 100.0),
    (4.10, 90.0),
    (4.00, 80.0),
    (3.93, 70.0),
    (3.87, 60.0),
    (3.80, 50.0),
    (3.74, 40.0),
    (3.68, 30.0),
    (3.61, 20.0),
    (3.50, 10.0),
    (3.40, 5.0),
    (3.30, 3.0),
    (3.00, 0.0),
)

# 低电告警阈值（按单体电压，pack = cell * 2）
WARN_CELL_V = 3.50      # 7.00 V
LOW_CELL_V = 3.30       # 6.60 V
CRITICAL_CELL_V = 3.00  # 6.00 V

# OCV 重新锚定条件：电流足够小且持续足够久（否则电压受内阻压降污染）
OCV_REST_CURRENT_A = 0.15
OCV_REST_SECONDS = 60.0
OCV_BLEND_ALPHA = 0.25
OCV_MIN_CORRECTION_PCT = 2.0


def ocv_to_soc(cell_v: float) -> float:
    """单体开路电压 -> 剩余电量百分比（线性插值，超出表格则钳位）。"""
    if cell_v >= OCV_TABLE[0][0]:
        return OCV_TABLE[0][1]
    if cell_v <= OCV_TABLE[-1][0]:
        return OCV_TABLE[-1][1]
    for (v_hi, soc_hi), (v_lo, soc_lo) in zip(OCV_TABLE, OCV_TABLE[1:]):
        if v_lo <= cell_v <= v_hi:
            ratio = (cell_v - v_lo) / (v_hi - v_lo)
            return soc_lo + ratio * (soc_hi - soc_lo)
    return 0.0


class BatteryModel:
    """电压 + 库仑积分 + OCV 校准 的剩余电量估计。

    库仑积分负责短期精度，OCV 锚定负责消除长期漂移，
    两者结合才不会出现"电量越用越多"或"永远停在 100%"。
    """

    def __init__(
        self,
        capacity_ah: float = PACK_CAPACITY_AH,
        cell_count: int = CELL_COUNT,
        soc_pct: float = None,
        state_path: str = None,
    ):
        self.capacity_ah = capacity_ah
        self.cell_count = cell_count
        self.state_path = state_path

        self.remaining_ah = None
        self.soc_pct = float(soc_pct) if soc_pct is not None else 100.0
        if soc_pct is not None:
            self.remaining_ah = self.capacity_ah * self.soc_pct / 100.0

        self.total_discharged_ah = 0.0
        self._rest_seconds = 0.0
        self._last_anchor_reason = None

    # ---- 持久化 ----------------------------------------------------------

    def load(self) -> bool:
        """从状态文件恢复剩余电量。返回是否成功。"""
        if not self.state_path or not os.path.exists(self.state_path):
            return False
        try:
            with open(self.state_path, "r", encoding="utf-8") as f:
                data = json.load(f)
            soc = data.get("soc_pct")
            if soc is None:
                return False
            self.soc_pct = min(max(float(soc), 0.0), 100.0)
            self.remaining_ah = self.capacity_ah * self.soc_pct / 100.0
            self.total_discharged_ah = float(data.get("total_discharged_ah", 0.0))
            self.saved_at = data.get("saved_at")
            # 停机期间电池仍在放电，积分不可信 -> 等静置后用 OCV 重新锚定
            self._rest_seconds = 0.0
            self._last_anchor_reason = "restored"
            return True
        except (OSError, ValueError):
            return False

    def save(self) -> bool:
        if not self.state_path:
            return False
        try:
            atomic_write_json(
                self.state_path,
                {
                    "soc_pct": round(self.soc_pct, 3),
                    "remaining_ah": round(self.remaining_ah or 0.0, 4),
                    "capacity_ah": self.capacity_ah,
                    "total_discharged_ah": round(self.total_discharged_ah, 4),
                    "saved_at": time.time(),
                },
            )
            return True
        except OSError:
            return False

    # ---- 估计 ------------------------------------------------------------

    def update(self, pack_v: float, current_a: float, dt: float) -> float:
        """推进一次估计。

        Args:
            pack_v: 电池组总电压(V)
            current_a: INA219 原始带符号电流(A)，放电为负
            dt: 距上次更新的秒数
        """
        if self.remaining_ah is None:
            self.remaining_ah = self.capacity_ah * self.soc_pct / 100.0

        # 统一成"放电为正"
        discharge_a = -current_a if DISCHARGE_IS_NEGATIVE else current_a

        # --- 库仑积分 ---
        if dt > 0:
            delta_ah = discharge_a * dt / 3600.0
            self.remaining_ah -= delta_ah
            if discharge_a > 0:
                self.total_discharged_ah += delta_ah
            self.remaining_ah = min(max(self.remaining_ah, 0.0), self.capacity_ah)
            self.soc_pct = self.remaining_ah / self.capacity_ah * 100.0

        cell_v = pack_v / self.cell_count
        self._last_anchor_reason = None

        # --- 满电锚定 ---
        if cell_v >= CV_FULL_V - 0.02 and abs(discharge_a) < OCV_REST_CURRENT_A:
            self._anchor(100.0, "full")

        # --- 空电锚定 ---
        elif cell_v <= CV_EMPTY_V:
            self._anchor(0.0, "empty")

        # --- 静置 OCV 锚定 ---
        else:
            if abs(discharge_a) < OCV_REST_CURRENT_A:
                self._rest_seconds += dt
            else:
                self._rest_seconds = 0.0

            if self._rest_seconds >= OCV_REST_SECONDS:
                target = ocv_to_soc(cell_v)
                if abs(target - self.soc_pct) >= OCV_MIN_CORRECTION_PCT:
                    blended = self.soc_pct + OCV_BLEND_ALPHA * (target - self.soc_pct)
                    self._anchor(blended, "ocv")
                    # 校准后重新起算，避免连续叠加
                    self._rest_seconds = 0.0

        return self.soc_pct

    def _anchor(self, soc_pct: float, reason: str):
        self.soc_pct = min(max(soc_pct, 0.0), 100.0)
        self.remaining_ah = self.capacity_ah * self.soc_pct / 100.0
        self._last_anchor_reason = reason

    def level(self, pack_v: float) -> str:
        """低电告警等级（只告警，不主动下电）。"""
        cell_v = pack_v / self.cell_count
        if cell_v <= CRITICAL_CELL_V:
            return "CRITICAL"
        if cell_v <= LOW_CELL_V:
            return "LOW"
        if cell_v <= WARN_CELL_V:
            return "WARN"
        return "OK"

    def to_dict(self) -> dict:
        return {
            "soc_pct": round(self.soc_pct, 1),
            "remaining_ah": round(self.remaining_ah or 0.0, 3),
            "capacity_ah": self.capacity_ah,
            "total_discharged_ah": round(self.total_discharged_ah, 3),
            "anchor": self._last_anchor_reason,
        }


# --------------------------------------------------------------------------
# 温度 -> 占空比
# --------------------------------------------------------------------------

TEMP_CURVES = {
    # 名字: ((温度°C, 占空比%), ...)  按温度升序，中间线性插值
    "off": ((0.0, 0), (999.0, 0)),
    "silent": ((0.0, 0), (60.0, 0), (70.0, 60), (80.0, 85), (85.0, 100)),
    "balanced": ((0.0, 0), (45.0, 0), (55.0, 40), (65.0, 70), (75.0, 100)),
    "aggressive": ((0.0, 0), (35.0, 0), (45.0, 50), (55.0, 100)),
    "always30": ((0.0, 30), (999.0, 30)),
}

DEFAULT_CURVE = "silent"
"""默认曲线：60°C 以下完全停转（省电、静音）。"""

EMERGENCY_TEMP_C = 85.0
"""超过此温度直接满速，跳过迟滞与斜率限制。"""

THERMAL_ZONES = (
    "/sys/class/thermal/thermal_zone0/temp",  # thermal-ddr
    "/sys/class/thermal/thermal_zone1/temp",  # thermal-cpu
)


def curve_duty(curve, temp_c: float) -> int:
    """按折线曲线把温度映射成占空比。"""
    points = TEMP_CURVES.get(curve) if isinstance(curve, str) else curve
    if not points:
        raise ValueError(f"unknown curve: {curve}")
    if temp_c <= points[0][0]:
        return int(round(points[0][1]))
    if temp_c >= points[-1][0]:
        return int(round(points[-1][1]))
    for (t_lo, d_lo), (t_hi, d_hi) in zip(points, points[1:]):
        if t_lo <= temp_c <= t_hi:
            ratio = (temp_c - t_lo) / (t_hi - t_lo)
            return int(round(d_lo + ratio * (d_hi - d_lo)))
    return int(round(points[-1][1]))


class ThermalController:
    """带迟滞与斜率限制的风扇曲线控制器。

    迟滞避免在曲线拐点附近来回抖动，斜率限制避免占空比突变
    （既有电气噪声考虑，也避免风扇转速阶跃产生可听噪声）。
    """

    def __init__(
        self,
        curve: str = DEFAULT_CURVE,
        hysteresis_c: float = 3.0,
        max_slew_pct_per_s: float = 10.0,
        emergency_c: float = EMERGENCY_TEMP_C,
    ):
        if curve not in TEMP_CURVES:
            raise ValueError(f"unknown curve: {curve}")
        self.curve = curve
        self.hysteresis_c = hysteresis_c
        self.max_slew_pct_per_s = max_slew_pct_per_s
        self.emergency_c = emergency_c

        self.duty = 0
        self._change_temp = None

    def update(self, temp_c: float, dt: float) -> int:
        emergency = temp_c >= self.emergency_c

        if emergency:
            # 紧急温度：直接满速，既不看迟滞也不做斜率限制
            target = 100
            self._change_temp = temp_c
        elif (
            self._change_temp is not None
            and abs(temp_c - self._change_temp) < self.hysteresis_c
        ):
            # 温度相对上次决策点未移出迟滞带 -> 维持当前值
            target = self.duty
        else:
            target = curve_duty(self.curve, temp_c)
            self._change_temp = temp_c

        # 斜率限制（紧急路径已提前返回，不受此限制）
        if not emergency and self.max_slew_pct_per_s and dt > 0:
            max_step = max(1, int(self.max_slew_pct_per_s * dt))
            if abs(target - self.duty) > max_step:
                target = self.duty + max_step * (1 if target > self.duty else -1)

        self.duty = min(max(int(target), 0), 100)
        return self.duty


# --------------------------------------------------------------------------
# STM32F411 协控制器 UART
# --------------------------------------------------------------------------


class FanLinkError(RuntimeError):
    pass


class FanLink:
    """STM32F411 风扇协控制器 UART 链路。

    协议是行式的 ASCII：发一行命令，MCU 回 "ok" / "err"（get 命令先回一行转速）。
    所有写操作都在本类内部串行，调用方不需要考虑并发。
    """

    def __init__(self, port: str = UART_PORT, baud: int = UART_BAUD,
                 timeout: float = UART_TIMEOUT_S):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self._ser = None
        self.failures = 0
        self.last_error = None

    # ---- 生命周期 --------------------------------------------------------

    def open(self):
        if serial is None:
            raise FanLinkError("未安装 pyserial（pip install pyserial）")
        if self._ser is not None:
            return
        self._ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
        self._ser.reset_input_buffer()
        self._ser.reset_output_buffer()

    @property
    def is_open(self) -> bool:
        return self._ser is not None and getattr(self._ser, "is_open", False)

    def close(self):
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:  # noqa: BLE001
                pass
            self._ser = None

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
        return False

    # ---- 协议 ------------------------------------------------------------

    def _read_reply(self):
        """读取一行回复。返回 (reply, extra_lines)。reply 为 'ok'/'err'/None(超时)。"""
        deadline = time.monotonic() + self.timeout
        extra = []
        while time.monotonic() < deadline:
            raw = self._ser.readline()
            if not raw:
                break
            line = raw.decode("ascii", errors="replace").strip()
            if not line:
                continue
            if line in ("ok", "err"):
                return line, extra
            extra.append(line)
        return None, extra

    def _transact(self, command: str):
        """发一条命令并读回复。失败抛 FanLinkError。"""
        self.open()
        try:
            self._ser.write((command + "\n").encode("ascii"))
            self._ser.flush()
            reply, extra = self._read_reply()
        except Exception as exc:  # noqa: BLE001 - 串口异常需要能自愈重连
            self.last_error = f"{type(exc).__name__}: {exc}"
            self.close()
            raise FanLinkError(self.last_error) from exc

        if reply is None:
            self.failures += 1
            self.last_error = f"timeout waiting for reply to {command!r}"
            raise FanLinkError(self.last_error)
        if reply == "err":
            self.failures += 1
            self.last_error = f"device returned err for {command!r}"
            raise FanLinkError(self.last_error)

        self.failures = 0
        self.last_error = None
        return extra

    # ---- 命令 ------------------------------------------------------------

    def set_duty(self, duty: int) -> bool:
        """设置 PWM 占空比 0-100（百分比）。"""
        duty = int(duty)
        if not 0 <= duty <= 100:
            raise ValueError(f"duty out of range: {duty}")
        self._transact(f"dut,{duty:03d}")
        return True

    def get_rpm(self) -> int:
        """读取风扇转速。

        !! 实测警告（2026-10-01）: 协处理器的转速遥测不可信 !!

        风扇转慢或停转时，固件似乎取不到测速脉冲，于是**保持上一次的
        读数不变**，而不是回 0。实测在 duty=0 下连续 18 次采样（跨 36 秒）
        读到完全相同的 2865，真实转速计不可能如此。

        因此这个值**不能**用来判断风扇是否在转。要验证风扇状态，
        用 INA219 的总电流做旁证：实测 duty 0% 比 100% 低约 55 mA
        （0/25/50/75/100% 单调，见 power_manager_test.py --fan-curve-test）。
        """
        extra = self._transact("get")
        if not extra:
            raise FanLinkError("device did not report rpm")
        try:
            return int(float(extra[0]))
        except ValueError as exc:
            raise FanLinkError(f"bad rpm line: {extra[0]!r}") from exc

    def send_temperature(self, temp_c: float) -> bool:
        """把温度回传给协控制器（MCU 仅打印，不参与控制）。"""
        self._transact(f"tmp,{temp_c:.1f}")
        return True


# --------------------------------------------------------------------------
# 工具
# --------------------------------------------------------------------------


def atomic_write_json(path: str, payload: dict):
    """原子写 JSON：先写临时文件再 os.replace，读者永远看不到半截文件。"""
    directory = os.path.dirname(os.path.abspath(path))
    os.makedirs(directory, exist_ok=True)
    tmp = f"{path}.tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False, indent=2)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, path)


def read_thermal_zones(paths=THERMAL_ZONES) -> dict:
    """读 SoC 温度（毫摄氏度 -> 摄氏度）。读不到的区直接跳过。"""
    out = {}
    for path in paths:
        try:
            with open(path, "r", encoding="ascii") as f:
                name = os.path.basename(os.path.dirname(path))
                out[name] = int(f.read().strip()) / 1000.0
        except (OSError, ValueError):
            continue
    return out


# --------------------------------------------------------------------------
# 主服务
# --------------------------------------------------------------------------


class PowerManager:
    """电量监测 + 温控的执行体。

    不自己起线程也不自己 sleep 地跑循环；run() 提供轻量循环，
    tick() 可以单独调用（方便测试和嵌入别的调度器）。
    """

    def __init__(
        self,
        curve: str = DEFAULT_CURVE,
        control_fan: bool = False,
        use_uart: bool = True,
        state_path: str = None,
        max_current_a: float = MAX_CURRENT_A,
        shunt_ohm: float = SHUNT_OHM,
    ):
        self.control_fan = control_fan
        self.use_uart = use_uart

        self.ina = None
        self.fan = None
        self.battery = BatteryModel(state_path=state_path)
        self.thermal = ThermalController(curve=curve)

        self._max_current_a = max_current_a
        self._shunt_ohm = shunt_ohm
        self._last_duty_sent = None
        self._last_update = None
        self._battery_state = {}
        self._temp_state = {}

    # ---- 硬件生命周期 ----------------------------------------------------

    def open(self):
        self.ina = INA219(address=INA219_ADDRESS)
        self.ina.configure_defaults(shunt=self._shunt_ohm,
                                    max_current=self._max_current_a)
        if self.use_uart:
            self.fan = FanLink()
            self.fan.open()
        if self.battery.state_path:
            self.battery.load()

    def close(self, on_exit_duty=None):
        """收尾。on_exit_duty 为 None 表示保持当前转速不动。"""
        if self.fan is not None:
            if on_exit_duty is not None and self._last_duty_sent != on_exit_duty:
                try:
                    self.fan.set_duty(on_exit_duty)
                except FanLinkError:
                    pass
            self.fan.close()
        if self.ina is not None:
            self.ina.close()
        self.battery.save()

    # ---- 采集 ------------------------------------------------------------

    def read_battery(self):
        snap = self.ina.snapshot()
        self._battery_state = snap
        return snap

    def read_temperature(self):
        zones = read_thermal_zones()
        self._temp_state = zones
        return zones

    def tick(self):
        """执行一个周期。任何异常都不应该让服务退出。"""
        now = time.monotonic()
        dt = 0.0 if self._last_update is None else max(0.0, now - self._last_update)
        self._last_update = now

        # --- 温度 + 风扇 ---
        zones = self.read_temperature()
        temp_c = max(zones.values()) if zones else None
        duty = None
        rpm = None

        if temp_c is not None:
            duty = self.thermal.update(temp_c, dt)
            if self.fan is not None:
                if self.control_fan and duty != self._last_duty_sent:
                    try:
                        self.fan.set_duty(duty)
                        self._last_duty_sent = duty
                    except FanLinkError:
                        pass  # 看门狗在 status 里体现，不打断循环
                try:
                    rpm = self.fan.get_rpm()
                except FanLinkError:
                    rpm = None

        # --- 电量 ---
        snap = self.read_battery()
        battery = None
        if snap["ok"] and snap["bus_v"] is not None and snap["current_a"] is not None:
            soc = self.battery.update(snap["bus_v"], snap["current_a"], dt)
            level = self.battery.level(snap["bus_v"])
            battery = {
                "pack_v": round(snap["bus_v"], 3),
                "current_a": round(snap["current_a"], 3),
                "power_w": round(snap["power_w"], 3),
                "cell_v": round(snap["bus_v"] / CELL_COUNT, 3),
                "soc_pct": round(soc, 1),
                "level": level,
                "overflow": snap["overflow"],  # 电流被削顶，SoC 不可信
                "ok": True,
            }
        else:
            battery = {"ok": False, "error": snap.get("error"), "level": "UNKNOWN"}

        return {
            "battery": battery,
            "thermal": {
                "zones_c": {k: round(v, 1) for k, v in zones.items()},
                "max_c": round(temp_c, 1) if temp_c is not None else None,
                "curve": self.thermal.curve,
                "duty_pct": duty,
                "duty_sent": self._last_duty_sent,
                "fan_rpm": rpm,
                "control_enabled": self.control_fan,
            },
        }

    def status(self):
        """汇总当前状态（对外输出用）。"""
        return {
            "ts": time.time(),
            "battery_model": self.battery.to_dict(),
            "uart": None
            if self.fan is None
            else {
                "port": self.fan.port,
                "open": self.fan.is_open,
                "consecutive_failures": self.fan.failures,
                "last_error": self.fan.last_error,
            },
        }

    # ---- 循环 ------------------------------------------------------------

    def run(
        self,
        period: float = 1.0,
        iterations: int = None,
        status_path: str = None,
        state_interval: float = 30.0,
        log_path: str = None,
        print_status: bool = True,
        on_exit_duty: int = None,
    ):
        """主循环。sleep 到下一个 tick，不忙等。

        Args:
            on_exit_duty: 退出时下发的占空比。None 表示保持当前转速不动。
                          只有在 --control-fan 打开时才有效。
        """
        self.open()
        stop = {"flag": False}

        def _on_signal(signum, _frame):
            stop["flag"] = True

        for sig in (signal.SIGINT, signal.SIGTERM):
            try:
                signal.signal(sig, _on_signal)
            except (ValueError, OSError):
                pass

        log_file = open(log_path, "a", encoding="utf-8") if log_path else None
        last_state_save = time.monotonic()
        count = 0

        try:
            while not stop["flag"]:
                t0 = time.monotonic()
                try:
                    result = self.tick()
                except Exception as exc:  # noqa: BLE001 - 后台服务必须活着
                    print(f"[power_manager] tick error: {type(exc).__name__}: {exc}",
                          file=sys.stderr)
                    result = None

                if result is not None:
                    payload = dict(result)
                    payload.update(self.status())
                    if status_path:
                        try:
                            atomic_write_json(status_path, payload)
                        except OSError as exc:
                            print(f"[power_manager] status write failed: {exc}",
                                  file=sys.stderr)
                    if log_file:
                        log_file.write(json.dumps(payload, ensure_ascii=False) + "\n")
                        log_file.flush()
                    if print_status:
                        print(self._format_line(payload))

                    now = time.monotonic()
                    if state_interval and now - last_state_save >= state_interval:
                        self.battery.save()
                        last_state_save = now

                count += 1
                if iterations is not None and count >= iterations:
                    break

                elapsed = time.monotonic() - t0
                sleep_for = period - elapsed
                if sleep_for > 0:
                    time.sleep(sleep_for)
        finally:
            if log_file:
                log_file.close()
            # 只有开启写通道时才在退出时改转速；默认保持不动更安全
            self.close(on_exit_duty=on_exit_duty if self.control_fan else None)

    @staticmethod
    def _format_line(payload: dict) -> str:
        b = payload.get("battery", {})
        t = payload.get("thermal", {})
        if b.get("ok"):
            volt = f"{b['pack_v']:.2f}V {b['current_a']:+.2f}A {b['power_w']:+.2f}W"
            batt = f"{volt} {b['soc_pct']:.0f}% [{b['level']}]"
        else:
            batt = f"n/a [{b.get('level', 'UNKNOWN')}]"
        thermal = (
            f"{t.get('max_c')}C duty={t.get('duty_pct')}"
            f"(sent {t.get('duty_sent')}) rpm={t.get('fan_rpm')}"
        )
        return f"[power] {batt} | {thermal}"


# --------------------------------------------------------------------------
# 自检（不接触硬件）
# --------------------------------------------------------------------------


def selftest():
    """纯算法自检，不需要 INA219 / UART。"""
    failures = []

    def check(name, got, want, tol=1e-6):
        ok = abs(got - want) <= tol if isinstance(want, (int, float)) else got == want
        print(f"  {'PASS' if ok else 'FAIL'}  {name}: got={got} want={want}")
        if not ok:
            failures.append(name)

    print("[1] OCV 表")
    check("4.20V", ocv_to_soc(4.20), 100.0)
    check("3.80V", ocv_to_soc(3.80), 50.0)
    check("3.00V", ocv_to_soc(3.00), 0.0)
    check("超出上限钳位", ocv_to_soc(4.35), 100.0)
    check("超出下限钳位", ocv_to_soc(2.50), 0.0)
    check("插值 3.965V", ocv_to_soc(3.965), 75.0, tol=0.5)

    print("[2] 温度曲线")
    check("silent@40C", curve_duty("silent", 40.0), 0)
    check("silent@65C", curve_duty("silent", 65.0), 30)
    check("silent@90C", curve_duty("silent", 90.0), 100)
    check("off@90C", curve_duty("off", 90.0), 0)

    print("[3] 迟滞与斜率限制")
    # balanced 曲线: (45,0) (55,40) (65,70) (75,100)，斜率上限 10%/s
    tc = ThermalController(curve="balanced", hysteresis_c=3.0, max_slew_pct_per_s=10.0)
    check("50C: 曲线要 20%，被斜率限制到", tc.update(50.0, 1.0), 10)
    check("54C: 移出迟滞带，继续爬升", tc.update(54.0, 1.0), 20)
    check("55.5C: 迟滞带内维持不变", tc.update(55.5, 1.0), 20)
    check("69C: 移出迟滞带，继续爬升", tc.update(69.0, 1.0), 30)
    check("90C: 紧急，跳过斜率限制直接满速", tc.update(90.0, 1.0), 100)
    check("70C: 降温后受斜率限制回落", tc.update(70.0, 1.0), 90)

    print("[4] 库仑积分")
    bm = BatteryModel(capacity_ah=7.0, soc_pct=100.0)
    check("满电 100%", bm.soc_pct, 100.0)
    # 1A 放电 1 小时 -> 消耗 1Ah / 7Ah
    for _ in range(360):
        bm.update(pack_v=7.4, current_a=-1.0, dt=10.0)
    check("1A 放 1h 后", bm.soc_pct, 100.0 - 100.0 / 7.0, tol=0.05)

    print("[5] 符号约定（放电为负 -> 电量下降）")
    bm2 = BatteryModel(capacity_ah=7.0, soc_pct=50.0)
    bm2.update(pack_v=7.4, current_a=-1.0, dt=3600.0)
    check("放电后电量下降", bm2.soc_pct < 50.0, True)
    bm3 = BatteryModel(capacity_ah=7.0, soc_pct=50.0)
    bm3.update(pack_v=8.0, current_a=+1.0, dt=3600.0)
    check("充电后电量上升", bm3.soc_pct > 50.0, True)

    print("[6] 低电分级")
    check("8.0V", BatteryModel(soc_pct=100).level(8.0), "OK")
    check("7.0V", BatteryModel(soc_pct=100).level(7.0), "WARN")
    check("6.6V", BatteryModel(soc_pct=100).level(6.6), "LOW")
    check("6.0V", BatteryModel(soc_pct=100).level(6.0), "CRITICAL")

    print("[7] 占空比指令格式")
    check("dut 格式", f"{50:03d}", "050")

    print()
    if failures:
        print(f"自检失败 {len(failures)} 项: {failures}")
        return 1
    print("自检全部通过")
    return 0


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------


def build_parser():
    p = argparse.ArgumentParser(
        description="电源管理后台服务（默认只读，不改变风扇转速）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("--loop", action="store_true", help="持续运行（默认只跑一次）")
    p.add_argument("--period", type=float, default=1.0, help="采样周期秒，默认 1.0")
    p.add_argument("--iterations", type=int, default=None, help="只跑 N 个周期")
    p.add_argument("--curve", default=DEFAULT_CURVE, choices=sorted(TEMP_CURVES),
                   help=f"温度曲线，默认 {DEFAULT_CURVE}")
    p.add_argument("--control-fan", action="store_true",
                   help="开启温控写通道（会下发 dut 命令改变风扇转速）")
    p.add_argument("--duty", type=int, default=None,
                   help="固定占空比 0-100 用于手动测试（隐含 --control-fan）")
    p.add_argument("--no-uart", action="store_true", help="完全不打开串口")
    p.add_argument("--on-exit-duty", type=int, default=0,
                   help="退出时下发的占空比，默认 0（停转）；-1 表示保持不动")
    p.add_argument("--status-file", default=None,
                   help="状态文件路径（原子写，供其它进程读取）")
    p.add_argument("--state-file", default=None,
                   help="电量持久化文件路径（掉电保留剩余电量）")
    p.add_argument("--state-interval", type=float, default=30.0,
                   help="电量持久化间隔秒，默认 30")
    p.add_argument("--log", default=None, help="JSONL 日志文件路径")
    p.add_argument("--quiet", action="store_true", help="不打印每周期状态行")
    p.add_argument("--selftest", action="store_true", help="运行纯算法自检后退出")
    return p


def main(argv=None):
    args = build_parser().parse_args(argv)

    if args.selftest:
        return selftest()

    control_fan = args.control_fan or args.duty is not None
    if control_fan and not args.control_fan:
        print("[power_manager] --duty 隐含开启温控写通道")

    pm = PowerManager(
        curve=args.curve,
        control_fan=control_fan,
        use_uart=not args.no_uart,
        state_path=args.state_file,
    )

    if args.duty is not None:
        # 固定占空比：只下发一次，不做闭环
        pm.open()
        try:
            pm.fan.set_duty(args.duty)
            print(f"[power_manager] duty set to {args.duty}%, rpm={pm.fan.get_rpm()}")
        finally:
            pm.close(on_exit_duty=None)
        return 0

    if not args.loop:
        pm.open()
        try:
            result = pm.tick()
            payload = dict(result)
            payload.update(pm.status())
            print(json.dumps(payload, ensure_ascii=False, indent=2))
            if args.status_file:
                atomic_write_json(args.status_file, payload)
        finally:
            pm.close(on_exit_duty=None)
        return 0

    pm.run(
        period=args.period,
        iterations=args.iterations,
        status_path=args.status_file,
        state_interval=args.state_interval,
        log_path=args.log,
        print_status=not args.quiet,
        on_exit_duty=None if args.on_exit_duty < 0 else args.on_exit_duty,
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
