"""
INA219 Python Driver
====================

适用于 Texas Instruments INA219 电流/电压检测芯片。

本项目的实机硬件事实（2026-10-01 实测，RDK X5 /dev/i2c-5 @ 0x40）:

    分流电阻 : 10 mΩ   (0.01 Ω)
    总线量程 : 16 V    (2S 电池组满电 8.4 V)
    接线方向 : 放电时电流读数为负（详见 power_manager.DISCHARGE_IS_NEGATIVE）

注意：INA219 与 BNO055(IMU) 挂在同一条 I2C 总线上，总线号由
platform_compat.get_i2c_bus_number() 决定，不要硬编码。

依赖:
    pip install smbus2

快速使用
--------
from ina219 import INA219, MODE_CONTINUOUS

ina = INA219(address=0x40)

ina.set_brng(16)              # 总线量程: 16V/32V
ina.set_gain(160)             # 分流量程: 40/80/160/320(mV)
ina.set_bus_adc(16)           # 9/10/11/12 或 2/4/8/16/32/64/128(平均)
ina.set_shunt_adc(16)
ina.set_mode(MODE_CONTINUOUS)

ina.set_calibration(shunt=0.01, max_current=16.0)

print(ina.snapshot())
ina.close()
"""

import os
import sys
import time

try:
    from ..platform_compat import get_i2c_bus_number
except ImportError:  # 作为脚本直接运行时没有父包
    _PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if _PKG_DIR not in sys.path:
        sys.path.insert(0, _PKG_DIR)
    from platform_compat import get_i2c_bus_number

from smbus2 import SMBus

REG_CONFIG = 0x00
REG_SHUNT = 0x01
REG_BUS = 0x02
REG_POWER = 0x03
REG_CURRENT = 0x04
REG_CAL = 0x05

BRNG_16V = 0
BRNG_32V = 1

PGA_MAP = {40: 0, 80: 1, 160: 2, 320: 3}

MODE_POWERDOWN = 0
MODE_SHUNT_TRIGGERED = 1
MODE_BUS_TRIGGERED = 2
MODE_SHUNT_BUS_TRIGGERED = 3
MODE_ADC_OFF = 4
MODE_SHUNT_CONTINUOUS = 5
MODE_BUS_CONTINUOUS = 6
MODE_CONTINUOUS = 7

ADC_MAP = {9: 0, 10: 1, 11: 2, 12: 3, 2: 9, 4: 10, 8: 11, 16: 12, 32: 13, 64: 14, 128: 15}

# 分流电压寄存器 LSB，固定 10uV，与电阻无关
SHUNT_VOLTAGE_LSB = 10e-6
# 总线电压寄存器 LSB，固定 4mV
BUS_VOLTAGE_LSB = 0.004
# 12-bit 单次 ADC 转换时间
ADC_SINGLE_CONVERSION_S = 532e-6


class INA219NotCalibrated(RuntimeError):
    """未调用 set_calibration 就读取电流/功率时抛出。

    历史上这里会静默返回 0.0，导致"读数一直是 0 但看起来一切正常"。
    """


class INA219:
    """INA219 驱动。"""

    def __init__(self, bus=None, address=0x40):
        """
        Args:
            bus: I2C 总线号。None 表示按平台自动选择
                 (RDK X5 -> 5, Raspberry Pi -> 1)。
            address: 芯片 I2C 地址，本机为 0x40。
        """
        if bus is None:
            bus = get_i2c_bus_number()
        self.bus_number = bus
        self.bus = SMBus(bus)
        self.address = address
        self.config = 0x399F
        self.current_lsb = 0.0
        self.power_lsb = 0.0
        self.shunt_ohm = None
        self.max_current_a = None

    def close(self):
        self.bus.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
        return False

    @staticmethod
    def _swap(v: int) -> int:
        return ((v & 0xFF) << 8) | ((v >> 8) & 0xFF)

    @staticmethod
    def _signed(v: int) -> int:
        return v - 0x10000 if v & 0x8000 else v

    def read_reg(self, reg: int) -> int:
        """读取16位寄存器(自动处理大小端)。"""
        return self._swap(self.bus.read_word_data(self.address, reg))

    def write_reg(self, reg: int, value: int):
        """写16位寄存器。"""
        self.bus.write_word_data(self.address, reg, self._swap(value & 0xFFFF))

    @staticmethod
    def _apply_mask(config: int, mask: int, shift: int, value: int) -> int:
        """把 value 按 shift/mask 合并进 config 字，返回新 config。"""
        return (config & ~mask) | ((value << shift) & mask)

    def _update(self, mask, shift, value):
        self.config = self._apply_mask(self.config, mask, shift, value)
        self.write_reg(REG_CONFIG, self.config)

    def reset(self):
        """软件复位。"""
        self.write_reg(REG_CONFIG, 0x8000)
        self.config = 0x399F

    def set_brng(self, voltage: int):
        """设置Bus量程，仅支持16或32(V)。"""
        if voltage not in (16, 32):
            raise ValueError("bus range must be 16 or 32")
        self._update(1 << 13, 13, BRNG_32V if voltage == 32 else BRNG_16V)

    def set_gain(self, mv: int):
        """设置PGA量程：40/80/160/320(mV)。"""
        if mv not in PGA_MAP:
            raise ValueError(f"gain must be one of {sorted(PGA_MAP)}")
        self._update(0x3 << 11, 11, PGA_MAP[mv])

    def set_bus_adc(self, opt: int):
        """Bus ADC: 9~12(bit) 或 2~128(平均次数)。"""
        self._update(0xF << 7, 7, ADC_MAP[opt])

    def set_shunt_adc(self, opt: int):
        """Shunt ADC: 9~12(bit) 或 2~128(平均次数)。"""
        self._update(0xF << 3, 3, ADC_MAP[opt])

    def set_mode(self, mode: int):
        """设置工作模式(0~7)。推荐 MODE_CONTINUOUS。"""
        self._update(0x7, 0, mode)

    def set_default_config(self):
        self.config = 0x399F
        self.write_reg(REG_CONFIG, self.config)

    def set_calibration(self, shunt: float, max_current: float):
        """计算并写入 Calibration 寄存器。

        Args:
            shunt: 分流电阻阻值(欧姆)，本机为 0.01。
            max_current: 期望的可测最大电流(A)，决定电流寄存器 LSB。
                         注意它只影响标度，硬件钳位由 PGA 量程决定，
                         两者必须匹配，否则大电流时会被削顶。
        """
        if shunt <= 0:
            raise ValueError("shunt must be > 0")
        if max_current <= 0:
            raise ValueError("max_current must be > 0")

        self.shunt_ohm = shunt
        self.max_current_a = max_current
        self.current_lsb = max_current / 32768
        self.power_lsb = self.current_lsb * 20

        # 0.04096 = 0.00512 * 8 (内部标度常数)，用 round 而非 int，
        # 避免截断引入固定的比例误差。
        cal = round(0.04096 / (self.current_lsb * shunt))
        if cal > 0xFFFF:
            raise ValueError(
                f"calibration overflow ({cal}); 减小 max_current 或增大分流电阻"
            )
        self.write_reg(REG_CAL, cal)

        # 已知硬件怪癖：本芯片 CAL 寄存器回读时 bit0 恒为 0
        # （写 8389 读回 8388，写 1 读回 0；CONFIG 的 bit0 正常）。
        # 影响 1/8389 ≈ 0.012%，忽略；不要用回读值做断言。

    def is_calibrated(self) -> bool:
        return self.current_lsb > 0.0

    def _require_calibration(self):
        if not self.is_calibrated():
            raise INA219NotCalibrated(
                "先调用 set_calibration(shunt, max_current) 再读电流/功率"
            )

    def _adc_samples(self, config: int, shift: int) -> int:
        """从 ADC 配置位解出平均次数（9~11bit/12bit 都是 1 次）。"""
        v = (config >> shift) & 0x0F
        return 1 if v <= 3 else (1 << (v - 8))

    def conversion_period_s(self) -> float:
        """一次完整(分流 + 总线)转换的周期。

        连续模式下 CURRENT / POWER 寄存器只在转换结束时刷新，
        这个周期就是读数的最短有效间隔。
        """
        config = self.read_reg(REG_CONFIG)
        samples = self._adc_samples(config, 7) + self._adc_samples(config, 3)
        return samples * ADC_SINGLE_CONVERSION_S

    def wait_for_conversion(self, cycles: float = 2.0):
        """等待若干次转换。

        改了 CAL 之后必须等一次转换，否则 CURRENT 寄存器里还是
        用旧标度算出来的值（实测 CAL 从 old->new 变 5 倍时，
        紧跟着的读数正好差 5 倍）。
        """
        time.sleep(self.conversion_period_s() * cycles)

    def configure_defaults(self, shunt=0.01, max_current=16.0):
        """按本项目实机硬件一次性配置。返回后所有读数才是有效的。"""
        self.set_brng(16)
        self.set_gain(160 if max_current > 8 else 80)
        self.set_bus_adc(16)
        self.set_shunt_adc(16)
        self.set_mode(MODE_CONTINUOUS)
        self.set_calibration(shunt=shunt, max_current=max_current)
        # 等新标度生效，避免第一次读数是用旧 CAL 算出来的陈旧值
        self.wait_for_conversion(2.0)

    def bus_voltage(self) -> float:
        return (self.read_reg(REG_BUS) >> 3) * BUS_VOLTAGE_LSB

    def math_overflow(self) -> bool:
        """电流/功率计算的数学溢出标志（寄存器 0x02 的 bit0）。

        INA219 总线电压寄存器 0x02 的低位：
            bit0 = OVF  : 电流/功率计算溢出（分流电压超出 PGA 量程）
            bit1 = CNVR : 转换完成（连续模式下会一直翻转，不要当溢出用）
        """
        return bool(self.read_reg(REG_BUS) & 0x0001)

    def conversion_ready(self) -> bool:
        """一次转换是否已完成（寄存器 0x02 的 bit1）。"""
        return bool(self.read_reg(REG_BUS) & 0x0002)

    def shunt_voltage(self) -> float:
        return self._signed(self.read_reg(REG_SHUNT)) * SHUNT_VOLTAGE_LSB

    def current(self) -> float:
        """电流(A)。正负号取决于分流电阻接线方向。"""
        self._require_calibration()
        return self._signed(self.read_reg(REG_CURRENT)) * self.current_lsb

    def power(self) -> float:
        """硬件功率寄存器(W)。

        注意：该寄存器为无符号，会丢掉电流方向。需要带符号功率时用
        power_measured() 或自己算 voltage * current。
        """
        self._require_calibration()
        return self.read_reg(REG_POWER) * self.power_lsb

    def power_measured(self) -> float:
        """用 bus_voltage * current 计算的带符号功率(W)。"""
        self._require_calibration()
        return self.bus_voltage() * self.current()

    def snapshot(self) -> dict:
        """一次读取电压/电流/功率，返回字典。

        读失败不抛异常，返回 ok=False，方便后台服务做退避重试。
        """
        out = {
            "ok": False,
            "error": None,
            "bus_v": None,
            "shunt_v": None,
            "current_a": None,
            "power_w": None,
            "overflow": None,   # 电流/功率计算溢出(0x02 bit0)，1 表示读数被削顶
        }
        try:
            raw_bus = self.read_reg(REG_BUS)
            out["overflow"] = bool(raw_bus & 0x0001)
            out["bus_v"] = (raw_bus >> 3) * BUS_VOLTAGE_LSB
            out["shunt_v"] = self._signed(self.read_reg(REG_SHUNT)) * SHUNT_VOLTAGE_LSB
            if self.is_calibrated():
                out["current_a"] = (
                    self._signed(self.read_reg(REG_CURRENT)) * self.current_lsb
                )
                out["power_w"] = out["bus_v"] * out["current_a"]
            else:
                # 未校准时用分流电阻直接换算，避免返回 None 让人误以为没接线
                out["current_a"] = (
                    out["shunt_v"] / self.shunt_ohm if self.shunt_ohm else None
                )
                if out["current_a"] is not None:
                    out["power_w"] = out["bus_v"] * out["current_a"]
            out["ok"] = True
        except Exception as exc:  # noqa: BLE001 - 后台服务需要吞掉总线抖动
            out["error"] = f"{type(exc).__name__}: {exc}"
        return out

    def dump_registers(self):
        """打印所有寄存器。"""
        for n, r in [
            ("CONFIG", 0),
            ("SHUNT", 1),
            ("BUS", 2),
            ("POWER", 3),
            ("CURRENT", 4),
            ("CAL", 5),
        ]:
            print(f"{n:8}: 0x{self.read_reg(r):04X}")

    def print_config(self):
        """
        打印当前 Configuration Register 的配置。
        """

        config = self.read_reg(REG_CONFIG)

        brng = (config >> 13) & 0x01
        pga = (config >> 11) & 0x03
        badc = (config >> 7) & 0x0F
        sadc = (config >> 3) & 0x0F
        mode = config & 0x07

        print("========== INA219 Configuration ==========")
        print(f"CONFIG Register : 0x{config:04X}")
        print()

        print(f"Bus Voltage Range : {16 if brng == 0 else 32} V")

        pga_table = {
            0: "±40mV (Gain ×1)",
            1: "±80mV (Gain ×2)",
            2: "±160mV (Gain ×4)",
            3: "±320mV (Gain ×8)",
        }

        print(f"PGA Gain          : {pga_table[pga]}")

        adc_table = {
            0: "9-bit",
            1: "10-bit",
            2: "11-bit",
            3: "12-bit",
            9: "12-bit, 2 Samples",
            10: "12-bit, 4 Samples",
            11: "12-bit, 8 Samples",
            12: "12-bit, 16 Samples",
            13: "12-bit, 32 Samples",
            14: "12-bit, 64 Samples",
            15: "12-bit, 128 Samples",
        }

        print(f"Bus ADC           : {adc_table.get(badc, 'Unknown')}")
        print(f"Shunt ADC         : {adc_table.get(sadc, 'Unknown')}")

        mode_table = {
            0: "Power Down",
            1: "Shunt Triggered",
            2: "Bus Triggered",
            3: "Shunt + Bus Triggered",
            4: "ADC Off",
            5: "Shunt Continuous",
            6: "Bus Continuous",
            7: "Shunt + Bus Continuous",
        }

        print(f"Mode              : {mode_table[mode]}")
        print("==========================================")


if __name__ == "__main__":
    import time

    # 本项目实机配置：10mΩ 分流，±160mV PGA -> ±16A 硬件钳位
    ina = INA219(address=0x40)
    ina.configure_defaults(shunt=0.01, max_current=16.0)

    print(f"I2C bus: /dev/i2c-{ina.bus_number}, address: 0x{ina.address:02X}")
    ina.print_config()

    try:
        while True:
            s = ina.snapshot()
            if s["ok"]:
                print(
                    f"Bus: {s['bus_v']:.3f} V   "
                    f"Shunt: {s['shunt_v'] * 1000:+.3f} mV   "
                    f"Current: {s['current_a']:+.3f} A   "
                    f"Power: {s['power_w']:+.3f} W",
                    end="\n\n",
                )
            else:
                print(f"read failed: {s['error']}")
            time.sleep(2)

    except KeyboardInterrupt:
        ina.close()
