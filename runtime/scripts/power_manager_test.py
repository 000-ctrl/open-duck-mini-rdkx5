#!/usr/bin/env python3
"""电源管理硬件诊断脚本。

默认只读：读电量、温度、风扇转速，**不会改变风扇转速**。
改变转速必须显式加 --yes。

用法
----
    # 硬件只读诊断
    python scripts/power_manager_test.py --read-only

    # 连续观察（不写转速）
    python scripts/power_manager_test.py --watch --period 2

    # 纯算法自检（不需要硬件）
    python scripts/power_manager_test.py --selftest

    # 改变风扇转速（需要显式确认）
    python scripts/power_manager_test.py --duty 40 --yes
    python scripts/power_manager_test.py --fan-curve-test --yes
"""

import argparse
import os
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
_SRC = os.path.join(os.path.dirname(_HERE), "mini_bdx_runtime")
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

try:
    from mini_bdx_runtime.peripheral import power_manager as pm
except ImportError:  # 源码目录未安装时直接按路径导入
    sys.path.insert(0, os.path.join(_SRC, "mini_bdx_runtime", "peripheral"))
    import power_manager as pm


def print_battery(snap):
    if not snap.get("ok"):
        print(f"  INA219 读取失败: {snap.get('error')}")
        return
    print(f"  总线电压 : {snap['bus_v']:.3f} V")
    print(f"  分流电压 : {snap['shunt_v'] * 1000:+.3f} mV")
    print(f"  电流     : {snap['current_a']:+.3f} A"
          f"   (放电为{'负' if pm.DISCHARGE_IS_NEGATIVE else '正'})")
    print(f"  功率     : {snap['power_w']:+.3f} W")
    print(f"  单体电压 : {snap['bus_v'] / pm.CELL_COUNT:.3f} V"
          f"   电量 ≈ {pm.ocv_to_soc(snap['bus_v'] / pm.CELL_COUNT):.1f}% (OCV)")
    if snap.get("overflow"):
        print("  !! 电流计算溢出：分流电压超出 PGA 量程，读数被削顶，SoC 不可信")


def print_temperature(zones):
    if not zones:
        print("  读不到任何 thermal zone")
        return
    for name, temp in sorted(zones.items()):
        print(f"  {name:16s}: {temp:.1f} °C")
    print(f"  {'最高':16s}: {max(zones.values()):.1f} °C")


def cmd_selftest(_args):
    return pm.selftest()


def cmd_read_only(args):
    print("=== INA219 电量 ===")
    pm_instance = pm.PowerManager(use_uart=not args.no_uart,
                                  state_path=args.state_file)
    pm_instance.open()

    def once(idx=None):
        tag = "" if idx is None else f"[{idx}] "
        snap = pm_instance.read_battery()
        print(f"{tag}--- 电量 ---")
        print_battery(snap)
        print(f"{tag}--- 温度 ---")
        print_temperature(pm_instance.read_temperature())
        if pm_instance.fan is not None:
            print(f"{tag}--- 风扇 ---")
            try:
                print(f"  转速     : {pm_instance.fan.get_rpm()} RPM")
                # 固件在风扇慢转/停转时会保持上一次读数（实测 duty=0 下
                # 连续 36 秒一模一样），所以这个数字不能当风扇状态用。
                print("  (注意：该遥测在低速/停转时会冻结，不可作为风扇状态证据)")
            except pm.FanLinkError as exc:
                print(f"  读取失败 : {exc}")

    try:
        if args.watch:
            idx = 0
            while True:
                idx += 1
                once(idx)
                print()
                time.sleep(args.period)
        else:
            once()
    except KeyboardInterrupt:
        print("\n中断")
    finally:
        pm_instance.close(on_exit_duty=None)
    return 0


def cmd_duty(args):
    if not args.yes:
        print("拒绝执行：改变风扇转速需要显式加 --yes")
        return 2
    link = pm.FanLink()
    link.open()
    try:
        before = link.get_rpm()
        link.set_duty(args.duty)
        time.sleep(args.settle)
        after = link.get_rpm()
        print(f"占空比已设为 {args.duty}%")
        print(f"转速: {before} RPM -> {after} RPM")
    finally:
        link.close()
    return 0


def cmd_fan_curve_test(args):
    """逐个占空比扫一遍，确认风扇响应。

    注意 RPM 只作参考：固件在低速/停转时会冻结上一次读数。
    要确认占空比真的改变了功耗，同时用 INA219 看总电流
    （实测 0/25/50/75/100% 单调，0% 比 100% 低约 55 mA）。
    """
    if not args.yes:
        print("拒绝执行：改变风扇转速需要显式加 --yes")
        return 2
    steps = [int(x) for x in args.steps.split(",")]
    link = pm.FanLink()
    link.open()
    rows = []
    try:
        for duty in steps:
            # 单步失败不要中断整个扫描：协议格式若有问题，
            # 需要一次看到全部档位的反应，而不是停在第一档。
            try:
                link.set_duty(duty)
            except pm.FanLinkError as exc:
                rows.append((duty, None, str(exc)))
                print(f"  duty={duty:3d}%  ->  设置失败: {exc}")
                continue
            time.sleep(args.settle)
            try:
                rpm = link.get_rpm()
            except pm.FanLinkError as exc:
                rpm = f"err({exc})"
            rows.append((duty, rpm, None))
            print(f"  duty={duty:3d}%  ->  {rpm} RPM")

        # 结束状态必须明确：设置失败时要说清风扇停在哪一档，
        # 不能让操作员以为它已经归零。
        final = 0 if args.leave_at_zero else steps[-1]
        last_ok = [d for d, _r, e in rows if e is None]
        try:
            link.set_duty(final)
            print(f"结束，占空比留在 {final}%")
        except pm.FanLinkError as exc:
            stuck = last_ok[-1] if last_ok else None
            print(f"结束设置 {final}% 失败: {exc}")
            if stuck is None:
                print("  !! 本次没有任何一条 dut 成功，风扇状态未知，请人工确认")
            else:
                print(f"  !! 风扇停在最后一次成功设置的 {stuck}%")
            return 3
    finally:
        link.close()
    return 0


def build_parser():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--read-only", action="store_true",
                   help="只读诊断电量/温度/转速（默认行为）")
    p.add_argument("--watch", action="store_true", help="持续观察")
    p.add_argument("--period", type=float, default=2.0, help="观察周期秒")
    p.add_argument("--no-uart", action="store_true", help="完全不打开串口")
    p.add_argument("--state-file", default=None, help="电量持久化文件路径")
    p.add_argument("--selftest", action="store_true", help="纯算法自检，不需要硬件")
    p.add_argument("--duty", type=int, default=None, help="设置固定占空比 0-100")
    p.add_argument("--fan-curve-test", action="store_true", help="占空比扫描测试")
    p.add_argument("--steps", default="0,10,20,30,40,50,70,100",
                   help="扫描的占空比列表，逗号分隔")
    p.add_argument("--settle", type=float, default=2.0, help="每次改变后的等待秒数")
    p.add_argument("--leave-at-zero", action="store_true", help="扫描结束后停在 0%%")
    p.add_argument("--yes", action="store_true", help="确认改变风扇转速")
    return p


def main():
    args = build_parser().parse_args()

    if args.selftest:
        return cmd_selftest(args)
    if args.duty is not None:
        return cmd_duty(args)
    if args.fan_curve_test:
        return cmd_fan_curve_test(args)
    return cmd_read_only(args)


if __name__ == "__main__":
    sys.exit(main())
