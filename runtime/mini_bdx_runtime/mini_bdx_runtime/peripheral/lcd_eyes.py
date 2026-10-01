from gc9a01a import GC9A01A, Color
import time
from PIL import Image, ImageFont, ImageDraw, ImageSequence
import os
from ina219 import INA219
import numpy as np
from pathlib import Path

FPS_SETTING = 12


def gif_to_fixed_fps(path, fps=15):
    gif = Image.open(path)

    frames = []
    times = []

    t = 0

    # 读取原始帧
    for frame in ImageSequence.Iterator(gif):
        duration = frame.info.get("duration", 100)
        frames.append(frame.convert("RGB").resize((240, 240)))
        times.append(t)
        t += duration

    total_time = t

    # 新时间间隔
    interval = 1000 / fps
    result = []
    current = 0
    index = 0

    while current < total_time:
        # 找当前时间对应原帧
        while (
                index + 1 < len(times)
                and times[index + 1] <= current
        ):
            index += 1
        result.append(frames[index])
        current += interval

    return result


def rgb888_to_rgb565(img):
    arr = np.array(img, dtype=np.uint16)

    r = arr[:, :, 0]
    g = arr[:, :, 1]
    b = arr[:, :, 2]

    rgb565 = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)

    return rgb565.astype(">u2").tobytes()


def render_text_region(lcd, x, y, text, font, color="black", bg="white"):
    # 1. 计算区域
    dummy = Image.new("RGB", (1, 1))
    d = ImageDraw.Draw(dummy)
    x1, y1, x2, y2 = d.textbbox((0, 0), text, font=font)

    w, h = x2 - x1, y2 - y1

    # 2. 创建局部画布
    img = Image.new("RGB", (int(w), int(h)), bg)
    draw = ImageDraw.Draw(img)

    # 3. 画文字
    draw.text((-x1, -y1), text, font=font, fill=color)

    # 4. 转 RGB565
    buf = rgb888_to_rgb565(img)

    # 5. 局部刷新
    lcd.draw_image(x, y, w, h, buf)


if __name__ == "__main__":
    lcd_l = GC9A01A(dc=27, rst=29, blk=37, freq=20_000_000, device=0)
    lcd_r = GC9A01A(dc=27, rst=31, blk=37, freq=20_000_000, device=1)

    lcd_l.init()
    lcd_r.init()

    lcd_l.fill(Color.BLACK)
    lcd_r.fill(Color.BLACK)

    # lcd_l.fill(Color.WHITE)
    # lcd_r.fill(Color.WHITE)

    # images = gif_to_fixed_fps(f"{os.getcwd()}/test-hardware/resource/cat.gif", FPS_SETTING)
    # lx1, ly1, lx2, ly2 = None, None, None, None
    # frame_time = 1 / FPS_SETTING
    # next_time = time.perf_counter()
    # while True:
    #     for i, f in enumerate(images):
    #         x1, y1, x2, y2 = f.getbbox()
    #         nx1, ny1, nx2, ny2 = x1, y1, x2, y2

    #         if lx1 is not None:
    #             nx1 = min(x1, lx1)
    #             ny1 = min(y1, ly1) # type: ignore
    #             nx2 = max(x2, lx2) # type: ignore
    #             ny2 = max(y2, ly2) # type: ignore

    #         f = f.crop((nx1, ny1, nx2, ny2))
    #         lx1, ly1, lx2, ly2 = x1, y1, x2, y2

    #         lcd_l.draw_image(nx1, ny1, nx2-nx1, ny2-ny1, rgb888_to_rgb565(f))

    #         # 稳定帧率
    #         next_time += frame_time
    #         time.sleep(max(0, next_time - time.perf_counter()))

    # pho = Image.open(f"{Path(__file__).parent}/resource/eyes.png").resize((180, 180))
    pho = Image.open(f"/home/sunrise/openduckmini/test-hardware/resource/eyes.png").resize((180, 180))

    w, h = pho.size[0], pho.size[1]
    # print(w, h)

    fps = 0
    start = time.time()
    count = 0

    while True:
        lcd_l.draw_image(120 - w // 2, 120 - h // 2, w, h, rgb888_to_rgb565(pho))
        lcd_r.draw_image(120 - w // 2, 120 - h // 2, w, h, rgb888_to_rgb565(pho))
        count += 2
        if time.time() - start >= 1:
            print(f"FPS: {((count / 2) / (time.time() - start)):.4f}")
            count = 0
            start = time.time()

