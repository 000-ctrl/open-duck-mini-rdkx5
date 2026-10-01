''' gc9a01a LCD driver for rdkx5 '''

import Hobot.GPIO as GPIO
from Hobot.GPIO import HIGH, LOW
import spidev
import time
from enum import IntEnum, IntFlag
from dataclasses import dataclass


class Config:
    TFTWIDTH = 240  # Display width in pixels
    TFTHEIGHT = 240  # Display height in pixels


class Madctl(IntFlag):
    MADCTL_MY = 0x80  # Bottom to top
    MADCTL_MX = 0x40  # Right to left
    MADCTL_MV = 0x20  # Reverse Mode
    MADCTL_ML = 0x10  # LCD refresh Bottom to top
    MADCTL_RGB = 0x00  # Red-Green-Blue pixel order
    MADCTL_BGR = 0x08  # Blue-Green-Red pixel order
    MADCTL_MH = 0x04  # LCD refresh right to left


class Command(IntEnum):
    SWRESET = 0x01  # Software Reset (maybe, not documented)
    RDDID = 0x04  # Read display identification information
    RDDST = 0x09  # Read Display Status
    SLPIN = 0x10  # Enter Sleep Mode
    SLPOUT = 0x11  # Sleep Out
    PTLON = 0x12  # Partial Mode ON
    NORON = 0x13  # Normal Display Mode ON
    INVOFF = 0x20  # Display Inversion OFF
    INVON = 0x21  # Display Inversion ON
    DISPOFF = 0x28  # Display OFF
    DISPON = 0x29  # Display ON
    CASET = 0x2A  # Column Address Set
    RASET = 0x2B  # Row Address Set
    RAMWR = 0x2C  # Memory Write
    PTLAR = 0x30  # Partial Area
    VSCRDEF = 0x33  # Vertical Scrolling Definition
    TEOFF = 0x34  # Tearing Effect Line OFF
    TEON = 0x35  # Tearing Effect Line ON
    MADCTL = 0x36  # Memory Access Control
    VSCRSADD = 0x37  # Vertical Scrolling Start Address
    IDLEOFF = 0x38  # Idle mode OFF
    IDLEON = 0x39  # Idle mode ON
    COLMOD = 0x3A  # Pixel Format Set
    CONTINUE = 0x3C  # Write Memory Continue
    TEARSET = 0x44  # Set Tear Scanline
    GETLINE = 0x45  # Get Scanline
    SETBRIGHT = 0x51  # Write Display Brightness
    SETCTRL = 0x53  # Write CTRL Display
    GC9A01A1_POWER7 = 0xA7  # Power Control 7
    TEWC = 0xBA  # Tearing effect width control
    GC9A01A1_POWER1 = 0xC1  # Power Control 1
    GC9A01A1_POWER2 = 0xC3  # Power Control 2
    GC9A01A1_POWER3 = 0xC4  # Power Control 3
    GC9A01A1_POWER4 = 0xC9  # Power Control 4
    RDID1 = 0xDA  # Read ID 1
    RDID2 = 0xDB  # Read ID 2
    RDID3 = 0xDC  # Read ID 3
    FRAMERATE = 0xE8  # Frame rate control
    SPI2DATA = 0xE9  # SPI 2DATA control
    INREGEN2 = 0xEF  # Inter register enable 2
    GAMMA1 = 0xF0  # Set gamma 1
    GAMMA2 = 0xF1  # Set gamma 2
    GAMMA3 = 0xF2  # Set gamma 3
    GAMMA4 = 0xF3  # Set gamma 4
    IFACE = 0xF6  # Interface control
    INREGEN1 = 0xFE  # Inter register enable 1


# Color definitions
class Color(IntEnum):
    BLACK = 0x0000  # 0,   0,   0
    NAVY = 0x000F  # 0,   0, 123
    DARKGREEN = 0x03E0  # 0, 125,   0
    DARKCYAN = 0x03EF  # 0, 125, 123
    MAROON = 0x7800  # 123,   0,   0
    PURPLE = 0x780F  # 123,   0, 123
    OLIVE = 0x7BE0  # 123, 125,   0
    LIGHTGREY = 0xC618  # 198, 195, 198
    DARKGREY = 0x7BEF  # 123, 125, 123
    BLUE = 0x001F  # 0,   0, 255
    GREEN = 0x07E0  # 0, 255,   0
    CYAN = 0x07FF  # 0, 255, 255
    RED = 0xF800  # 255,   0,   0
    MAGENTA = 0xF81F  # 255,   0, 255
    YELLOW = 0xFFE0  # 255, 255,   0
    WHITE = 0xFFFF  # 255, 255, 255
    ORANGE = 0xFD20  # 255, 165,   0
    GREENYELLOW = 0xAFE5  # 173, 255,  41
    PINK = 0xFC18  # 255, 130, 198


@dataclass(frozen=True)
class InitCmd:
    cmd: int
    data: list[int] | None = None
    delay: float = 0.00  # 单位：ms


# Initialization sequence came from some early code provided by the
# manufacturer. Many of these registers are undocumented, some might
# be unnecessary, just playing along...
init_list = [
    InitCmd(Command.INREGEN2),
    InitCmd(0xEB, [0x14]),
    InitCmd(Command.INREGEN1),
    InitCmd(Command.INREGEN2),
    InitCmd(0xEB, [0x14]),
    InitCmd(0x84, [0x40]),
    InitCmd(0x85, [0xFF]),
    InitCmd(0x86, [0xFF]),
    InitCmd(0x87, [0xFF]),
    InitCmd(0x88, [0x0A]),
    InitCmd(0x89, [0x21]),
    InitCmd(0x8A, [0x00]),
    InitCmd(0x8B, [0x80]),
    InitCmd(0x8C, [0x01]),
    InitCmd(0x8D, [0x01]),
    InitCmd(0x8E, [0xFF]),
    InitCmd(0x8F, [0xFF]),
    InitCmd(0xB6, [0x00, 0x00]),
    InitCmd(Command.MADCTL, [Madctl.MADCTL_MX | Madctl.MADCTL_BGR]),
    InitCmd(Command.COLMOD, [0x05]),
    InitCmd(0x90, [0x08, 0x08, 0x08, 0x08]),
    InitCmd(0xBD, [0x06]),
    InitCmd(0xBC, [0x00]),
    InitCmd(0xFF, [0x60, 0x01, 0x04]),
    InitCmd(Command.GC9A01A1_POWER2, [0x13]),
    InitCmd(Command.GC9A01A1_POWER3, [0x13]),
    InitCmd(Command.GC9A01A1_POWER4, [0x22]),
    InitCmd(0xBE, [0x11]),
    InitCmd(0xE1, [0x10, 0x0E]),
    InitCmd(0xDF, [0x21, 0x0c, 0x02]),
    InitCmd(Command.GAMMA1, [0x45, 0x09, 0x08, 0x08, 0x26, 0x2A]),
    InitCmd(Command.GAMMA2, [0x43, 0x70, 0x72, 0x36, 0x37, 0x6F]),
    InitCmd(Command.GAMMA3, [0x45, 0x09, 0x08, 0x08, 0x26, 0x2A]),
    InitCmd(Command.GAMMA4, [0x43, 0x70, 0x72, 0x36, 0x37, 0x6F]),
    InitCmd(0xED, [0x1B, 0x0B]),
    InitCmd(0xAE, [0x77]),
    InitCmd(0xCD, [0x63]),
    # Unsure what this line (from manufacturer's boilerplate code) is
    # meant to do, but users reported issues, seems to work OK without:
    # 0x70, 9, 0x07, 0x07, 0x04, 0x0E, 0x0F, 0x09, 0x07, 0x08, 0x03,
    InitCmd(Command.FRAMERATE, [0x34]),
    InitCmd(0x62, [0x18, 0x0D, 0x71, 0xED, 0x70, 0x70, 0x18, 0x0F, 0x71, 0xEF, 0x70, 0x70]),
    InitCmd(0x63, [0x18, 0x11, 0x71, 0xF1, 0x70, 0x70, 0x18, 0x13, 0x71, 0xF3, 0x70, 0x70]),
    InitCmd(0x64, [0x28, 0x29, 0xF1, 0x01, 0xF1, 0x00, 0x07]),
    InitCmd(0x66, [0x3C, 0x00, 0xCD, 0x67, 0x45, 0x45, 0x10, 0x00, 0x00, 0x00]),
    InitCmd(0x67, [0x00, 0x3C, 0x00, 0x00, 0x00, 0x01, 0x54, 0x10, 0x32, 0x98]),
    InitCmd(0x74, [0x10, 0x85, 0x80, 0x00, 0x00, 0x4E, 0x00]),
    InitCmd(0x98, [0x3e, 0x07]),
    InitCmd(Command.TEON),
    InitCmd(Command.INVON),
    InitCmd(Command.SLPOUT, delay=0.15),  # Exit sleep
    InitCmd(Command.DISPON, delay=0.15),  # Display on
]


class GC9A01A:
    def __init__(self, dc: int, rst: int = 0, blk: int = 0, bus: int = 1, device: int = 0, freq: int = 16_670_000,
                 n_rst: bool = True) -> None:
        """
        Initialize a GC9A01A display driver instance.

        Args:
            dc (int): GPIO pin connected to the D/C (Data/Command) signal.
            rst (int): GPIO pin connected to the RESET signal. Set to 0 if not used.
            blk (int): GPIO pin connected to the backlight control. Set to 0 if not used.
            bus (int): SPI bus number.
            device (int): SPI device (chip select) number.
            freq (int): SPI clock frequency in Hz.

        Returns:
            None
        """
        self.spi: spidev.SpiDev = None
        self.dc = dc
        self.rst = rst
        self.blk = blk
        self.bus = bus
        self.device = device
        self.freq = freq
        self.n_rst = n_rst
        self.width = Config.TFTWIDTH
        self.height = Config.TFTHEIGHT

    def _gpio_init(self):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup([self.rst, self.blk] if self.blk else [self.rst], GPIO.OUT, initial=HIGH)
        GPIO.setup(self.dc, GPIO.OUT)

    def _spi_init(self):
        self.spi = spidev.SpiDev()
        self.spi.open(self.bus, self.device)
        self.spi.max_speed_hz = self.freq
        self.spi.mode = 0
        self.spi.bits_per_word = 8

    def _hard_reset(self):
        GPIO.output(self.rst, LOW)
        time.sleep(0.02)
        GPIO.output(self.rst, HIGH)
        time.sleep(0.12)

    def _soft_reset(self):
        self._write_cmd(Command.SWRESET)
        time.sleep(0.15)

    def _write_cmd(self, cmd: int):
        GPIO.output(self.dc, LOW)
        self.spi.writebytes2([cmd])

    def _write_data(self, data: int | list[int]):
        GPIO.output(self.dc, HIGH)

        if isinstance(data, int):
            data = [data]

        self.spi.writebytes2(data)

    def _write(self, cmd: int, data: int | list[int] | None = None):

        self._write_cmd(cmd)

        if data is not None:
            self._write_data(data)

    def init(self):
        '''
        Initialize the LCD screen.
        '''
        self._gpio_init()
        self._spi_init()

        if self.n_rst:
            if self.rst:
                self._hard_reset()
            else:
                self._soft_reset()

        for item in init_list:
            self._write_cmd(item.cmd)

            if item.data:
                self._write_data(item.data)

            if item.delay:
                time.sleep(item.delay)

        self.set_rotation(0)
        self.turn_on()

    def turn_on(self):
        '''
        Turn on the backlight.
        '''
        if self.blk:
            GPIO.output(self.blk, HIGH)

    def turn_off(self):
        '''
        Turn off the backlight.
        '''
        if self.blk:
            GPIO.output(self.blk, LOW)

    def set_rotation(self, m: int):
        '''
        Set screen rotation angle.

        Args:
            m (int): 0, 1, 2, 3 -> 0°,90°, 180°, 270°
        '''
        if m not in [0, 1, 2, 3]:
            raise ValueError("Input value must be within [0, 1, 2, 3]")

        rotation = m % 4
        if rotation == 0:
            m = (Madctl.MADCTL_MX | Madctl.MADCTL_BGR)
            self.width = Config.TFTWIDTH
            self.height = Config.TFTHEIGHT
        elif rotation == 1:
            m = (Madctl.MADCTL_MV | Madctl.MADCTL_BGR)
            self.width = Config.TFTWIDTH
            self.height = Config.TFTHEIGHT
        elif rotation == 2:
            m = (Madctl.MADCTL_MY | Madctl.MADCTL_BGR)
            self.width = Config.TFTWIDTH
            self.height = Config.TFTHEIGHT
        elif rotation == 3:
            m = (Madctl.MADCTL_MX | Madctl.MADCTL_MY | Madctl.MADCTL_MV | Madctl.MADCTL_BGR)
            self.width = Config.TFTWIDTH
            self.height = Config.TFTHEIGHT

        self._write(Command.MADCTL, m)

    def close(self):
        '''
        Free up hardware resources.
        '''
        if self.spi is not None:
            self.spi.close()
            self.spi = None

        GPIO.cleanup()

    def set_lcd_size(self, w: int = 240, h: int = 240):
        '''
        Set screen resolution size.

        Args:
            w (int): Resolution Width.
            h (int): Resolution Height.
        '''
        self.width = w
        self.height = h

    def invert_display(self, invert: bool):
        '''
        Invert colors.

        Args:
            invert (int): True on, False off.
        '''
        self._write_cmd(Command.INVON if invert else Command.INVOFF)

    def _u16_to_bytes(self, value: int) -> list[int]:
        return [(value >> 8) & 0xFF, value & 0xFF]

    def _set_window(self, x: int, y: int, w: int, h: int):
        x2 = x + w - 1
        y2 = y + h - 1

        self._write(Command.CASET, [*self._u16_to_bytes(x), *self._u16_to_bytes(x2)])
        self._write(Command.RASET, [*self._u16_to_bytes(y), *self._u16_to_bytes(y2)])
        self._write(Command.RAMWR)

    def _write_pixels(self, data: bytes | bytearray | memoryview):
        GPIO.output(self.dc, HIGH)
        self.spi.writebytes2(data)

    @staticmethod
    def color565(r: int, g: int, b: int) -> int:
        '''
        Create an RGB565 color.

        Args:
            r (int): R value.
            g (int): G value.
            b (int): B value.

        Returns:
            int: Hex value.
        '''
        return (((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3))

    def draw_pixel(self, x: int, y: int, color: int):
        """
        Draw a single pixel at the specified coordinate.

        Args:
            x (int): X coordinate (0 to width-1)
            y (int): Y coordinate (0 to height-1)
            color (int): Pixel color in RGB565 format

        Returns:
            None
        """
        if not (0 <= x < self.width):
            return

        if not (0 <= y < self.height):
            return

        self._set_window(x, y, 1, 1)
        self._write_data(self._u16_to_bytes(color))

    def fill_rect(self, x: int, y: int, w: int, h: int, color: int):
        """
        Fill a rectangular area with a single color.

        Args:
            x (int): Top-left X coordinate
            y (int): Top-left Y coordinate
            w (int): Rectangle width in pixels
            h (int): Rectangle height in pixels
            color (int): Fill color in RGB565 format

        Returns:
            None
        """
        if w <= 0 or h <= 0:
            return

        if x >= self.width or y >= self.height:
            return

        if x + w > self.width:
            w = self.width - x

        if y + h > self.height:
            h = self.height - y

        self._set_window(x, y, w, h)

        hi = color >> 8
        lo = color & 0xFF

        pixel = bytes((hi, lo))
        chunk = bytearray(pixel * 8192)
        remain = w * h

        while remain >= 8192:
            self._write_pixels(chunk)
            remain -= 8192

        if remain:
            self._write_pixels(bytearray(pixel * remain))

    def fill(self, color: int):
        """
        Fill the entire screen with a single color.

        Args:
            color (int): Fill color in RGB565 format

        Returns:
            None
        """
        self.fill_rect(0, 0, self.width, self.height, color)

    def draw_fast_hline(self, x: int, y: int, w: int, color: int):
        """
        Draw a fast horizontal line.

        Args:
            x (int): Starting X coordinate
            y (int): Y coordinate
            w (int): Line width in pixels
            color (int): Line color in RGB565 format

        Returns:
            None
        """
        self.fill_rect(x, y, w, 1, color)

    def draw_fast_vline(self, x: int, y: int, h: int, color: int):
        """
        Draw a fast vertical line.

        Args:
            x (int): X coordinate
            y (int): Starting Y coordinate
            h (int): Line height in pixels
            color (int): Line color in RGB565 format

        Returns:
            None
        """
        self.fill_rect(x, y, 1, h, color)

    def draw_image(self, x: int, y: int, w: int, h: int, image: bytes | bytearray | memoryview):
        """
        Draw a raw image buffer to the display.

        Args:
            x (int): Top-left X coordinate
            y (int): Top-left Y coordinate
            w (int): Image width in pixels
            h (int): Image height in pixels
            image (bytes | bytearray | memoryview): Raw RGB565 image data

        Returns:
            None
        """
        self._set_window(x, y, w, h)
        GPIO.output(self.dc, HIGH)
        self._write_pixels(image)

    def draw_rgb565_buffer(self, x: int, y: int, w: int, h: int, buf):
        """
        Draw an RGB565 pixel buffer to a specified region.

        Args:
            x (int): Top-left X coordinate
            y (int): Top-left Y coordinate
            w (int): Width of the buffer region
            h (int): Height of the buffer region
            buf (bytes | bytearray | memoryview): RGB565 pixel buffer

        Returns:
            None
        """
        self._set_window(x, y, w, h)
        GPIO.output(self.dc, HIGH)
        self._write_pixels(buf)
