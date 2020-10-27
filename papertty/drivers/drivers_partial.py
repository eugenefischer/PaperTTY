#     Copyright (c) 2018 Jouko Strömmer
#     Copyright (c) 2017 Waveshare
#
#     This program is free software: you can redistribute it and/or modify
#     it under the terms of the GNU General Public License as published by
#     the Free Software Foundation, either version 3 of the License, or
#     (at your option) any later version.
#
#     This program is distributed in the hope that it will be useful,
#     but WITHOUT ANY WARRANTY; without even the implied warranty of
#     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#     GNU General Public License for more details.
#
#     You should have received a copy of the GNU General Public License
#     along with this program.  If not, see <https://www.gnu.org/licenses/>.
from papertty.drivers.drivers_base import WaveshareEPD


class WavesharePartial(WaveshareEPD):
    """Displays that support partial refresh (*monochrome*): 1.54", 2.13", 2.9". 
    The code is almost entirely identical with these, just small differences in the 2.13"."""

    BOOSTER_SOFT_START_CONTROL = 0x0C
    BORDER_WAVEFORM_CONTROL = 0x3C
    DATA_ENTRY_MODE_SETTING = 0x11
    DEEP_SLEEP_MODE = 0x10
    DISPLAY_UPDATE_CONTROL_1 = 0x21
    DISPLAY_UPDATE_CONTROL_2 = 0x22
    DRIVER_OUTPUT_CONTROL = 0x01
    GATE_SCAN_START_POSITION = 0x0F
    MASTER_ACTIVATION = 0x20
    SET_DUMMY_LINE_PERIOD = 0x3A
    SET_GATE_TIME = 0x3B
    SET_RAM_X_ADDRESS_COUNTER = 0x4E
    SET_RAM_X_ADDRESS_START_END_POSITION = 0x44
    SET_RAM_Y_ADDRESS_COUNTER = 0x4F
    SET_RAM_Y_ADDRESS_START_END_POSITION = 0x45
    SW_RESET = 0x12
    TEMPERATURE_SENSOR_CONTROL = 0x1A
    TERMINATE_FRAME_READ_WRITE = 0xFF
    WRITE_LUT_REGISTER = 0x32
    WRITE_RAM = 0x24
    WRITE_VCOM_REGISTER = 0x2C

    # these LUTs are used by 1.54" and 2.9" - 2.13" overrides them
    lut_full_update = [
        0x02, 0x02, 0x01, 0x11, 0x12, 0x12, 0x22, 0x22,
        0x66, 0x69, 0x69, 0x59, 0x58, 0x99, 0x99, 0x88,
        0x00, 0x00, 0x00, 0x00, 0xF8, 0xB4, 0x13, 0x51,
        0x35, 0x51, 0x51, 0x19, 0x01, 0x00
    ]

    lut_partial_update = [
        0x10, 0x18, 0x18, 0x08, 0x18, 0x18, 0x08, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x13, 0x14, 0x44, 0x12,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    ]

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.supports_partial = True
        self.colors = 2
        self.lut = None

    def init(self, partial=True):
        self.partial_refresh = partial
        if self.epd_init() != 0:
            return -1
        # EPD hardware init start
        self.lut = self.lut_partial_update if partial else self.lut_full_update
        self.reset()
        self.send_command(self.DRIVER_OUTPUT_CONTROL)
        self.send_data((self.height - 1) & 0xFF)
        self.send_data(((self.height - 1) >> 8) & 0xFF)
        self.send_data(0x00)  # GD = 0 SM = 0 TB = 0
        self.send_command(self.BOOSTER_SOFT_START_CONTROL)
        self.send_data(0xD7)
        self.send_data(0xD6)
        self.send_data(0x9D)
        self.send_command(self.WRITE_VCOM_REGISTER)
        self.send_data(0xA8)  # VCOM 7C
        self.send_command(self.SET_DUMMY_LINE_PERIOD)
        self.send_data(0x1A)  # 4 dummy lines per gate
        self.send_command(self.SET_GATE_TIME)
        self.send_data(0x08)  # 2us per line
        self.send_command(self.DATA_ENTRY_MODE_SETTING)
        self.send_data(0x03)  # X increment Y increment
        self.set_lut(self.lut)
        # EPD hardware init end
        return 0

    def wait_until_idle(self):
        while self.digital_read(self.BUSY_PIN) == 1:  # 0: idle, 1: busy
            self.delay_ms(100)

    def set_lut(self, lut):
        self.lut = lut
        self.send_command(self.WRITE_LUT_REGISTER)
        # the length of look-up table is 30 bytes
        for i in range(0, len(lut)):
            self.send_data(self.lut[i])

    def get_frame_buffer(self, image):
        buf = [0x00] * int(self.width * self.height / 8)
        # Set buffer to value of Python Imaging Library image.
        # Image must be in mode 1.
        image_monocolor = image.convert('1')
        imwidth, imheight = image_monocolor.size
        if imwidth != self.width or imheight != self.height:
            raise ValueError('Image must be same dimensions as display \
                ({0}x{1}).'.format(self.width, self.height))

        pixels = image_monocolor.load()
        for y in range(self.height):
            for x in range(self.width):
                # Set the bits for the column of pixels at the current position.
                if pixels[x, y] != 0:
                    buf[int((x + y * self.width) / 8)] |= 0x80 >> (x % 8)
        return buf

    # this differs with 2.13" but is the same for 1.54" and 2.9"
    def set_frame_memory(self, image, x, y):
        if image is None or x < 0 or y < 0:
            return
        image_monocolor = image.convert('1')
        image_width, image_height = image_monocolor.size
        # x point must be the multiple of 8 or the last 3 bits will be ignored
        x = x & 0xF8
        image_width = image_width & 0xF8
        if x + image_width >= self.width:
            x_end = self.width - 1
        else:
            x_end = x + image_width - 1
        if y + image_height >= self.height:
            y_end = self.height - 1
        else:
            y_end = y + image_height - 1
        self.set_memory_area(x, y, x_end, y_end)
        self.set_memory_pointer(x, y)
        self.send_command(self.WRITE_RAM)
        # send the image data
        pixels = image_monocolor.load()
        byte_to_send = 0x00
        for j in range(0, y_end - y + 1):
            # 1 byte = 8 pixels, steps of i = 8
            for i in range(0, x_end - x + 1):
                # Set the bits for the column of pixels at the current position.
                if pixels[i, j] != 0:
                    byte_to_send |= 0x80 >> (i % 8)
                if i % 8 == 7:
                    self.send_data(byte_to_send)
                    byte_to_send = 0x00

    def clear_frame_memory(self, color):
        self.set_memory_area(0, 0, self.width - 1, self.height - 1)
        self.set_memory_pointer(0, 0)
        self.send_command(self.WRITE_RAM)
        # send the color data
        for i in range(0, int(self.width / 8 * self.height)):
            self.send_data(color)

    def display_frame(self):
        self.send_command(self.DISPLAY_UPDATE_CONTROL_2)
        self.send_data(0xC4)
        self.send_command(self.MASTER_ACTIVATION)
        self.send_command(self.TERMINATE_FRAME_READ_WRITE)
        self.wait_until_idle()

    def set_memory_area(self, x_start, y_start, x_end, y_end):
        self.send_command(self.SET_RAM_X_ADDRESS_START_END_POSITION)
        # x point must be the multiple of 8 or the last 3 bits will be ignored
        self.send_data((x_start >> 3) & 0xFF)
        self.send_data((x_end >> 3) & 0xFF)
        self.send_command(self.SET_RAM_Y_ADDRESS_START_END_POSITION)
        self.send_data(y_start & 0xFF)
        self.send_data((y_start >> 8) & 0xFF)
        self.send_data(y_end & 0xFF)
        self.send_data((y_end >> 8) & 0xFF)

    def set_memory_pointer(self, x, y):
        self.send_command(self.SET_RAM_X_ADDRESS_COUNTER)
        # x point must be the multiple of 8 or the last 3 bits will be ignored
        self.send_data((x >> 3) & 0xFF)
        self.send_command(self.SET_RAM_Y_ADDRESS_COUNTER)
        self.send_data(y & 0xFF)
        self.send_data((y >> 8) & 0xFF)
        self.wait_until_idle()

    def sleep(self):
        self.send_command(self.DEEP_SLEEP_MODE)
        self.wait_until_idle()

    def draw(self, x, y, image):
        """Replace a particular area on the display with an image"""
        self.set_frame_memory(image, x, y)
        self.display_frame()
        if self.partial_refresh:
            # set the memory again if partial refresh LUT is used
            self.set_frame_memory(image, x, y)
            self.display_frame()
            

class rpi_epd2in7(object)
    #code from the rpi_epd2in7 partial update driver by Elad Alfassa
    def __init__(self, partial_refresh_limit=32, fast_refresh=True):
        """ Initialize the EPD class.
        `partial_refresh_limit` - number of partial refreshes before a full refrersh is forced
        `fast_frefresh` - enable or disable the fast refresh mode,
                          see smart_update() method documentation for details"""
        self.width = EPD_WIDTH
        """ Display width, in pixels """
        self.height = EPD_HEIGHT
        """ Display height, in pixels """
        self.fast_refresh = fast_refresh
        """ enable or disable the fast refresh mode """
        self.partial_refresh_limit = partial_refresh_limit
        """ number of partial refreshes before a full refrersh is forced """

        self._last_frame = None
        self._partial_refresh_count = 0
        self._init_performed = False
        self.spi = spidev.SpiDev(0, 0)

    def digital_write(self, pin, value):
        return GPIO.output(pin, value)

    def digital_read(self, pin):
        return GPIO.input(pin)

    def delay_ms(self, delaytime):
        time.sleep(delaytime / 1000.0)

    def send_command(self, command):
        self.digital_write(DC_PIN, GPIO.LOW)
        self.spi.writebytes([command])

    def send_data(self, data):
        self.digital_write(DC_PIN, GPIO.HIGH)
        self.spi.writebytes([data])

    def init(self):
        """ Preform the hardware initialization sequence """
        # Interface initialization:
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(RST_PIN, GPIO.OUT)
        GPIO.setup(DC_PIN, GPIO.OUT)
        GPIO.setup(CS_PIN, GPIO.OUT)
        GPIO.setup(BUSY_PIN, GPIO.IN)

        self.spi.max_speed_hz = 2000000
        self.spi.mode = 0b00
        # EPD hardware init
        # The specifics of how this works or what "power optimization" actually means
        # are unclear to me, so I'm leaving it as-is.
        self.reset()
        self.send_command(POWER_SETTING)
        self.send_data(0x03)                  # VDS_EN, VDG_EN
        self.send_data(0x00)                  # VCOM_HV, VGHL_LV[1], VGHL_LV[0]
        self.send_data(0x2b)                  # VDH
        self.send_data(0x2b)                  # VDL
        self.send_data(0x09)                  # VDHR
        self.send_command(BOOSTER_SOFT_START)
        self.send_data(0x07)
        self.send_data(0x07)
        self.send_data(0x17)
        # Power optimization
        self.send_command(0xF8)
        self.send_data(0x60)
        self.send_data(0xA5)
        # Power optimization
        self.send_command(0xF8)
        self.send_data(0x89)
        self.send_data(0xA5)
        # Power optimization
        self.send_command(0xF8)
        self.send_data(0x90)
        self.send_data(0x00)
        # Power optimization
        self.send_command(0xF8)
        self.send_data(0x93)
        self.send_data(0x2A)
        # Power optimization
        self.send_command(0xF8)
        self.send_data(0xA0)
        self.send_data(0xA5)
        # Power optimization
        self.send_command(0xF8)
        self.send_data(0xA1)
        self.send_data(0x00)
        # Power optimization
        self.send_command(0xF8)
        self.send_data(0x73)
        self.send_data(0x41)
        self.send_command(PARTIAL_DISPLAY_REFRESH)
        self.send_data(0x00)
        self.send_command(POWER_ON)
        self.wait_until_idle()

        self.send_command(PANEL_SETTING)
        self.send_data(0xAF)        # KW-BF   KWR-AF    BWROTP 0f
        self.send_command(PLL_CONTROL)
        self.send_data(0x3A)        # 3A 100HZ   29 150Hz 39 200HZ    31 171HZ
        self.send_command(VCM_DC_SETTING_REGISTER)
        self.send_data(0x12)
        self.delay_ms(2)
        self.set_lut()
        # EPD hardware init end
        self._init_performed = True

    def wait_until_idle(self):
        """ Wait until screen is idle by polling the busy pin """
        while(self.digital_read(BUSY_PIN) == 0):      # 0: busy, 1: idle
            self.delay_ms(50)

    def reset(self):
        """ Module reset """
        self.digital_write(RST_PIN, GPIO.LOW)
        self.delay_ms(200)
        self.digital_write(RST_PIN, GPIO.HIGH)
        self.delay_ms(200)

    def set_lut(self, fast=False):
        """ Set LUT for the controller.
        If `fast` is srt to True, quick update LUTs from Ben Krasnow will be used"""
        lut_to_use = LUT if not fast else QuickLUT

        # Quick LUTs courtsey of Ben Krasnow:
        # http://benkrasnow.blogspot.co.il/2017/10/fast-partial-refresh-on-42-e-paper.html
        # https://www.youtube.com/watch?v=MsbiO8EAsGw

        self.send_command(LUT_FOR_VCOM)               # vcom
        for byte in lut_to_use.lut_vcom_dc:
            self.send_data(byte)

        self.send_command(LUT_WHITE_TO_WHITE)         # ww --
        for byte in lut_to_use.lut_ww:
            self.send_data(byte)

        self.send_command(LUT_BLACK_TO_WHITE)         # bw r
        for byte in lut_to_use.lut_bw:
            self.send_data(byte)

        self.send_command(LUT_WHITE_TO_BLACK)         # wb w
        for byte in lut_to_use.lut_wb:
            self.send_data(byte)

        self.send_command(LUT_BLACK_TO_BLACK)         # bb b
        for byte in lut_to_use.lut_bb:
            self.send_data(byte)

    def _get_frame_buffer(self, image):
        """ Get a full frame buffer from a PIL Image object """
        image_monocolor = image.convert('1')
        imwidth, imheight = image_monocolor.size
        if imwidth != self.width or imheight != self.height:
            raise ValueError('Image must be same dimensions as display \
                ({0}x{1}).' .format(self.width, self.height))
        return self._get_frame_buffer_for_size(image_monocolor, self.height, self.width)

    def _get_frame_buffer_for_size(self, image_monocolor, height, width):
        """ Get a frame buffer object from a PIL Image object assuming a specific size"""
        buf = [0x00] * (width * height // 8)
        pixels = image_monocolor.load()
        for y in range(height):
            for x in range(width):
                # Set the bits for the column of pixels at the current position
                if pixels[x, y] != 0:
                    buf[(x + y * width) // 8] |= (0x80 >> (x % 8))
        return buf

    def display_frame(self, image):
        """ Display a full frame, doing a full screen refresh """
        if not self._init_performed:
            # Initialize the hardware if it wasn't already initialized
            self.init()
        self.set_lut()
        frame_buffer = self._get_frame_buffer(image)
        self.send_command(DATA_START_TRANSMISSION_1)
        self.delay_ms(2)
        for _ in range(0, self.width * self.height // 8):
            self.send_data(0xFF)
        self.delay_ms(2)
        self.send_command(DATA_START_TRANSMISSION_2)
        self.delay_ms(2)
        for i in range(0, self.width * self.height // 8):
            self.send_data(frame_buffer[i])
        self.delay_ms(2)
        self.send_command(DISPLAY_REFRESH)
        self.wait_until_idle()
        self._last_frame = image.copy()
        self._partial_refresh_count = 0  # reset the partial refreshes counter

    def _send_partial_frame_dimensions(self, x, y, l, w):
        self.send_data(x >> 8)
        self.send_data(x & 0xf8)
        self.send_data(y >> 8)
        self.send_data(y & 0xff)
        self.send_data(w >> 8)
        self.send_data(w & 0xf8)
        self.send_data(l >> 8)
        self.send_data(l & 0xff)

    def display_partial_frame(self, image, x, y, h, w, fast=False):
        """ Display a partial frame, only refreshing the changed area.
        `image` is a Pillow Image object
        `x` and `y` are the top left coordinates
        `h` is the height of the area to update
        `w` is the width of the area to update.
        if `fast` is True, fast refresh lookup tables will be used.
        see `smart_update()` method documentation for details."""
        if fast:
            self.set_lut(fast=True)
            self.delay_ms(2)

        # According to the spec, x and w have to be multiples of 8.
        # round them up and down accordingly to make sure they fit the requirement
        # adding a few more pixels to the refreshed area.
        # This is mandatory, otherwise the display might get corrupted until
        # the next valid update that touches the same area.
        x = _nearest_mult_of_8(x, False)
        w = _nearest_mult_of_8(w)

        self.send_command(PARTIAL_DATA_START_TRANSMISSION_1)
        self.delay_ms(2)

        self._send_partial_frame_dimensions(x, y, h, w)
        self.delay_ms(2)

        # Send the old values, as per spec
        old_image = self._last_frame.crop((x, y, x+w, y+h))
        old_fb = self._get_frame_buffer_for_size(old_image, h, w)
        for i in range(0, w * h // 8):
            self.send_data(old_fb[i])
        self.delay_ms(2)

        self.send_command(PARTIAL_DATA_START_TRANSMISSION_2)
        self.delay_ms(2)

        self._send_partial_frame_dimensions(x, y, h, w)

        # Send new data
        self._last_frame = image.copy()
        image = image.crop((x, y, x+w, y+h))
        new_fb = self._get_frame_buffer_for_size(image, h, w)
        for i in range(0, w * h // 8):
            self.send_data(new_fb[i])
        self.delay_ms(2)

        self.send_command(PARTIAL_DISPLAY_REFRESH)
        self.delay_ms(2)
        self._send_partial_frame_dimensions(x, y, h, w)
        self.wait_until_idle()
        if fast:
            self.set_lut()  # restore LUT to normal mode
        self._partial_refresh_count += 1

    def smart_update(self, image):
        """ Display a frame, automatically deciding which refresh method to use.
        If `fast_frefresh` is enabled, it would use optimized LUTs that shorten
        the refresh cycle, and don't do the full "inverse,black,white,black again,
        then draw" flush cycle.
        The fast refresh mode is much faster, but causes the image to apper
        gray instead of black, and can cause burn-in if it's overused.
        It's recommended to do a full flush "soon" after using the fast mode,
        to avoid degrading the panel. You can tweak `partial_refresh_limit`
        or
        """
        if self._last_frame is None or self._partial_refresh_count == self.partial_refresh_limit:
            # Doing a full refresh when:
            # - No frame has been displayed in this run, do a full refresh
            # - The display has been partially refreshed more than LIMIT times
            # the last full refresh (to prevent burn-in)
            self.display_frame(image)
        else:
            # Partial update. Let's start by figuring out the bounding box
            # of the changed area
            difference = ImageChops.difference(self._last_frame, image)
            bbox = difference.getbbox()
            if bbox is not None:
                # the old picture and new picture are different, partial
                # update is needed.
                # Get the update area. x and w have to be multiples of 8
                # as per the spec, so round down for x, and round up for w
                x = _nearest_mult_of_8(bbox[0], False)
                y = bbox[1]
                w = _nearest_mult_of_8(bbox[2] - x)
                if w > self.width:
                    w = self.width
                h = bbox[3] - y
                if h > self.height:
                    h = self.height
                # now let's figure out if fast mode is an option.
                # If the area was all white before - fast mode will be used.
                # otherwise, a slow refresh will be used (to avoid ghosting).
                # Since the image only has one color, meaning each pixel is either
                # 0 or 255, the following convinent one liner can be used
                fast = 0 not in self._last_frame.crop(bbox).getdata() and self.fast_refresh
                self.display_partial_frame(image, x, y, h, w, fast)

    def sleep(self):
        """Put the chip into a deep-sleep mode to save power.
        The deep sleep mode would return to standby by hardware reset.
        Use EPD.reset() to awaken and use EPD.init() to initialize. """
        self.send_command(DEEP_SLEEP)
        self.delay_ms(2)
        self.send_data(0xa5)  # deep sleep requires 0xa5 as a "check code" parameter
    class LUT(object):
    lut_vcom_dc = [
        0x00, 0x00,
        0x00, 0x0F, 0x0F, 0x00, 0x00, 0x05,
        0x00, 0x32, 0x32, 0x00, 0x00, 0x02,
        0x00, 0x0F, 0x0F, 0x00, 0x00, 0x05,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    ]

    # R21H
    lut_ww = [
        0x50, 0x0F, 0x0F, 0x00, 0x00, 0x05,
        0x60, 0x32, 0x32, 0x00, 0x00, 0x02,
        0xA0, 0x0F, 0x0F, 0x00, 0x00, 0x05,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    ]

    # R22H    r
    lut_bw = [
        0x50, 0x0F, 0x0F, 0x00, 0x00, 0x05,
        0x60, 0x32, 0x32, 0x00, 0x00, 0x02,
        0xA0, 0x0F, 0x0F, 0x00, 0x00, 0x05,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    ]

    # R24H    b
    lut_bb = [
        0xA0, 0x0F, 0x0F, 0x00, 0x00, 0x05,
        0x60, 0x32, 0x32, 0x00, 0x00, 0x02,
        0x50, 0x0F, 0x0F, 0x00, 0x00, 0x05,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    ]

    # R23H    w
    lut_wb = [
        0xA0, 0x0F, 0x0F, 0x00, 0x00, 0x05,
        0x60, 0x32, 0x32, 0x00, 0x00, 0x02,
        0x50, 0x0F, 0x0F, 0x00, 0x00, 0x05,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    ]
    class QuickLUT(object):
    # Quick LUTs courtsey of Ben Krasnow:
    # http://benkrasnow.blogspot.co.il/2017/10/fast-partial-refresh-on-42-e-paper.html

    lut_vcom_dc = [
        0x00, 0x0E, 0x00, 0x00, 0x00, 0x01,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    ]

    lut_ww = [
        0xA0, 0x0E, 0x00, 0x00, 0x00, 0x01,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00]

    lut_bw = [
        0xA0, 0x0E, 0x00, 0x00, 0x00, 0x01,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00]

    lut_bb = [
        0x50, 0x0E, 0x00, 0x00, 0x00, 0x01,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00]

    lut_wb = [
        0x50, 0x0E, 0x00, 0x00, 0x00, 0x01,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        
class EPD1in54(WavesharePartial):
    """Waveshare 1.54" - monochrome"""

    def __init__(self):
        super().__init__(name='1.54" BW', width=200, height=200)


class EPD2in13(WavesharePartial):
    """Waveshare 2.13" - monochrome"""

    lut_full_update = [
        0x22, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x11,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E,
        0x01, 0x00, 0x00, 0x00, 0x00, 0x00
    ]

    lut_partial_update = [
        0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x0F, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    ]

    def __init__(self):
        # the actual pixel width is 122, but 128 is the 'logical' width
        super().__init__(name='2.13" BW', width=128, height=250)

    def set_frame_memory(self, image, x, y):
        if image is None or x < 0 or y < 0:
            return
        image_monocolor = image.convert('1')
        image_width, image_height = image_monocolor.size
        # x point must be the multiple of 8 or the last 3 bits will be ignored
        x = x & 0xF8
        image_width = image_width & 0xF8
        if x + image_width >= self.width:
            x_end = self.width - 1
        else:
            x_end = x + image_width - 1
        if y + image_height >= self.height:
            y_end = self.height - 1
        else:
            y_end = y + image_height - 1
        self.set_memory_area(x, y, x_end, y_end)
        # send the image data
        pixels = image_monocolor.load()
        byte_to_send = 0x00
        for j in range(y, y_end + 1):
            self.set_memory_pointer(x, j)
            self.send_command(self.WRITE_RAM)
            # 1 byte = 8 pixels, steps of i = 8
            for i in range(x, x_end + 1):
                # Set the bits for the column of pixels at the current position.
                if pixels[i - x, j - y] != 0:
                    byte_to_send |= 0x80 >> (i % 8)
                if i % 8 == 7:
                    self.send_data(byte_to_send)
                    byte_to_send = 0x00

    
                    
                    
class EPD2in13v2(WavesharePartial):
    """Waveshare 2.13" V2 - monochrome"""

    lut_full_update = [
        0x80,0x60,0x40,0x00,0x00,0x00,0x00,             #LUT0: BB:     VS 0 ~7
        0x10,0x60,0x20,0x00,0x00,0x00,0x00,             #LUT1: BW:     VS 0 ~7
        0x80,0x60,0x40,0x00,0x00,0x00,0x00,             #LUT2: WB:     VS 0 ~7
        0x10,0x60,0x20,0x00,0x00,0x00,0x00,             #LUT3: WW:     VS 0 ~7
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,             #LUT4: VCOM:   VS 0 ~7

        0x03,0x03,0x00,0x00,0x02,                       # TP0 A~D RP0
        0x09,0x09,0x00,0x00,0x02,                       # TP1 A~D RP1
        0x03,0x03,0x00,0x00,0x02,                       # TP2 A~D RP2
        0x00,0x00,0x00,0x00,0x00,                       # TP3 A~D RP3
        0x00,0x00,0x00,0x00,0x00,                       # TP4 A~D RP4
        0x00,0x00,0x00,0x00,0x00,                       # TP5 A~D RP5
        0x00,0x00,0x00,0x00,0x00,                       # TP6 A~D RP6

        0x15,0x41,0xA8,0x32,0x30,0x0A
    ]

    lut_partial_update = [
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,             #LUT0: BB:     VS 0 ~7
        0x80,0x00,0x00,0x00,0x00,0x00,0x00,             #LUT1: BW:     VS 0 ~7
        0x40,0x00,0x00,0x00,0x00,0x00,0x00,             #LUT2: WB:     VS 0 ~7
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,             #LUT3: WW:     VS 0 ~7
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,             #LUT4: VCOM:   VS 0 ~7

        0x0A,0x00,0x00,0x00,0x00,                       # TP0 A~D RP0
        0x00,0x00,0x00,0x00,0x00,                       # TP1 A~D RP1
        0x00,0x00,0x00,0x00,0x00,                       # TP2 A~D RP2
        0x00,0x00,0x00,0x00,0x00,                       # TP3 A~D RP3
        0x00,0x00,0x00,0x00,0x00,                       # TP4 A~D RP4
        0x00,0x00,0x00,0x00,0x00,                       # TP5 A~D RP5
        0x00,0x00,0x00,0x00,0x00,                       # TP6 A~D RP6

        0x15,0x41,0xA8,0x32,0x30,0x0A,
    ]

    def __init__(self):
        # the actual pixel width is 122, but 128 is the 'logical' width
        super().__init__(name='2.13" BW V2 (full refresh only)', width=128, height=250)


class EPD2in9(WavesharePartial):
    """Waveshare 2.9" - monochrome"""

    def __init__(self):
        super().__init__(name='2.9" BW', width=128, height=296)


class EPD2in13d(WavesharePartial):
    """Waveshare 2.13" D - monochrome (flexible)"""

    # Note: the original code for this display was pretty broken and seemed
    # to have been written by some other person than the rest of the drivers.

    def __init__(self):
        super().__init__(name='2.13" D', width=104, height=212)

    lut_vcomDC = [
        0x00, 0x08, 0x00, 0x00, 0x00, 0x02,
        0x60, 0x28, 0x28, 0x00, 0x00, 0x01,
        0x00, 0x14, 0x00, 0x00, 0x00, 0x01,
        0x00, 0x12, 0x12, 0x00, 0x00, 0x01,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00,
    ]

    lut_ww = [
        0x40, 0x08, 0x00, 0x00, 0x00, 0x02,
        0x90, 0x28, 0x28, 0x00, 0x00, 0x01,
        0x40, 0x14, 0x00, 0x00, 0x00, 0x01,
        0xA0, 0x12, 0x12, 0x00, 0x00, 0x01,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    ]

    lut_bw = [
        0x40, 0x17, 0x00, 0x00, 0x00, 0x02,
        0x90, 0x0F, 0x0F, 0x00, 0x00, 0x03,
        0x40, 0x0A, 0x01, 0x00, 0x00, 0x01,
        0xA0, 0x0E, 0x0E, 0x00, 0x00, 0x02,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    ]

    lut_wb = [
        0x80, 0x08, 0x00, 0x00, 0x00, 0x02,
        0x90, 0x28, 0x28, 0x00, 0x00, 0x01,
        0x80, 0x14, 0x00, 0x00, 0x00, 0x01,
        0x50, 0x12, 0x12, 0x00, 0x00, 0x01,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    ]

    lut_bb = [
        0x80, 0x08, 0x00, 0x00, 0x00, 0x02,
        0x90, 0x28, 0x28, 0x00, 0x00, 0x01,
        0x80, 0x14, 0x00, 0x00, 0x00, 0x01,
        0x50, 0x12, 0x12, 0x00, 0x00, 0x01,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    ]

    lut_vcom1 = [
        0x00, 0x19, 0x01, 0x00, 0x00, 0x01,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00,
    ]

    lut_ww1 = [
        0x00, 0x19, 0x01, 0x00, 0x00, 0x01,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    ]

    lut_bw1 = [
        0x80, 0x19, 0x01, 0x00, 0x00, 0x01,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    ]

    lut_wb1 = [
        0x40, 0x19, 0x01, 0x00, 0x00, 0x01,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    ]

    lut_bb1 = [
        0x00, 0x19, 0x01, 0x00, 0x00, 0x01,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    ]

    def wait_until_idle(self):
        """This particular model's code sends the GET_STATUS command while waiting - dunno why."""
        while self.digital_read(self.BUSY_PIN) == 0:  # 0: idle, 1: busy
            self.send_command(self.GET_STATUS)
            self.delay_ms(100)

    def init(self, **kwargs):
        if self.epd_init() != 0:
            return -1
        self.reset()

        self.send_command(0x01)  # POWER SETTING
        self.send_data(0x03)
        self.send_data(0x00)
        self.send_data(0x2b)
        self.send_data(0x2b)
        self.send_data(0x03)

        self.send_command(0x06)  # boost soft start
        self.send_data(0x17)  # A
        self.send_data(0x17)  # B
        self.send_data(0x17)  # C

        self.send_command(0x04)
        self.wait_until_idle()

        self.send_command(0x00)  # panel setting
        self.send_data(0xbf)  # LUT from OTP,128x296
        self.send_data(0x0d)  # VCOM to 0V fast

        self.send_command(0x30)  # PLL setting
        self.send_data(0x3a)  # 3a 100HZ   29 150Hz 39 200HZ	31 171HZ

        self.send_command(0x61)  # resolution setting
        self.send_data(self.width)
        self.send_data((self.height >> 8) & 0xff)
        self.send_data(self.height & 0xff)

        self.send_command(0x82)  # vcom_DC setting
        self.send_data(0x28)

        # self.send_command(0X50)			#VCOM AND DATA INTERVAL SETTING
        # self.send_data(0xb7)		#WBmode:VBDF 17|D7 VBDW 97 VBDB 57		WBRmode:VBDF F7 VBDW 77 VBDB 37  VBDR B7
        return 0

    def set_full_reg(self):
        self.send_command(0x82)
        self.send_data(0x00)
        self.send_command(0X50)
        self.send_data(0xb7)

        self.send_command(0x20)  # vcom
        for count in range(0, 44):
            self.send_data(self.lut_vcomDC[count])
        self.send_command(0x21)  # ww --
        for count in range(0, 42):
            self.send_data(self.lut_ww[count])
        self.send_command(0x22)  # bw r
        for count in range(0, 42):
            self.send_data(self.lut_bw[count])
        self.send_command(0x23)  # wb w
        for count in range(0, 42):
            self.send_data(self.lut_wb[count])
        self.send_command(0x24)  # bb b
        for count in range(0, 42):
            self.send_data(self.lut_bb[count])

    def set_part_reg(self):
        self.send_command(0x82)
        self.send_data(0x03)
        self.send_command(0X50)
        self.send_data(0x47)

        self.send_command(0x20)  # vcom
        for count in range(0, 44):
            self.send_data(self.lut_vcom1[count])
        self.send_command(0x21)  # ww --
        for count in range(0, 42):
            self.send_data(self.lut_ww1[count])
        self.send_command(0x22)  # bw r
        for count in range(0, 42):
            self.send_data(self.lut_bw1[count])
        self.send_command(0x23)  # wb w
        for count in range(0, 42):
            self.send_data(self.lut_wb1[count])
        self.send_command(0x24)  # bb b
        for count in range(0, 42):
            self.send_data(self.lut_bb1[count])

    def turn_on_display(self):
        self.send_command(0x12)
        self.delay_ms(10)
        self.wait_until_idle()

    def clear(self):
        self.send_command(0x10)
        for i in range(0, int(self.width * self.height / 8)):
            self.send_data(0x00)
        self.delay_ms(10)

        self.send_command(0x13)
        for i in range(0, int(self.width * self.height / 8)):
            self.send_data(0xFF)
        self.delay_ms(10)

        self.set_full_reg()
        self.turn_on_display()

    def display_full(self, frame_buffer):
        if not frame_buffer:
            return

        self.send_command(0x10)
        for i in range(0, int(self.width * self.height / 8)):
            self.send_data(0x00)
        self.delay_ms(10)

        self.send_command(0x13)
        for i in range(0, int(self.width * self.height / 8)):
            self.send_data(frame_buffer[i])
        self.delay_ms(10)

        self.set_full_reg()
        self.turn_on_display()

    def display_partial(self, frame_buffer, x_start, y_start, x_end, y_end):
        if not frame_buffer:
            return

        self.set_part_reg()
        self.send_command(0x91)
        self.send_command(0x90)
        self.send_data(x_start)
        self.send_data(x_end - 1)

        self.send_data(y_start / 256)
        self.send_data(y_start % 256)
        self.send_data(y_end / 256)
        self.send_data(y_end % 256 - 1)
        self.send_data(0x28)

        self.send_command(0x10)
        for i in range(0, int(self.width * self.height / 8)):
            # print(frame_buffer[i],'%d','0x10')
            self.send_data(frame_buffer[i])
        self.delay_ms(10)

        self.send_command(0x13)
        for i in range(0, int(self.width * self.height / 8)):
            # print(~frame_buffer[i],'%d','0x13')
            self.send_data(~frame_buffer[i])
        self.delay_ms(10)

        # self.set_full_reg()
        self.turn_on_display()

    # after this, call epd.init() to awaken the module
    def sleep(self):
        self.send_command(0x50)
        self.send_data(0xf7)
        self.send_command(0x02)  # power off
        self.send_command(0x07)  # deep sleep
        self.send_data(0xA5)

    def draw(self, x, y, image):
        """Replace a particular area on the display with an image"""
        if self.partial_refresh:
            self.display_partial(self.get_frame_buffer(image), x, y, x + image.width, x + image.height)
        else:
            self.display_full(self.get_frame_buffer(image))
