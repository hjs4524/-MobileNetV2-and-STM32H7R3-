import sensor
import time
from pyb import UART, LED

# OpenMV single target pixel center -> MCU
# UART frame:
#   Target found : cx,cy\r\n
#   No target    : NONE\r\n

UART_ID = 3
UART_BAUD = 115200

ROI = (80, 60, 160, 120)
thresholds = [(18, 50, -35, 116, -124, 127)]

MIN_PIXELS = 120
MIN_AREA = 120
MERGE_BLOBS = False
SEND_NONE_INTERVAL = 10
SEND_PIXEL_THRESHOLD = 2

uart = UART(UART_ID, UART_BAUD, timeout_char=1000)
led = LED(2)

sensor.reset()
sensor.set_vflip(True)
sensor.set_hmirror(True)
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)

clock = time.clock()
none_count = 0
last_cx = None
last_cy = None


def blob_score(blob):
    return blob.pixels()


while True:
    clock.tick()
    img = sensor.snapshot()

    blobs = img.find_blobs(
        thresholds,
        roi=ROI,
        pixels_threshold=MIN_PIXELS,
        area_threshold=MIN_AREA,
        merge=MERGE_BLOBS
    )

    img.draw_rectangle(ROI, color=(0, 0, 255))

    if blobs:
        blob = max(blobs, key=blob_score)
        cx = blob.cx()
        cy = blob.cy()

        img.draw_rectangle(blob.rect(), color=(255, 0, 0))
        img.draw_cross(cx, cy, color=(0, 255, 0))
        img.draw_string(cx + 4, cy + 4, "(%d,%d)" % (cx, cy),
                        color=(255, 255, 0), scale=1)

        if (last_cx is None or
                abs(cx - last_cx) >= SEND_PIXEL_THRESHOLD or
                abs(cy - last_cy) >= SEND_PIXEL_THRESHOLD):
            uart.write("%d,%d\r\n" % (cx, cy))
            last_cx = cx
            last_cy = cy
        led.on()
        none_count = 0
    else:
        led.off()
        last_cx = None
        last_cy = None
        none_count += 1
        if none_count >= SEND_NONE_INTERVAL:
            uart.write("NONE\r\n")
            none_count = 0

    time.sleep_ms(100)
