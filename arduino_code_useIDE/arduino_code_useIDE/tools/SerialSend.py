import serial
import time

# 强制指定 DTR/RTS 初始电平
ser = serial.Serial(
    port="COM5",
    baudrate=115200,
    timeout=1,
    dsrdtr=False,
    rtscts=False,
)

time.sleep(3)  # 延长等待时间（Due 冷启动需要约 1.5 秒）
ser.write(b"Hello, Arduino!\n")
