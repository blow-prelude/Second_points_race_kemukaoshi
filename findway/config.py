# config.py

# 摄像头设置
CAM_INDEX=0
# CAM_INDEX='/dev/video0'
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# ROI区域
VER_ROI_TOP_RATIO=0.4
VER_ROI_BOTTOM_RATIO=0.6
VER_ROI_LEFT_RATIO=0.4
VER_ROI_RIGHT_RATIO=0.6

HOR_ROI_TOP_RATIO=0.7
HOR_ROI_BOTTOM_RATIO=0.9
HOR_ROI_LEFT_RATIO=0.2
HOR_ROI_RIGHT_RATIO=0.4


# 斜率判断
SLOPE_VERTICAL_THRESHOLD = 30
MIN_LINE_HEIGHT_RATIO = 0.1
# 理想的高度线
HEIGHT_LINE = 0.7

# 检测逻辑参数
LINE_DETECTION_COUNT = 4
DETECTION_TIME_INTERVAL = 5  
MAX_REVERSE = 2   #倒车任务总次数
REVERSING_TIMEOUT = 20   #倒车时间
LEAVE_GARAGE_TIMEOUT=20   #出库时间


# 显示
STACK_SCALE = 0.5

# 串口设置
# SERIAL_PORT = '/dev/ttyAMA0'
# BAUD_RATE = 115200


# PID参数
Kp = 0
Ki = 0
Kd = 0

