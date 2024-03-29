# 定义全局变量
step_distance = 0.00001171875  # 步进电机一步滑台移动的距离，例如每步0.1米
distance_threshold = 1.0  # 距离阈值，例如1米
STEP_Y = 1  # 丝杠上下移动的步数
mode = 1  # 运行模式
speed = 1000  # 运行速度
route = 600 # 行程 600mm
lead = 75 # 电机转动一圈滑台移动的距离 75mm
step_angle = 1.8 # 步距角 1.8度
segmentation = 32 # 细分 32细分
actual_step_angle = 1.8/32  

# 设置步进电机一步滑台移动的距离
def set_step_distance(value):
    global step_distance
    step_distance = value

# 获取步进电机一步滑台移动的距离
def get_step_distance():
    return step_distance

# 设置距离阈值
def set_distance_threshold(value):
    global distance_threshold
    distance_threshold = value

# 获取距离阈值
def get_distance_threshold():
    return distance_threshold

# 设置丝杠上下移动的步数
def set_steps(value):
    global STEP_Y
    STEP_Y = value

# 获取丝杠上下移动的步数
def get_steps():
    return STEP_Y

# 设置运行模式
def set_mode(value):
    global mode
    mode = value

# 获取运行模式
def get_mode():
    return mode

# 设置运行速度
def set_speed(value):
    global speed
    speed = value

# 获取运行速度
def get_speed():
    return speed