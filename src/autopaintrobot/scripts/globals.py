# 定义全局变量
step_distance = 0.1  # 步进电机一步滑台移动的距离，例如每步0.1米
distance_threshold = 1.0  # 距离阈值，例如1米
steps = 1  # 丝杠上下移动的步数

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
    global steps
    steps = value

# 获取丝杠上下移动的步数
def get_steps():
    return steps
