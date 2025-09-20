from pyb import Servo
import time, sensor

SERVO_MAX_DUTY = 2.5
SERVO_MIN_DUTY = 0.5

servo_x = Servo(1)
servo_y = Servo(2)


sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.VGA)    #320x240
sensor.set_auto_gain(False)         #关闭自动增益
sensor.set_auto_whitebal(False)     #关闭白平衡
sensor.set_vflip(True)
sensor.set_hmirror(True)
sensor.skip_frames(time=2000)
sensor.set_auto_exposure(False, exposure_us=12000)

#################################################
#转换电机角度至脉冲宽度
#输入值：angle
#返回值：int_pulse_width
#################################################
def angle_to_pulse(angle):
    return int(((angle/180)*(SERVO_MAX_DUTY-SERVO_MIN_DUTY)+SERVO_MIN_DUTY)*1000)

#################################################
#寻找画面中最大的色块，并保证色块是max_size以内的大小
#输入值：blobs列表，max_size阈值（用于筛选掉不合适的色块）
#返回值：max_blob, 未找到返回None
#################################################
def find_max_blob(blobs, max_size):
    if blobs:
        max_blob = None
        max_size = max_size
        for blob in blobs:
            if blob[2]*blob[3]>max_size:
                max_size = blob[2]*blob[3]
                max_blob = blob
            #if max_blob is not None:
                #print("找到黄色色块！")
        return max_blob
    else:
        #print("未找到黄色色块，返回")
        return None

INIT_POS_X = 90
INIT_POS_Y = 90

green_threshold = (45, 89, -69, -19, 13, 58)

image_width = 320
image_height = 240
4
target_x_pos = INIT_POS_X
target_y_pos = INIT_POS_Y

last_err_x_pos = 0
last_err_y_pos = 0

FILTER_FACTOR = 0.15

servo_x.pulse_width(angle_to_pulse(INIT_POS_X))
time.sleep(0.5)
servo_y.pulse_width(angle_to_pulse(INIT_POS_Y))

while True:
    img = sensor.snapshot()
    blobs = img.find_blobs([green_threshold], area_threshold= 1000)
    max_blob = find_max_blob(blobs, 500)

    print("target_x_y:",target_x_pos,target_y_pos)
    if max_blob:
        cx = max_blob.x()
        cy = max_blob.y()
        cw = max_blob.w()
        ch = max_blob.h()
        img.draw_rectangle(cx,cy,cw,ch,[255,0,0])

        # 横向偏差计算（左为正，右为负）
        err_x_pos = image_width - (cx + cw/2)
        err_x_pos = FILTER_FACTOR*err_x_pos + (1-FILTER_FACTOR)*last_err_x_pos

        # X轴PID输出（左偏时增大角度，右偏时减小角度）
        delta_x_pos = 0.2*(err_x_pos - last_err_x_pos) + 0.018*err_x_pos
        last_err_x_pos = err_x_pos
        target_x_pos += delta_x_pos
        target_x_pos = max(0, min(180, target_x_pos))  # 限幅

        # 纵向偏差计算（上为正，下为负）
        err_y_pos = (image_height - (cy + ch/2))  # 直接计算上侧为正偏差
        err_y_pos = FILTER_FACTOR*err_y_pos + (1-FILTER_FACTOR)*last_err_y_pos

        # Y轴PID输出（上偏时减小角度，下偏时增大角度）
        delta_y_pos = 0.2*(err_y_pos - last_err_y_pos) + 0.018*err_y_pos
        last_err_y_pos = err_y_pos
        target_y_pos -= delta_y_pos  # 关键：Y轴用减法，使上偏时角度减小（上转）
        target_y_pos = max(0, min(180, target_y_pos))  # 限幅




        # #舵机控制
        servo_x.pulse_width(angle_to_pulse(target_x_pos))
        servo_y.pulse_width(angle_to_pulse(target_y_pos))
    else:
        continue


