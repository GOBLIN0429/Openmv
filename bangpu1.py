# 校赛 - By: 何颂禧 - Sat Apr 19 2025

import sensor, time, pyb
from pyb import UART, Servo

EXPOSURE_TIME_SCALE = 1.0
#黄色色块阈值
YELLOW_WINDOW_LEFT = 128
YELLOW_WINDOW_RIGHT = 128+384
YELLOW_WINDOW_TOP = 96
YELLOW_WINDOW_BOTTOM = 96+288
#红色杆阈值
RED_WINDOW_LEFT = 256
RED_WINDOW_RIGHT = 256+128

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.VGA)    #320x240
current_exposure_time_in_microseconds = sensor.get_exposure_us()
print("Initial Exposure Time:%d us" % sensor.get_exposure_us())  # 打印初始曝光值，用于后续比较曝光值的变化
sensor.set_auto_gain(False)         #关闭自动增益
sensor.set_auto_whitebal(False)     #关闭白平衡
sensor.set_vflip(True)
sensor.set_hmirror(True)
sensor.skip_frames(time=2000)
#sensor.set_auto_exposure(False,
     #exposure_us=int(current_exposure_time_in_microseconds * EXPOSURE_TIME_SCALE))
#sensor.set_auto_exposure(False, exposure_us=1000)
sensor.set_auto_exposure(False, exposure_us=3000)
print("Initial Exposure Time:%d us" % sensor.get_exposure_us())

#初始化标志位
yellow_is_find_Flag = 0  #是否寻找到黄色色块,0为否,1为是
red_is_find_Flag = 0    #是否寻找到红色色块,0为否,1为是
First_in_mid_Flag = 0 #红杆第一次进入到红杆阈值内时置1
QR_code_is_find_Flag = 0 #是否寻找到杆上二维码,0为否,1为是
WORK_STATE = 2           #工作状态标志位,0为不工作,1为找黄色，2为找杆二维码
is_receive_Flag = 0      #飞控是否接收到数据,0为否,1为是
openmv_receive_Flag = 0  #openmv是否收到一组正确的数据包，接收后置为1
servo_state = None         #0时舵机为0度，1时为90度
clock = time.clock()    #用于计算FPS
mission_step = 0
counter_IS_IN = 0 #保证没有误识别

#滤波初始化
last_cx = 0
last_cy = 0
alpha = 0.6  # 滤波系数，调节平滑程度

#初始化三个板载灯
red_led = pyb.LED(1)
green_led = pyb.LED(2)
blue_led = pyb.LED(3)
#red_led.on();#OpenMV初始化成功长亮红灯

#初始化串口
uart = UART(3, 115200)  #初始化UART3,RxP5,TxP4

#初始化舵机
servo_vertical = Servo(1)   #p7888
#舵机初始位置判断
if servo_vertical.angle() >= -90 and servo_vertical.angle() <= -70:
    servo_state = 1
else:
    servo_state = 0
print(f"舵机初始位置状态为{servo_state}")

#黄色阈值
#yellow_threshold = (37, 100, -19, 58, 17, 127)#傍晚（6点左右）
#yellow_threshold = (50, 86, -128, 5, 0, 127)#早10
#yellow_threshold = (15, 68, -20, 69, 30, 70)
yellow_threshold = (56, 100, -34, 45, 17, 127)
#yellow_threshold = (14, 27, -16, 2, 21, 34)#晚上
#红色阈值
red_rhreshold = (39, 96, 60, 127, -1, 75)

################################
#检查openmv_receive_Flag标志位，并重置
#输入值：None
#返回值：bool
#################################################
def is_openmv_receive():
    global openmv_receive_Flag
    if openmv_receive_Flag:
        openmv_receive_Flag = 0
        return True
    else:
        return None


#################################################
#控制舵机运动
#输入值：舵机角度
#返回值：None
#################################################
def servo_angle(servo, angle):
    servo.angle(-angle)
    print(f"当前舵机角度为{servo.angle()}")
    time.sleep(1)

#################################################
#寻找画面中最大的色块，并保证色块是10cm以内的大小
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

#################################################
#拍三次照片保存至sd卡
#返回值：void
#################################################
def capture_image_3_times():
    for i in range(3):
        clock.tick()
        img = sensor.snapshot()
        filename = "imgae_%d" %i
        try:
            img.save(filename)
            print("已保存第%d张图片"%i)
        except Exception as e:
            print("Error saving %s: %s" % (filename, e))
        time.sleep(1)

#################################################
#串口发送函数
#返回值：void
#包格式：帧头0xFF,目标地址0xAA,功能码0x01,数据长度data_length,和校验位checksum
#################################################
HEADER_SEND = b'\xFF\xAA\x01'
# data_to_send = [yellow_is_find_Flag, QR_code_if_find_Flag, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
# tx_buff = bytes(data_to_send)

# def UART3_sendMyPack(data):
#     data_length = len(data)     #计算数据长度
#     checksum = sum(data)&0xFF   #计算和校验位
#     packet = HEADER_SEND + bytes([data_length]) + data + bytes([checksum])
#     uart.write(packet)
#     print("发送数据", packet)

def UART3_sendMyPack(yellow_is_find_Flag, QR_code_if_find_Flag, LEFT_FLAG, RIGHT_FLAG, UP_FLAG, DOWM_FLAG, red_pole_is_find_Flag,openMV_is_receive):
    data_to_send = bytes([yellow_is_find_Flag, QR_code_if_find_Flag, LEFT_FLAG, RIGHT_FLAG, UP_FLAG, DOWM_FLAG, red_pole_is_find_Flag, openMV_is_receive])
    data_length = len(data_to_send)     #计算数据长度
    packet = HEADER_SEND + bytes([data_length]) + data_to_send
    checksum = sum(packet)&0xFF   #计算和校验位
    packet += bytes([checksum])
    uart.write(packet)
    #print("发送数据", packet)
################################################
#串口接收函数
#返回值：void
#包格式：帧头0xFF,目标地址0xCC,功能码0x01,数据长度data_length,和校验位checksum
#################################################
state = 0
rx_buff = []    #接收有效数据包,共八位,rx_buff[0]为WROK_STATE,rx_buff[1]为is_receive_Flag
data = []   #数据包，包含包头包内包尾
checksum = 0

def UART3_receiveData():
    global state
    global data
    global rx_buff
    global checksum
    global openmv_receive_Flag
    if state == 0:#判断帧头
        byte = uart.readchar()
        data.insert(0,byte)
        checksum = 0
        if byte == 0xFF:
            state = 1
            #print("帧头识别成功")
        else:
            state = 0
            data.clear()
            rx_buff.clear()
    elif state == 1:#判断目标地址
        byte = uart.readchar()
        if byte == 0xCC:
            data.insert(1,byte)
            state = 2
            #print("目标地址正确")
        else:
            state = 0
            data.clear()
            rx_buff.clear()
    elif state == 2:#判断功能码
        byte = uart.readchar()
        if byte == 0x01:
            data.insert(2,byte)
            state = 3
            #print("功能码正确")
        else:
            state = 0
            data.clear()
            rx_buff.clear()
    elif state == 3:#读取数据长度
        byte = uart.readchar()
        data.insert(3,byte)
        state = 4
        #print(f"数据长度读取完毕{byte}")
    elif state == 4:#接收数据到rx_buff
        data_length = data[3]
        #print(data_length)
        for i in range(data_length):
            byte = uart.readchar()
            data.insert(4+i,byte)
            rx_buff.append(byte)
        state = 5
        #print("数据接收完成")
    elif state == 5:#计算并判断和校验位
        byte = uart.readchar()
        data.insert(12,byte)
        for i in range(len(data)-1):
            checksum += data[i]
        checksum = checksum & 0xFF
        print("checksum=",checksum)
        if checksum == byte:
            print("成功接收数据", rx_buff)
            openmv_receive_Flag = 1
        else:
            state = 0
            data.clear()
            rx_buff.clear()

def analyze_rx_data(data):
    WORK_STATE = data[0]
    is_receive_yellow_Flag = data[1]
    #data.clear()
    return WORK_STATE, is_receive_yellow_Flag

def learn_color_threshold(ROI,img):
    statistics_Data = img.get_statistics(ROI)

    print(statistics_Data.l_mode()) #LAB众数，打印出来看看效果稳定不稳定
    print(statistics_Data.a_mode())
    print(statistics_Data.b_mode())

    color_L_Mode = statistics_Data.l_mode()     #分别赋值LAB的众数
    color_A_Mode = statistics_Data.a_mode()
    color_B_Mode = statistics_Data.b_mode()

    yellow_thresholds = (color_L_Mode-20, color_L_Mode+20, color_A_Mode-20, color_A_Mode+20, color_B_Mode-20, color_B_Mode+20)

    print("当前的阈值为:"[yellow_thresholds])

    return yellow_thresholds

servo_angle(servo_vertical, -10)

while True:
    clock.tick()
    img = sensor.snapshot()
    #img.lens_corr(strength=1.8, zoom=1.0)  # 矫正镜头畸变

    if WORK_STATE == -2:
        #测阈值使用
        blobs = img.find_blobs([yellow_threshold],margin = 20,merge = True, area_threshold= 1000)
        max_blob = find_max_blob(blobs, 1500)
        if max_blob:
            #画出位置
            img.draw_rectangle(max_blob.rect(),color = (255,0,0))
            center_x = max_blob.cx()
            center_y = max_blob.cy()
            img.draw_cross(center_x,center_y,(0,255,0))

    # if WORK_STATE == -1:
    #     ROI=(310,230,20,20)
    #     img.draw_rectangle(ROI,color=(255,255,255),thickness=2)
    #     statistics_Data = img.get_statistics(roi= (310,230,20,20))
    #     #threshold = learn_color_threshold([ROI],img)
    #     color_L_Mode = statistics_Data.l_mode()     #分别赋值LAB的众数
    #     color_A_Mode = statistics_Data.a_mode()
    #     color_B_Mode = statistics_Data.b_mode()

    #     yellow_thresholds = (color_L_Mode-20, color_L_Mode+20, color_A_Mode-20, color_A_Mode+20, color_B_Mode-20, color_B_Mode+20)

    #     print("当前的阈值为:",[yellow_thresholds])
    #     img.binary([yellow_thresholds])

    if WORK_STATE == 0:
        #先确认当前工作状态
        red_led.on()
        if uart.any()>0:
            UART3_receiveData()
            if is_openmv_receive():
                WORK_STATE, is_receive_Flag = analyze_rx_data(rx_buff)
                #确认接收
                UART3_sendMyPack(0,0,0,0,0,0,0,1)
                print(WORK_STATE, is_receive_Flag)


    #如果处于找黄色状态
    if WORK_STATE == 1:
        red_led.off()
        #img.draw_rectangle(128, 96, 384, 288, (0,0,255), 2)
        if mission_step == 0:
            if not servo_state:
                #舵机面向正前方
                servo_angle(servo_vertical, 90)
                servo_state = 1
            mission_step += 1
        if mission_step == 1:
            green_led.off()
            #img.median(1)           # 去噪
            # 查找色块
            blobs = img.find_blobs([yellow_threshold],area_threshold= 1200 ,margin = True,merge = True)
            max_blob = find_max_blob(blobs, 1500)
            #如果找到黄色色块
            if max_blob:
                green_led.on()
                #画出位置
                #img.draw_rectangle(max_blob.rect(),color = (255,0,0))
                center_x = max_blob.cx()
                center_y = max_blob.cy()
                # 应用一阶低通滤波
                cx_filtered = int(alpha * last_cx + (1 - alpha) * center_x)
                cy_filtered = int(alpha * last_cy + (1 - alpha) * center_y)

                # 保存当前作为下次的“上一次”
                last_cx = cx_filtered
                last_cy = cy_filtered

                #img.draw_cross(cx_filtered, cy_filtered, size=10, color=(0, 255, 0))
                if center_x < YELLOW_WINDOW_LEFT:
                    #在画面左端，需左移
                    UART3_sendMyPack(0,0,1,0,0,0,0,0)
                    print("偏左")
                elif center_x > YELLOW_WINDOW_RIGHT:
                    #在画面右端，需右移
                    UART3_sendMyPack(0,0,0,1,0,0,0,0)
                    print("偏右")
                elif center_y < YELLOW_WINDOW_TOP:
                    #在画面上端，需上移
                    UART3_sendMyPack(0,0,0,0,1,0,0,0)
                    print("偏上")
                elif center_y > YELLOW_WINDOW_BOTTOM:
                    #在画面下端，需下移
                    UART3_sendMyPack(0,0,0,0,0,1,0,0)
                    print("偏下")
                else:
                    #在画面中间，无需移动，发送悬停
                    UART3_sendMyPack(0,0,0,0,0,0,0,0)
                    if counter_IS_IN == 3:
                        #在画面中间，悬停！！！！！！！！！！！
                        yellow_is_find_Flag = 1
                        mission_step += 1
                    else:
                        # 停止0.5s使无人机稳定
                        time.sleep(0.5)
                        #拍照
                        green_led.off()
                        file_name = "img_yellow_%d"%counter_IS_IN
                        img.save(file_name)
                        counter_IS_IN += 1

        if mission_step ==2:
            blue_led.on()
            yellow_is_find_Flag = 1
            for _ in range(20):
                UART3_sendMyPack(yellow_is_find_Flag,0,0,0,0,0,0,0)
            #初始化
            mission_step = 0
            counter_IS_IN = 0
            yellow_is_find_Flag=0
            last_cx=0
            last_cy=0
            blue_led.off()
            #进入到工作状态2，绕杆
            WORK_STATE = 2
            # if uart.any()>0:
            #     UART3_receiveData()
            #     if is_openmv_receive():
            #         WORK_STATE, is_receive_Flag = analyze_rx_data(rx_buff)
            #         if WORK_STATE == 0:
            #             #初始化
            #             for _ in range(100):
            #                 UART3_sendMyPack(0,0,0,0,0,0,0,1)
            #             mission_step = 0
            #             counter_IS_IN = 0
            #             yellow_is_find_Flag=0
            #             blue_led.off()

        #     # green_led.off()
        #     # #告诉飞控停车并等待飞控返回确认
        #     # if is_receive_Flag == 0:
        #     #     UART3_sendMyPack(yellow_is_find_Flag,0,0,0,0,0,0,0)
        #     #     if uart.any()>0:
        #     #         UART3_receiveData()
        #     #         if is_openmv_receive():
        #     #             WORK_STATE, is_receive_Flag = analyze_rx_data(rx_buff)
        #     # if is_receive_Flag == 1:
        #     #     mission_step += 1
        # if mission_step == 3:
        #     blue_led.on()
        #     #拍照
        #     #capture_image_3_times()
        #     #重置标志位
        #     yellow_is_find_Flag = 0
        #     mission_step += 1
        # if mission_step == 4:
        #     blue_led.off()
        #     #告诉飞控拍照完成并等待飞控返回下一工作状态
        #     UART3_sendMyPack(yellow_is_find_Flag,0,0,0,0,0,0)
        #     if uart.any()>0:
        #         UART3_receiveData()
        #         if is_openmv_receive():
        #             WORK_STATE, is_receive_Flag = analyze_rx_data(rx_buff)
        #             if WORK_STATE == 0:
        #                 #工作状态发生更新，工作步骤清零
        #                 for _ in range(10):
        #                     UART3_sendMyPack(0,0,0,0,0,0,0,1)
        #                     time.sleep(0.1)
        #                 mission_step = 0
        #                 counter_IS_IN = 0


    #如果处于绕杆状态
    if WORK_STATE == 2:
        red_led.off()
        #画出标识线
        img.draw_line(256,0,256,480,(0,0,255),2)
        img.draw_line(384,0,384,480,(0,0,255),2)
        if mission_step == 0:
            # 查找红色色块
            blobs = img.find_blobs([red_rhreshold], area_threshold = 1000,merge = True, margin = 20)
            max_blob = find_max_blob(blobs,2000)
            if max_blob:
                print("第一次找到红色色块")
                # 发送给飞控，开始进入绕杆模式
                # 飞控接收到后，右移速度减慢，保证绕杆稳定性
                red_is_find_Flag = 1
                UART3_sendMyPack(0,0,0,0,0,0,red_is_find_Flag,0)
                mission_step += 1
        if mission_step == 1:
            #print("开始查找红色色块")
            # 查找红色色块
            blobs = img.find_blobs([red_rhreshold])
            max_blob = find_max_blob(blobs,2000)
            if max_blob:
                #red_is_find_Flag = 1
                # 在图中画出
                img.draw_rectangle(max_blob.rect())
                center_x = max_blob.cx()
                center_y = max_blob.cy()
                img.draw_cross(center_x, center_y, size=10, color=(0, 255, 0))
                # 检验红杆中心是否第一次进入阈值内
                if First_in_mid_Flag == 0:
                    if center_x>RED_WINDOW_LEFT and center_x<RED_WINDOW_RIGHT:
                        First_in_mid_Flag = 1
                        print("红杆第一次进入阈值框内")
                        UART3_sendMyPack(0,0,0,0,0,0,First_in_mid_Flag,0)
                        for i in range(3):
                            blue_led.on()
                            file_name = "img_red_%d"%i
                            img.save(file_name)
                        mission_step += 1
        if mission_step == 2:
            red_led.on()
            # 开始绕杆
            # 先接收串口信息
            if uart.any()>0:
                UART3_receiveData()
                if is_openmv_receive():
                    # 接收到数据返回给飞控
                    UART3_sendMyPack(0,0,0,0,0,0,0,1)
                    WORK_STATE, is_receive_Flag = analyze_rx_data(rx_buff)
            # 查找红色色块
            blobs = img.find_blobs([red_rhreshold], area_threshold = 1000,merge = True, margin = 20)
            max_blob = find_max_blob(blobs,2000)
            if max_blob:
                red_led.on()
                # 找杆中心点并发聩给飞控，直到飞控更新OpenMV工作状态
                if First_in_mid_Flag == 1:
                    img.draw_rectangle(max_blob.rect())
                    center_x = max_blob.cx()
                    center_y = max_blob.cy()
                    # 应用一阶低通滤波
                    cx_filtered = int(alpha * last_cx + (1 - alpha) * center_x)
                    cy_filtered = int(alpha * last_cy + (1 - alpha) * center_y)
                    # 保存当前作为下次的“上一次”
                    last_cx = cx_filtered
                    last_cy = cy_filtered

                    img.draw_cross(cx_filtered, cy_filtered, size=10, color=(0, 255, 0))
                    if WORK_STATE != 0:
                        if max_blob.area()>60000:
                            print("危险，需要后退！")
                        elif cx_filtered < RED_WINDOW_LEFT:
                            # 红杆在图像左端，无人机需向左旋
                            UART3_sendMyPack(0,0,1,0,0,0,red_is_find_Flag,0)
                            print("无人机需左旋")
                        elif cx_filtered > RED_WINDOW_RIGHT:
                            # 红杆在图像左端，无人机需向右旋
                            UART3_sendMyPack(0,0,0,1,0,0,red_is_find_Flag,0)
                            print("无人机右旋")
                        else:
                            # 在画面中间，只需继续向右移动
                            UART3_sendMyPack(0,0,0,0,0,0,red_is_find_Flag,0)
                    else:
                        # 结束该工作流程，重置标志位
                        red_led.off()

                        mission_step = 0
                        First_in_mid_Flag = 0
                        red_is_find_Flag = 0
                red_led.off()
    #print(clock.fps())
