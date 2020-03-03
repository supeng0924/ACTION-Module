# coding: utf-8
import numpy as np
import os
import six.moves.urllib as urllib
import sys
import tarfile
import tensorflow as tf
import zipfile
import time
import multiprocessing as mp

from collections import defaultdict
from io import StringIO
import cv2
import time
from xml.etree import ElementTree as ET
import serial

sys.path.append("..")
from object_detection.utils import ops as utils_ops


from utils import label_map_util
from utils import visualization_utils as vis_util


# What model to download.
MODEL_NAME = 'jd_12_18_27180'








# Path to frozen detection graph. This is the actual model that is used for the object detection.
PATH_TO_CKPT = MODEL_NAME + '/frozen_inference_graph.pb'

# List of the strings that is used to add correct label for each box.
PATH_TO_LABELS = os.path.join('training', 'object-detection.pbtxt')



# -----------------------------控制变量区域-------------------------------------
# commodity=np.array([[1,9,3,11,5,13,7,15],[2,10,4,12,6,14,8,16,9,15],[1,10,3,12,5,14,7,16],[2,9,4,11,6,13,8,15],[1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16]])
commodity=np.array([[1,9,3,11,5,13,7,15,4,6],[2,10,4,12,6,14,8,16,9,15],[1,10,3,12,5,14,7,16,2,8],[2,9,4,11,6,13,8,15,17,18],[1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16]])
# things_width=np.array([15,7,9,24,8,5,7,11,17,16,15,13,15,7,13,8])
# 第i套题 需要 title_num-1
title_num = 3
NUM_CLASSES = 20
# 录像标志
use_video = 1


detection_graph = tf.Graph()
with detection_graph.as_default():
    od_graph_def = tf.GraphDef()
    with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
        serialized_graph = fid.read()
        od_graph_def.ParseFromString(serialized_graph)
        tf.import_graph_def(od_graph_def, name='')


label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
category_index = label_map_util.create_category_index(categories)


def load_image_into_numpy_array(image):
    (im_width, im_height) = image.size
    return np.array(image.getdata()).reshape(
        (im_height, im_width, 3)).astype(np.uint8)


path_D435 = "D435Camera/build/"
data_x_weight = np.append((np.ones([1,320],np.int8)*(-1)),np.ones([1,320],np.int8))


def run_inference_for_single_image(graph,numberCam_deal, dataPackUp, dataPackDown,car_stop_up,car_stop_down):
    with graph.as_default():
        with tf.Session() as sess:
            # Get handles to input and output tensors
            ops = tf.get_default_graph().get_operations()
            all_tensor_names = {output.name for op in ops for output in op.outputs}
            tensor_dict = {}
            for key in [
                'num_detections', 'detection_boxes', 'detection_scores',
                'detection_classes', 'detection_masks'
            ]:
                tensor_name = key + ':0'
                if tensor_name in all_tensor_names:
                    tensor_dict[key] = tf.get_default_graph().get_tensor_by_name(
                        tensor_name)
            if 'detection_masks' in tensor_dict:
                # The following processing is only for single image
                detection_boxes = tf.squeeze(tensor_dict['detection_boxes'], [0])
                detection_masks = tf.squeeze(tensor_dict['detection_masks'], [0])
                # Reframe is required to translate mask from box coordinates to image coordinates and fit the image size.
                real_num_detection = tf.cast(tensor_dict['num_detections'][0], tf.int32)
                detection_boxes = tf.slice(detection_boxes, [0, 0], [real_num_detection, -1])
                detection_masks = tf.slice(detection_masks, [0, 0, 0], [real_num_detection, -1, -1])
                detection_masks_reframed = utils_ops.reframe_box_masks_to_image_masks(
                    detection_masks, detection_boxes, image.shape[0], image.shape[1])
                detection_masks_reframed = tf.cast(
                    tf.greater(detection_masks_reframed, 0.5), tf.uint8)
                # Follow the convention by adding back the batch dimension
                tensor_dict['detection_masks'] = tf.expand_dims(
                    detection_masks_reframed, 0)
            image_tensor = tf.get_default_graph().get_tensor_by_name('image_tensor:0')

            camera_list_total = np.array(["UP", "DOWN"])
            img_index = 0

            # 最开始设置为检测模式 1:彩色图像检测 0:彩色检测+深度对齐
            cam_detect = 1
            # 上摄像头检测到标志
            cam_up_signal = 0
            # 下摄像头检测到标志
            cam_down_signal = 0
            # debug_mode为1时调试
            debug_mode = 0



            # 商品序号                  1    2   3     4       5    6     7    8     9   10    11  12    13   14    15   16   17   18
            # 商品序列                  薯片 牛奶 薯条  牙膏   可乐  钙片   橙子  挂钩  饼干  纸抽   书  奥利奥  果珍 洗发液  肠  牙刷  香水  杯子 ---宽度 高度
            things_thesh = np.array([0.7, 0.75, 0.7, 0.85, 0.75, 0.70, 0.7, 0.8, 0.7, 0.7, 0.75, 0.7, 0.8, 0.7, 0.7, 0.7, 0.55, 0.65])
            # things_thesh = np.array([0.6, 0.60, 0.6, 0.60, 0.60, 0.60, 0.6, 0.6, 0.6, 0.6, 0.60, 0.6, 0.6, 0.6, 0.6, 0.6, 0.60, 0.60])

            while True:
                st_time = time.time()
                # cam_detect为1时只进行彩色图像检测阶段
                # cam_detect为0时进行彩色和深度对齐
                if cam_detect:
                    # 摄像头检测计数
                    cam_rgbd = 0
                    camera_list = camera_list_total[:numberCam_deal.value]
                    # print(camera_list)
                    for path_head in camera_list:
                        files = os.listdir(path_D435 + path_head + "RGB")
                        max_index = 0

                        for file_t in files:
                            a = file_t.split('.')[0]
                            if int(a) > max_index:
                                max_index = int(a)
                        if max_index == img_index:
                            break

                        img_index = max_index
                        # 读取彩色图像
                        color_img = cv2.imread(path_D435 + "/" + path_head + "RGB/" + str(max_index - 1) + ".jpg")
                        image = cv2.cvtColor(color_img, cv2.COLOR_BGR2RGB)

                        # Run inference
                        output_dict = sess.run(tensor_dict,
                                               feed_dict={image_tensor: np.expand_dims(image, 0)})

                        # all outputs are float32 numpy arrays, so convert types as appropriate
                        output_dict['num_detections'] = int(output_dict['num_detections'][0])
                        output_dict['detection_classes'] = output_dict['detection_classes'][0].astype(np.uint8)
                        output_dict['detection_boxes'] = output_dict['detection_boxes'][0]
                        output_dict['detection_scores'] = output_dict['detection_scores'][0]
                        if 'detection_masks' in output_dict:
                            output_dict['detection_masks'] = output_dict['detection_masks'][0]

                        dict_part = {}
                        for key in [
                            'detection_boxes', 'detection_scores',
                            'detection_classes',
                        ]:
                            dict_part[key] = []

                        # 筛选出阈值符合要求的框
                        for num_dete in range(len(output_dict['detection_scores'])):
                            class_index=int(output_dict['detection_classes'][num_dete])
                            if class_index in commodity[title_num] \
                                    and output_dict['detection_scores'][num_dete]>things_thesh[class_index-1]:
                                dict_part['detection_scores'].append(output_dict['detection_scores'][num_dete])
                                dict_part['detection_classes'].append(output_dict['detection_classes'][num_dete])
                                dict_part['detection_boxes'].append(output_dict['detection_boxes'][num_dete])

                        # 阈值概率
                        pro_thresh = 0.65
                        # 对于重合率IOU高的框，只保留相似度高的，其余进行删除
                        if len(dict_part['detection_classes']) > 1:
                            for num_dete in range(len(dict_part['detection_classes']) - 1):
                                ymin, xmin, ymax, xmax = dict_part['detection_boxes'][num_dete]
                                for num_dete_2 in range(num_dete + 1, len(dict_part['detection_classes'])):
                                    ymin2, xmin2, ymax2, xmax2 = dict_part['detection_boxes'][num_dete_2]
                                    y_min = max(ymin2, ymin)
                                    x_min = max(xmin2, xmin)
                                    y_max = min(ymax2, ymax)
                                    x_max = min(xmax2, xmax)
                                    bili = (x_max - x_min) * (y_max - y_min) / (
                                                (max(xmax2, xmax) - min(xmin2, xmin)) * (max(ymax2, ymax) - min(ymin2, ymin)))
                                    if bili > 0.5:
                                        if dict_part['detection_scores'][num_dete] > \
                                                dict_part['detection_scores'][num_dete_2]:
                                            dict_part['detection_scores'][num_dete_2] = 0
                                        else:
                                            dict_part['detection_scores'][num_dete] = 0
                        # 筛选出距离符合要求的，同时绘制方框
                        for num_dete in range(len(dict_part['detection_scores'])):
                            class_index = int(dict_part['detection_classes'][num_dete])
                            if dict_part['detection_scores'][num_dete]>things_thesh[class_index-1]:
                            # 必须加阈值限制，因为上一步将重复的清零了
                            # if dict_part['detection_scores'][num_dete] > pro_thresh:
                                # 得到框的左上角和右下角坐标
                                ymin, xmin, ymax, xmax = dict_part['detection_boxes'][num_dete]
                                # 得到的是0-1范围的，需要转化为480,640
                                ymin_i = int(ymin * 480)
                                ymax_i = int(ymax * 480)
                                xmin_i = int(xmin * 640)
                                xmax_i = int(xmax * 640)
                                # if (xmax_i-xmin_i)>400:
                                #     continue
                                # 计算图像坐标系下x坐标的中间值，作为摄像头到该点的距离作为参考值
                                x_pos = int(xmin_i + xmax_i) // 2

                                middle_val = x_pos
                                color_paint = 0
                                if debug_mode:
                                    print(path_head, middle_val)
                                else:
                                    if path_head[0] == 'U':
                                        if middle_val > 340 and middle_val < 450:
                                            cam_up_signal = 1
                                            car_stop_up[0] = 1
                                            car_stop_up[1] = int(dict_part['detection_classes'][num_dete])
                                            cam_rgbd += 1
                                            color_paint = 255
                                            cam_detect = 0
                                            class_index = int(dict_part['detection_classes'][num_dete])
                                            print("find kind ", class_index)
                                    else:
                                        if middle_val > 328 and middle_val < 448:
                                            cam_down_signal = 1
                                            car_stop_down[0] = 1
                                            car_stop_down[1] = int(dict_part['detection_classes'][num_dete])
                                            cam_rgbd += 1
                                            color_paint = 255
                                            cam_detect = 0
                                            class_index = int(dict_part['detection_classes'][num_dete])
                                            print("find kind ", class_index)

                                    if cam_rgbd == 1:
                                        f = open("D435Camera/build/camdep.txt", 'w')
                                        f.write('6')
                                        f.close()


                                # cv2.line(color_img,(left_border, 240),(right_border, 240),(255, 255, 255),4)
                                # cv2.circle(color_img, (middle_val, 240), 4, (0, 0, 255))
                                cv2.rectangle(color_img, (xmin_i, ymin_i), (xmax_i, ymax_i), (color_paint, 255, 255), 1)
                                score_temp = int(dict_part['detection_scores'][num_dete] * 100)
                                st_get = category_index[dict_part['detection_classes'][num_dete]][
                                             'name'] + ' sco ' + str(score_temp)
                                cv2.putText(color_img, st_get, (xmin_i, ymax_i), cv2.FONT_HERSHEY_COMPLEX, 0.5,
                                            (255, 255, 0), 1)
                        # if path_head == "UP":
                        #     cv2.line(color_img,(208, 37),(573, 27),(255, 255, 255),4,cv2.LINE_AA )
                        #     cv2.line(color_img,(143, 159),(64, 330),(255, 255, 255),4,cv2.LINE_AA )
                        # else:
                        #     cv2.line(color_img,(168, 126),(594, 124),(255, 255, 255),4,cv2.LINE_AA )
                        #     cv2.line(color_img,(55, 249),(17, 312),(255, 255, 255),4,cv2.LINE_AA )
                        cv2.imshow(path_head, color_img)
                else:
                    st_time = time.time()
                    out_for = 0
                    while True:
                        img_file = open("D435Camera/build/camdep.txt", 'r')
                        a = img_file.read(1)
                        img_file.close()

                        if a is '5':
                            break
                        time_wait = time.time() - st_time
                        if time_wait > 0.4:
                            print("wait too many time no image")
                            out_for = 1
                            break
                    # print("wait ok")
                    if out_for:
                        continue


                    max_index = 0
                    files = os.listdir(path_D435 + "UP" + "depth")
                    for file_t in files:
                        a = file_t.split('.')[0]
                        if int(a) > max_index:
                            max_index = int(a)
                    # img_index = max_index

                    camera_list = camera_list_total[:numberCam_deal.value]
                    cam_detect = 1
                    for path_head in camera_list:
                        if path_head == "UP":
                            if cam_up_signal:
                                cam_up_signal = 0
                            else:
                                continue

                        if path_head == "DOWN":
                            if cam_down_signal:
                                cam_down_signal = 0
                            else:
                                continue



                        # 读取彩色图像
                        color_img = cv2.imread(path_D435 + "/" + path_head + "color/" + str(max_index) + ".jpg")
                        # 读取深度图像
                        depth_img = cv2.imread(path_D435 + "/" + path_head + "depth/" + str(max_index) + ".png",
                                               cv2.IMREAD_ANYDEPTH)
                        # 读取世界坐标系下的坐标
                        xdata_img = cv2.imread(path_D435 + "/" + path_head + "xdata/" + str(max_index) + ".png",
                                               cv2.IMREAD_ANYDEPTH)
                        xdata_img = xdata_img * data_x_weight
                        # xyz是一个列表list存放数据  行*640+列*3  0：x 1:y 2:z
                        image = cv2.cvtColor(color_img, cv2.COLOR_BGR2RGB)


                        # Run inference
                        output_dict = sess.run(tensor_dict,
                                               feed_dict={image_tensor: np.expand_dims(image, 0)})

                        # all outputs are float32 numpy arrays, so convert types as appropriate
                        output_dict['num_detections'] = int(output_dict['num_detections'][0])
                        output_dict['detection_classes'] = output_dict['detection_classes'][0].astype(np.uint8)
                        output_dict['detection_boxes'] = output_dict['detection_boxes'][0]
                        output_dict['detection_scores'] = output_dict['detection_scores'][0]
                        if 'detection_masks' in output_dict:
                            output_dict['detection_masks'] = output_dict['detection_masks'][0]

                        dict_part = {}
                        for key in [
                            'detection_boxes', 'detection_scores',
                            'detection_classes'
                        ]:
                            dict_part[key] = []

                        # 阈值概率
                        pro_thresh = 0.65

                        # 筛选出阈值符合要求的框
                        for num_dete in range(len(output_dict['detection_scores'])):
                            class_index=int(output_dict['detection_classes'][num_dete])
                            if class_index in commodity[title_num] \
                                    and output_dict['detection_scores'][num_dete]>things_thesh[class_index-1]:
                                dict_part['detection_scores'].append(output_dict['detection_scores'][num_dete])
                                dict_part['detection_classes'].append(output_dict['detection_classes'][num_dete])
                                dict_part['detection_boxes'].append(output_dict['detection_boxes'][num_dete])

                        # 对于重合率IOU高的框，只保留相似度高的，其余进行删除
                        if len(dict_part['detection_classes']) > 1:
                            for num_dete in range(len(dict_part['detection_classes']) - 1):
                                ymin, xmin, ymax, xmax = dict_part['detection_boxes'][num_dete]
                                for num_dete_2 in range(num_dete + 1, len(dict_part['detection_classes'])):
                                    ymin2, xmin2, ymax2, xmax2 = dict_part['detection_boxes'][num_dete_2]
                                    y_min = max(ymin2, ymin)
                                    x_min = max(xmin2, xmin)
                                    y_max = min(ymax2, ymax)
                                    x_max = min(xmax2, xmax)
                                    bili = (x_max - x_min) * (y_max - y_min) / (
                                                (max(xmax2, xmax) - min(xmin2, xmin)) * (
                                                max(ymax2, ymax) - min(ymin2, ymin)))
                                    if bili > 0.5:
                                        if dict_part['detection_scores'][num_dete] > \
                                                dict_part['detection_scores'][num_dete_2]:
                                            dict_part['detection_scores'][num_dete_2] = 0
                                        else:
                                            dict_part['detection_scores'][num_dete] = 0
                        # 筛选出距离符合要求的，同时绘制方框
                        for num_dete in range(len(dict_part['detection_scores'])):
                            class_index = int(dict_part['detection_classes'][num_dete])
                            if class_index in commodity[title_num] and dict_part['detection_scores'][num_dete] > things_thesh[class_index - 1]:
                            # 必须加阈值限制，因为上一步将重复的清零了
                            # if dict_part['detection_scores'][num_dete] > pro_thresh:
                                # 得到框的左上角和右下角坐标
                                ymin, xmin, ymax, xmax = dict_part['detection_boxes'][num_dete]
                                # 得到的是0-1范围的，需要转化为480,640
                                ymin_i = int(ymin * 480)
                                ymax_i = int(ymax * 480)
                                xmin_i = int(xmin * 640)
                                xmax_i = int(xmax * 640)
                                # 计算图像坐标系下x坐标的中间值，作为摄像头到该点的距离作为参考值
                                x_pos = int(xmin_i + xmax_i) // 2
                                # 计算图像坐标系下y坐标的中间值
                                y_pos_middle = int(ymin_i + ymax_i) // 2

                                if x_pos > 639:
                                    x_pos = 639
                                if ymax_i > 479:
                                    ymax_i = 479
                                if xmax_i > 639:
                                    xmax_i = 639

                                distance = depth_img[ymax_i, x_pos]
                                while not distance:
                                    ymax_i -= 1
                                    distance = depth_img[ymax_i, x_pos]
                                    if ymax_i == 0:
                                        break

                                distance2 = depth_img[y_pos_middle, x_pos]
                                while not distance2:
                                    y_pos_middle -= 1
                                    distance2 = depth_img[y_pos_middle, x_pos]
                                    if y_pos_middle == 0:
                                        break

                                if distance > 800 or distance2 > 1200:
                                    continue
                                # 对准的中间值采用商品最下边中间点(distance对应的那个点)
                                # 注意:这个点上面已经进行如果是0的处理,针对非0才算通过
                                middle_val = xdata_img[ymax_i, x_pos]

                                print("middle", middle_val)

                                color_paint = 0
                                # 38 3cm
                                # if middle_val > -20 and middle_val < 52:
                                if middle_val > -10 and middle_val < 80:
                                    color_paint = 255
                                    if path_head[0] == 'U':
                                        dataPackUp[0] = int(dict_part['detection_classes'][num_dete])
                                        dataPackUp[1] = distance
                                        dataPackUp[2] = 1
                                    else:
                                        dataPackDown[0] = int(dict_part['detection_classes'][num_dete])
                                        dataPackDown[1] = distance
                                        dataPackDown[2] = 1
                                # cv2.circle(color_img, (x_pos, ymax_i), 2, (255, 255, 255))
                                cv2.rectangle(color_img, (xmin_i, ymin_i), (xmax_i, ymax_i), (color_paint, 255, 255), 1)
                                score_temp = int(dict_part['detection_scores'][num_dete] * 100)
                                # st_get = category_index[dict_part['detection_classes'][num_dete]][
                                #              'name'] + ' dis:' + str(
                                #     distance) + ' sco ' + str(score_temp)
                                st_get = category_index[dict_part['detection_classes'][num_dete]][
                                             'name'] + ' sco '+str(score_temp)
                                # cv2.putText(color_img, st_get, (xmin_i, ymin_i), cv2.FONT_HERSHEY_COMPLEX, 0.5,
                                #             (255, 255, 0), 1)
                                cv2.putText(color_img, st_get, (xmin_i, ymax_i), cv2.FONT_HERSHEY_COMPLEX, 0.5,
                                            (255, 255, 0), 1)

                            cv2.imshow(path_head, color_img)
                            cam_detect = 1



                if cv2.waitKey(1) & 0xff == ord('q'):
                    cv2.destroyAllWindows()
                    break

                # print(time.time()-st_time)


def serialData(numberCam_deal,dataPackUp, dataPackDown,car_stop_up,car_stop_down):
    # 串口初始化
    ser = serial.Serial()
    ser.baudrate = 115200
    ser.port = '/dev/ttyUSB0'
    ser.open()
    print("open success")

    #                         1    2   3   4    5   6   7   8    9  10  11  12   13   14  15   16  17  18
    #商品序列                  薯片 牛奶 薯条 牙膏 可乐 钙片 橙子 挂钩 饼干 纸抽 书 奥利奥 果珍 洗发液 肠  牙刷 香水 杯子 ---宽度 高度
    things_width = np.array([ 14,   6,   8,  5, 7,  4,  6,  10, 16, 16, 17, 12,  14,   6,  11,  7,  5, 11])
    things_length = np.array([12,   4,   5, 12, 3,  3,  4,   8,  5,  6, 11,  3,   6,   3,   4, 12,  3,  5])

    # 转子长度12cm 12/2
    zhuazi_length_up = 6
    zhuazi_length_down = 13

    class CameraPar:
        def __init__(self, class_index, first_char, pawlength, distance):
            # class 索引是从1开始的
            self.class_record = class_index
            self.sendfirst = first_char
            self.pawlength = pawlength
            self.distance = distance

        def calculatedepth(self):
            dist = self.distance // 10
            if self.distance <= 300:
                print("error: distance lower 300mm")
                dist_temp = 16
                dist_temp += things_length[self.class_record - 1]
                dist_temp += self.pawlength
                dist = int(dist_temp)
            else:
                dist_temp = np.sqrt(self.distance * self.distance - 300 * 300)
                dist_temp /= 10
                dist_temp += things_length[self.class_record-1]
                dist_temp += self.pawlength
                dist_int = int(dist_temp)
                if dist_int > 250:
                    dist_int = 250
                dist = dist_int
            return dist

        def send2PC(self,ser):
            dist = self.calculatedepth()
            if self.class_record == 4:
                dist = 34
            for i in range(2):
                # 发送信息
                # 'A' 'C' '上'/'下' 宽度 距离  'B' 'D'
                ser.write('A'.encode('utf-8'))
                ser.write('C'.encode('utf-8'))
                ser.write(self.sendfirst.encode('utf-8'))
                # ser.write(bytes([self.class_record]))

                ser.write(bytes([things_width[self.class_record-1]]))
                ser.write(bytes([dist]))

                ser.write('B'.encode('utf-8'))
                ser.write('D'.encode('utf-8'))

            print("inform",self.sendfirst, "class", self.class_record, "dis", dist)

    # 识别的货物清单索引
    dataPackUp[-1]=0
    dataPackDown[-1]=0

    CamUp = CameraPar(0, 'U', zhuazi_length_up, 0)
    CamDown = CameraPar(0, 'D', zhuazi_length_down, 0)
    car_stop_up[0]=0
    car_stop_down[0]=0

    def sendStop(cam, class_record):
        for i in range(2):
            # 发送信息
            # 'A' 'C' '上'/'下' 类别  'B' 'D'
            ser.write('A'.encode('utf-8'))
            ser.write('C'.encode('utf-8'))

            ser.write(cam.encode('utf-8'))
            ser.write('S'.encode('utf-8'))
            ser.write(bytes([class_record]))


            ser.write('B'.encode('utf-8'))
            ser.write('D'.encode('utf-8'))
        print("stop",cam,class_record)

    while True:
        if car_stop_down[0]:
            car_stop_down[0]=0
            sendStop('D', car_stop_down[1])
        if car_stop_up[0]:
            car_stop_up[0]=0
            sendStop('U', car_stop_up[1])

        #   上面的摄像头
        if dataPackUp[-1]:
            dataPackUp[-1] = 0
            CamUp.class_record = dataPackUp[0]
            CamUp.distance = dataPackUp[1]
            CamUp.send2PC(ser)
        #   下面的摄像头
        if dataPackDown[-1]:
            dataPackDown[-1] = 0
            CamDown.class_record = dataPackDown[0]
            CamDown.distance = dataPackDown[1]
            CamDown.send2PC(ser)

    return


if __name__ == '__main__':
    # f = open("D435Camera/build/camdep.txt", 'r')
    # f = open("test.txt", 'r')
    # a=f.read(1)
    # if a is not '2':
    #     print("asdf")

    # img_file = open("D435Camera/build/camdep.txt", 'r')
    # a = img_file.read(1)
    # print(a)
    # # while True:
    # #     a = img_file.read(1)
    # #     if a is '5':
    # #         print(a)
    # #         break
    # img_file = open("D435Camera/build/camdep.txt", 'r')
    # a = img_file.read(1)
    # print(a)

    task_signal=2
    numberCam_deal=mp.Value("i",2)
    dataPackUp=mp.Array("i",range(3))
    dataPackDown=mp.Array("i",range(3))
    car_stop_up = mp.Array("i", range(2))
    car_stop_down = mp.Array("i", range(2))


    # dealSerial = mp.Process(target=serialData, args=(numberCam_deal,send_goods_index,goods_depth))
    # dealSerial.start()
    # dealSerial.join()


    if task_signal==1:
        dealim = mp.Process(target=run_inference_for_single_image, args=(detection_graph,numberCam_deal,dataPackUp,dataPackDown,car_stop_up,car_stop_down))
        dealim.start()
        dealim.join()
    elif task_signal==2:
        dealSerial = mp.Process(target=serialData, args=(numberCam_deal,dataPackUp,dataPackDown,car_stop_up,car_stop_down))
        dealSerial.start()
        dealim = mp.Process(target=run_inference_for_single_image, args=(detection_graph,numberCam_deal,dataPackUp,dataPackDown,car_stop_up,car_stop_down))
        dealim.start()
        dealim.join()
        dealSerial.join()
