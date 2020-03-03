# Product recognition
以京东X机器人挑战赛为背景，设计了基于SSD检测框架与深度相机的商品识别与三维定位解决方案。  

比赛规则和场地图链接[link](https://x.jdwl.com/onLineActivities/jrc/game_issues)，比赛视频链接[link](https://v.qq.com/x/page/b0837wctzi2.html)

D435Camera文件夹下是深度相机采集图片的C++代码，jdx_18_1217.py是调用深度模型进行商品识别的Python代码。  

工程分成两个部分：

- 图像采集和对齐

  ```
  1.识别时有两个摄像头同时工作，通过两个线程抓取两个摄像头的图像。
  2.图像配准对齐时，调用官方提供的库函数将深度图像向彩色图像配准对齐
  rs2::align align(RS2_STREAM_COLOR);    		
  //Get processed aligned frame
  auto processed = align.process(frameset);
  3.然后将结果保存到build文件夹下，通过C++中的文件扫描，将文件夹更新，使得每次只有10张图片
  ```

- 调动深度神经网络进行推理预测

  ```
  1.去build文件夹下抓取图片
  2.调用sess.run()进行前向推理，得到预测结果
  3.通过逐类别执行soft nms实现筛选，然后通过深度坐标和图像二维坐标，判断筛选的检测框中是否有在3cm误差允许范围内有正对着的要抓取的商品
  4.通过serial包操作串口，与机器人主控进行通信
  5.当要识别的商品正对着机器人抓取的爪子时，发送结果给机器人主控。信息包括摄像头ID(即上/下摄像头)，正对面的商品的类别，商品的深度距离
  ```

工程两个部分一个是C++，另一个是Python。通信是通过向两个txt文件夹中写入数据作为信号来进行通信。