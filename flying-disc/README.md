# flying disc
本工程主要完成第十六届亚太机器人比赛中防守任务，第十六届亚太机器人比赛主题和规则的视频[链接](https://www.youtube.com/watch?v=3Xvxbw_YCvQ)，最终国内赛决赛视频[链接](https://www.bilibili.com/video/av12405640/)。

程序识别主函数在src/main.cpp, 部分识别的api在src/recognition.cpp。  
由于方案和调试时的需求时刻在变，当时代码比较乱，先梳理大概思路如下：

工程中包括如下几个部分

- 手动调节参数，主要是平时调试和试场时使用：调节R,G,B三个的增益，使得达到白平衡效果

- 初始化(结合场地的光照，根据摄像头视野中的白条，进行自动白平衡调节)

- 手动防守   通过人观察飞盘的位置，手动点击触摸板大概位置，然后通过程序捕获坐标，进而控制机器人实现防守

- 防守

  ```
  1.通过图像二值化得到圆台的二值图像，并加入形态学操作
  2.通过模板匹配找到圆台
  3.通过hsv阈值，对红、蓝两种飞盘进行二值化，加入形态学操作
  4.对两种飞盘通过边界点确定左上角和右下角，确定出相对圆台的位置
  5.防守策略
  	如果着陆台上没有自己颜色的飞盘，发送给机器人，然后机器人放一个自己的飞盘
  	如果着陆台上有敌人颜色的飞盘，将着陆台分成6个区域，将飞盘所在区域的号，发送给机器人，机器人通过区域号，调用调试时保存的参数，控制电机将飞盘打掉。
  ```