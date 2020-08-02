# digital_recognition
# 2020年新工科联盟-Xilinx暑期学校

# 项目名称
  基于FPGA的一位数字识别系统设计
# 项目介绍
## 项目概要
    该设计是基于FPGA的一位印刷体数字识别系统
    通过OV5647摄像头采集图像，将采集到的图像进行rgb转ycbcr处理、二值化处理以及投影处理后，经过基于数字特征算法实现对数字的识别，并将结果显示在LCD屏幕上
## 工具版本
    vivado2018.3
## 板卡型号与外设
    板卡：SEA Board
    芯片型号：Xilinx Spartan7 xcs15
    外设：摄像头OV5647，mini HDMI转HDMI线，3.5寸LCD显示屏（MPI3508）
 ## 仓库目录介绍
  ### Camero_Demo模块
      完成对图像的采集，并且将捕捉到的图像显示在LCD上
  ### LCD模块
      该模块由两个子模块构成：时钟分频模块clk_div，LCD驱动模块
  ### VIP(Vedio_Image_Process)模块
      该模块由4个子模块构成：rgb2ycbcr模块、二值化模块、投影模块、数字识别模块
      ·rgb2ycbcr模块：将LCD输出的RGB数据转化为YCbCr的数据
      ·二值化模块：进一步对YCbCr数据进行二值化操作
      ·投影模块：分别对数据进行水平和垂直投影，从而实现对图像的分割，再通过对行和列的像素累加值的判断，可以确定数字的位置
      ·数字识别模块：通过对数字特征的判断，实现数字识别，并将结果显示到LCD屏幕上
## 项目框图
      ![Image text](https://github.com/qishiyemeishashi/digital_recognition/blob/master/Project_block_diagram.png)
## 完成功能展示
      
