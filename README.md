# SJTU JiaoLong RM2018 Buff

## 环境要求

- Ubuntu 14.04或更高
- ROS indigo 或更高
- CMake
- OpenCV（推荐OpenCV3以上版本）
- Caffe
- Cuda, Cudnn（对于Caffe GPU版本）

## Nodes

- control node：接受所有的信号并控制所有其他结点
- image publish node：读取图像，发布给其他结点，后来为了加速，直接在这里进行图像处理
- ~~sudoku_node~~(合并到 image_publish_node): 处理整个图像，找出九宫格和数码管区域并发布
- ~~led_node~~(合并到 image_publish_node): 接受数码管区域信号，给出数码管数字
- ~~mnist_node/fire_node~~(合并到 image_publish_node): 接受九宫格区域，给出手写/火焰数字
- aim node：用来自动标定

## 工作流程

- 初始状态：control node等待串口消息，其他结点休眠
- 收到串口消息后，进入不同的阶段（大/小符，自动/手动标定）
- sudoku node工作，找到九宫格和数码管区域后发布，led node和mnist/fire node继续休眠
- led node和mnist/fire node工作，发布识别到的数字
- control node获取九宫格和数码管数字，判断九宫格是否变化、现在打中了几个、应该击打哪个宫格，确认后发送消息给串口

## Topic

- sudoku rect：九宫格的范围
- led rect：数码管的范围
- led num：数码管的数字
- mnist num：手写数字九宫格对应值
- fire num：火焰数字九宫格对应值
- *_ctr：结点控制信号
- *_param：结点参数更改信号

## 画面分割算法

- 找出高亮区域，寻找轮廓
- 找到两侧5个小方块
- 中间的区域是九宫格，上方是数码管

## 数码管算法

- 寻找轮廓，找出5个数码管
- 用穿线法，判断数字

## 手写/火焰数字算法

- 使用神经网络
- 在Google Colab上训练保存模型（h5py格式），训练代码在`train_model`文件夹
- 使用MMdnn转换成Caffe模型（妙算上TensorFlow兼容不佳）

## 标定算法

- 先使用画面分割算法，找到左上，中间，右下三个宫格
- 使用KCFTracker追踪宫格，直到标定成功

---

## Requirements

- Ubuntu 14.04 or higher
- ROS indigo or higher
- CMake
- OpenCV (version 3 recommended)
- Caffe

## Nodes

- control_node: Receive signals and control other nodes.
- image_publish_node: Fetch image from camera and publish to other nodes. To speed up, the processing of image is also added.
- ~~sudoku_node~~(merged to image_publish_node): Process the whole image and give the region of sudoku and led.
- ~~led_node~~(merged to image_publish_node): Receive the region of led and give the digits
- ~~mnist_node/fire_node~~(merged to image_publish_node): Receive the region of led and give the mnist/fire numbers.
- aim_node: Track the block and give the position.

## Topic

- sudoku_rect: The region of sudoku
- led_rect: The region of led
- led_num: The number of led
- mnist_num: The number of handwriting numbers on the sudoku
- fire_num: The fire numbers on the sudoku
- *_ctr: The control signal
- *_param: To adjust params

## Sudoku Algorithm

- Use gray image, threshold and findContour
- Find the 5 small blocks on the side
- The region between them is the sudoku and the region above is led

## Led Algorithm

- Find the five digits
- For one digit, cross it to recognize it.

## Mnist/Fire Algorithm

- Use CNN
- Train on Google Colab with Keras (TensorFlow backend)
- Convert to Caffe Model with MMdnn

## Aim Algorithm

- Sudoku Algorithm to find the sudoku
- KCFTracker