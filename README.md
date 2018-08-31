# SJTU JiaoLong RM2018 Buff

## Requirements
- Ubuntu 14.04 or higher
- ROS indigo or higher
- CMake
- OpenCV (version 3 recommended)
- Caffe

## Nodes:

- control_node: Receive signals and control other nodes.
- image_publish_node: Fetch image from camera and publish to other nodes. To speed up, the processing of image is also added.
- ~~sudoku_node~~(merged to image_publish_node): Process the whole image and give the region of sudoku and led.
- ~~led_node~~(merged to image_publish_node): Receive the region of led and give the digits
- ~~mnist_node/fire_node~~(merged to image_publish_node): Receive the region of led and give the mnist/fire numbers.
- aim_node: Track the block and give the position.

## Topic:

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