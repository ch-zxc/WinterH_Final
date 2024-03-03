# WinterH_Homework

## 完成情况

实装了ROI区域、pnp解算、卡尔曼滤波器、识别颜色切换等部分功能

关于识别颜色切换，目前感觉蓝色识别效果比红色好一点（

## 代码&效果说明

关于红蓝色识别切换的控制代码在Line 136，通过修改mode值来切换识别的颜色

关于图片窗口

|窗口名称|图片意义|
|----|----|
|sec|截取的视频当前帧图像|
|source|根据sec大小形成的黑色底板图像|
|grey|将sec二值化后的图像|
|dilatemat|将grey进行一系列腐蚀与膨胀之后的图像|
|ROI|按照ROI尺寸从dilatemat截取的区域图像|
|dilatemat after ROI|根据感兴趣区域将dilatemat进行处理后的图像|
|Thresholded Image|突出识别颜色的图像|
|imgBGR|根据识别的目标颜色进行黑色处理后的dilatemat图像|
|final|最终效果图像|

## final图像说明

![final_sample](https://github.com/ch-zxc/WinterH_Final/blob/master/final_pic_sample.png)

绿色矩形为ROI区域

红色旋转矩形为识别的合格灯条

蓝色圆形为以装甲板中心为圆心形成的圆形

黄色点为通过卡尔曼滤波进行的装甲板移动预测（这个效果一般

## 尚未解决的问题

关于识别效果，现在存在一个问题就是同时存在两个目标车辆的时候识别框会在两个车之间来回频繁切换，目前想到的解决办法是加上装甲板数字识别功能，然后通过筛选装甲板数字进行目标车辆追踪