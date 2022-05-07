# 图像处理加速库(ImageUtil)
地平线的旭日3、征程3芯片内部，内置了一些强大的图像处理组件，这些模块都是以硬件的方式来对外提供，其中包括如下能力：
1. 图像接入层(VIN)：图像信号处理（ISP)，镜头畸变校正（LDC），图像防抖矫正（LDC）；
2. 图像处理层(VPS)：图像拼接（GDC），提供图像裁剪，缩放，镜像，旋转，流控，金字塔的图像处理（IPU）模块；
3. 图像编解码(CODEC)：图像JPEG编解码以及H264/265编解码；
4. AI加速器（BPU）：深度学习AI推理以及内置了图像处理相关指令，补充了resize/crop能力；

相关能力你可以参考平台软件相关文档来获取。而我们这里图像处理加速库(ImageUtil)定位是一个能力补充，他主要基于libjpeg-turbo等软件库来实现，以C接口来提供图像解码，图像格式转换、图像缩放、抠图、padding、镜像翻转、中心旋转等功能。

其中包括如下重要的接口函数：

<table frame="border" width="100%" style="border-collapse:collapse; border:1px solid #ddd">
	<tr>
	    <th style="border:1px solid #ddd; background-color:#eee; padding:10px; width:150px">功能类别</th>
        <th style="border:1px solid #ddd; background-color:#eee; padding:6px">功能函数</th>
	    <th style="border:1px solid #ddd; background-color:#eee; padding:6px">功能描述</th>
	</tr >
	<tr>
	    <td rowspan="2" style="border:1px solid #ddd; padding:6px"><B>内存管理</B></td>
        <td align="left" style="border:1px solid #ddd; padding:6px">HobotXStreamAllocImage</td>
	    <td style="border:1px solid #ddd; padding:6px">基于图像区域大小，分配一个图像内存空间</td>
	</tr>
    <tr>
        <td align="left" style="border:1px solid #ddd; padding:6px">HobotXStreamFreeImage</td>
	    <td style="border:1px solid #ddd; padding:6px">释放HobotXStreamAllocImage接口返回的内存</td>
	</tr>
    <tr>
	    <td style="border:1px solid #ddd; padding:6px"><B>图像解码</B></td>
        <td align="left" style="border:1px solid #ddd; padding:6px">HobotXStreamDecodeImage</td>
	    <td style="border:1px solid #ddd; padding:6px">将jpg/png/bmp格式的二进制图片解码成RGB/BGR/YUV等格式，采用的软件进行解码</td>
	</tr>
    <tr>
	    <td rowspan="2" style="border:1px solid #ddd; padding:6px"><B>图像转换</B></td>
        <td align="left" style="border:1px solid #ddd; padding:6px">HobotXStreamConvertImage</td>
	    <td style="border:1px solid #ddd; padding:6px">支持RGB/BGR/YUV等格式之间的互相转换</td>
	</tr>
    <tr>
        <td align="left" style="border:1px solid #ddd; padding:6px">HobotXStreamConvertYuvImage</td>
	    <td style="border:1px solid #ddd; padding:6px">对YUV420图像进行图像格式转换，支持y、u、v存储空间可能不连续的应用场景</td>
	</tr>
    <tr>
	    <td rowspan="2" style="border:1px solid #ddd; padding:6px"><B>图像缩放</B></td>
        <td align="left" style="border:1px solid #ddd; padding:6px">HobotXStreamResizeImage</td>
	    <td style="border:1px solid #ddd; padding:6px">根据目标尺寸缩放图像，支持等比例、非等比例缩放</td>
	</tr>
    <tr>
        <td align="left" style="border:1px solid #ddd; padding:6px">HobotXStreamResizeImageByRatio</td>
	    <td style="border:1px solid #ddd; padding:6px">根据缩放因子缩放图像，支持等比例、非等比例缩放</td>
	</tr>
    <tr>
	    <td rowspan="4" style="border:1px solid #ddd; padding:6px"><B>图像抠图</B></td>
        <td align="left" style="border:1px solid #ddd; padding:6px">HobotXStreamCropImage</td>
	    <td style="border:1px solid #ddd; padding:6px">根据ROI区域进行抠图</td>
	</tr>
    <tr>
        <td align="left" style="border:1px solid #ddd; padding:6px">HobotXStreamCropYUVImage</td>
	    <td style="border:1px solid #ddd; padding:6px">对YUV420图像进行ROI区域进行抠图，支持y、u、v存储空间可能不连续的应用场景</td>
	</tr>
    <tr>
        <td align="left" style="border:1px solid #ddd; padding:6px">HobotXStreamCropImageWithPaddingBlack</td>
	    <td style="border:1px solid #ddd; padding:6px">根据ROI区域进行抠图，若抠图区域超过图像边界，则超过区域补黑色</td>
	</tr>
    <tr>
        <td align="left" style="border:1px solid #ddd; padding:6px">HobotXStreamCropYuvImageWithPaddingBlack</td>
	    <td style="border:1px solid #ddd; padding:6px">对YUV420图像进行ROI区域进行抠图，支持y、u、v存储空间可能不连续的应用场景，若抠图区域超过图像边界，则超过区域补黑色</td>
	</tr>
    <tr>
        <td style="border:1px solid #ddd; padding:6px"><B>图像填充</B></td>
        <td align="left" style="border:1px solid #ddd; padding:6px">HobotXStreamPadImage</td>
	    <td style="border:1px solid #ddd; padding:6px">对图像四周进行元素填充，放大图像分辨率</td>
	</tr>
    <tr>
        <td style="border:1px solid #ddd; padding:6px"><B>图像旋转</B></td>
        <td align="left" style="border:1px solid #ddd; padding:6px">HobotXStreamRotateImage</td>
	    <td style="border:1px solid #ddd; padding:6px">对图像顺时针进行90/180/270度的旋转</td>
	</tr>
</table>

这些函数接口使用都较为简单，请直接参考`include/image_utils/image_utils.h`头文件来熟悉接口的详细使用。

```
注意：

图像处理加速库(ImageUtil)我们在实现的采用libjpeg-turbo等软件库进行软件加速，性能上我们也是做了最大的软件优化，但是相比硬件实现还是有较大的性能gap，如果可以，请尽量使用前文中的硬件进行加速。

其次，内部使用libjpeg-turbo，OpenCV等依赖库的默认是提供了so/h，但是如果你需要源码，请联系我们获取。我们对源码未做过多修改，主要在做了芯片交叉环境的适配编译。
```
