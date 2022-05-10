vec_diff工具旨在帮助用户定位精度异常问题。当出现精度异常时，可能有几种原因：
the vec_diff tool is used for helping users finding out accuracy problems.
When accuracy problem pops up, there can be several causes:

校准或者量化导致某一layer的输出存在误差，经过可能的误差放大，导致最终结果差异较大。
Accuracy error in the output of a certain layer caused by calibration or quantization.
Then through some possible error enlargement in the process led to dramatic accuracy error in the end.

模型转换工具的某一步存在未知问题，浮点模型到定点模型转换过程中，经过了几次优化和变换，这些变换可能会存在问题，导致异常。
Some unknow problem in the conversion process. During the floating-point model to fixed-point model 
conversion process, changes happened in the several optimization and conversion steps caused the accuracy problem.

为了定位这些问题，使用向量比较工具，用户可以比较不同阶段模型的卷积层输出差异。
To locate these problems, users can compare the conv layer output differences at different stages using the vec_diff tool.

该工具可直接对接detection和classification示例, 需要用户将示例中的yaml文件内 layer_out_dump 改为True后重新编译模型, 并在 01_inference_rt.sh 脚本中将所需config文件路径配置好, 便可直接运行。示例中默认为 03_classification/01_mobilenet 的路径配置。
This tool can work in conjunction with detection and classification samples.
Users need to recompile the model after modifying the value of layer_out_dump parameter in yaml file into True,
and configure the required file path in the 01_inference_rt.sh script, then perform it.
As default, in the sample path is 03_classification/01_mobilenet.


脚本步骤讲解:
Explain script steps:

第一步, 01_inference_rt.sh 帮助用户得到原始模型, 定点模型以及混合模型(*.bin模型)的模拟器的infer输出结果。
Step 1, 01_inference_rt.sh script helps users get the infer results of raw model, fixed-point model 
and hybrid model (*.bin model) on simulator.

第二步, 02_vec_diff.sh 帮助用户使用vec_diff 命令, 得到对比结果。其使用命令为:
step 2, 02_vec_diff.sh helps users perform the vec_diff command to get comparison results. The command is as follows:

vec_diff [OPTIONS] left_file/folder right_file/folder

参数:
Parameters:

left_file/folder
文件名，或者文件夹名。
File name or folder name.

right_file/folder
文件名，或者文件夹名。
File name or folder name.

-o, --output-file FILENAME
输出的结果文件的文件名。
The file name of output results.

vec_diff_out.csv 文件，列表（表项为：左侧文件名，右侧文件名，余弦相似度、相对欧拉距、最大绝对误差、方差），参考如下：
vec_diff_out.csv, list
(whose items contain: Left Files, Right Files, Cosine Similarity,Relative Euclidean Distance, Max Absolute Error and Mean Square Error). 
See below:

Left Files         |   Right Files        |  Cosine Similarity | Relative Euclidean Distance | Max Absolute Error | Mean Square Error   

Layerxxx-input.txt |   Layerxxx-input.txt |  xxx               | xxx                         | xxx                | xxx

Layerxxx-param.txt |   Layerxxx-param.txt |  xxx               | xxx                         | xxx                | xxx

!!! 如果模型的 input_type_rt 为 nv12 类型, 则在验证时给到 hb_mapper infer 的输入类型应为 input_type_train, 给到 hrt_bin_dump 工具的输入类型应为 nv12 数据类型 !!!
!!! If input_type_rt setting of the model is set to nv12, then the data input type for hb_mapper infer tool shall be set to input_type_train type, and the data input type for hrt_bin_dump shall be set to nv12 !!!
