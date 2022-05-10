# Custom Op

## 准备模型和数据 

1. Onnx 模型修改
	可以通过 horizon_nn 中提供的 save/load, 以及 helper 功能, 对已有onnx模型进行修改. 用户可以选择对已有node修改, 也可选择添加自定义node. 
	需要注意
	1.1 在添加完pyop之后, 还需添加对应的value_info信息, 否则模型转换时无法判断该节点的输出shape, 进而会报错
	1.2 示例代码中读取的googlenet为分类模型示例中使用的googlenet模型. 
	用户可参考示例代码 onnx_modify.py 查看具体修改	

2. Pytorch 转 Onnx 模型
    可以pytorch中的 register_custom_op_symbolic 函数将指定op转为 pyop, 作为自定义算子, 并给出相应的运算逻辑, 完成模型转换. 
	需要注意:
	2.1 该OP的参数定义时需要附带数据类型, 例如: s : UTF-8 string, i: A 64-bit integer value. 详见: https://github.com/onnx/onnx/blob/master/docs/IR.md
	2.2 googlenet 模型的前处理部分 _transform_input 函数需要拿掉, 因此示例代码中修改了原模型中的 _transform_input 定义
	2.3 该模型的输出shape需要在 output_shape_s 中进行定义, 否则模型转换时无法判断该节点的输出shape, 进而会报错
	
	用户可参考示例代码 torch_export.py 查看具体修改.
