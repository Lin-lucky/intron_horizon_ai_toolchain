import torch
from horizon_nn.horizon_onnx.onnx_pb import TensorProto
from torch.onnx.symbolic_helper import parse_args
from torch.onnx.utils import register_custom_op_symbolic
from torch import Tensor

model = torch.hub.load('pytorch/vision:v0.10.0', 'googlenet', pretrained=True)


def _transform_input(x: Tensor) -> Tensor:
    return x


model._transform_input = _transform_input


@parse_args("v", "v")
def horizon_pool(g, input, output_size):
    return g.op(
        'horizon.custom::PyOp',  #required, ! must be 'horizon.custom' domain !
        input,
        class_name_s=
        "GlobalAveragePool",  #required ! must match the class def name in sample_custom python file !
        compute_s="compute",  #optional, 'compute' by default
        module_s=
        "sample_custom",  #required ! must match the file name of the "op_register_files" !
        input_types_i=[TensorProto.FLOAT],  #required
        output_types_i=[TensorProto.FLOAT],  #required
        output_shape_s=["1, 1024, 1, 1"])  #required


d_input = torch.rand(1, 3, 224, 224)
register_custom_op_symbolic('::adaptive_avg_pool2d',
                            horizon_pool,
                            opset_version=11)
torch.onnx.export(model, d_input, "googlenet_cop.onnx", opset_version=11)
