from horizon_nn import horizon_onnx
from horizon_nn.horizon_onnx import helper
from horizon_nn.horizon_onnx.onnx_pb import TensorProto

model = horizon_onnx.load("googlenet.onnx")

# print(helper.printable_graph(model.graph))

py1_node = helper.make_node(
    op_type='PyOp',  #required, must be 'PyOp'
    name='hr_op',  #required
    inputs=['368'],  #required
    outputs=['368_py'],  #required
    domain='horizon.custom',  #required, ! must be 'horizon.custom' domain !
    input_types=[TensorProto.FLOAT],  #required
    output_types=[TensorProto.FLOAT],  #required
    module=
    'sample_custom',  #required, ! must match the file name of the "op_register_files" !
    class_name=
    'CustomIdentity',  #required ! must match the class def name in sample_custom python file !
    compute='compute',  #optional, 'compute' by default
    kernel_size='10',  #optional,
    threshold='1.2')  #optional,

custom_index = -1
for node_index in range(len(model.graph.node)):
    if model.graph.node[
            node_index].name == "Conv_4":  # insert pyop at certain point in the original model
        model.graph.node[node_index].input[0] = "368_py"
        custom_index = node_index

if custom_index == -1:
    raise ValueError(f"target node not found")

model.graph.node.insert(custom_index, py1_node)

pyop_value_info = helper.make_tensor_value_info(
    "368_py",
    TensorProto.FLOAT,  #  value info is needed for onnx shape infer.
    [1, 64, 56, 56])

model.graph.value_info.append(pyop_value_info)

pyop_opset = helper.make_operatorsetid("horizon.custom", 1)
model.opset_import.append(pyop_opset)

horizon_onnx.save(model, "googlenet_cop.onnx")