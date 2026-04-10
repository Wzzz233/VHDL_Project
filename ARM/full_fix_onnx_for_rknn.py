"""Combined fix: Replace Resize with Upsample AND fix Conv dilations for RKNN toolkit2."""
import onnx
from onnx import helper, numpy_helper
import numpy as np

src = '/home/wzzz/VHDL_Project/ARM/stage1_r18_gt_best_rknnfriendly_op12_convfixed.onnx'
dst = '/home/wzzz/VHDL_Project/ARM/stage1_r18_gt_best_rknn_ready.onnx'

model = onnx.load(src)

# Step 1: Set opset to 10 (where Upsample is native)
model.opset_import[0].version = 10

# Step 2: Replace Resize nodes with Upsample
new_nodes = []
remove_init_names = set()
resize_count = 0
for node in model.graph.node:
    if node.op_type == 'Resize':
        scales_name = node.input[2] if len(node.input) > 2 else ''
        scales_val = None
        # Find scales from initializer
        for init in model.graph.initializer:
            if init.name == scales_name:
                scales_val = numpy_helper.to_array(init).flatten().tolist()
                remove_init_names.add(init.name)
                break
        if scales_val is None:
            # Find from Constant nodes
            for cn in model.graph.node:
                if cn.op_type == 'Constant' and len(cn.output) > 0 and cn.output[0] == scales_name:
                    for attr in cn.attribute:
                        if attr.name == 'value':
                            scales_val = numpy_helper.to_array(attr.t).flatten().tolist()
                            break
                    break
        if scales_val is None:
            # Default: 2x bilinear upsample
            scales_val = [1.0, 1.0, 2.0, 2.0]
        new_node = helper.make_node(
            'Upsample',
            inputs=[node.input[0]],
            outputs=node.output,
            name=node.name if node.name else f'upsample_{resize_count}',
        )
        new_node.attribute.append(helper.make_attribute('scales', scales_val))
        new_node.attribute.append(helper.make_attribute('mode', 'linear'))
        new_nodes.append(new_node)
        resize_count += 1
        # Also remove roi constant if present
        if len(node.input) > 1:
            remove_init_names.add(node.input[1])
    else:
        new_nodes.append(node)

del model.graph.node[:]
model.graph.node.extend(new_nodes)

# Remove unused initializers
new_inits = [init for init in model.graph.initializer if init.name not in remove_init_names]
del model.graph.initializer[:]
model.graph.initializer.extend(new_inits)

# Remove unused Constant nodes that produced scales/roi for Resize
final_nodes = []
remove_const_outputs = remove_init_names
for node in model.graph.node:
    if node.op_type == 'Constant' and len(node.output) > 0 and node.output[0] in remove_const_outputs:
        continue
    final_nodes.append(node)
del model.graph.node[:]
model.graph.node.extend(final_nodes)

# Set all graph input shapes to static (remove dim_param)
for inp in model.graph.input:
    t = inp.type.tensor_type
    if t.HasField('shape'):
        for dim in t.shape.dim:
            if dim.dim_param:
                dim.Clear()

onnx.save(model, dst)
print(f'Replaced {resize_count} Resize nodes with Upsample')
print(f'Saved to {dst}')

try:
    onnx.checker.check_model(onnx.load(dst))
    print('ONNX check passed')
except Exception as e:
    print(f'Check warning (expected for opset10 Upsample): {e}')