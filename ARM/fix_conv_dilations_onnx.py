"""Fix ONNX graph for RKNN-toolkit2 compatibility.
Force all Conv node attributes to explicit 2D int64 lists."""
import onnx
from onnx import helper, numpy_helper
import numpy as np

src = '/home/wzzz/VHDL_Project/ARM/stage1_r18_gt_best_rknnfriendly_op12.onnx'
dst = '/home/wzzz/VHDL_Project/ARM/stage1_r18_gt_best_rknnfriendly_op12_convfixed.onnx'

model = onnx.load(src)
fixed = 0

for node in model.graph.node:
    if node.op_type != 'Conv':
        continue
    new_attrs = []
    for attr in node.attribute:
        val = helper.get_attribute_value(attr)
        if attr.name == 'dilations':
            if isinstance(val, np.ndarray):
                val = val.tolist()
            if isinstance(val, (list, tuple)):
                if len(val) == 1:
                    val = [int(val[0]), int(val[0])]
                elif len(val) == 2:
                    val = [int(val[0]), int(val[1])]
                else:
                    val = [1, 1]
            else:
                val = [1, 1]
            new_attrs.append(helper.make_attribute('dilations', val))
            fixed += 1
        elif attr.name == 'strides':
            if isinstance(val, np.ndarray):
                val = val.tolist()
            if isinstance(val, (list, tuple)):
                val = [int(v) for v in val]
            new_attrs.append(helper.make_attribute('strides', val))
        elif attr.name == 'pads':
            if isinstance(val, np.ndarray):
                val = val.tolist()
            if isinstance(val, (list, tuple)):
                val = [int(v) for v in val]
                if len(val) == 2:
                    val = [val[0], val[1], val[0], val[1]]
            new_attrs.append(helper.make_attribute('pads', val))
        elif attr.name == 'kernel_shape':
            if isinstance(val, np.ndarray):
                val = val.tolist()
            if isinstance(val, (list, tuple)):
                val = [int(v) for v in val]
            new_attrs.append(helper.make_attribute('kernel_shape', val))
        else:
            new_attrs.append(attr)
    del node.attribute[:]
    node.attribute.extend(new_attrs)

# Also remove dynamic axes from graph inputs (make them static)
# This is critical for RKNN
for inp in model.graph.input:
    t = inp.type.tensor_type
    if t.HasField('shape'):
        for dim in t.shape.dim:
            if dim.dim_param:
                dim.Clear()
                # Will be set by input_size_list anyway

onnx.save(model, dst)
print(f'Fixed {fixed} Conv dilations')
print(f'Saved to {dst}')

# Verify
try:
    onnx.checker.check_model(onnx.load(dst))
    print('ONNX check passed')
except Exception as e:
    print(f'Check warning: {e}')