from rknn.api import RKNN

onnx_path = '/home/wzzz/VHDL_Project/ARM/stage1_r18_gt_best_sim.onnx'
out_path = '/home/wzzz/VHDL_Project/ARM/stage1_r18_gt_best.rknn'

rknn = RKNN(verbose=True)
try:
    print('=> config')
    rknn.config(target_platform='rk3568')
    print('=> load onnx (use embedded static shapes; do not pass inputs/input_size_list)')
    ret = rknn.load_onnx(model=onnx_path)
    if ret != 0:
        raise RuntimeError(f'load_onnx failed: {ret}')
    print('=> build')
    ret = rknn.build(do_quantization=False)
    if ret != 0:
        raise RuntimeError(f'build failed: {ret}')
    print('=> export')
    ret = rknn.export_rknn(out_path)
    if ret != 0:
        raise RuntimeError(f'export_rknn failed: {ret}')
    print(out_path)
finally:
    try:
        rknn.release()
    except Exception:
        pass

raise SystemExit(0)
