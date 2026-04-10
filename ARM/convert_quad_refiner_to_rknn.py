from rknn.api import RKNN

onnx_path = '/home/wzzz/VHDL_Project/ARM/stage1_r18_gt_best_sim.onnx'
out_path = '/home/wzzz/VHDL_Project/ARM/stage1_r18_gt_best.rknn'

trials = [
    ('nchw', ['image'], [[3, 128, 256]]),
    ('nhwc', ['image'], [[128, 256, 3]]),
]

last_err = None
for tag, inputs, input_size_list in trials:
    print('=== trial', tag, '===')
    rknn = RKNN(verbose=True)
    try:
        print('=> config')
        rknn.config(target_platform='rk3568')
        print('=> load onnx', input_size_list)
        ret = rknn.load_onnx(model=onnx_path, inputs=inputs, input_size_list=input_size_list)
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
        rknn.release()
        raise SystemExit(0)
    except Exception as e:
        last_err = e
        print('trial failed:', repr(e))
        try:
            rknn.release()
        except Exception:
            pass

raise SystemExit(f'all trials failed: {last_err!r}')
