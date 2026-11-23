"""Fixed-shape TensorRT execution with PyTorch-owned CUDA buffers."""
import hashlib
from pathlib import Path
import threading
import torch

_CACHE = {}
_CACHE_LOCK = threading.Lock()

class StaticTensorRTEngine:
    def __init__(self, path, expected_sha256, device='cuda'):
        try:
            import tensorrt as trt
        except ImportError:
            import tensorrt_bindings as trt
        path = Path(path).resolve()
        data = path.read_bytes()
        if not expected_sha256 or hashlib.sha256(data).hexdigest() != expected_sha256:
            raise ValueError("TensorRT engine integrity mismatch")
        self.device = torch.device(device)
        if self.device.type != 'cuda':
            raise ValueError("TensorRT requires CUDA")
        index = self.device.index if self.device.index is not None else torch.cuda.current_device()
        self.device = torch.device('cuda',index)
        key = (str(path),expected_sha256,index)
        with _CACHE_LOCK, torch.cuda.device(self.device):
            if key not in _CACHE:
                logger = trt.Logger(trt.Logger.WARNING)
                runtime = trt.Runtime(logger)
                engine = runtime.deserialize_cuda_engine(data)
                if engine is None:
                    raise RuntimeError("Cannot deserialize TensorRT engine")
                _CACHE[key] = (logger,runtime,engine)
            self.logger,self.runtime,self.engine = _CACHE[key]
            self.context = self.engine.create_execution_context()
            if self.context is None:
                raise RuntimeError("Cannot create TensorRT execution context")
            self.inputs,self.outputs = {},{}
            dtypes = {trt.float32:torch.float32,trt.float16:torch.float16,
                      trt.int32:torch.int32,trt.int64:torch.int64,trt.bool:torch.bool}
            for index in range(self.engine.num_io_tensors):
                name = self.engine.get_tensor_name(index)
                shape = tuple(self.engine.get_tensor_shape(name))
                dtype = self.engine.get_tensor_dtype(name)
                if not shape or min(shape) <= 0 or dtype not in dtypes:
                    raise ValueError("Unsupported TensorRT tensor shape or dtype")
                buffer = torch.empty(shape,device=self.device,dtype=dtypes[dtype])
                table = self.inputs if self.engine.get_tensor_mode(name) == trt.TensorIOMode.INPUT else self.outputs
                table[name] = buffer
                if not self.context.set_tensor_address(name,buffer.data_ptr()):
                    raise RuntimeError("Cannot bind TensorRT tensor " + name)
        self.lock = threading.Lock()
        self.completion = torch.cuda.Event()
        self.stream = torch.cuda.Stream(device=self.device)

    def run_tensors(self, inputs):
        if inputs.keys() != self.inputs.keys():
            raise ValueError("TensorRT input names mismatch")
        producer = torch.cuda.current_stream(self.device)
        with self.lock, torch.cuda.device(self.device):
            with torch.cuda.stream(self.stream):
                self.stream.wait_stream(producer)
                for name,value in inputs.items():
                    target = self.inputs[name]
                    if tuple(value.shape) != tuple(target.shape):
                        raise ValueError("TensorRT input shape mismatch for " + name)
                    target.copy_(value,non_blocking=True)
                    if value.is_cuda:
                        value.record_stream(self.stream)
                if not self.context.execute_async_v3(self.stream.cuda_stream):
                    raise RuntimeError("TensorRT execution failed")
                result = {name:value.clone() for name,value in self.outputs.items()}
                self.completion.record(self.stream)
            producer.wait_event(self.completion)
            for value in result.values():
                value.record_stream(producer)
            return result
