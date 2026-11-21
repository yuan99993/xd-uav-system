"""Optional CUDA JPEG decode with a bounded image and explicit CPU fallback."""
import io
import numpy as np

class JPEGDecoder:
    def __init__(self, device='cpu', maximum_bytes=16777216):
        self.device='cpu';self.error='';self.maximum_bytes=maximum_bytes
        if device=='cpu':return
        try:
            import torch
            from torchvision.io import decode_jpeg,ImageReadMode
            if not torch.cuda.is_available():raise RuntimeError('CUDA unavailable')
            self.torch=torch;self.decode_jpeg=decode_jpeg;self.mode=ImageReadMode.RGB
            self.cuda_device=torch.device('cuda:0' if device in ('cuda','auto') else device)
            self.stream=torch.cuda.Stream(device=self.cuda_device,priority=-1)
            self.device=str(self.cuda_device)
        except (ImportError,RuntimeError) as exc:self.error=str(exc)

    def __call__(self,data):
        import cv2
        from PIL import Image
        if len(data)>self.maximum_bytes:raise ValueError('JPEG payload limit')
        with Image.open(io.BytesIO(data)) as image:
            if image.format!='JPEG' or image.width*image.height*3>self.maximum_bytes:raise ValueError('JPEG dimensions limit')
        if self.device!='cpu':
            try:
                with self.torch.cuda.stream(self.stream):
                    encoded=self.torch.frombuffer(bytearray(data),dtype=self.torch.uint8)
                    pixels=self.decode_jpeg(encoded,mode=self.mode,device=self.cuda_device)
                    result=pixels.permute(1,2,0).contiguous().cpu().numpy()[:,:,::-1].copy()
                return result
            except RuntimeError as exc:
                self.error=str(exc);self.device='cpu'
        return cv2.imdecode(np.frombuffer(data,np.uint8),cv2.IMREAD_COLOR)
