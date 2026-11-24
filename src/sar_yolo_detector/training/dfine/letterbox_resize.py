"""D-FINE training transform matched by the runtime Letterbox preprocessor."""
import torch
from torchvision.transforms import InterpolationMode
import torchvision.transforms.v2.functional as F
from PIL import Image

from src.core import register
from src.data._misc import convert_to_tv_tensor


@register()
class LetterboxResize:
    def __init__(self, size, fill=114):
        if isinstance(size, int):
            size = (size, size)
        if len(size) != 2 or min(map(int, size)) <= 0:
            raise ValueError("LetterboxResize size must be [height, width]")
        self.out_height, self.out_width = map(int, size)
        self.fill = int(fill)

    def __call__(self, *inputs):
        if len(inputs) < 2 or not isinstance(inputs[0], Image.Image):
            raise ValueError("LetterboxResize expects a PIL image and D-FINE target")
        image, target, *rest = inputs
        width, height = image.size
        scale = min(self.out_width / float(width), self.out_height / float(height))
        content_width = max(1, min(self.out_width, int(width * scale + .5)))
        content_height = max(1, min(self.out_height, int(height * scale + .5)))
        pad_x = (self.out_width - content_width) // 2
        pad_y = (self.out_height - content_height) // 2
        image = F.resize(image, [content_height, content_width],
                         interpolation=InterpolationMode.BILINEAR, antialias=True)
        image = F.pad(image, [pad_x, pad_y,
            self.out_width - content_width - pad_x,
            self.out_height - content_height - pad_y], fill=self.fill)

        target = target.copy()
        scale_x, scale_y = content_width / float(width), content_height / float(height)
        if "boxes" in target:
            boxes = torch.as_tensor(target["boxes"]).clone().float()
            boxes[:, 0::2] = boxes[:, 0::2] * scale_x + pad_x
            boxes[:, 1::2] = boxes[:, 1::2] * scale_y + pad_y
            target["boxes"] = convert_to_tv_tensor(
                boxes, key="boxes", box_format="xyxy",
                spatial_size=(self.out_height, self.out_width))
            target["area"] = ((boxes[:, 2] - boxes[:, 0]).clamp(min=0) *
                              (boxes[:, 3] - boxes[:, 1]).clamp(min=0))
        if "masks" in target:
            masks = torch.as_tensor(target["masks"])
            resized = torch.nn.functional.interpolate(masks[:, None].float(),
                size=(content_height, content_width), mode="nearest")[:, 0].to(masks.dtype)
            target["masks"] = convert_to_tv_tensor(torch.nn.functional.pad(resized,
                (pad_x, self.out_width-content_width-pad_x,
                 pad_y, self.out_height-content_height-pad_y)), key="masks")
        target["size"] = torch.tensor([self.out_height, self.out_width])
        return (image, target, *rest)
