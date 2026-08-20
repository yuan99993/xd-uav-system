#!/usr/bin/env python3
"""Reproducible FloodNet SegFormer training runner.

The runner deliberately preserves FloodNet's official train/validation/test
directories. It crops only inside an already assigned split, so no image or
adjacent crop crosses into validation. The 4 GB pilot uses gradient accumulation
to make an effective batch of 12 without claiming that 12 full 1024 px images
fit on the GPU simultaneously.
"""

import argparse
import json
import math
import random
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
import torch
import torch.nn.functional as functional
import yaml
from PIL import Image, ImageEnhance
from torch.utils.data import DataLoader, Dataset
from tqdm import tqdm
from transformers import SegformerForSemanticSegmentation


IMAGE_MEAN = (0.485, 0.456, 0.406)
IMAGE_STD = (0.229, 0.224, 0.225)


@dataclass(frozen=True)
class Sample:
    image_path: Path
    mask_path: Path


def load_yaml(path: Path) -> Dict:
    with path.open("r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def resolve_samples(root: Path, split: str) -> List[Sample]:
    split_directory = root / split
    images_directory = split_directory / f"{split}-org-img"
    labels_directory = split_directory / f"{split}-label-img"
    if not images_directory.is_dir() or not labels_directory.is_dir():
        raise FileNotFoundError(f"FloodNet {split} split is incomplete under {root}.")
    samples: List[Sample] = []
    for image_path in sorted(images_directory.glob("*.jpg")):
        mask_path = labels_directory / f"{image_path.stem}_lab.png"
        if not mask_path.is_file():
            raise FileNotFoundError(f"Missing label for {image_path.name}: {mask_path}")
        samples.append(Sample(image_path=image_path, mask_path=mask_path))
    if not samples:
        raise RuntimeError(f"No {split} images found under {images_directory}.")
    return samples


def crop_origin(mask: Image.Image, crop_size: Tuple[int, int], training: bool,
                rare_probability: float, rare_classes: Sequence[int]) -> Tuple[int, int]:
    width, height = mask.size
    crop_width, crop_height = crop_size
    maximum_x = max(0, width - crop_width)
    maximum_y = max(0, height - crop_height)
    if not training:
        return maximum_x // 2, maximum_y // 2
    if random.random() < rare_probability:
        mask_array = np.asarray(mask, dtype=np.uint8)
        for _ in range(12):
            origin_x = random.randint(0, maximum_x)
            origin_y = random.randint(0, maximum_y)
            window = mask_array[origin_y:origin_y + crop_height,
                                origin_x:origin_x + crop_width]
            if np.isin(window, rare_classes).any():
                return origin_x, origin_y
    return random.randint(0, maximum_x), random.randint(0, maximum_y)


class FloodNetDataset(Dataset):
    def __init__(self, samples: Sequence[Sample], crop_size: Tuple[int, int], training: bool,
                 rare_probability: float, rare_classes: Sequence[int], horizontal_flip: float):
        self.samples = list(samples)
        self.crop_size = crop_size
        self.training = training
        self.rare_probability = rare_probability
        self.rare_classes = list(rare_classes)
        self.horizontal_flip = horizontal_flip

    def __len__(self) -> int:
        return len(self.samples)

    def __getitem__(self, index: int) -> Dict[str, torch.Tensor]:
        sample = self.samples[index]
        with Image.open(sample.image_path) as image_source:
            image = image_source.convert("RGB")
        with Image.open(sample.mask_path) as mask_source:
            mask = mask_source.convert("L")
        if image.size != mask.size:
            raise RuntimeError(f"Image/mask dimension mismatch for {sample.image_path.name}")

        origin_x, origin_y = crop_origin(mask, self.crop_size, self.training,
                                         self.rare_probability, self.rare_classes)
        crop_box = (origin_x, origin_y, origin_x + self.crop_size[0],
                    origin_y + self.crop_size[1])
        image = image.crop(crop_box)
        mask = mask.crop(crop_box)
        if self.training and random.random() < self.horizontal_flip:
            image = image.transpose(Image.FLIP_LEFT_RIGHT)
            mask = mask.transpose(Image.FLIP_LEFT_RIGHT)
        if self.training:
            image = ImageEnhance.Brightness(image).enhance(random.uniform(0.85, 1.15))
            image = ImageEnhance.Contrast(image).enhance(random.uniform(0.85, 1.15))

        image_array = np.asarray(image, dtype=np.float32) / 255.0
        image_tensor = torch.from_numpy(image_array.transpose(2, 0, 1))
        mean = torch.tensor(IMAGE_MEAN, dtype=torch.float32).view(3, 1, 1)
        std = torch.tensor(IMAGE_STD, dtype=torch.float32).view(3, 1, 1)
        image_tensor = (image_tensor - mean) / std
        label_tensor = torch.from_numpy(np.asarray(mask, dtype=np.int64).copy())
        if label_tensor.min().item() < 0 or label_tensor.max().item() > 9:
            raise RuntimeError(f"Invalid FloodNet class value in {sample.mask_path}")
        return {"pixel_values": image_tensor, "labels": label_tensor}


def class_weights(samples: Sequence[Sample], num_classes: int, cap: float,
                  cache_path: Optional[Path] = None) -> torch.Tensor:
    if cache_path is not None and cache_path.exists():
        cached = np.load(cache_path)
        if cached.shape == (num_classes,) and np.all(np.isfinite(cached)):
            return torch.tensor(cached, dtype=torch.float32)
    counts = np.zeros(num_classes, dtype=np.float64)
    for sample in tqdm(samples, desc="Computing FloodNet class weights", unit="mask"):
        with Image.open(sample.mask_path) as source:
            values = np.asarray(source.convert("L"), dtype=np.uint8)
        counts += np.bincount(values.ravel(), minlength=num_classes)[:num_classes]
    frequencies = counts / max(1.0, counts.sum())
    valid_frequencies = frequencies[frequencies > 0.0]
    median_frequency = float(np.median(valid_frequencies))
    weights = median_frequency / np.maximum(frequencies, 1.0e-12)
    weights = np.minimum(weights, cap)
    weights[frequencies == 0.0] = 0.0
    weights /= np.mean(weights[weights > 0.0])
    if cache_path is not None:
        temporary_path = cache_path.with_name(cache_path.name + ".tmp")
        with temporary_path.open("wb") as stream:
            np.save(stream, weights)
        temporary_path.replace(cache_path)
    return torch.tensor(weights, dtype=torch.float32)


def multiclass_dice_loss(logits: torch.Tensor, labels: torch.Tensor, num_classes: int) -> torch.Tensor:
    probabilities = torch.softmax(logits, dim=1)
    labels_one_hot = functional.one_hot(labels, num_classes=num_classes).permute(0, 3, 1, 2)
    labels_one_hot = labels_one_hot.to(dtype=probabilities.dtype)
    intersection = (probabilities * labels_one_hot).sum(dim=(0, 2, 3))
    denominator = probabilities.sum(dim=(0, 2, 3)) + labels_one_hot.sum(dim=(0, 2, 3))
    return 1.0 - ((2.0 * intersection + 1.0) / (denominator + 1.0)).mean()


def evaluate(model: torch.nn.Module, loader: DataLoader, device: torch.device,
             num_classes: int, amp_enabled: bool) -> Dict[str, float]:
    confusion = torch.zeros((num_classes, num_classes), dtype=torch.float64, device=device)
    model.eval()
    with torch.no_grad():
        for batch in tqdm(loader, desc="Validation", leave=False, unit="batch"):
            images = batch["pixel_values"].to(device, non_blocking=True)
            labels = batch["labels"].to(device, non_blocking=True)
            with torch.cuda.amp.autocast(enabled=amp_enabled):
                logits = model(pixel_values=images).logits
                logits = functional.interpolate(logits, size=labels.shape[-2:], mode="bilinear",
                                                align_corners=False)
            predictions = logits.argmax(dim=1)
            indices = labels.reshape(-1) * num_classes + predictions.reshape(-1)
            confusion += torch.bincount(indices, minlength=num_classes * num_classes).reshape(
                num_classes, num_classes).to(dtype=torch.float64)
    true_positive = confusion.diag()
    denominator = confusion.sum(dim=0) + confusion.sum(dim=1) - true_positive
    iou = true_positive / denominator.clamp_min(1.0)
    accuracy = true_positive.sum() / confusion.sum().clamp_min(1.0)
    result = {"pixel_accuracy": float(accuracy.cpu()), "mean_iou": float(iou.mean().cpu())}
    for class_id, value in enumerate(iou.cpu().tolist()):
        result[f"iou_class_{class_id}"] = float(value)
    result["flood_water_iou"] = float(iou[[1, 3, 5]].mean().cpu())
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description="Train a FloodNet SegFormer-B0 profile.")
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--epochs", type=int, default=None)
    parser.add_argument("--num-workers", type=int, default=2)
    parser.add_argument("--resume", type=Path, default=None)
    parser.add_argument(
        "--stop-after-epoch", type=int, default=None,
        help="Stop after this epoch while retaining the configured scheduler horizon.",
    )
    parser.add_argument(
        "--eval-only", action="store_true",
        help="Load the checkpoint and run the fixed-center validation without training.",
    )
    arguments = parser.parse_args()

    config = load_yaml(arguments.config)
    output_root = Path(config["output"]["root"])
    output_root.mkdir(parents=True, exist_ok=True)
    seed = int(config["output"]["seed"])
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    if not torch.cuda.is_available():
        raise RuntimeError("CUDA is required for this FloodNet training profile.")
    torch.cuda.manual_seed_all(seed)
    device = torch.device("cuda")
    torch.backends.cudnn.benchmark = True

    dataset_config = config["dataset"]
    dataset_root = Path(dataset_config["extraction_root"]) / dataset_config["supervised_root"]
    train_samples = resolve_samples(dataset_root, "train")
    validation_samples = resolve_samples(dataset_root, "val")
    expected_train = int(dataset_config["official_split"]["train_images"])
    expected_validation = int(dataset_config["official_split"]["val_images"])
    if len(train_samples) != expected_train or len(validation_samples) != expected_validation:
        raise RuntimeError("FloodNet split count differs from the verified official split.")

    model_config = config["model"]
    crop_size = tuple(int(value) for value in model_config["input_crop_px"])
    optimization = config["optimization"]
    micro_batch = int(optimization["micro_batch_size"])
    accumulation = int(optimization["gradient_accumulation_steps"])
    effective_batch = int(optimization["effective_batch_size"])
    if micro_batch * accumulation != effective_batch:
        raise ValueError("micro_batch_size * gradient_accumulation_steps must equal effective_batch_size")
    sampling = config["sampling"]
    augmentation = config["augmentation"]
    train_dataset = FloodNetDataset(train_samples, crop_size, True,
                                    float(sampling["rare_hazard_crop_probability"]),
                                    sampling["rare_hazard_class_ids"],
                                    float(augmentation["horizontal_flip_probability"]))
    validation_dataset = FloodNetDataset(validation_samples, crop_size, False, 0.0, [], 0.0)
    worker_count = max(0, int(arguments.num_workers))
    loader_options = {"num_workers": worker_count, "pin_memory": True}
    if worker_count > 0:
        loader_options["persistent_workers"] = True
    train_loader = DataLoader(train_dataset, batch_size=micro_batch, shuffle=True,
                              drop_last=False, **loader_options)
    validation_loader = DataLoader(validation_dataset, batch_size=micro_batch, shuffle=False,
                                   drop_last=False, **loader_options)

    num_classes = int(model_config["num_classes"])
    class_weight = class_weights(
        train_samples, num_classes,
        float(optimization["loss"]["class_weight_cap"]),
        output_root / "class_weights.npy",
    ).to(device)
    id2label = {index: name for index, name in enumerate(dataset_config["class_names"])}
    model = SegformerForSemanticSegmentation.from_pretrained(
        model_config["checkpoint"], num_labels=num_classes, id2label=id2label,
        label2id={name: index for index, name in id2label.items()}, ignore_mismatched_sizes=True)
    gradient_checkpointing_enabled = False
    if bool(model_config.get("gradient_checkpointing", False)):
        try:
            model.gradient_checkpointing_enable()
            gradient_checkpointing_enabled = True
        except ValueError:
            # Current Hugging Face SegFormer semantic-segmentation models do
            # not expose checkpointing. Do not fail halfway through setup or
            # silently claim that a memory-saving mode was active.
            print("Warning: SegFormer gradient checkpointing is unavailable; "
                  "using the configured micro-batch size instead.", flush=True)
    model.to(device)

    encoder_parameters, decoder_parameters = [], []
    for name, parameter in model.named_parameters():
        (encoder_parameters if name.startswith("segformer.encoder") else decoder_parameters).append(parameter)
    optimizer = torch.optim.AdamW([
        {"params": encoder_parameters, "lr": float(optimization["backbone_learning_rate"])},
        {"params": decoder_parameters, "lr": float(optimization["decoder_learning_rate"])}],
        weight_decay=float(optimization["weight_decay"]))
    total_epochs = int(arguments.epochs or optimization["epochs"])
    updates_per_epoch = math.ceil(len(train_loader) / accumulation)
    total_updates = total_epochs * updates_per_epoch
    warmup_updates = min(int(optimization["warmup_steps"]), total_updates)

    def learning_rate_scale(update: int) -> float:
        if update < warmup_updates:
            return float(update + 1) / max(1, warmup_updates)
        progress = (update - warmup_updates) / max(1, total_updates - warmup_updates)
        return max(0.0, (1.0 - progress) ** 0.9)

    scheduler = torch.optim.lr_scheduler.LambdaLR(optimizer, learning_rate_scale)
    scaler = torch.cuda.amp.GradScaler(enabled=bool(optimization["amp"]))
    start_epoch = 1
    best_miou = -1.0
    if arguments.resume:
        checkpoint = torch.load(arguments.resume, map_location=device)
        model.load_state_dict(checkpoint["model"])
        optimizer.load_state_dict(checkpoint["optimizer"])
        scheduler.load_state_dict(checkpoint["scheduler"])
        # Optimizer state contains the previous run's parameter-group learning
        # rates.  A continuation experiment is allowed to deliberately use a
        # lower (or otherwise changed) rate, so restore the configured values
        # after loading state and keep LambdaLR's base rates consistent too.
        configured_learning_rates = [
            float(optimization["backbone_learning_rate"]),
            float(optimization["decoder_learning_rate"]),
        ]
        for parameter_group, learning_rate in zip(optimizer.param_groups, configured_learning_rates):
            parameter_group["lr"] = learning_rate
        scheduler.base_lrs = list(configured_learning_rates)
        scaler.load_state_dict(checkpoint["scaler"])
        start_epoch = int(checkpoint["epoch"]) + 1
        best_miou = float(checkpoint.get("best_miou", -1.0))

    if arguments.eval_only:
        metrics = evaluate(model, validation_loader, device, num_classes, bool(optimization["amp"]))
        print(json.dumps(metrics, sort_keys=True), flush=True)
        return

    run_metadata = {
        "profile": config["profile_name"], "config": str(arguments.config.resolve()),
        "model_checkpoint": model_config["checkpoint"], "device": torch.cuda.get_device_name(0),
        "crop_size": crop_size, "micro_batch_size": micro_batch,
        "gradient_accumulation_steps": accumulation, "effective_batch_size": effective_batch,
        "gradient_checkpointing_enabled": gradient_checkpointing_enabled,
        "train_images": len(train_samples), "val_images": len(validation_samples),
        "epochs": total_epochs, "started_unix": time.time(),
    }
    (output_root / "run_metadata.json").write_text(json.dumps(run_metadata, indent=2), encoding="utf-8")
    history: List[Dict[str, float]] = []
    dice_multiplier = float(optimization["loss"]["dice"])
    cross_entropy_multiplier = float(optimization["loss"]["weighted_cross_entropy"])
    amp_enabled = bool(optimization["amp"])
    global_update = 0
    for epoch in range(start_epoch, total_epochs + 1):
        model.train()
        optimizer.zero_grad(set_to_none=True)
        running_loss = 0.0
        epoch_start = time.monotonic()
        progress = tqdm(train_loader, desc=f"Epoch {epoch}/{total_epochs}", unit="batch")
        for batch_index, batch in enumerate(progress, start=1):
            images = batch["pixel_values"].to(device, non_blocking=True)
            labels = batch["labels"].to(device, non_blocking=True)
            with torch.cuda.amp.autocast(enabled=amp_enabled):
                logits = model(pixel_values=images).logits
                logits = functional.interpolate(logits, size=labels.shape[-2:], mode="bilinear",
                                                align_corners=False)
                cross_entropy = functional.cross_entropy(logits, labels, weight=class_weight)
                dice = multiclass_dice_loss(logits, labels, num_classes)
                loss = cross_entropy_multiplier * cross_entropy + dice_multiplier * dice
                scaled_loss = loss / accumulation
            scaler.scale(scaled_loss).backward()
            if batch_index % accumulation == 0 or batch_index == len(train_loader):
                scaler.unscale_(optimizer)
                torch.nn.utils.clip_grad_norm_(model.parameters(),
                                               float(optimization["max_gradient_norm"]))
                scaler.step(optimizer)
                scaler.update()
                optimizer.zero_grad(set_to_none=True)
                scheduler.step()
                global_update += 1
            running_loss += float(loss.detach().cpu())
            progress.set_postfix(loss=f"{running_loss / batch_index:.4f}",
                                 lr=f"{optimizer.param_groups[0]['lr']:.2e}")

        metrics = evaluate(model, validation_loader, device, num_classes, amp_enabled)
        metrics.update({"epoch": epoch, "train_loss": running_loss / len(train_loader),
                        "elapsed_sec": time.monotonic() - epoch_start,
                        "optimizer_updates": global_update})
        history.append(metrics)
        (output_root / "history.json").write_text(json.dumps(history, indent=2), encoding="utf-8")
        checkpoint = {"epoch": epoch, "model": model.state_dict(), "optimizer": optimizer.state_dict(),
                      "scheduler": scheduler.state_dict(), "scaler": scaler.state_dict(),
                      "best_miou": best_miou, "metrics": metrics, "run_metadata": run_metadata}
        if bool(config["output"].get("save_every_epoch", True)):
            torch.save(checkpoint, output_root / f"checkpoint_epoch_{epoch:02d}.pt")
        if metrics["mean_iou"] > best_miou:
            best_miou = metrics["mean_iou"]
            checkpoint["best_miou"] = best_miou
            torch.save(checkpoint, output_root / "best.pt")
        print(json.dumps(metrics, sort_keys=True), flush=True)
        if arguments.stop_after_epoch is not None and epoch >= arguments.stop_after_epoch:
            break


if __name__ == "__main__":
    main()
