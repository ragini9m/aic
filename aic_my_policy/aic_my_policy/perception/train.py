"""Train a KeypointHeatmapNet for a single port type.

Example:
  python -m aic_my_policy.perception.train \
      --data_dir ~/aic_data/raw \
      --port_type sfp \
      --out ~/aic_data/models/sfp_keypoints.pt
"""

from __future__ import annotations

import argparse
from pathlib import Path

import torch
import torch.nn as nn
from torch.utils.data import DataLoader, random_split
import torch.nn.functional as F
from aic_my_policy.perception.dataset import PortKeypointDataset
from aic_my_policy.perception.keypoints import NUM_KEYPOINTS
from aic_my_policy.perception.model import KeypointHeatmapNet


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--data_dir", required=True)
    p.add_argument("--port_type", choices=["sfp", "sc"], required=True)
    p.add_argument("--out", required=True)
    p.add_argument("--epochs", type=int, default=30)
    p.add_argument("--batch_size", type=int, default=16)
    p.add_argument("--lr", type=float, default=1e-3)
    p.add_argument("--val_frac", type=float, default=0.1)
    p.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    p.add_argument("--workers", type=int, default=4)
    args = p.parse_args()

    full = PortKeypointDataset(args.data_dir, port_type=args.port_type)
    print(f"dataset {len(full)} samples for port_type={args.port_type}")
    n_val = max(1, int(len(full) * args.val_frac))
    n_train = len(full) - n_val
    train_ds, val_ds = random_split(full, [n_train, n_val])

    train_loader = DataLoader(
        train_ds, batch_size=args.batch_size, shuffle=True,
        num_workers=args.workers, collate_fn=_collate,
    )
    val_loader = DataLoader(
        val_ds, batch_size=args.batch_size, shuffle=False,
        num_workers=args.workers, collate_fn=_collate,
    )

    model = KeypointHeatmapNet(num_keypoints=NUM_KEYPOINTS).to(args.device)
    opt = torch.optim.AdamW(model.parameters(), lr=args.lr, weight_decay=1e-4)
    sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, T_max=args.epochs)
    # criterion = nn.MSELoss(reduction="none")
    criterion = _weighted_mse
    best_val = float("inf")
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)

    for epoch in range(args.epochs):
        model.train()
        tr_loss, tr_n, tr_skipped = _run_epoch(model, train_loader, opt, criterion, args.device, training=True)
        model.eval()
        with torch.no_grad():
            val_loss, val_n, val_skip = _run_epoch(model, val_loader, None, criterion, args.device, training=False)
        sched.step()
        print(f"[ep {epoch:02d}] train={tr_loss:.5f} (n={tr_n} skip={tr_skipped}) val={val_loss:.5f} (n={val_n} skip={val_skip})")
        if val_loss > 0 and val_loss < best_val:
            best_val = val_loss
            torch.save(
                {"model": model.state_dict(), "port_type": args.port_type},
                args.out,
            )
            print(f"  -> saved {args.out}")

    print(f"Done. best val={best_val:.5f}")
def _weighted_mse(pred: torch.Tensor, target: torch.Tensor, pos_weight: float=50.0) -> torch.Tensor:
    weight = 1.0 + (pos_weight - 1.0) * target
    return (weight * (pred - target) ** 2).mean()

def _collate(batch):
    imgs, heats, metas = zip(*batch)
    valid = torch.tensor([m["valid"] for m in metas], dtype=torch.bool)
    return torch.stack(imgs), torch.stack(heats), valid


def _run_epoch(model, loader, opt, criterion, device, training: bool):
    total, count, skipped = 0.0, 0, 0
    for imgs, heats, valid in loader:
        if not valid.any():
            skipped += imgs.size(0)
            continue
        imgs = imgs[valid].to(device)
        heats = heats[valid].to(device)
        pred = model(imgs)
        # Heatmap regression: MSE between predicted logits (sigmoid) and target peaks.
        loss = criterion(torch.sigmoid(pred), heats)
        if training:
            opt.zero_grad()
            loss.backward()
            opt.step()
        total += loss.item() * imgs.size(0)
        count += imgs.size(0)
    return total / max(1, count), count, skipped


if __name__ == "__main__":
    main()
