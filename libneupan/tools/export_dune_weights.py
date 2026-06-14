"""
Export a NeuPAN DUNE checkpoint (.pth) to the NPTF binary format read by
libneupan's MLP::load.

Usage (from the upstream NeuPAN repo root, with its venv):
    python export_dune_weights.py <checkpoint.pth> <out.bin> [edge_dim]

Part of neupan_cpp, a C++ port of NeuPAN (Copyright (c) 2025 Ruihua Han),
distributed under the GNU General Public License v3 or later.
"""

import sys

import torch
import torch.nn as nn

from nptf import write_nptf


def export(checkpoint: str, out_path: str, edge_dim: int = 4):
    from neupan.blocks import ObsPointNet

    model = ObsPointNet(2, edge_dim)
    model.load_state_dict(torch.load(checkpoint, map_location="cpu"))
    model.eval()

    records = []
    for i, layer in enumerate(model.MLP):
        prefix = f"L{i:02d}"
        if isinstance(layer, nn.Linear):
            records.append((f"{prefix}.linear.weight",
                            layer.weight.detach().numpy()))
            records.append((f"{prefix}.linear.bias",
                            layer.bias.detach().numpy()))
        elif isinstance(layer, nn.LayerNorm):
            records.append((f"{prefix}.ln.gamma",
                            layer.weight.detach().numpy()))
            records.append((f"{prefix}.ln.beta",
                            layer.bias.detach().numpy()))
        elif isinstance(layer, nn.Tanh):
            records.append((f"{prefix}.tanh", torch.zeros(0, 0).numpy()))
        elif isinstance(layer, nn.ReLU):
            records.append((f"{prefix}.relu", torch.zeros(0, 0).numpy()))
        else:
            raise TypeError(f"unsupported layer {type(layer)}")

    write_nptf(out_path, records)
    print(f"exported {len(records)} records to {out_path}")


if __name__ == "__main__":
    ckpt, out = sys.argv[1], sys.argv[2]
    edge_dim = int(sys.argv[3]) if len(sys.argv) > 3 else 4
    export(ckpt, out, edge_dim)
