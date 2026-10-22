"""
Export a NeuPAN DUNE checkpoint (.pth) to the NPTF binary format read by
libneupan's MLP::load.

Usage (from the upstream NeuPAN repo root, with its venv):
    python export_dune_weights.py <checkpoint.pth> <out.bin> <planner.yaml>

The footprint is read from the yaml's robot section (vertices, or length,
width and wheelbase) the way upstream builds it, and stored in the file so
libneupan refuses a robot it was not trained for. Pass the yaml the
checkpoint was trained with or the planner yaml that uses it.

Part of neupan_cpp, a C++ port of NeuPAN (Copyright (c) 2025 Ruihua Han),
distributed under the GNU General Public License v3 or later.
"""

import sys

import torch
import torch.nn as nn
import yaml

from nptf import write_nptf


def export(checkpoint: str, out_path: str, yaml_path: str):
    from neupan.blocks import ObsPointNet
    from neupan.robot import robot

    with open(yaml_path) as f:
        footprint = robot(**yaml.safe_load(f)["robot"])
    G, h = footprint.G, footprint.h

    model = ObsPointNet(2, len(G))
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

    records.append(("meta.G", G))
    records.append(("meta.h", h))

    write_nptf(out_path, records)
    print(f"exported {len(records)} records to {out_path} "
          f"(footprint vertices {footprint.vertices.T.tolist()})")


if __name__ == "__main__":
    export(sys.argv[1], sys.argv[2], sys.argv[3])
