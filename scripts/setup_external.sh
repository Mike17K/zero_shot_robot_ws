#!/bin/bash
# Non-ROS libraries under external/ (git submodules, pinned) + model weights.
# Found at runtime via shared_utils.external (add_external_to_path).
#
#   external/MobileSAM              ChaoningZhang/MobileSAM   (package mobile_sam)
#   external/pytorch-image-models   huggingface timm v1.0.30  (package timm, needed by MobileSAM's TinyViT)
#   external/weights/mobile_sam.pt  MobileSAM ViT-T checkpoint (gitignored)
#
# Idempotent: re-running only fills in what is missing. Run from anywhere
# inside the workspace (inside the container: /workspaces/isaac_ros-dev).
set -euo pipefail
cd "$(git -C "$(dirname "$0")" rev-parse --show-toplevel)"

TIMM_TAG=v1.0.30

add_submodule() {  # <url> <path>
    if ! git config -f .gitmodules --get "submodule.$2.path" >/dev/null 2>&1; then
        git submodule add "$1" "$2"
    fi
    git submodule update --init "$2"
}

mkdir -p external/weights
add_submodule https://github.com/ChaoningZhang/MobileSAM.git external/MobileSAM
add_submodule https://github.com/huggingface/pytorch-image-models.git external/pytorch-image-models
git -C external/pytorch-image-models fetch --depth 1 origin "refs/tags/$TIMM_TAG:refs/tags/$TIMM_TAG"
git -C external/pytorch-image-models checkout -q "$TIMM_TAG"

weights=external/weights/mobile_sam.pt
if [ ! -s "$weights" ]; then
    if [ -s external/MobileSAM/weights/mobile_sam.pt ]; then
        cp external/MobileSAM/weights/mobile_sam.pt "$weights"
    else
        wget -O "$weights" https://github.com/ChaoningZhang/MobileSAM/raw/master/weights/mobile_sam.pt
    fi
fi

echo "external/ ready:"
ls -la external external/weights
echo "Commit the submodule pins: git add .gitmodules external/MobileSAM external/pytorch-image-models"
