#!/bin/bash
export PYTHONPATH=/workspace:/workspace/contact_graspnet_pytorch:$PYTHONPATH
cd /workspace
python3 contact_graspnet_pytorch/inference.py $@
