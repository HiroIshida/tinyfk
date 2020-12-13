#!/bin/bash

python -c "
import os.path as osp
import gdown
print(\"downloading robot models\")
gdown.cached_download(
    url='https://drive.google.com/uc?id=1y7Jc3QoVW6J072CrSNupfKpyLp4NNxuH',
    path=osp.join(\"./data\", 'fetch_description.tar.gz'),
    md5='fbe29ab5f3d029d165a625175b43a265',
    postprocess=gdown.extractall,
    quiet=True,
)
"
rm ./data/fetch_description.tar.gz
