# If you have any Docker Question
We will write down some common question you will face here. Feel free to ask for any question.

# Wrong PyTorch (GPU Version)

> **Note**: Check out your cuda version if you are using GPU image.In this image the default `pytorch` will be for `CUDA12.4` or `CUDA12.8`

There are two existing version CUDA12.4 and CUDA12.8
You can change in `run_gpu.sh` 
```bash
...
CONTAINER_NAME="DL_lab_cuda"
HUB_USER="hrcnthu"
REPO_NAME="dl_lab_cuda"
AS_ROOT=0
TAG_DEFAULT="humble-cuda12.4"       # humble-cuda12.4 | humble-cuda12.8 
TAG="${TAG_DEFAULT}"
...
```

## 1. ) Change Temporary
If you are already in the container, you will only can change temporary,
It will be reset if you restart the container.

```bash
# uninstall the default version
pip uninstall torch torchvision torchaudio -y

# check out your version 
nvidia-smi
```
#### download your version from pytorch website
https://pytorch.org/get-started/locally/

## 2. ) Change Permanent (Recommend)
You will need to change the dockerfile by yourself, and rebuild it.
Find the file at `~/Docker/GPU/Dockerfile.gpu`
```Dockerfile
# ===================================================================
# Python pkg（with PyTorch CUDA 12.4 ）
# ===================================================================
RUN pip install --upgrade pip && \
    pip install numpy==1.26.4 matplotlib==3.8.* scipy pandas \
    opencv-python opencv-python-headless \
    scikit-learn scikit-image Pillow \
    pyserial pexpect future pyquaternion pyyaml \
    tensorflow tensorboard && \
    pip install --no-cache-dir --ignore-installed \
      torch torchvision torchaudio \
      --index-url https://download.pytorch.org/whl/cu124     # you will only need to change this line
```
Build your own Image (it will take a while).
```bash
cd ~/Docker/GPU
./build_gpu.sh humble-cuda12.6    # fill in your version e.g. humble-cuda12.6
```
Rewrite your `run_gpu.sh` 
```bash
# -------------------------------
# Config
# -------------------------------
CONTAINER_NAME="DL_lab_cuda"
HUB_USER="hrcnthu"                  # delete this line
REPO_NAME="dl_lab_cuda"
AS_ROOT=0
TAG_DEFAULT="humble-cuda12.4"       # humble-cuda{yourversion} 
TAG="${TAG_DEFAULT}"
```
Open your container
```bash
./run_gpu.sh
```


