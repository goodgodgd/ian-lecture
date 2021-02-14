---
layout: post
title:  "[Det] Environment Setup"
date:   2021-02-06 09:00:13
categories: 2021-1-detector
---



## Ubuntu Setting

- Ubuntu 20.04 Download: <https://ubuntu.com/download/desktop>

- 우분투 기본 설치
  - `sudo apt update && sudo apt upgrade -y`
  - `sudo apt install build-essential cmake git curl`
  
- NVIDIA driver: `sudo ubuntu-drivers autoinstall`

- CUDA Download: <https://developer.nvidia.com/cuda-toolkit-archive>
  - Currently(2021.02), download CUDA 11.0 for Tensorflow 2.4 and Pytorch 1.7
  - `chmod a+x cuda_<version>.run`
  - `sudo ./cuda_<version>.run`

- cuDNN: <https://developer.nvidia.com/rdp/cudnn-archive>
  - NVIDIA 계정 로그인
  
  - Select "cuDNN-xxx for CUDA 11.0"
  
  - Download "cuDNN Library for Linux (x86_64)"

  - tgz 압축해제
  
  - 헤더와 라이브러리 복사
  
    ```
    sudo cp <extracted directory>/cuda/include/* /usr/local/cuda/include
    sudo cp <extracted directory>/cuda/lib64/* /usr/local/cuda/lib64
    ```
  
  - `~/.profile`에 아래 내용 없으면 추가
  
    ```
    export LD_LIBARARY_PATH="/usr/local/cuda/lib64:${LD_LIBARARY_PATH}"
    export PATH="/usr/local/cuda/bin:${PATH}"
    ```
  
- Pyenv: <https://github.com/pyenv/pyenv-installer>
  
  - pyenv prerequisites: `https://github.com/pyenv/pyenv/wiki/Common-build-problems`
  
  - `curl https://pyenv.run | bash`
  
  - add the three lines to `~/.bashrc`
  
    ```
    export PATH="$HOME/.pyenv/bin:$PATH"
    eval "$(pyenv init -)"
    eval "$(pyenv virtualenv-init -)"
    ```

- PyCharm: `sudo snap install pycharm-community --classic`

- 한글 설치

  - Settings -> Region & Language -> Add "Korean(101/104 key compatible)" -> Manage Installed Languages -> Update
  - `sudo apt install fcitx-hangul`
  - Settings -> Region & Language -> Manage Installed Languages -> Keyboard input method system: "fcitx"
  - 재부팅
  - 한/영키 등록: 오른쪽 위 키보드 아이콘 -> Configure -> "+" -> "Hangul" 추가 -> Global Config 탭 -> Trigger Input Method

- Naver Whale: <https://whale.naver.com/ko/download>



## Pyenv Setting

Pyenv를 이용한 파이썬 설치

`pyenv install 3.8.x` (latest)



### Setup Virtual Environment

```
pyenv virtualenv 3.8.x py38dl
mkdir -p ~/workspace/detlec
cd ~/workspace/detlec
pyenv local py38dl
pip install --upgrade pip

pip install numpy opencv-python scikit-learn matplotlib pydot
sudo apt install graphviz

pip install tensorflow==2.4

pip install torch==1.7.1+cu110 torchvision==0.8.2+cu110 \
torchaudio===0.7.2 -f https://download.pytorch.org/whl/torch_stable.html
```



### Test TF2 and Pytorch

- Open PyCharm at `~/workspace/detlec`

- Set pycharm interpreter at `~/.pyenv/versions/py38dl/bin/python`

- Test tensorflow by

  ```python
  import tensorflow as tf
  x = tf.random.uniform((2, 3, 4))
  print(tf.reduce_sum(x, axis=(1, 2)))
  
  ---------------------------
  Output:
  tf.Tensor([4.5362177 5.611371 ], shape=(2,), dtype=float32)
  ```

- Test pytorch by

  ```python
  import tensorflow as tf
  x = tf.random.uniform((2, 3, 4))
  print(tf.reduce_sum(x, axis=(1, 2)))
  
  ---------------------------
  Output:
  tensor([8.4671, 4.6608])
  ```



## Troubleshooting



### 우분투 반응 느림

- RTX 2070 이 장착된 랩탑에서 Ubuntu 20.04를 설치했더니 자꾸 반응이 느려지는 현상 발생  

- 키보드나 마우스 입력 후 3~4초 후에 반응함
- Ubuntu 18.04에서는 이상없음
- Ubuntu 18.04에서 20.04로 업그레이드를 해도 같은 현상 발생
- `software-properties-gtk`에서 그래픽 드라이버 버전을 460에서 450으로 수정했더니 다시 빨라짐
- CUDA 11.1까지는 450 버전에서 작동


