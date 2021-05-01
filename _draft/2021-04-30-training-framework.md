---
layout: post
title:  "[Det] Training Framework"
date:   2021-04-30 09:00:13
categories: 2021-1-detector
---



##  1. Design Philosophy

이번 자료에서는 모델을 학습시키는 프레임워크에 대해 설명한다. 이 구조의 설계 의도는 코드들을 조립가능한 부품(component)처럼 만들어서 입출력 규격만 같으면 언제든 부품을 다른 부품으로 교체 가능한 구조로 만드는 것이다. 모든 구조 설계는 config.py 에 들어있고 프레임워크는 이에 따라 클래스들을 조립하여 학습 알고리즘을 완성한다. yaml 같은 텍스트로 설정을 저장하는 경우도 있지만 어차피 파이썬을 쓸거라면 파이썬 스크립트에 설정을 저장하는 것이 파이썬의 다양한 기능을 활용할 수 있어서 유리하다.  

### 1.1. config.py 

config.py에 들어있는 `Config`에는 이 프로젝트에서 변경할만한 모든 파라미터들이 들어있다. 파라미터들은 전역변수처럼 쓸수 있도록 static 변수로 선언되었으며 파라미터의 용도에 따라 서브 클래스로 나누어 그룹화를 하였다.  

- Config: 모든 파라미터를 담은 최상위 클래스
  - Paths: 주요 경로들을 담은 클래스
  - Datasets: 각 데이터셋 별로 서브 클래스를 만들어 데이터셋마다 다른 파라미터 값 저장
  - Tfrdata: tfrecord를 생성하는데 필요한 파라미터 저장
  - Model.Output: 모델의 출력에 관한 파라미터 저장
  - Model.Structure: 모델의 내부 구조에 대한 파라미터 저장
  - Train: 학습에 필요한 파라미터 저장



### 1.2. parameter_pool.py

파라미터 중에는 가능한 설정이 너무 다양하거나 파라미터 값 자체가 list, dict 등의 복잡한 구조를 가지고 있는 것들이 있다. 그런 경우 파라미터 값을 그때그때 써넣기 보다는 미리 여러가지 가능한 값들을 변수에 저장해놓고 그 중에 하나를 선택하여 쓰는 것이 편하고 config 관리도 쉽다. 그런데 모든 가능한 설정을 미리 써둔다면 config이 너무 길어진다. 그래서 파라미터 값 자체가 길거나 선택할 수 있는 옵션이 정해진 경우 parameter_pool.py 라는 파일을 만드어 그곳에서 미리 다 선언해놓고 config.py에서는 paramter pool에서 원하는 것만 가져다 쓴다. 아래는 현재까지 만든 paramter pool의 구조다.

- LossComb: 객체 검출기의 학습은 다양한 손실 함수를 조합하여 만든다. Dict에서 key는 사용할 손실 함수 종류고 value는 손실 함수 결과로 나온 값에 대한 가중치다. 모델 출력 채널마다 어떤 손실 함수들을 쓰고 어떤 가중치로 학습할지에 따라서 매우 다양한 설정이 나올 수 있으므로 parameter pool에 입력한다. 이는 추후 TrainingPlan에 사용된다.
- Anchor: anchor는 모델의 출력 객체의 표준 크기다. 현재는 `COCO_YOLOv3`를 쓰는데 YOLO v3에서 COCO 데이터셋으로부터 만든 anchor들이다. `Config.Tfrdata.set_anchors()` 함수에서 픽셀 단위의 anchor를 목표 데이터셋에 맞춰서 0~1 사이의 비율 anchor로 변환하여 `Config.Tfrdata.ANCHORS_RATIO`에 저장한다.
- TrainingPlan: 학습 과정에 대한 계획, 자세한 내용은 아래 section에서 설명한다.



## 2. Training Plan

모델 학습은 보통 여러 epoch에 걸쳐서하고 하나의 모델을 여러 데이터셋으로 학습하기도 한다. 모델은 같아도 epoch 별로 학습률(learning rate)이나 손실 함수 등의 설정이 달라질 수 있다. Epoch에 따라 다른 학습 설정을 미리 TrainingPlan에 저장해놓으면 epoch 별로 설정을 바꿔가며 학습을 여러번 실행하지 않고 한번의 실행으로 모델을 끝까지 학습시킬 수 있다. 다음은 parameter pool에 저장된 training plan의 예시다.

```python
    KITTI_SIMPLE = [
        ("kitti", 10, 0.0001, LossComb.STANDARD, True),
        ("kitti", 10, 0.00001, LossComb.STANDARD, True)
    ]
```

Training plan은 list of tuples로 만들어지는데 각 tuple의 의미는 다음과 같다.

> (학습 데이터셋, 동일한 학습 설정을 가진 epoch 수, 학습률, 손실 함수, checkpoint 저장 여부)

Parameter pool의 training plan 중 하나가 `cfg.Train.TRAINING_PLAN`에 저장되고 train_main.py에서는 이를 이용해 학습을 진행한다.

```python
def train_main():
    uf.set_gpu_configs()
    end_epoch = 0
    for dataset_name, epochs, learning_rate, loss_weights, model_save in cfg.Train.TRAINING_PLAN:
        end_epoch += epochs
        train_by_plan(dataset_name, end_epoch, learning_rate, loss_weights, model_save)
```



## 3. TrainVal

학습을 할 때는 학습(training) 데이터에서 나온 손실 함수를 줄이도록 모델 파라미터를 갱신한다. 보통 한 에폭의 학습이 끝나고 나면 소량의 검증(validation) 데이터를 통해 학습되지 않은 데이터에 대해서도 좋은 결과가 나오는지를 확인한다. 이를 통해 overfitting 여부를 판단하여 학습을 조기에 정지시킬수도 있다.  

train_val.py에는 학습과 검증 알고리즘이 클래스로 구현되어 있다. 둘 다 데이터셋에 대해 반복 연산을 하고 결과로 나온 로그 데이터를 저장하는 점은 공통이므로 `TrainValBase`라는 공통의 부모 클래스를 상속한다. 둘의 차이점은 학습 클래스에서는 optimizer를 통해 모델의 파라미터를 갱신한다는 것 뿐이다. 또한 학습과 검증 알고리즘은 각각 2가지의 클래스를 가지고 있는데 하나는 텐서플로의 Eager 모드를 사용하고 다른 하나는 Graph 모드를 사용한다. 현재 총 4가지의 학습/검증 클래스가 있는데 epoch 단위의 알고리즘은 공통이고 batch 단위의 알고리즘만 차이가 난다.  

모드 설정에 따라 `trainer_factory()`와 `validater_factory()`에서 해당하는 클래스 객체를 출력한다.



## 4. Model Factory

model_factory.py의 `ModelFactory` 클래스에서는 `Config.Model`의 설정에 따라 `tf.keras.Model` 객체를 만들어 리턴한다.

