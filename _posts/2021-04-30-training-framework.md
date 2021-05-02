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

def train_by_plan(dataset_name, end_epoch, learning_rate, loss_weights, model_save):
    batch_size, train_mode = cfg.Train.BATCH_SIZE, cfg.Train.MODE
    tfrd_path, ckpt_path = cfg.Paths.TFRECORD, op.join(cfg.Paths.CHECK_POINT, cfg.Train.CKPT_NAME)
    start_epoch = read_previous_epoch(ckpt_path)
    if end_epoch <= start_epoch:
        print(f"!! end_epoch {end_epoch} <= start_epoch {start_epoch}, no need to train")
        return

    dataset_train, train_steps, imshape, anchors_per_scale = \
        get_dataset(tfrd_path, dataset_name, False, batch_size, "train")
    dataset_val, val_steps, _, _ = get_dataset(tfrd_path, dataset_name, False, batch_size, "val")

    model, loss, optimizer = create_training_parts(batch_size, imshape, anchors_per_scale, ckpt_path,
                                                   learning_rate, loss_weights, dataset_name)

    trainer = tv.trainer_factory(train_mode, model, loss, optimizer, train_steps)
    validater = tv.validater_factory(train_mode, model, loss, val_steps)
    logger = Logger()

    for epoch in range(start_epoch, end_epoch):
        print(f"========== Start dataset : {dataset_name} epoch: {epoch + 1}/{end_epoch} ==========")
        train_result = trainer.run_epoch(dataset_train)
        val_result = validater.run_epoch(dataset_val)
        save_model_ckpt(ckpt_path, model)
        logger.save_log(epoch, train_result, val_result)

    if model_save:
        save_model_ckpt(ckpt_path, model, f"ep{end_epoch:02d}")
```

`train_by_plan()`은 plan의 한 period를 처리하는데 처리 과정은 다음과 같다.

> 참고. 지질시대 구분 단위
>
> epoch: '세(世)' 단위, e.g. 홀로세  
>
> period: '기(期)' 단위, e.g. 제4기, 쥐라기
>
> era: '대(代)' 단위, e.g. 중생대, 신생대

1. `read_previous_epoch()`: 학습은 오랜 시간이 걸리므로 중간에 중단됐다 다시 시작할 수도 있다. 학습을 재개할 때 저장된 모델이 어디까지 학습한 모델인지를 로깅(logging) 정보를 통해 알아내서 `start_epoch`을 정하고 이것이 현재 period의 `end_epoch` 보다 크면 이미 학습이 완료된 plan이기 때문에 넘어간다.
2. `get_dataset()`: 학습과 검증 데이터를 위한 tfrecord 파일이 저장된 경로를 입력하여 `Dataset` 객체를 만든다. tfr_config.txt 파일에서 데이터의 다양한 속성도 함께 리턴한다. 데이터셋을 전부 반복하기 위한 스텝 수, 입력 이미지의 shape, 스케일 별 anchor 크기
3. `create_training_parts()`: 학습에 필요한 model, loss, optimizer를 생성한다. Model과 loss는 config에 관련된 파라미터들이 있으므로 각각 `ModelFactory`와 `LossFactory`에서 파라미터에 맞춰 model과 loss 객체를 생성한다. Model 객체를 만들고 나서 기존에 학습된 모델이 저장되어 있다면 저장된 모델에서 모델 파라미터를 불러온다. Optimizer는 현재까지는 학습률 외에 다른 파라미터가 없으므로 Adam optimizer를 바로 생성하여 사용한다.
4. `trainer/validater_factory()`: 학습과 검증을 수행할 객체를 만든다. 자세한 내용은 아래 섹션 참조
5. `trainer/validater.run_epoch()`: 앞서 만들어진 객체들을 이용해 한 epoch의 학습과 검증을 순차적으로 진행한다.
6. `save_model_ckpt()`: 매 epoch 마다 모델을 `latest`라는 접미사가 붙은 파일명으로 저장한다. 모델 저장은 모델 파라미터만 저장할 수도 있고 모델 구조까지 한 번에 저장할 수도 있는데 여기서는 파라미터만 저장하는 방법을 사용한다. (`model.save_weights(ckpt_file)`) 하나의 period가 끝날때의 모델은 다음에 재사용될 수 있으므로 epoch 숫자를 붙여 다른 이름으로 저장한다.



## 3. TrainVal

학습을 할 때는 학습(training) 데이터에서 나온 손실 함수를 줄이도록 모델 파라미터를 갱신한다. 보통 한 에폭의 학습이 끝나고 나면 소량의 검증(validation) 데이터를 통해 학습되지 않은 데이터에 대해서도 좋은 결과가 나오는지를 확인한다. 이를 통해 overfitting 여부를 판단하여 학습을 조기에 정지시킬수도 있다.  

train_val.py에는 학습과 검증 알고리즘이 클래스로 구현되어 있다. 둘 다 데이터셋에 대해 반복 연산을 하고 결과로 나온 로그 데이터를 저장하는 점은 공통이므로 `TrainValBase`라는 공통의 부모 클래스를 상속한다. 둘의 차이점은 학습 클래스에서는 optimizer를 통해 모델의 파라미터를 갱신한다는 것 뿐이다. 또한 학습과 검증 알고리즘은 각각 2가지의 클래스를 가지고 있는데 하나는 텐서플로의 Eager 모드를 사용하고 다른 하나는 Graph 모드를 사용한다. 현재 총 4가지의 학습/검증 클래스가 있는데 epoch 단위의 알고리즘은 공통이고 batch 단위의 알고리즘만 차이가 난다.  

모드 설정에 따라 `trainer_factory()`와 `validater_factory()`에서 해당하는 클래스 객체를 출력한다.  

`TrainValBase`에는 모든 자식 클래스에서 공유하는 epoch 단위의 처리과정이 정의되어있다. 자식 클래스에서 달라지는 부분은 `run_batch()` 함수고 그 전후로 로깅에 관련된 내용은 공통이다.  

```python
    def run_epoch(self, dataset):
        epoch_start = time.time()
        self.model_log.clear()
        for step, features in enumerate(dataset):
            start = time.time()
            outputs = self.run_batch(features)
            self.model_log.append_batch_result(outputs)
            uf.print_progress(f"training {step}/{self.epoch_steps} steps, "
                              f"time={time.time() - start:.2f}... ")
            if step > 20:
                break

        print("")
        self.model_log.append_epoch_result(time=time.time() - epoch_start)
        return self.model_log
```



## 4. Model Factory

model_factory.py의 `ModelFactory` 클래스에서는 `Config.Model`의 설정에 따라 `tf.keras.Model` 객체를 만들어 리턴한다. 생성자 함수에서는 모델에 대한 각종 설정 파라미터를 입력인자로 받는데 대부분의 인자들의 기본 값을 config 파라미터로 지정한다. 그리고 입력 인자로 받은 파라미터들을 멤버 변수로 저장한다. 이러한 패턴의 의도는 `ModelFactory`를 config에 덜 의존적인 클래스로 만드는 것이다. Config의 파라미터는 다양한 레벨에서 사용할 수 있다. 최상위 main 함수에서부터 파라미터가 실제 사용되는 곳까지 입력 인자로 전달할 수도 있고, 실제 사용되는 곳에서 바로 import 해서 쓸 수도 있고, 그 중간에서 사용할 수도 있다. Config을 너무 상위 레벨에서 사용하면 전달하는 입력인자가 너무 많아지게 되고, 너무 하위 레벨에서 쓰면 모든 스크립트에서 config.py를 import 하게 돼서 어떤 클래스나 함수를 다른 프로젝트에서 재사용 할 때마다 config까지 같이 가져가거나 일일이 찾아서 수정해야 하는 불편함이 있다.

그래서 여기서는 어떤 객체를 생성하는 팩토리에서 config 파라미터를 생성자의 기본 값으로 받게 하였다. 만약 다른 프로젝트에서 이 코드를 재사용한다면 기본 값을 config 대신 다른 값을 넣거나 아니면 기본 값을 아예 안쓰고 객체를 생성할 때 모든 입력 인자를 직접 입력하면 된다. 필수 입력 인자 수는 늘리지 않으면서 다른 프로젝트에 이식할 때 config 설정을 여기저기 찾는 수고를 덜 수 있다.

```python
class ModelFactory:
    def __init__(self, batch_size, input_shape, anchors_per_scale,
                 backbone_name=cfg.Model.Structure.BACKBONE,
                 head_name=cfg.Model.Structure.HEAD,
                 backbone_conv_args=cfg.Model.Structure.BACKBONE_CONV_ARGS,
                 head_conv_args=cfg.Model.Structure.HEAD_CONV_ARGS,
                 num_anchors_per_scale=cfg.Model.Output.NUM_ANCHORS_PER_SCALE,
                 out_channels=cfg.Model.Output.OUT_CHANNELS,
                 output_compos=cfg.Model.Output.OUT_COMPOSITION
                 ):
        self.batch_size = batch_size
        self.input_shape = input_shape
        self.anchors_per_scale = anchors_per_scale
        self.backbone_name = backbone_name
        self.head_name = head_name
        self.backbone_conv_args = backbone_conv_args
        self.head_conv_args = head_conv_args
        self.num_anchors_per_scale = num_anchors_per_scale
        self.out_channels = out_channels
        self.output_compos = output_compos
        mu.CustomConv2D.CALL_COUNT = -1
        print(f"[ModelFactory] batch size={batch_size}, input shape={input_shape}")
        print(f"[ModelFactory] backbone={self.backbone_name}, HEAD={self.head_name}")
```

모델을 만드는 과정은 `get_model()`에 나타나 있다. 모델을 이루는 backbone과 head 또한 팩토리로 생성한다. 입력 이미지에 해당하는 `input_tensor`를 만들고 이것이 backbone, head, decoder를 순차적으로 통과하여 모델의 출력인 `output_features`를 만든다. 이렇게 입력으로부터 최종 출력까지 가는 그래프를 만들고 입력부터 출력까지의 그래프를 `tf.keras.Model`로 묶으면 된다.  

입력에서부터 convolution 등의 텐서 연산 과정이 코드에서 명시적인 그래프 객체에 쌓이는 것은 아니지만 텐서플로 패키지에의 전역변수로 그래프 모델을 관리하고 있다. 텐서플로 1.x에서는 그래프 변수를 직접 관리할 수도 있었으나 사용법이 너무 헷갈려서 2.x에서는 텐서플로에서 알아서 관리하게 됐다.

```python
    def get_model(self):
        backbone_model = back.backbone_factory(self.backbone_name, self.backbone_conv_args)
        head_model = head.head_factory(self.head_name, self.head_conv_args, self.num_anchors_per_scale, self.out_channels)
        input_tensor = tf.keras.layers.Input(shape=self.input_shape, batch_size=self.batch_size)
        backbone_features = backbone_model(input_tensor)
        head_features = head_model(backbone_features)
        output_features = dict()
        decode_features = head.FeatureDecoder(self.output_compos, self.anchors_per_scale)
        for i, (scale, feature) in enumerate(head_features.items()):
            output_features[scale] = decode_features(feature, scale)
        yolo_model = tf.keras.Model(inputs=input_tensor, outputs=output_features, name="yolo_model")
        return yolo_model
```







