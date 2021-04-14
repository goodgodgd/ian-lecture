---
layout: post
title:  "[Det] Dataset and Tfrecords"
date:   2021-03-08 09:00:13
categories: 2021-1-detector
---



## Why Tfrecord?

딥러닝 학습은 고속의 연산능력을 요구한다. 딥러닝 학습을 하려면 하드디스크에서 데이터를 불러와서 RAM에 올리고 CPU를 거쳐 GPU로 갈 것이다. 이 과정에서 가장 느린 장치가 하드디스크다. 하드디스크에서 파일을 읽어오는게 느리면 GPU의 성능을 최대한으로 쓸 수 없다. 요즘은 SSD가 HDD를 대체해가고 있지만 여전히 10TB급의 대용량은 HDD의 영역이다. 갈수록 데이터셋도 대용량화되어 요즘은 TB급의 데이터셋도 있기 때문에 HDD를 써야하는 경우가 많다.  

텐서플로에서는 `tf.data.Dataset` 클래스 객체로 학습에 필요한 데이터를 받는다. `Dataset`이 원본 데이터셋 파일에서 데이터를 읽어온다면 디스크에 산재된 여러개의 파일을 찾고 읽어오는데 시간이 오래 걸릴 것이다. 하지만 데이터가 하나 혹은 소수의 파일에 학습에 필요한 세트단위로 잘 정리가 되어있다면 데이터를 읽어오는 속도가 크게 향상될 것이다.

Tfrecord는 텐서플로에서 제공하는 데이터 포맷이다. 데이터를 저장할 때는 여러가지 학습 데이터를 스텝단위로 잘 정리하여 소수의 파일에 저장할 수 있게 한다. 데이터를 읽어올 때는 다수의 개별 파일을 읽어오는 것보다 빠른 읽기 속도를 제공하고 추가적으로 앞으로 사용할 데이터를 미리 읽는다던가 다수의 스레드에서 읽는 등의 최적화도 적용할 수 있다. 그래서 텐서플로에서 대용량 학습데이터를 다루러면 먼저 Tfrecord 파일을 만들고 읽을 줄 아는게 좋다.

#### 참고자료

<https://www.tensorflow.org/tutorials/load_data/tfrecord>
<https://www.tensorflow.org/guide/data>



## 1. CIFAR-10 Example

먼저 간단한 예시로 cifar10 데이터셋으로부터 tfrecord를 만들고 이를 읽어서 분류 모델을 학습시키고자 한다. 학습을 위한 한 세트마다 **image, class index (0~9), class name ('plane'~'truck')** 세 가지 요소를 포함할 것이다. 



### 1.1. TfrecordWriter

Tfrecord는 여러가지 크기와 종류의 데이터들을 dictionary에 담은 후 dictionary 전체 데이터를 serialize 하여 단순히 byte들의 나열로 저장한다. 문자열 key와 데이터가 dictionary로 묶여서 하나의 *Example*을 이루고 example들이 차례로 디스크에 저장된다. Example은 tfrecord 데이터를 저장하고 읽는 최소 단위다. 문자열, 정수, 실수(의 배열) 등 다양한 타입을 저장하고 읽을 수 있다. Tfrecord 파일을 만드는 과정은 다음과 같다.

1. 하나의 example에 필요한 데이터 불러오기 (numpy, int, float, list, str 등)
2. 데이터를 dictionary로 묶어 example 원본 만들기
3. example을 serialize 하기
4. serialized example을 파일에 쓰기
5. 모든 데이터에 대해 1~4를 반복



#### a) 코드 구조

가장 위에 보이는 `Config` 클래스는 설정 값들을 모아놓은 클래스다. 객체 없이 편리하게 쓸수 있도록 static 변수를 사용하였다. 전역적으로 관리해야 하는 모든 설정값들은 하나의 클래스로 모아서 관리하는 게 좋다. PC 마다 달라질 수 있는 경로, 모델이나 데이터 관련 상수나 파라미터 등을 모아서 관리하면 편리하게 설정을 바꿔서 테스트해 볼 수 있다.

메인 함수는 `write_cifar10_tfrecord()`이고 tfrecord를 만드는 중요한 과정은 `TfrSerializer` 클래스에 정리하였다. 파일명은 **cifar10_tfr_writer.py**로 지정하였다.

```python
import os
import tensorflow as tf
import numpy as np
import pickle
from glob import glob

class Config:
    RAW_DATA_PATH = "/home/ian/workspace/detlec/dataset/cifar-10-batches-py"
    TFRECORD_PATH = "/home/ian/workspace/detlec/dataset/tfrecords"
    CLASS_NAMES = ('plane', 'car', 'bird', 'cat', 'deer', 'dog', 'frog', 'horse', 'ship', 'truck')
    CIFAR_IMG_SHAPE = (32, 32, 3)

def write_cifar10_tfrecord():
    test_serializer()
    train_set, test_set = load_cifar10_dataset(Config.RAW_DATA_PATH, Config.CIFAR_IMG_SHAPE)
    make_tfrecord(train_set, "cifar10", "train", Config.CLASS_NAMES, Config.TFRECORD_PATH)
    make_tfrecord(test_set, "cifar10", "test", Config.CLASS_NAMES, Config.TFRECORD_PATH)

def test_serializer():
    pass

def load_cifar10_dataset(data_path, img_shape):
    pass

def read_data(file, img_shape):
    pass

def make_tfrecord(dataset, dataname, split, class_names, tfr_path):
    pass

def open_tfr_writer(writer, tfr_path, dataname, split, shard_index):
    pass

class TfrSerializer:
    def __call__(self, raw_example):
        features = self.convert_to_feature(raw_example)
        features = tf.train.Features(feature=features)
        tf_example = tf.train.Example(features=features)
        serialized = tf_example.SerializeToString()
        return serialized

    def convert_to_feature(self, raw_example):
        pass
    
    def _bytes_feature(value):
        pass

    def _float_feature(value):
        pass

    def _int64_feature(value):
        pass

if __name__ == "__main__":
    write_cifar10_tfrecord()
```

실제 개발시에는 Config과 TfrSerializer를 독립적인 파일에 만들어주는 것이 좋다.



#### b) 데이터 불러오기

이전에는 CIFAR-10 데이터를 텐서플로 API를 이용해 가져왔다면 이번에는 [홈페이지](https://www.cs.toronto.edu/~kriz/cifar.html)에서 직접 다운로드 받아보자. **CIFAR-10 python version**을 받으면 된다. 다운로드 받고 적당한 위치에 압축을 풀고 그 경로를 `Config.RAW_DATA_PATH`에 입력한다. 압축을 풀면 학습 데이터 파일 5개(data_batch_*)와 테스트 데이터 파일 1개(test_batch)가 있다. `glob`으로 학습 데이터 파일 리스트를 받아 하나씩 읽어 리스트에 이미지와 레이블을 쌓은 뒤 합친다.  

`read_data()` 내부에서는 `pickle` 패키지를 이용해 파일에서 데이터를 읽는다. CIFAR-10 데이터셋은 이미지를 *channel-first* 형식으로 저장했기 때문에 이를 일반적으로 많이 쓰이는 *channel-last* 형식으로 변환한다.

```python
def load_cifar10_dataset(data_path, img_shape):
    train_files = glob(os.path.join(data_path, "data_*"))
    train_labels = []
    train_images = []
    for file in train_files:
        labels, images = read_data(file, img_shape)
        train_labels += labels
        train_images.append(images)
    train_images = np.concatenate(train_images, axis=0)

    test_file = os.path.join(data_path, "test_batch")
    test_labels, test_images = read_data(test_file, img_shape)

    print("[load_cifar10_dataset] train image and label shape:", train_images.shape, len(train_labels))
    print("[load_cifar10_dataset] test image and label shape: ", test_images.shape, len(test_labels))
    return (train_images, train_labels), (test_images, test_labels)

def read_data(file, img_shape):
    with open(file, 'rb') as fr:
        data = pickle.load(fr, encoding='bytes')
        labels = data[b"labels"]    # list of category indices, [10000], int
        images = data[b"data"]      # numpy array, [10000, 3072(=32x32x3)], np.uint8
        # CIFAR dataset is encoded in channel-first format
        images = images.reshape((-1, img_shape[2], img_shape[0], img_shape[1]))
        # convert to back channel-last format
        images = np.transpose(images, (0, 2, 3, 1))
    return labels, images
```



#### c) 데이터 쓰기 loop

아래 코드는 데이터를 받아 이를 변환하여 출력 파일을 쓰는 과정을 보여준다. 레이블을 숫자와 문자 두 가지 다 저장하기 위해 숫자 레이블 `ys`를 이에 해당하는 문자열 배열로 변환한 `labels`를 만든다. (`labels = labels[ys]`)  

저장할 데이터에 대해 for loop을 돌면서 저장할 example을 만들고 example을 serialize 한 결과를 tfrecord 파일에 저장하면 된다. 여기서 serialize 하는 과정이 중요한데 그건 다음에 `TfrSerializer` 클래스에서 설명한다.    

작은 데이터셋이기 때문에 하나의 파일에 다 담아도 되지만 대량의 데이터셋의 경우 하나의 파일에 다 담으면 파일이 너무 커지고 파일을 나누는게 읽는 속도도 빠르기 때문에 여러 파일에 나눠서 저장하는 예제를 만들었다. Tfrecord에서 파일을 나누는 단위를 **shard**(조각)라고 한다. 여기서는 `open_tfr_writer()` 함수에서 파일에서 shard를 구분하여 새 파일을 만들어준다. Example 10,000개마다 기존 shard 파일을 닫고 새로운 shard 파일을 연다. 

```python
def make_tfrecord(dataset, dataname, split, class_names, tfr_path):
    xs, ys = dataset
    labels = np.array(class_names)
    labels = labels[ys]
    writer = None
    serializer = TfrSerializer()
    examples_per_shard = 10000

    for i, (x, y, label) in enumerate(zip(xs, ys, labels)):
        if i % examples_per_shard == 0:
            writer = open_tfr_writer(writer, tfr_path, dataname, split, i//examples_per_shard)

        example = {"image": x, "label_index": y, "label_name": label}
        serialized = serializer(example)
        writer.write(serialized)
    writer.close()

def open_tfr_writer(writer, tfr_path, dataname, split, shard_index):
    if writer:
        writer.close()

    tfrdata_path = os.path.join(tfr_path, f"{dataname}_{split}")
    if os.path.isdir(tfr_path) and not os.path.isdir(tfrdata_path):
        os.makedirs(tfrdata_path)
    tfrfile = os.path.join(tfrdata_path, f"shard_{shard_index:03d}.tfrecord")
    writer = tf.io.TFRecordWriter(tfrfile)
    print(f"create tfrecord file: {tfrfile}")
    return writer
```



#### d) TfrSerializer

Tfrecord를 만드는 과정에서 가장 중요한 과정이 일반적인 자료 형식을 serialized bytes로 변환하는 과정이다. 중요하긴 하지만 아래 클래스를 쓰면 대부분의 경우 변환이 가능하므로 한 번 만들면 수정할 일이 별로 없다.  

Dictionary로 입력이 들어오면 `convert_to_feature()`에서 value로 저장된 내부 데이터를 하나씩 `tf.train.Feature` 타입으로 변환해준다. 그 뒤는 dict 전체를 `tf.train.Features`, `tf.train.Example`로 타입 변환한다. `tf.train.Example`이 만들어지면 `SerializeToString()` 함수를 통해 파일에 데이터 구조 전체를 bytes 데이터로 변환한다.

```python
class TfrSerializer:
    def __call__(self, raw_example):
        features = self.convert_to_feature(raw_example)
        # wrap the data as TensorFlow Features.
        features = tf.train.Features(feature=features)
        # wrap again as a TensorFlow Example.
        tf_example = tf.train.Example(features=features)
        # serialize the data.
        serialized = tf_example.SerializeToString()
        return serialized
```



첫 번째 단계인 `convert_to_feature()`가 중요한데 입력 dictionary에 들어있는 value들을 데이터 타입에 따라 다른 방식으로 `tf.train.Feature` 타입으로 변환해준다. 타입에 따른 변환 과정은 다음과 같다.

- `tf.train.FloatList`: float 하나 또는 배열을 담아준다.  
- `tf.train.Int64List`: int 하나 또는 배열을 담아준다.  
- `tf.train.BytesList`: float이나 int가 아닌 나머지 데이터를 bytes로 저장한다.  
  - `str`: 문자열은 `bytes(str, encoding)` 함수로 bytes 타입으로 변환할 수 있다. 인코딩은 `utf-8`이 기본적으로 쓰이는데 `euc-kr`처럼 다른 인코딩을 쓴다면 디코딩 할때도 같은 인코딩 타입을 입력해야 한다. 
  - `np.ndarray`: 일반적으로 이미지 등의 중요한 데이터는 처음에 numpy 배열로 불러오게 된다. 이걸 `FloatList`나 `Int64List`로 저장할 수도 있지만 1 byte 이미지 한 픽셀을 8 byte로 저장하는 것은 대단한 낭비다. 다행히 numpy는 배열의 raw bytes에 접근할 수 있는 `tobytes()` 함수를 제공한다. 이를 이용해 효율적으로 디스크에 저장하고 나중에 데이터 타입과 형태를 맞춰서 배열을 복원하면 된다.



```python
    def convert_to_feature(self, raw_example):
        features = dict()
        for key, value in raw_example.items():
            if value is None:
                continue
            elif isinstance(value, np.ndarray):
                # method 1: encode into raw bytes
                # fast but losing shape, 2 seconds to make training dataset
                value = value.tobytes()
                # method 2: encode into png format
                # slow but keeping shape, 10 seconds to make training dataset
                # value = tf.io.encode_png(value)
                # BytesList won't unpack a tf.string from an EagerTensor.
                # value = value.numpy()
                features[key] = self._bytes_feature(value)
            elif isinstance(value, str):
                value = bytes(value, 'utf-8')
                features[key] = self._bytes_feature(value)
            elif isinstance(value, int):
                features[key] = self._int64_feature(value)
            elif isinstance(value, float):
                features[key] = self._float_feature(value)
            else:
                assert 0, f"[convert_to_feature] Wrong data type: {type(value)}"
        return features

    @staticmethod
    def _bytes_feature(value):
        """Returns a bytes_list from a string / byte."""
        return tf.train.Feature(bytes_list=tf.train.BytesList(value=[value]))
    
    @staticmethod
    def _float_feature(value):
        """Returns a float_list from a float / double."""
        return tf.train.Feature(float_list=tf.train.FloatList(value=[value]))

    @staticmethod
    def _int64_feature(value):
        """Returns an int64_list from a bool / enum / int / uint."""
        return tf.train.Feature(int64_list=tf.train.Int64List(value=[value]))
```



위 변환과정을 확인할 수 있는 함수를 만들어 보았다. 결과가 너무 길어서 다 담지 못하니 직접 실행해보기 바란다.

```python
def test_serializer():
    example = {"name": "car", "int": 10, "float": 1.1, "np": np.array([1, 2, 3]).astype(np.uint8)}
    features = TfrSerializer().convert_to_feature(example)
    print("=== dict of tf.train.Feature:\n", features)
    features = tf.train.Features(feature=features)
    print("=== tf.train.Features:\n", features)
    tf_example = tf.train.Example(features=features)
    print("=== tf.train.Example\n", tf_example)
    serialized = tf_example.SerializeToString()
    print("=== serialized\n", serialized)
    print("")
```

결과

```
=== dict of tf.train.Feature:
{'name': bytes_list {
  value: "car"
}
, 'int': int64_list {
  value: 10
}
, 'float': float_list {
  value: 1.100000023841858
}
, 'np': bytes_list {
  value: "\001\002\003"
}
}
=== tf.train.Features:
 feature {
  key: "float"
  value {
    float_list {
      value: 1.100000023841858
...
=== tf.train.Example
 features {
  feature {
    key: "float"
    value {
      float_list {
        value: 1.100000023841858
...
=== serialized
b'\nA\n\x0f\n\x04name\x12\x07\n\x05\n\x03car\n\x0c\n\x03int\x12\x05\x1a\x03\n\x01\n\n\x11\n\x05float\x12\x08\x12\x06\n\x04\xcd\xcc\x8c?\n\r\n\x02np\x12\x07\n\x05\n\x03\x01\x02\x03'

```



### 1.2. TfrecordReader

**cifar10_tfr_writer.py**으로 tfrecord 파일을 저장했다면 이제 이를 읽어서 분류 모델 학습에 적용해보자. 먼저 tfrecord 파일을 읽어들이는 `tf.data.Dataset` 객체를 만든다. Tfrecord 파일은 raw bytes로 저장되어 있기 때문에 저장된 데이터의 타입을 미리 알고있어야 이를 원래대로 복구할 수 있다. 데이터의 타입과 형태(shape)을 복구하고 batch, epoch, shuffle 등의 설정을 추가하여 학습 또는 테스트에 사용할 Dataset 객체를 만든다.  

Dataset 객체를 만들고 나면 나머지 과정은 지난번 했던 *Advanced Model*과 거의 유사하다.  



#### a) 코드 구조

위 설명처럼 메인 함수인 `read_cifar10_tfrecord()`은 먼저 학습 데이터셋과 테스트 데이터셋 객체를 만들고 이후 `AdvancedClassifier`를 이용해 학습과 평가를 진행한다. 파일명은 **cifar10_tfr_reader.py**로 지정하였다.

```python
import os.path as op
import tensorflow as tf
import numpy as np
import matplotlib.pyplot as plt

class Config:
    RAW_DATA_PATH = "/home/ian/workspace/detlec/dataset/cifar-10-batches-py"
    TFRECORD_PATH = "/home/ian/workspace/detlec/dataset/tfrecords"
    CLASS_NAMES = ('plane', 'car', 'bird', 'cat', 'deer', 'dog', 'frog', 'horse', 'ship', 'truck')
    CIFAR_IMG_SHAPE = (32, 32, 3)
    BATCH_SIZE = 32

def read_cifar10_tfrecord():
    gpu_config()
    train_dataset = get_dataset(Config.TFRECORD_PATH, "cifar10", "train", True, Config.BATCH_SIZE)
    test_dataset = get_dataset(Config.TFRECORD_PATH, "cifar10", "test", False, Config.BATCH_SIZE)
    check_data(train_dataset)
    classifier = AdvancedClassifier(Config.BATCH_SIZE)
    classifier.build_model(Config.CIFAR_IMG_SHAPE, len(Config.CLASS_NAMES))
    classifier.train(train_dataset, test_dataset, 5)
    classifier.evaluate(test_dataset)

def get_dataset(tfr_path, dataname, split, shuffle=False, batch_size=32, epochs=1):
    pass

def parse_example(example):
    pass

def set_properties(dataset, shuffle, epochs, batch_size):
    pass

def check_data(dataset):
    pass

def show_samples(images, labels, grid=(3, 3)):
    pass

"""
classfier model
"""
from tensorflow import keras
from tensorflow.keras import layers
from timeit import default_timer as timer

class DurationTime:
    pass	# same as tf_classifier_adv.py

def gpu_config():
    pass	# same as tf_classifier_adv.py

class AdvancedClassifier:
    def __init__(self, batch_size=32):
        pass	# same as tf_classifier_adv.py

    def build_model(self, input_shape, output_shape):
        pass	# same as tf_classifier_adv.py

    def train(self, train_dataset, val_dataset, epochs):
        pass

    @tf.function
    def train_batch_graph(self, x_batch, y_batch):
        pass	# same as tf_classifier_adv.py

    def evaluate(self, dataset, verbose=True):
        pass

if __name__ == "__main__":
    read_cifar10_tfrecord()
```

여기서는 예제로 보여주기 위해  하나의 파일로 만들었지만 Config과 AdvancedClassifier 독립적인 파일로 만드는게 좋다.



#### b) 데이터셋 불러오기

데이터셋 객체를 만드는 과정은 다음과 같고 `get_dataset()` 함수로 이를 구현하였다.

1. Tfrecord 파일 목록으로부터 기본 Dataset 객체 만들기
2. `map()` 함수 이용하여 Dataset 객체에서 데이터를 읽어올 때 전처리 과정 적용
3. batch, epoch, shuffle 등의 데이터셋 속성 추가

```python
def get_dataset(tfr_path, dataname, split, shuffle=False, batch_size=32, epochs=1):
    tfr_files = tf.io.gfile.glob(op.join(tfr_path, f"{dataname}_{split}", "*.tfrecord"))
    tfr_files.sort()
    print("[TfrecordReader] tfr files:", tfr_files)
    dataset = tf.data.TFRecordDataset(tfr_files)
    dataset = dataset.map(parse_example)
    dataset = set_properties(dataset, shuffle, epochs, batch_size)
    return dataset
```



`parse_example()`은 파일에 저장된 raw bytes를 example 단위로 읽어들이고 `tf.io.parse_single_example()` 함수를 통해 원하는 타입으로 변환한다. 이때 원하는 타입은 `tf.io.FixedLenFeature`의 인자로 넣어주고 이를 데이터를 만들때 입력한 key와 짝을 맞춰 dictionary 변수(features)로 입력한다. 이렇게 하면 `example`의 value들이 원래 타입으로 변환되어 어느정도 볼 수 있는 데이터가 된다.  

다만 numpy array의 경우에는 string에서 원래 배열의 타입과 형태로 복원해야 하기 때문에 추가적인 작업이 더필요하다. 이미지는 원래 uint8 타입이었기 때문에 `tf.io.decode_raw()` 함수로 `tf.uint8` 타입으로 변환 후, `tf.reshape()`까지 적용하면 원래의 (32, 32, 3)의 형태를 복원하게 된다.  

추가적으로 Convolution등의 연산에 입력할 때는 0~1의 normalized image가 적합하기 때문에 `tf.image.convert_image_dtype()` 함수를 적용해 0~255 사이의 정수 타입 이미지를 0~1 사이의 실수(float) 타입으로 변환한다.  `image`라는 key는 float 이미지를 위해 사용하고 이미지 시각화를 위해 원본 이미지를 따로 `image_u8`이라는 key에 저장한다.

```python
def parse_example(example):
    features = {
        "image": tf.io.FixedLenFeature(shape=(), dtype=tf.string, default_value=""),
        "label_index": tf.io.FixedLenFeature(shape=(), dtype=tf.int64, default_value=0),
        "label_name": tf.io.FixedLenFeature(shape=(), dtype=tf.string, default_value=""),
    }
    parsed = tf.io.parse_single_example(example, features)
    # method 1. decode from raw bytes
    parsed["image"] = tf.io.decode_raw(parsed["image"], tf.uint8)
    # only for visualization
    parsed["image_u8"] = tf.reshape(parsed["image"], Config.CIFAR_IMG_SHAPE)
    parsed["image"] = tf.image.convert_image_dtype(parsed["image_u8"], dtype=tf.float32)     # for model input
    # method 2. decode from png format
    # parsed["image"] = tf.io.decode_png(parsed["image"])
    return parsed
```



`set_properties()` 함수에서는 Dataset 객체의 속성을 지정한다.

- shuffle: 다수의 데이터를 미리 읽어들여서 순서를 섞어 출력하게 한다. 입력인자로 넣은 숫자(100)은 미리 읽어들여 버퍼에 저장하는 데이터의 양이다. 너무 작게 하면 shuffling의 효과가 적고 이를 너무 크게 하면 메인 메모리가 꽉 차버릴 수 있으니 데이터의 용량에 따라 적당히 조절해야 한다.
- batch: 한번에 출력한 example의 개수를 지정한다. 딥러닝에서는 보통 mini batch 단위로 여러개의 example을 한번에 학습하기 때문에 batch size를 지정한다. 보통 학습 속도를 올리기 위해 GPU 메모리가 허락하는 한 batch size를 크게 잡는데 batch size가 작을때 오히려 모델의 성능이 좋다는 연구결과도 있다.  
- repeat: 데이터셋 객체로 for loop을 진행할 때 전체 데이터를 몇 번씩 사용할 것인지 epoch 수를 결정한다.  

```python
def set_properties(dataset, shuffle: bool, epochs: int, batch_size: int):
    if shuffle:
        dataset = dataset.shuffle(100)
    dataset = dataset.batch(batch_size).repeat(epochs)
    return dataset
```



#### c) 데이터 확인하기

데이터셋 객체로 불러오는 데이터가 의도한대로 나오는지 확인해본다. `dataset`에 대해 for loop을 돌면서 각 데이터의 shape과 값들을 확인해본다. `show_samples()` 함수를 통해 이미지와 레이블의 짝이 잘 맞는지도 확인한다.

```python
def check_data(dataset):
    for i, features in enumerate(dataset):
        print("sample:", i, features["image"].shape,
              features["label_index"][:8].numpy(), features["label_name"][:8].numpy())
        if i == 0:
            show_samples(features["image_u8"], features["label_name"])
        if i > 5:
            break

def show_samples(images, labels, grid=(3, 3)):
    plt.figure(figsize=grid)
    num_samples = grid[0] * grid[1]
    for i in range(num_samples):
        plt.subplot(grid[0], grid[1], i+1)
        plt.xticks([])
        plt.yticks([])
        plt.grid(False)
        plt.imshow(images[i].numpy())
        plt.xlabel(labels[i].numpy().decode())
    plt.show()
```

실행 결과

```
sample: 0 (32, 32, 32, 3) [8 8 4 0 0 4 9 4] [b'ship' b'ship' b'deer' b'plane' b'plane' b'deer' b'truck' b'deer']
sample: 1 (32, 32, 32, 3) [0 2 7 3 1 8 9 6] [b'plane' b'bird' b'horse' b'cat' b'car' b'ship' b'truck' b'frog']
sample: 2 (32, 32, 32, 3) [6 6 0 5 6 4 5 4] [b'frog' b'frog' b'plane' b'dog' b'frog' b'deer' b'dog' b'deer']
sample: 3 (32, 32, 32, 3) [7 9 2 3 4 1 1 6] [b'horse' b'truck' b'bird' b'cat' b'deer' b'car' b'car' b'frog']
sample: 4 (32, 32, 32, 3) [7 6 7 3 4 9 4 8] [b'horse' b'frog' b'horse' b'cat' b'deer' b'truck' b'deer' b'ship']
sample: 5 (32, 32, 32, 3) [0 9 4 5 0 6 4 0] [b'plane' b'truck' b'deer' b'dog' b'plane' b'frog' b'deer' b'plane']
sample: 6 (32, 32, 32, 3) [3 5 1 1 9 1 2 9] [b'cat' b'dog' b'car' b'car' b'truck' b'car' b'bird' b'truck']
```



#### d) 모델 학습하기

모델에 관련된 코드는 이전 강의에서 사용한 **tf_classifier_adv.py**와 거의 유사하다. 데이터셋 객체를 numpy 배열로부터 만드느냐 파일로부터 만드느냐의 차이가 있을 뿐이다. 여기서는 이전 강의와 달라진 함수만 설명한다. 

`train()` 함수에서 달라진 점은 학습 데이터를 쪼개서 validation 데이터를 만들수 없다는 것이다. 원본 데이터가 아니라 이미 Dataset 객체로 포장이 돼서 입력으로 들어오기 때문에 이를 쪼갤 수 없다. 그래서 메인 함수를 보면 아래와 같이 test 데이터를 validation을 위해 사용했다. 엄밀하게 말하면 test 데이터와 validation 데이터는 구분되어야 하고 그렇게 하기 위해서는 validation을 위한 tfrecord 파일을 만들어야 하지만 단순한 예시를 위해 validation 데이터는 생략하였다.  

> classifier.train(train_dataset, test_dataset, 5)

```python
    def train(self, train_dataset, val_dataset, epochs):
        with DurationTime("** training time"):
            for epoch in range(epochs):
                for i, features in enumerate(train_dataset):
                    self.train_batch_graph(features["image"], features["label_index"])
                loss, accuracy = self.evaluate(val_dataset, verbose=False)
                print(f"[Training] epoch={epoch}, val_loss={loss:1.4f}, val_accuracy={accuracy:1.4f}")
```



#### e) 모델 평가하기

`evaluate()` 함수에서는 학습할 때와 마찬가지로 Dataset 객체에 대해 for문을 돌면서 batch 단위로 모델 출력을 만들고 이를 쌓아서 평가에 사용한다. Accuracy와 loss를 계산하는 과정은 **tf_classifier_adv.py**와 거의 동일하다.

```python
    def evaluate(self, dataset, verbose=True):
        y_true, y_pred = [], []
        for features in dataset:
            probs = self.model(features["image"])
            y_pred.append(probs)
            y_true.append(features["label_index"])
        y_pred = tf.concat(y_pred, axis=0).numpy()
        y_true = tf.concat(y_true, axis=0).numpy()
        accuracy = np.mean(np.argmax(y_pred, axis=1) == y_true)
        loss = self.loss_object(y_true, y_pred).numpy()
        if verbose:
            print("=== evaluate result ===")
            np.set_printoptions(precision=4, suppress=True)
            print("  prediction shape:", y_pred.shape, y_true.shape)
            print("  pred indices", np.argmax(y_pred, axis=1)[:20])
            print("  true indices", y_true[:20])
            print(f"  loss={loss:1.4f}, accuracy={accuracy:1.4f}")
        return loss, accuracy
```

실행 결과

```
[Training] epoch=0, val_loss=1.1968, val_accuracy=0.5800
[Training] epoch=1, val_loss=0.9574, val_accuracy=0.6663
[Training] epoch=2, val_loss=0.9745, val_accuracy=0.6684
[Training] epoch=3, val_loss=0.9576, val_accuracy=0.6842
[Training] epoch=4, val_loss=0.8841, val_accuracy=0.6993
** training time: 18.85
=== evaluate result ===
  prediction shape: (10000, 10) (10000,)
  pred indices [3 8 8 8 6 6 1 6 3 1 0 9 6 7 9 8 5 7 8 6]
  true indices [3 8 8 0 6 6 1 6 3 1 0 9 5 7 9 8 5 7 8 6]
  loss=0.8841, accuracy=0.6993
```


