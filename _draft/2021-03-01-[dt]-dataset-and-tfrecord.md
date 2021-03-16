---
layout: post
title:  "[Det] Dataset and Tfrecords"
date:   2021-03-01 09:00:13
categories: 2021-1-detector
---



## Why Tfrecord?

딥러닝 학습은 고속의 연산능력을 요구한다. 딥러닝 학습을 하려면 하드디스크에서 데이터를 불러와서 RAM에 올리고 CPU를 거쳐 GPU로 갈 것이다. 이 과정에서 가장 느린 장치가 하드디스크다. GPU, CPU 성능은 날로 발전하고 RAM은 용량이 늘어나면 속도도 올라갈 수 있는데 하드디스크에서 파일을 읽어오는게 느리면 GPU의 성능을 최대한으로 쓸 수 없다. 요즘은 SSD가 HDD를 대체해가고 있지만 여전히 10TB급의 대용량은 HDD의 영역이다. 갈수록 데이터셋도 대용량화되어 요즘은 TB급의 데이터셋도 있기 때문에 HDD를 써야하는 경우가 많다.  

텐서플로에서는 `tf.data.Dataset` 클래스 객체로 학습에 필요한 데이터를 받는다. `Dataset`이 원본 데이터셋 파일에서 데이터를 읽어온다면 디스크에 산재된 여러개의 파일을 찾고 읽어오는데 시간이 오래 걸릴 것이다. 하지만 데이터가 하나 혹은 소수의 파일에 학습에 필요한 세트단위로 잘 정리가 되어있다면 데이터를 읽어오는 속도가 크게 향상될 것이다.

Tfrecord는 텐서플로에서 제공하는 데이터 포맷이다. 데이터를 저장할 때는 여러가지 학습 데이터를 스텝단위로 잘 정리하여 소수의 파일에 저장할 수 있게 한다. 데이터를 읽어올 때는 다수의 개별 파일을 읽어오는 것보다 빠른 읽기 속도를 제공하고 추가적으로 앞으로 사용할 데이터를 미리 읽는다던가 다수의 스레드에서 읽는 등의 최적화도 적용할 수 있다. 그래서 텐서플로에서 대용량 학습데이터를 다루러면 먼저 Tfrecord 파일을 만들고 읽을 줄 알아야 한다.

#### 참고자료

https://www.tensorflow.org/tutorials/load_data/tfrecord
https://www.tensorflow.org/guide/data



## CIFAR-10 Example

먼저 간단한 cifar10 데이터셋으로 tfrecord를 만들고 읽어보고자 한다. 학습을 위한 한 세트마다 **image, class index (0~9), class name ('plane'~'truck')** 세 가지 요소를 포함할 것이다. 

### TfrecordWiter

cifar10으로 `{"cartegory": "car", "label": 1, "image": xxxx}` 형식 만들기



```python
class Serializer:
    def __call__(self, example_dict):
        features = self.convert_to_feature(example_dict)
        # wrap the data as TensorFlow Features.
        features = tf.train.Features(feature=features)
        # wrap again as a TensorFlow Example.
        example = tf.train.Example(features=features)
        # serialize the data.
        serialized = example.SerializeToString()
        return serialized

    def convert_to_feature(self, example_dict):
        features = dict()
        for key, value in example_dict.items():
            if value is None:
                continue
            elif isinstance(value, np.ndarray):
                features[key] = self._bytes_feature(value.tostring())
            elif isinstance(value, int):
                features[key] = self._int64_feature(value)
            elif isinstance(value, float):
                features[key] = self._int64_feature(value)
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







### TfrecordReader







## KITTI Example



### TfrecordWiter



tfr_config



### TfrecordReader







