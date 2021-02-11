---
layout: post
title:  "[Det] Image Classifiers"
date:   2021-02-10 09:00:13
categories: 2021-1-detector
---



# Implement Image Classifiers

이번 포스트에서는 영상 분류 모델을 생성, 학습, 평가 코드 구현을 텐서플로와 파이토치 두 가지 프레임워크에서 배워보고자 한다.



## 1. Tensorflow2



### 1.1. Tensor Operation

텐서플로에서 모든 연산은 `tf.Tensor`라는 타입의 데이터로 진행된다. 텐서플로 뿐만 아니라 모든 파이썬 기반 딥러닝 프레임워크는 Numpy와 유사한 사용법을 가지므로 Numpy를 알고 있고 다차원 배열에 대한 개념만 확실하다면 사용상 어려움은 없다.  

`tf.Tensor`의 자세한 사용법은 이 튜토리얼에 자세히 나와있다. 그림과 함께 설명이 돼있어 이해하기 쉽다. Numpy에 아직 익숙하지 않은 사람은 정독이 필요하다.  

<https://www.tensorflow.org/guide/tensor>

다음은 Numpy 배열도 익숙하더라도 `tf.Tensor`를 사용하기 위해 알아야 할 내용을 간추린것이다.



#### immutable (불변 객체)

> All tensors are immutable like Python numbers and strings: you can never update the contents of a tensor, only create a new one.

`tf.Tensor`는 string 처럼 만들수만 있고 수정할 수는 없다. 파이썬에서 `word="hello"; word[2]='a'`  이런건 실행되지 않는다. 마찬가지로 어떤 텐서 객체가 있을 때 그 내부의 상태는 바꿀수 없고 그것을 수정한 **새로운** 텐서를 만들수밖에 없다.  

반면 Numpy 배열은 얼마든지 내부 원소들을 수정할 수 있다. 이것이 텐서와 Numpy의 가장 큰 차이점이다. 텐서는 내부 상태를 수정할 수 없으므로 연산을 통해서 원하는 상태를 가진 객체를 새로 만들어야 한다. 그래서 텐서 연산은 프로그래밍시 생각이 더 필요하고 코드도 조금 더 길어진다.



#### 생성 및 기본 속성 확인

```python
import tensorflow as tf
# create Tensor from List with specific type
tensor_a = tf.constant([[[1, 2, 3], [4, 5, 6]], [[7, 8, 9], [10, 11, 12]]], dtype=tf.int32)

print("Type of every element:", tensor_a.dtype)
print("Number of axes (=Rank):", tensor_a.ndim)
print("Shape of tensor:", tensor_a.shape)
print("Total number of elements: ", tf.size(tensor_a).numpy())
```

결과

```
Type of every element: <dtype: 'int32'>
Number of axes (=Rank): 3
Shape of tensor: (2, 2, 3)
Total number of elements:  tf.Tensor(12, shape=(), dtype=int32)
```



#### Numpy 변환

`tf.Tensor`와 `np.array` 사이의 변환은 다음과 같다.

```python
print("To numpy array:\n", x.numpy()[0])
print("Back to Tensor:", tf.convert_to_tensor(x.numpy())[0])
```

결과

```
To numpy array:
 [[1 2 3]
 [4 5 6]]
Back to Tensor: tf.Tensor(
[[1 2 3]
 [4 5 6]], shape=(2, 3), dtype=int32)
```



#### Shape 심화

텐서플로에는 Tensor의 shape을 확인하는 세 가지 방법이 있는데 1, 2번은 같고 3번은 타입과 성질이 다르다.

1. `some_tensor.shape` (TensorShape, static shape)
2. `some_tensor.get_shape()` (TensorShape, static shape)
3. `tf.shape(some_tensor)` (Tensor, dynamic shape)

기본 설정인 *eager mode*에서는 1~3 모두 결과가 같지만 `tf.function` 아래서는 결과가 달라질 수 있다.

```python
def print_tensor_shape(tensor, title):
    print(f"{title} 1) Tensor.shape:", tensor.shape, type(tensor.shape))
    print(f"{title} 2) Tensor.get_shape():", tensor.get_shape())
    print(f"{title} 3) tf.shape():", tf.shape(tensor))
    h, w = tensor[0, 0, 1], tensor[0, 0, 2]
    zeros = tf.zeros((h, w))
    print(f"{title} 4) Tensor.shape:", zeros.shape)
    print(f"{title} 5) Tensor.get_shape():", zeros.get_shape())
    print(f"{title} 6) tf.shape():", tf.shape(zeros))
    return tf.shape(zeros)

@tf.function
def print_tensor_shape_graph(tensor, title):
    return print_tensor_shape(tensor, title)

# Shape: The length (number of elements) of each of the axes of a tensor
print_tensor_shape(tensor_a, "eager")
shape6 = print_tensor_shape_graph(tensor_a, "graph")
print("graph 6-1) shape:", shape6)
shape6 = print_tensor_shape_graph(tensor_a, "graph")
shape6 = print_tensor_shape_graph(tensor_a, "graph")
```

결과

```
eager 1) Tensor.shape: (2, 2, 3) <class 'tensorflow.python.framework.tensor_shape.TensorShape'>
eager 2) Tensor.get_shape(): (2, 2, 3)
eager 3) tf.shape(): tf.Tensor([2 2 3], shape=(3,), dtype=int32)
eager 4) Tensor.shape: (2, 3)
eager 5) Tensor.get_shape(): (2, 3)
eager 6) tf.shape(): tf.Tensor([2 3], shape=(2,), dtype=int32)

graph 1) Tensor.shape: (2, 2, 3) <class 'tensorflow.python.framework.tensor_shape.TensorShape'>
graph 2) Tensor.get_shape(): (2, 2, 3)
graph 3) tf.shape(): Tensor("Shape:0", shape=(3,), dtype=int32)
graph 4) Tensor.shape: (None, None)
graph 5) Tensor.get_shape(): (None, None)
graph 6) tf.shape(): Tensor("Shape_1:0", shape=(2,), dtype=int32)
graph 6-1) shape: tf.Tensor([2 3], shape=(2,), dtype=int32)
```



결과를 보면 1)~3)은 eager 모드건 graph 모드(tf.function)건 결과가 같다. 4)~6)은 언제든 변할수 있는 텐서의 값을 이용해 `zeros`의 shape을 결정하므로 Tensor 객체가 생성된 시점이 아닌 연산이 실행된 시점에 결정된다.  

Eager 모드에서는 일반 파이썬 프로그램처럼 한줄씩 실행하므로 객체 생성과 동시에 연산을 수행한다. 그러므로 `zeros`의 shape을 바로 알 수 있다.  

반면 graph 모드에서는 미리 연산을 수행할 객체들을 미리 준비한 다음 연산을 실행하는데 `print()`가 실행되는 시점은 객체를 준비하는 시점이라서 shape이 None으로 나온다. (`print_tensor_shape_graph`을 세 번 실행하지만 출력은 한번씩 밖에 되지 않는다.)  

`print_tensor_shape_graph`의 return을 통해 `tf.function`을 벗어난 텐서는 실제 값을 가지게 되고 6-1) 처럼 정확한 shape이 나오는 것을 볼 수 있다. 이것은 `tf.shape()` 함수가 연산 당시의 **dynamic shape**을 출력하기 때문이다. 따라서 `tf.function`을 쓰면서 내부에서 값에 따라 크기가 달라지는 텐서를 만들때는 `tf.shape()` 함수를 쓰는게 낫다.  

- Tensor.get_shape(): <https://www.tensorflow.org/api_docs/python/tf/Tensor#get_shape>

- tf.shape(): <https://www.tensorflow.org/api_docs/python/tf/shape>



### 1.2. Image Classification



<https://www.tensorflow.org/tutorials/quickstart/beginner>



## 2. Pytorch



### 2.1. Tensor Operation



<https://pytorch.org/tutorials/beginner/blitz/tensor_tutorial.html#sphx-glr-beginner-blitz-tensor-tutorial-py>



### 2.2. Image Classification



<https://pytorch.org/tutorials/beginner/blitz/cifar10_tutorial.html#sphx-glr-beginner-blitz-cifar10-tutorial-py>









