---
layout: post
title:  "[Det] NMS and Visual Log"
date:   2021-06-10 09:00:13
categories: 2021-1-detector
---



## 1. Slice Features

모델에서 예측(prediction, 이하 Pred) 데이터가 나오고 나면 이를 GT(ground truth) 데이터와 비교하여 손실 함수도 계산하고 모델의 최종 출력을 결정하여 성능도 평가해봐야 한다.  

GT와 Pred는 두 가지 종류의 객체 정보가 있다. Tfrecord에 저장되는 GT 데이터는 "bboxes"라는 key에 실제 객체정보를 일렬로 쌓은 정보를 가지고 있고 이 정보가 feature map에 분산된 데이터가 있다. YOLO v3가 세 가지 스케일을 출력하므로 feature map은 "feature_l", "feature_m", "feature_s" 세 가지가 있다. "bboxes"의 객체 정보가 세 가지 feature map의 해당 anchor에 저장되어 있고 나머지 anchor들은 모두 0으로 채워져 있다. 다음은 tfr_config.txt에 들어있는 관련 데이터 정보다.

> "bboxes": {"parse_type": "tf.string", "decode_type": "tf.float32", "shape": [20, 5]}, 
>
> "feature_s": {"parse_type": "tf.string", "decode_type": "tf.float32", "shape": [32, 104, 3, 6]}, 
>
> "feature_m": {"parse_type": "tf.string", "decode_type": "tf.float32", "shape": [16, 52, 3, 6]}, 
>
> "feature_l": {"parse_type": "tf.string", "decode_type": "tf.float32", "shape": [8, 26, 3, 6]},

GT feature map에서 마지막 차원 '6'은 (yxhw, objectness, category index)를 의미한다.  

Pred는 모델의 출력에서 세 가지 feature map들이 먼저 나온다. 손실 함수를 계산할 때는 Pred와 GT의 feature map을 이용한다. 반면 모델의 성능을 측정할 때는 모델의 최종 검출 결과가 있어야 하기 때문에 Pred feature map에 NMS(Non Maximum Suppression)를 적용하여 검출 결과인 Pred의 "bboxes"를 만든다. 이를 GT의 "bboxes"와 비교해서 TP(true positive), FP(false positive), FN(false negative) 등을 구분해야 recall과 precision을 계산할 수 있다. 다음은 GT와 Pred 데이터의 구조를 그린 것이다.

[그림]

