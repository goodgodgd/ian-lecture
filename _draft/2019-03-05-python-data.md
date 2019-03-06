---
layout: post
title:  "Python Data Types"
date:   2019-03-05 09:00:13
categories: 2019-1-systprog
---


# 파이썬의 데이터 타입

파이썬은 동적 타입이라 변수를 선언할 때 타입을 지정하지 않고 변수 내부적으로 타입을 가지고 있으나 언제든 바뀔 수 있다. 이러한 특성이 C언어에 익숙한 사람들에게는 *위험한* 코드로 보일 수 있으나 깊이 사용하다보면 변수명을 아끼는데 큰 도움이 된다.

```python
import numpy as np	# 수학적 행렬을 다루는 패키지
import pandas as pd	# 엑셀같은 표를 다루는 패키지
# 똑같은 행렬을 같은 변수명으로 list, numpy array, dataframe 여러가지로 바꿔가면서 사용한다.
myarray = [[1, 2, 3], [4, 5, 6], [7, 8, 9]]
# [[1, 2, 3], [4, 5, 6], [7, 8, 9]]
myarray = np.array(myarray)
# array([[1, 2, 3],
#       [4, 5, 6],
#       [7, 8, 9]])
myarray = pd.DataFrame(myarray)
#    0  1  2
# 0  1  2  3
# 1  4  5  6
# 2  7  8  9
```

## 숫자형 타입

