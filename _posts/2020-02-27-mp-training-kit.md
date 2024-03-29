---
layout: post
title:  "ATmega128 개발환경 설정"
date:   2020-02-27 09:00:03
categories: Microprocessor
---



# ATmega128 Training Kit



ATmega128 실험 보드의 구성요소 입니다.

![atmega128-kit](../assets/microprocessor/atmega128-kit.jpg)

트레이닝 보드에 대한 더 자세한 내용은 아래 링크에서 확인할 수 있습니다.

<http://www.lkembedded.co.kr/goods/goods_view.php?goodsNo=923>

ATmega128 실험 보드의 구성을 좀 더 보기쉽게 추상화 시킨 그림입니다. 실험할 때 항상 참고하게 될 그림입니다.

![atmega128-kit-layout](../assets/microprocessor/atmega128-kit-layout.png)

ATmega128a의 핀배치도 입니다. 일반적인 디지털 입출력 외에 특수기능을 사용할 때 필요한 핀의 이름을 찾아야 합니다.

![atmega128-pins](../assets/microprocessor/atmega128-pins.jpg)



# Environment Setup

- Microchip Studio: <https://www.microchip.com/en-us/tools-resources/develop/microchip-studio>
- Zadig (USB driver): <https://zadig.akeo.ie/>
- 드라이버 설치 가이드: <http://www.lkembedded.co.kr/goods/goods_view.php?goodsNo=1447>



Microchip Studio에서 Device Programming 할 때 "firmware update"하라고 뜰 경우  

:arrow_right: Tools - Options - Tools - Tool settings - Check firmware: False로 설정



# Test Program

```c
#define F_CPU 16000000L
#include <avr/io.h>
#include <util/delay.h>

int main(void)
{
	DDRE |= 0x01;
	while(1)
	{
		
		PORTE |= 0x01;
		_delay_ms(500);
		PORTE &= ~0x01;
		_delay_ms(500);
	}
	return 0;
}
```

