---
layout: post
title:  "ATmega128 레지스터 설정"
date:   2020-05-27 09:00:03
categories: 2020-1-MICRO
---



# CH09. UART

### UCSRnA

![ucsrnb](../assets/registers/UART_pins.png)

![ucsrnb](../assets/registers/UART_UCSRnA_bits.png)



![ucsrna](../assets/registers/UART_UCSRnA.png)



### UCSRnB

![ucsrnb](../assets/registers/UART_UCSRnB_bits.png)

![ucsrnb](../assets/registers/UART_UCSRnB.png)



### UCSRnC

![ucsrnc](../assets/registers/UART_UCSRnC_bits.png)

![ucsrnc](../assets/registers/UART_UCSRnC.png)



### UBRR

![ucsrnc](../assets/registers/UART_UBRR.png)



# CH10. ADC

### ADMUX

![adc_mux](../assets/registers/ADC_ADMUX.png)

### ADCSRA

![adc_sra](../assets/registers/ADC_ADCSRA.png)

![adc_sra](../assets/registers/ADC_ADCSRA_ADPS.png)



# CH12. External Interrupt

![ext_int](../assets/registers/EXT_INT.png)



# CH13~15 Timer/Counter

## 8 bit Timer/Counter

### TCCR0

![tccr0](../assets/registers/TC8b_TCCR0_bits.png)

![tccr0_mode](../assets/registers/TC8b_TCCR_mode.png)

![tccr0_mode](../assets/registers/TC8b_TCCR0_CS.png)

<img src="../assets/registers/TC8b_TCCR2_CS.png" alt="tccr0_prescaler" style="zoom:80%;" />

![tccr0_pulse_out](../assets/registers/TC8b_TCCR_com.png)



## 16 bit Timer/Counter

### TCCR1X

![tccr1](../assets/registers/TC16b_TCCR_bits.png)



![tccr1_mode](../assets/registers/TC16b_TCCR_mode.png)

![tccr1_prescaler](../assets/registers/TC16b_TCCRnB_CS.png)





## Timer/Counter Interrupt

### TIMSK/ETIMSK

![tc_timsk](../assets/registers/TC_TIMSK_bits.png)

![tc_etimsk](../assets/registers/TC_ETIMSK_bits.png)



![tc8b_intvect](../assets/registers/TC8b_interrupt_vector.png)

![tc16b_intvect](../assets/registers/TC16b_interrupt_vector.png)



## PWM

![tc_pwm](../assets/registers/TC_PWM_pins.png)



![tc_pwm_modes](../assets/registers/TC_PWM_modes.png)

