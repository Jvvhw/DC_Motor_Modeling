# DC_Motor_Modeling
DLL code for running Plecs simulation. Including speed &amp; current controller (PI)

# DC Motor Modeling
## 개요
DC 모터는 상대적으로 모델 구현이 간단하고 구동이 쉬워 모터 제어의 입문으로 적합

본래 시뮬레이션 자체는 모두 Plecs S/W로 가능하다

실제로는 모터와 인버터는 구현이 되어 있으므로, 제어기능의 코드를 MCU에 올리는것이 중요

이번 프로젝트에서는 Cascaded PI controller와 전압지령 오버슈팅 방지를 위한 Anti wind-up을 사용하여

모터가 시뮬레이션 상으로 제어되는지 확인하였다.

## Plecs 회로 구성
![DWI](https://user-images.githubusercontent.com/125186303/222142383-0c5303de-87a7-488b-91a4-75bc38dc2c5d.jpg)
Amature, Field basic DC motor

![DSP](https://user-images.githubusercontent.com/125186303/222142488-0f225346-25c3-4866-b154-26bef193f21e.jpg)
DLL input, output section

![CBPWM](https://user-images.githubusercontent.com/125186303/222142544-26f8228f-d370-4870-a313-f7c4fd5197b8.jpg)
Carrier Based PWM (Triangular wave)

## 제어기 코드
```
if (INVf_on) { //인버터 전원이 켜져있을때 동작
		If_err = If_ref - If; //오차 입력

		Vsf_ref_integ += Tsamp * Kif_cc * (If_err - Kaf_cc * (Vsf_ref - Vsf_ref_set)); //적분제어, 구분구적법과 동일한 원리

		Vsf_ref = Vsf_ref_integ + Kpf_cc * If_err;                                     //비례제어

		if (Vsf_ref > Vdc) Vsf_ref_set = Vdc;                                          //Anti-Windup, Vdc 이상의 지령을 방지
		else if (Vsf_ref < -Vdc) Vsf_ref_set = -Vdc;
		else                    Vsf_ref_set = Vsf_ref;

		nVsf_ref_set = -Vsf_ref_set;

		Duty_Af = 0.5 * Vsf_ref_set * INV_Vdc + 0.5;//0~1                               //Output Duty 

		Duty_Bf = 0.5 * nVsf_ref_set * INV_Vdc + 0.5;//0~1


	}
	else {
		Duty_Af = 0.5;//0~1

		Duty_Bf = 0.5;//0~1

	}
```
---
## 마치며
프로젝트 진행 전 DC motor에 대한 공부가 충분히 선행되어야 함, 회로설계에 있어서 부족한 부분을 느꼈음

본격적인 DLL파일을 작성해보았으므로 더욱 복잡한 3상, AC 모터 프로젝트에서도 코드를 작성하는 노력을 게을리 하지 않을것
