#include "main.h"

volatile union SYS_TIME g_st_SysTimeFlag;
struct CBC_ELEMENT CBC_Element;

static INT8 fac_us = 0;	 // us
static INT16 fac_ms = 0; // ms

UINT8 g_u81msCnt = 0;
UINT8 g_u810msClockCnt = 0;
UINT8 g_u81msClockCnt = 0;

UINT8 gu8_200msCnt = 0;
UINT8 gu8_200msAccClock_Flag = 0;

void IWDG_HaltConfig(void);

void InitDelay(void)
{
	SysTick->CTRL &= ~(1 << 2); // 使用外部时钟
	// 这句话到底什么情况？？？？？害死我的系统时钟慢了十几倍
	// fac_us = SystemCoreClock / 8000000;		//SysTick时钟是SYSCLK 8分频，即SysTick时钟频率=SYSCLK/8，1us要计的个数为还得/1MHz
	// fac_us = 6;
	// fac_us = (UINT8)(SystemCoreClock / 8000000);	//我又改过来了，但是还是不行

	switch (SystemCoreClock)
	{
	case 8000000:
		fac_us = 1;
		fac_ms = (INT16)fac_us * 1000; // 每个ms需要的systick时钟数
		break;
	case 12000000:
		// fac_us = 1.5;
		fac_us = 1; // us变得不准了，12M建议只能倍频
		fac_ms = (INT16)fac_us * 1500;
		break;
	case 48000000:
		fac_us = 6;
		fac_ms = (INT16)fac_us * 1000; // 每个ms需要的systick时钟数
		break;
	default:
		break;
	}
}

void __delay_us(UINT32 nus)
{
	UINT32 temp;
	SysTick->LOAD = nus * fac_us;			  // 时间加载
	SysTick->VAL = 0x00;					  // 清空计数器
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk; // 开始倒数
	do
	{
		temp = SysTick->CTRL;
	} while ((temp & 0x01) && !(temp & (1 << 16))); // 等待时间到达
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;		// 关闭计数器
	SysTick->VAL = 0X00;							// 清空计数器
}

// 这个是非中断方式的延时，倘若使用中断式延时，在中断中使用延时会出现中断嵌套问题，很容易出错
void __delay_ms(UINT16 ms)
{
	UINT32 temp;
	SysTick->LOAD = (UINT32)ms * fac_ms;	  // 时间加载(SysTick->LOAD为24bit)
	SysTick->VAL = 0x00;					  // 清空计数器
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk; // 开始倒数
	do
	{
		temp = SysTick->CTRL;
	} while (temp & 0x01 && !(temp & (1 << 16))); // 等待时间到达

	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk; // 关闭计数器
	SysTick->VAL = 0X00;					   // 清空计数器
}

void InitIO(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE); // 开启GPIOA的外设时钟
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE); // 开启GPIOB的外设时钟
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE); // 开启GPIOC的外设时钟
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOD, ENABLE); // 开启GPIOB的外设时钟
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOE, ENABLE); // 开启GPIOB的外设时钟
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOF, ENABLE); // 开启GPIOF的外设时钟

	// PB2_LED1
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_1;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_1;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// PF7_WAKEUP_AFE
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_1;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(GPIOF, &GPIO_InitStructure);

	// MCUO_DEBUG_LED1 = 1;
}

void InitTimer(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	// RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);		//时钟3使能
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM17, ENABLE);

	// 定时器初始化
	TIM_TimeBaseStructure.TIM_Period = 500 - 1;							 // 设置在下一个更新事件装入活动的自动重装载寄存器周期的值
	TIM_TimeBaseStructure.TIM_Prescaler = SystemCoreClock / 1000000 - 1; // 设置用来作为TIMx时钟频率除数的预分频值——计数分频
	// TIM_TimeBaseStructure.TIM_Prescaler = 1; 					//设置用来作为TIMx时钟频率除数的预分频值——计数分频
	// TIM_TimeBaseStructure.TIM_Prescaler = 16;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;		// 设置时钟分割:TDTS = Tck_tim——时钟分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // TIM向上计数模式
	TIM_TimeBaseInit(TIM17, &TIM_TimeBaseStructure);			// 根据指定的参数初始化TIMx的时间基数单位
	TIM_ITConfig(TIM17, TIM_IT_Update, ENABLE);					// 使能指定中断,允许更新中断

	/*中断嵌套设计*/
	NVIC_InitStructure.NVIC_IRQChannel = TIM17_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0; // 抢占优先级0级，没响应优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; // IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);

	TIM_Cmd(TIM17, ENABLE); // 使能TIMx
}

// 使用LSI，38KHz
void Init_IWDG(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE); // 使能PWR外设时钟，待机模式，RTC，看门狗
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);		// 打开独立看门狗寄存器操作权限
	IWDG_SetPrescaler(IWDG_Prescaler_64);				// 预分频系数
	IWDG_SetReload(160);								// 设置重载计数值，k = Xms / (1 / (40KHz/64)) = X/64*40; 4096最高
						 // 800——1.28s，80——128ms
	IWDG_ReloadCounter(); // 喂狗
	IWDG_Enable();		  // 使能IWDG
}

void App_SysTime(void)
{
	static UINT8 s_u8Cnt1ms = 0;

	static UINT8 s_u8Cnt10ms = 0;
	// static UINT8 s_u8Cnt20ms = 0;
	static UINT8 s_u8Cnt50ms = 0;
	static UINT8 s_u8Cnt100ms = 0;

	static UINT8 s_u8Cnt200ms1 = 0;
	static UINT8 s_u8Cnt200ms2 = 4;
	static UINT8 s_u8Cnt200ms3 = 8;
	static UINT8 s_u8Cnt200ms4 = 12;
	static UINT8 s_u8Cnt200ms5 = 16;

	static UINT8 s_u8Cnt1000ms1 = 0;
	static UINT8 s_u8Cnt1000ms2 = 33;
	static UINT8 s_u8Cnt1000ms3 = 66;

	g_st_SysTimeFlag.bits.b1Sys1msFlag = 0;
	if (s_u8Cnt1ms != g_u81msClockCnt)
	{ // 1ms定时标志
		s_u8Cnt1ms = g_u81msClockCnt;
		g_st_SysTimeFlag.bits.b1Sys1msFlag = 1;
	}

	g_st_SysTimeFlag.bits.b1Sys10msFlag1 = 0;
	g_st_SysTimeFlag.bits.b1Sys10msFlag2 = 0;
	g_st_SysTimeFlag.bits.b1Sys10msFlag3 = 0;
	g_st_SysTimeFlag.bits.b1Sys10msFlag4 = 0;
	g_st_SysTimeFlag.bits.b1Sys10msFlag5 = 0;
	if (s_u8Cnt10ms != g_u810msClockCnt)
	{ // 10ms定时标志
		s_u8Cnt10ms = g_u810msClockCnt;
		switch (g_u810msClockCnt)
		{
		case 0:
			g_st_SysTimeFlag.bits.b1Sys10msFlag1 = 1;
			break;

		case 1:
			s_u8Cnt50ms++;
			g_st_SysTimeFlag.bits.b1Sys10msFlag2 = 1;
			break;

		case 2:
			s_u8Cnt100ms++;
			g_st_SysTimeFlag.bits.b1Sys10msFlag3 = 1;
			break;

		case 3:
			s_u8Cnt200ms1++; // 本想用一个变量搞一个循环然后置位，发现有BUG，不行
			s_u8Cnt200ms2++; // 会持续进来一个10ms，必须改变标志位让其不再进来
			s_u8Cnt200ms3++;
			s_u8Cnt200ms4++;
			s_u8Cnt200ms5++;
			g_st_SysTimeFlag.bits.b1Sys10msFlag4 = 1;
			break;

		case 4:
			s_u8Cnt1000ms1++;
			s_u8Cnt1000ms2++;
			s_u8Cnt1000ms3++;
			g_st_SysTimeFlag.bits.b1Sys10msFlag5 = 1;
			break;

		default:
			break;
		}
	}

	g_st_SysTimeFlag.bits.b1Sys50msFlag = 0;
	if (s_u8Cnt50ms >= 5)
	{
		s_u8Cnt50ms = 0;
		g_st_SysTimeFlag.bits.b1Sys50msFlag = 1; // 50ms定时标志
	}

	g_st_SysTimeFlag.bits.b1Sys100msFlag = 0;
	if (s_u8Cnt100ms >= 10)
	{
		s_u8Cnt100ms = 0;
		g_st_SysTimeFlag.bits.b1Sys100msFlag = 1; // 100ms定时标志
	}

	g_st_SysTimeFlag.bits.b1Sys200msFlag1 = 0;
	g_st_SysTimeFlag.bits.b1Sys200msFlag2 = 0;
	g_st_SysTimeFlag.bits.b1Sys200msFlag3 = 0;
	g_st_SysTimeFlag.bits.b1Sys200msFlag4 = 0;
	g_st_SysTimeFlag.bits.b1Sys200msFlag5 = 0;
	if (s_u8Cnt200ms1 >= 20)
	{
		s_u8Cnt200ms1 = 0;
		g_st_SysTimeFlag.bits.b1Sys200msFlag1 = 1; // 200ms定时标志
	}
	if (s_u8Cnt200ms2 >= 20)
	{
		s_u8Cnt200ms2 = 0;
		g_st_SysTimeFlag.bits.b1Sys200msFlag2 = 1; // 200ms定时标志
	}
	if (s_u8Cnt200ms3 >= 20)
	{
		s_u8Cnt200ms3 = 0;
		g_st_SysTimeFlag.bits.b1Sys200msFlag3 = 1; // 200ms定时标志
	}
	if (s_u8Cnt200ms4 >= 20)
	{
		s_u8Cnt200ms4 = 0;
		g_st_SysTimeFlag.bits.b1Sys200msFlag4 = 1; // 200ms定时标志
#if 0
		if(!MCUI_SLEEP_DET) {
			MCUO_DEBUG_LED1 = !MCUO_DEBUG_LED1;
		}
#endif
		MCUO_DEBUG_LED1 = !MCUO_DEBUG_LED1;
	}
	if (s_u8Cnt200ms5 >= 20)
	{
		s_u8Cnt200ms5 = 0;
		g_st_SysTimeFlag.bits.b1Sys200msFlag5 = 1; // 200ms定时标志
	}

	g_st_SysTimeFlag.bits.b1Sys1000msFlag1 = 0;
	g_st_SysTimeFlag.bits.b1Sys1000msFlag2 = 0;
	g_st_SysTimeFlag.bits.b1Sys1000msFlag3 = 0;
	if (s_u8Cnt1000ms1 >= 100)
	{
		s_u8Cnt1000ms1 = 0;
		g_st_SysTimeFlag.bits.b1Sys1000msFlag1 = 1; // 1000ms定时标志
	}
	if (s_u8Cnt1000ms2 >= 100)
	{
		s_u8Cnt1000ms2 = 0;
		g_st_SysTimeFlag.bits.b1Sys1000msFlag2 = 1; // 1000ms定时标志
	}
	if (s_u8Cnt1000ms3 >= 100)
	{
		s_u8Cnt1000ms3 = 0;
		g_st_SysTimeFlag.bits.b1Sys1000msFlag3 = 1; // 1000ms定时标志
		// MCUO_DEBUG_LED2 = !MCUO_DEBUG_LED2;
	}
}

void TIM17_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM17, TIM_IT_Update) != RESET)
	{												 // 检查TIM3更新中断发生与否
		TIM_ClearITPendingBit(TIM17, TIM_IT_Update); // 清除TIMx更新中断标志
		if ((++g_u81msCnt) >= 2)
		{ // 1ms
			g_u81msCnt = 0;
			g_u81msClockCnt++;

			if (g_u81msClockCnt >= 2)
			{ // 2ms
				g_u81msClockCnt = 0;
				g_u810msClockCnt++;
				if (g_u810msClockCnt >= 5)
				{ // 10ms
					g_u810msClockCnt = 0;
				}
			}
			if (++gu8_200msCnt >= 200)
			{
				gu8_200msCnt = 0;
				gu8_200msAccClock_Flag = 1;
			}
		}
	}
}
