#include "bsp_board.h"

/**
	* @brief   配置NVIC控制器
	* @param   无
	* @retval  无
	*/
void nvic_init(void)
{	
	// 4bit抢占优先级位
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	
	// USART1中断配置
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;  
	NVIC_Init(&NVIC_InitStructure);
	
	// USART2中断配置
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;  
	NVIC_Init(&NVIC_InitStructure);
	
	// USART3中断配置
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;  
	NVIC_Init(&NVIC_InitStructure);
}

/**
	*	@brief		外设时钟初始化
	*	@param		无
	*	@retval		无
	*/
void clock_init(void)
{
	// 使能GPIOA、GPIOB、AFIO外设时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

	// 使能USART1外设时钟 (USART1在APB2总线上)
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	
	// 使能USART2外设时钟 (USART2在APB1总线上)
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	
	// 使能USART3外设时钟 (USART3在APB1总线上)
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	// 禁用JTAG
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
}

/**
	* @brief   初始化USART1
	* @param   无
	* @retval  无
	*/
void usart1_init(void)
{
/**********************************************************
***	初始化USART1引脚
**********************************************************/
	// PA9 - USART1_TX
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;				/* 复用推挽输出 */
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	// PA10 - USART1_RX
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;					/* 上拉输入 */
	GPIO_Init(GPIOA, &GPIO_InitStructure);

/**********************************************************
***	初始化USART1
**********************************************************/
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(USART1, &USART_InitStructure);

/**********************************************************
***	清除USART1中断
**********************************************************/
	USART1->SR; USART1->DR;
	USART_ClearITPendingBit(USART1, USART_IT_RXNE);

/**********************************************************
***	使能USART1中断
**********************************************************/	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);

/**********************************************************
***	使能USART1
**********************************************************/
	USART_Cmd(USART1, ENABLE);
}

/**
	* @brief   初始化USART2
	* @param   无
	* @retval  无
	*/
void usart2_init(void)
{
/**********************************************************
***	初始化USART2引脚
**********************************************************/
	// PA2 - USART2_TX
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;				/* 复用推挽输出 */
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	// PA3 - USART2_RX
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;		/* 浮空输入 */
	GPIO_Init(GPIOA, &GPIO_InitStructure);

/**********************************************************
***	初始化USART2
**********************************************************/
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 460800;				/* 九轴传感器波特率 */
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(USART2, &USART_InitStructure);

/**********************************************************
***	清除USART2中断
**********************************************************/
	USART2->SR; USART2->DR;
	USART_ClearITPendingBit(USART2, USART_IT_RXNE);

/**********************************************************
***	使能USART2中断
**********************************************************/	
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);

/**********************************************************
***	使能USART2
**********************************************************/
	USART_Cmd(USART2, ENABLE);
}

/**
	* @brief   初始化USART3
	* @param   无
	* @retval  无
	*/
void usart3_init(void)
{
/**********************************************************
***	初始化USART3引脚
**********************************************************/
	// PB10 - USART3_TX
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;				/* 复用推挽输出 */
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	// PB11 - USART3_RX
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;					/* 上拉输入 */
	GPIO_Init(GPIOB, &GPIO_InitStructure);

/**********************************************************
***	初始化USART3
**********************************************************/
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(USART3, &USART_InitStructure);

/**********************************************************
***	清除USART3中断
**********************************************************/
	USART3->SR; USART3->DR;
	USART_ClearITPendingBit(USART3, USART_IT_RXNE);

/**********************************************************
***	使能USART3中断
**********************************************************/	
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);

/**********************************************************
***	使能USART3
**********************************************************/
	USART_Cmd(USART3, ENABLE);
}

/**
	*	@brief		板载初始化
	*	@param		无
	*	@retval		无
	*/
void board_init(void)
{
	nvic_init();
	clock_init();
	usart1_init();
	usart2_init();
	usart3_init();
}
