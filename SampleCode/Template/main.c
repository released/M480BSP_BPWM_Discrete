/*_____ I N C L U D E S ____________________________________________________*/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#include "project_config.h"


/*_____ D E C L A R A T I O N S ____________________________________________*/

/*_____ D E F I N I T I O N S ______________________________________________*/
volatile uint32_t BitFlag = 0;
volatile uint32_t counter_tick = 0;




/*
	Target : 30K Freq
	DUTY : 50%
	
	SYS_CLK : 192M
	PSC : 1

	192 000 000/30 000 = PSC * (CNR + 1)
	CNR = (SYS_CLK/FREQ)/PSC - 1 = 6400 - 1

	DUTY ratio = CMR / (CNR + 1)
	50% = CMR / (CNR + 1)
	
*/
#define PLL_CLOCK           		            (192000000)
#define SYS_CLK 								(PLL_CLOCK)
#define PWM_PSC 								(1)	
#define PWM_FREQ 								(10000)	

#define PWM_RESOLUTION                        	(10000)
#define PWM_TARGET_DUTY(d)					    ((PWM_RESOLUTION*(100-d))/100)	//((PWM_RESOLUTION*d)/100)
#define PWM_DUTY                              	(PWM_TARGET_DUTY(50))		//percent

#define PWM_CHANNEL                           	(2)
#define PWM_CHANNEL_MASK                        (BPWM_CH_2_MASK)

//16 bit
/*
	Up-Down Counter Type : 
	BPWM period time =(2*PERIOD) * (CLKPSC+1) * BPWMx_CLK. 

	Up Counter Type  or Down Counter Type 
	BPWM period time = (PERIOD+1) *(CLKPSC+1)* BPWMx_CLK. 
	
*/

//#define PWM_CNR 								((SYS_CLK/PWM_FREQ)/PWM_PSC - 1)			//Up Counter Type  or Down Counter Type
#define PWM_CNR 								(((SYS_CLK/PWM_FREQ)/PWM_PSC - 1)>>1 )	//Up-Down Counter Type
#define PWM_CMR 								(PWM_DUTY * (PWM_CNR + 1)/PWM_RESOLUTION)

#define CalNewDutyCMR(pwm, u32ChannelNum, u32DutyCycle, u32CycleResolution)	    (u32DutyCycle * (BPWM_GET_CNR(pwm, u32ChannelNum) + 1) / u32CycleResolution)
#define CalNewDuty(pwm, u32ChannelNum, u32DutyCycle, u32CycleResolution)		(BPWM_SET_CMR(pwm, u32ChannelNum, CalNewDutyCMR(pwm, u32ChannelNum, u32DutyCycle, u32CycleResolution)))

float duty_change = 95;

/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/

void tick_counter(void)
{
	counter_tick++;
}

uint32_t get_tick(void)
{
	return (counter_tick);
}

void set_tick(uint32_t t)
{
	counter_tick = t;
}

void compare_buffer(uint8_t *src, uint8_t *des, int nBytes)
{
    uint16_t i = 0;	
	
    for (i = 0; i < nBytes; i++)
    {
        if (src[i] != des[i])
        {
            printf("error idx : %4d : 0x%2X , 0x%2X\r\n", i , src[i],des[i]);
			set_flag(flag_error , ENABLE);
        }
    }

	if (!is_flag_set(flag_error))
	{
    	printf("%s finish \r\n" , __FUNCTION__);	
		set_flag(flag_error , DISABLE);
	}

}

void reset_buffer(void *dest, unsigned int val, unsigned int size)
{
    uint8_t *pu8Dest;
//    unsigned int i;
    
    pu8Dest = (uint8_t *)dest;

	#if 1
	while (size-- > 0)
		*pu8Dest++ = val;
	#else
	memset(pu8Dest, val, size * (sizeof(pu8Dest[0]) ));
	#endif
	
}

void copy_buffer(void *dest, void *src, unsigned int size)
{
    uint8_t *pu8Src, *pu8Dest;
    unsigned int i;
    
    pu8Dest = (uint8_t *)dest;
    pu8Src  = (uint8_t *)src;


	#if 0
	  while (size--)
	    *pu8Dest++ = *pu8Src++;
	#else
    for (i = 0; i < size; i++)
        pu8Dest[i] = pu8Src[i];
	#endif
}

void dump_buffer(uint8_t *pucBuff, int nBytes)
{
    uint16_t i = 0;
    
    printf("dump_buffer : %2d\r\n" , nBytes);    
    for (i = 0 ; i < nBytes ; i++)
    {
        printf("0x%2X," , pucBuff[i]);
        if ((i+1)%8 ==0)
        {
            printf("\r\n");
        }            
    }
    printf("\r\n\r\n");
}

void dump_buffer_hex(uint8_t *pucBuff, int nBytes)
{
    int     nIdx, i;

    nIdx = 0;
    while (nBytes > 0)
    {
        printf("0x%04X  ", nIdx);
        for (i = 0; i < 16; i++)
            printf("%02X ", pucBuff[nIdx + i]);
        printf("  ");
        for (i = 0; i < 16; i++)
        {
            if ((pucBuff[nIdx + i] >= 0x20) && (pucBuff[nIdx + i] < 127))
                printf("%c", pucBuff[nIdx + i]);
            else
                printf(".");
            nBytes--;
        }
        nIdx += 16;
        printf("\n");
    }
    printf("\n");
}


void BPWM_Init(void)
{
    /* Set PWM0 timer clock prescaler */
    BPWM_SET_PRESCALER(BPWM0, PWM_CHANNEL, PWM_PSC - 1);
	
    /* Set PWM0 timer period */
    BPWM_SET_CNR(BPWM0, PWM_CHANNEL, PWM_CNR);
	
    /* Set PWM0 timer duty */
    BPWM_SET_CMR(BPWM0, PWM_CHANNEL, PWM_CMR);	

	BPWM_SET_ALIGNED_TYPE(BPWM0, PWM_CHANNEL_MASK, BPWM_CENTER_ALIGNED);
	
    /* Set output level at zero, compare up, period(center) and compare down of specified channel */
//    BPWM_SET_OUTPUT_LEVEL(BPWM0, PWM_CHANNEL_MASK, BPWM_OUTPUT_HIGH, BPWM_OUTPUT_LOW, BPWM_OUTPUT_NOTHING, BPWM_OUTPUT_NOTHING);
    BPWM_SET_OUTPUT_LEVEL(BPWM0, PWM_CHANNEL_MASK, BPWM_OUTPUT_LOW, BPWM_OUTPUT_HIGH, BPWM_OUTPUT_NOTHING, BPWM_OUTPUT_LOW);
	
    /* Enable output of PWM0 channel 0 */
    BPWM_EnableOutput(BPWM0, PWM_CHANNEL_MASK);
	
	BPWM_Start(BPWM0, PWM_CHANNEL_MASK);
	
}

void loop(void)
{
    float var = 0.0f;

    if (is_flag_set(flag_duty))
    {
        set_flag(flag_duty , DISABLE);
        var = duty_change;
        printf("duty : %2.2f\r\n" , var);
		CalNewDuty(BPWM0, PWM_CHANNEL, PWM_TARGET_DUTY(var), PWM_RESOLUTION);

    }

}

void TMR1_IRQHandler(void)
{
	static uint32_t LOG = 0;

	
    if(TIMER_GetIntFlag(TIMER1) == 1)
    {
        TIMER_ClearIntFlag(TIMER1);
		tick_counter();

		if ((get_tick() % 1000) == 0)
		{
			printf("%s : %4d\r\n",__FUNCTION__,LOG++);
			PH0 ^= 1;
		}

		if ((get_tick() % 50) == 0)
		{

		}	
    }
}


void TIMER1_Init(void)
{
    TIMER_Open(TIMER1, TIMER_PERIODIC_MODE, 1000);
    TIMER_EnableInt(TIMER1);
    NVIC_EnableIRQ(TMR1_IRQn);	
    TIMER_Start(TIMER1);
}

void UARTx_Process(void)
{
	uint8_t res = 0;
	res = UART_READ(UART0);

	if (res > 0x7F)
	{
		printf("invalid command\r\n");
	}
	else
	{
        printf("res : %c\r\n" , res);
		switch(res)
		{
			case '0':
                duty_change = 50;				
				break;			
			case '1':
                duty_change = 85.2f;			
				break;
			case '2':
                duty_change = 38.5f;		
				break;
			case '3':
                duty_change = 62.73f;
				break;
			case '4':
                duty_change = 100;
				break;                

			case 'a':
			case 'A':
                duty_change += 0.5f;
                if (duty_change >= 100)
                {
                    duty_change = 100;
                }	
				break;    
			case 'd':
			case 'D':
                duty_change -= 0.5f;
                if (duty_change <= 0.5f)
                {
                    duty_change = 0.5f;
                }	
				break;                            

			case 'X':
			case 'x':
			case 'Z':
			case 'z':
				NVIC_SystemReset();		
				break;
		}
	}
}


void UART0_IRQHandler(void)
{
    if(UART_GET_INT_FLAG(UART0, UART_INTSTS_RDAINT_Msk | UART_INTSTS_RXTOINT_Msk))     /* UART receive data available flag */
    {
        while(UART_GET_RX_EMPTY(UART0) == 0)
        {
			UARTx_Process();
            set_flag(flag_duty, ENABLE);
        }
    }

    if(UART0->FIFOSTS & (UART_FIFOSTS_BIF_Msk | UART_FIFOSTS_FEF_Msk | UART_FIFOSTS_PEF_Msk | UART_FIFOSTS_RXOVIF_Msk))
    {
        UART_ClearIntFlag(UART0, (UART_INTSTS_RLSINT_Msk| UART_INTSTS_BUFERRINT_Msk));
    }	
}

void UART0_Init(void)
{
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);

	/* Set UART receive time-out */
	UART_SetTimeoutCnt(UART0, 20);

	UART0->FIFO &= ~UART_FIFO_RFITL_4BYTES;
	UART0->FIFO |= UART_FIFO_RFITL_8BYTES;

	/* Enable UART Interrupt - */
	UART_ENABLE_INT(UART0, UART_INTEN_RDAIEN_Msk | UART_INTEN_TOCNTEN_Msk | UART_INTEN_RXTOIEN_Msk);
	
	NVIC_EnableIRQ(UART0_IRQn);

	#if (_debug_log_UART_ == 1)	//debug
	printf("\r\nCLK_GetCPUFreq : %8d\r\n",CLK_GetCPUFreq());
	printf("CLK_GetHXTFreq : %8d\r\n",CLK_GetHXTFreq());
	printf("CLK_GetLXTFreq : %8d\r\n",CLK_GetLXTFreq());	
	printf("CLK_GetPCLK0Freq : %8d\r\n",CLK_GetPCLK0Freq());
	printf("CLK_GetPCLK1Freq : %8d\r\n",CLK_GetPCLK1Freq());	

//    printf("Product ID 0x%8X\n", SYS->PDID);
	
	#endif
}

void Custom_Init(void)
{
	SYS->GPH_MFPL = (SYS->GPH_MFPL & ~(SYS_GPH_MFPL_PH0MFP_Msk)) | (SYS_GPH_MFPL_PH0MFP_GPIO);
	SYS->GPH_MFPL = (SYS->GPH_MFPL & ~(SYS_GPH_MFPL_PH1MFP_Msk)) | (SYS_GPH_MFPL_PH1MFP_GPIO);
	SYS->GPH_MFPL = (SYS->GPH_MFPL & ~(SYS_GPH_MFPL_PH2MFP_Msk)) | (SYS_GPH_MFPL_PH2MFP_GPIO);

	//EVM LED
	GPIO_SetMode(PH,BIT0,GPIO_MODE_OUTPUT);
	GPIO_SetMode(PH,BIT1,GPIO_MODE_OUTPUT);
	GPIO_SetMode(PH,BIT2,GPIO_MODE_OUTPUT);
	
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

//    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);
//    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

//    CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk);
//    CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk);

//    CLK_EnableXtalRC(CLK_PWRCTL_LXTEN_Msk);
//    CLK_WaitClockReady(CLK_STATUS_LXTSTB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(FREQ_192MHZ);
    /* Set PCLK0/PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);
    /* Select UART clock source from HXT */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk);
    SYS->GPB_MFPH |= (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Enable IP module clock */
    CLK_EnableModuleClock(BPWM0_MODULE);

    /* BPWM clock frequency is set double to PCLK: select BPWM module clock source as PLL */
    CLK_SetModuleClock(BPWM0_MODULE, CLK_CLKSEL2_BPWM0SEL_PLL, (uint32_t)NULL);

   	SYS->GPA_MFPH &= ~(SYS_GPA_MFPH_PA9MFP_Msk );
    SYS->GPA_MFPH |= (SYS_GPA_MFPH_PA9MFP_BPWM0_CH2 );	


    CLK_EnableModuleClock(TMR1_MODULE);
    CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_HIRC, 0);
	
    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Lock protected registers */
    SYS_LockReg();
}

/*
 * This is a template project for M480 series MCU. Users could based on this project to create their
 * own application without worry about the IAR/Keil project settings.
 *
 * This template application uses external crystal as HCLK source and configures UART0 to print out
 * "Hello World", users may need to do extra system configuration based on their system design.
 */

int main()
{
    SYS_Init();

	UART0_Init();
	Custom_Init();	
	TIMER1_Init();

    BPWM_Init();

    /* Got no where to go, just loop forever */
    while(1)
    {
        loop();

    }
}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
