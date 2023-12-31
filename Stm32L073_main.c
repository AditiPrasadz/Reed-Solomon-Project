/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <float.h>
#include <limits.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "profiling.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define MAX_RAND LONG_MAX    // Maximum value of random()

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEBUG_PRINTF printf
#define MAX_EVENT_COUNT 30// Adjust this value as needed
#define __PROF_STOPPED 0xFF
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef hlpuart1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */
int mm, nn, tt, kk, n;
int pp[10];
int alpha_to[20], index_of[20], gg[20] ;
int recd[20], data[20], bb[20] ;


static uint32_t time_start; // profiler start time
static const char *prof_name; // profiler name
static uint32_t time_event[MAX_EVENT_COUNT]; // events time
static const char *event_name[MAX_EVENT_COUNT]; // events name
static uint8_t event_count = __PROF_STOPPED; // events counter


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
void read_p(int m);
void generate_gf(void);
void gen_poly(void);
void encode_rs(void);
void decode_rs(void);
int weight(int word);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len)
{
	HAL_UART_Transmit(&hlpuart1, (uint8_t*)ptr,len, HAL_MAX_DELAY);
	return len;
}

void input (int count)
{
	printf("\n--------------------INPUT === %d-----------------------\n\n\r",count+1);
  if (count == 0)
  {
	  mm=4;
	  nn=10;
	  tt = 3;
	  kk = 4;
	  printf("\nThe input values for m = %d, N =%d, parity = %d\n\r",mm,nn,tt);
  }
  else if (count == 1)
  {
	  mm=4;
	  nn=6;
	  tt = 1;
	  kk = 3;
	  printf("\nThe input values for m = %d, N =%d, parity = %d\n\r",mm,nn,tt);
  }
  else if (count == 2)
  {
	  mm=4;
	  nn=20;
	  tt = 5;
	  kk = 10;
	  printf("\nThe input values for m = %d, N =%d, parity = %d\n\r",mm,nn,tt);
  }

}

void PROFILING_START(const char *profile_name)
{
  prof_name = profile_name;
  event_count = 0;

  //HAL_Init(); // Initialize HAL or CMSIS for your microcontroller
  //HAL_SYSTICK_Config(SystemCoreClock / 1000000); // Configure SysTick for 1 us resolution
  //HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  time_start = HAL_GetTick();
}
void PROFILING_EVENT(const char *event)
{
  if (event_count == __PROF_STOPPED)
    return;

  if (event_count < MAX_EVENT_COUNT)
  {
    time_event[event_count] = HAL_GetTick() - time_start;
    event_name[event_count] = event;
    event_count++;
  }
}
void PROFILING_STOP(void)
{
  int32_t timestamp;
  int32_t delta_t;
  //int32_t tick_per_1us = SystemCoreClock / 1000000;
  int32_t time_prev = 0;
  const char* targetString = "Runtime ECC Code";

  if (event_count == __PROF_STOPPED)
  {
    DEBUG_PRINTF("\r\nWarning: PROFILING_STOP WITHOUT START.\r\n");
    return;
  }

  DEBUG_PRINTF("Profiling \"%s\" sequence: \r\n\n"
               "--Event---------------------------------------------------------------|--timestamp--|----delta_t---\r\n\n", prof_name); //|--Cycles_per_µs--

  for (int i = 0; i < event_count; i++)
  {
	timestamp = time_event[i];  //timestamp = (time_event[i] - time_start) / tick_per_1us;
	if (strncmp(event_name[i], targetString, strlen(targetString)) == 0)
	    {
			delta_t = (i != 2) ? timestamp - time_event[i - 3] : timestamp;
	    }
	else
		{
			delta_t = timestamp - time_prev;
		}
	time_prev = timestamp;

    DEBUG_PRINTF("%-70s:%9lu µs | +%9lu µs\r\n", event_name[i], (unsigned long)timestamp, (unsigned long)delta_t); 		//| +%9lu µs 	,(unsigned long)tick_per_1us
  }

  DEBUG_PRINTF("\r\n");
  event_count = __PROF_STOPPED;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_LPUART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  int check =0;
  PROFILING_START("RS ECC startup timing");
  //Check variable to verify timing
  while (check<1)
  {
	  register int i=0;
	  for(int count=0;count<3;count++)
    	{
		   	if (count==0)
    		{
    		input(count);
    		}
    		else if (count==1)
    		{
    		input(count);
    		}
    		else if (count==2)
    		{
    		input(count);
    		}

    		//declaration(mm,nn);
    		read_p(mm);
    		generate_gf() ;
    		printf("Look-up tables for GF(2**%2d)\n\n\r",mm) ;
    		printf("  i   alpha_to[i]  index_of[i]\n\r") ;
    		for ( i=0; i<=nn; i++)
    		printf("%3d      %3d          %3d\n\r",i,alpha_to[i],index_of[i]) ;
    		printf("\n\r") ;

    		gen_poly() ;

    		for  (i=0; i<kk; i++)   data[i] = 0 ;
    		printf("The message is: ");
    		for  (i=0; i<kk; i++)
    		{
    		data[i] = (rand()>>10) % n;
    		printf("%5d",data[i]);
    		}
    		printf("\n\r");

    		if (count==0)
			{
			encode_rs() ;
			PROFILING_EVENT("(10,4,6) Encoder: Detect and Correct at Least 3 Error Values");

			}
			else if (count==1)
			{
			encode_rs() ;
			PROFILING_EVENT("(6,3,2) Encoder: Detect Only 1 Error");
			}
			else if (count==2)
			{
			encode_rs() ;
			PROFILING_EVENT("(20,10,5) Encoder: Big Input with High Error Correction Capability");
			}

    		//encode_rs() ;
    		for (i=0; i<nn-kk; i++)  recd[i] = bb[i] ;
    		for (i=0; i<kk; i++) recd[i+nn-kk] = data[i] ;

    		data[kk/2] = 3 ;
    		data[kk+1/2] = 2 ;

    		for (i=0; i<nn; i++)
    		recd[i] = index_of[recd[i]] ;

    		if (count==0)
			{
    		decode_rs() ;
			PROFILING_EVENT("(10,4,6) Decoder: Detect and Correct at Least 3 Error Values");

			}
			else if (count==1)
			{
			decode_rs() ;
			PROFILING_EVENT("(6,3,2) Decoder: Detect Only 1 Error Value");
			}
			else if (count==2)
			{
			decode_rs() ;
			PROFILING_EVENT("(20,10,5) Decoder: Big Input with High Error Correction Capability");
			}

    		printf("\n\rResults for Reed-Solomon code (n =%2d, k =%2d, t =%2d)\n\n\r",nn,kk,tt) ;
    		printf("  i  data[i]   recd[i](decoded)\n\r"); //(data, recd in polynomial form)
    		for (i=0; i<nn-kk; i++)
    		printf("%3d    %3d      %3d\n\r",i, bb[i], recd[i]) ;
    		for (i=nn-kk; i<nn; i++)
    		printf("%3d    %3d      %3d\n\r",i, data[i-nn+kk], recd[i]) ;
    		PROFILING_EVENT("Runtime ECC Code");
    	}
    	printf("\n\n\r");
    	check =check + 1;
  }
  PROFILING_STOP();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (0)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_LPUART1;
  PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 16-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 16-1; //65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void read_p(int m)
//      Read m, the degree of a primitive polynomial pp(x) used to compute the
//      Galois field GF(2**m).
{
  int i;

  for (i=1; i<m; i++)
    pp[i] = 0;
  pp[0] = pp[m] = 1;

  if (m == 2)             pp[1] = 1;
  else if (m == 3)        pp[1] = 1;
  else if (m == 4)        pp[3] = 1;
  // else if (m == 4)        pp[1] = 1;  // Commented out to match example pp. 68
  else if (m == 5)        pp[2] = 1;
  else if (m == 6)        pp[1] = 1;
  else if (m == 7)        pp[1] = 1;
  else if (m == 8)        pp[4] = pp[5] = pp[6] = 1;
  else if (m == 9)        pp[4] = 1;
  else if (m == 10)       pp[3] = 1;
  else if (m == 11)       pp[2] = 1;
  else if (m == 12)       pp[3] = pp[4] = pp[7] = 1;
  else if (m == 13)       pp[1] = pp[3] = pp[4] = 1;
  else if (m == 14)       pp[1] = pp[11] = pp[12] = 1;
  else if (m == 15)       pp[1] = 1;
  else if (m == 16)       pp[2] = pp[3] = pp[5] = 1;
  else if (m == 17)       pp[3] = 1;
  else if (m == 18)       pp[7] = 1;
  else if (m == 19)       pp[1] = pp[5] = pp[6] = 1;
  else if (m == 20)       pp[3] = 1;
  printf("Primitive polynomial of GF(2^%d), (LSB first)   pp(x) = ",m);

  n = 1;
  for (i = 0; i <= m; i++)
    {
      n *= 2;
      printf("%1d", pp[i]);
    }
  printf("\n\r");

  n = n / 2 - 1;
}



void generate_gf()
/* generate GF(2**mm) from the irreducible polynomial pp(X) in pp[0]..pp[mm]
*/
 {
   register int i, mask ;

  mask = 1 ;
  alpha_to[mm] = 0 ;
  for (i=0; i<mm; i++)
   { alpha_to[i] = mask ;
     index_of[alpha_to[i]] = i ;
     if (pp[i]!=0)
       alpha_to[mm] ^= mask ;
     mask <<= 1 ;
   }
  index_of[alpha_to[mm]] = mm ;
  mask >>= 1 ;
  for (i=mm+1; i<nn; i++)
   { if (alpha_to[i-1] >= mask)
        alpha_to[i] = alpha_to[mm] ^ ((alpha_to[i-1]^mask)<<1) ;
     else alpha_to[i] = alpha_to[i-1]<<1 ;
     index_of[alpha_to[i]] = i ;
   }
  index_of[0] = -1 ;
 }


void gen_poly()
/* Obtain the generator polynomial of the tt-error correcting, length
  nn=(2**mm -1) Reed Solomon code  from the product of (X+alpha**i), i=1..2*tt
*/
 {
   register int i,j ;

   gg[0] = 2 ;    /* primitive element alpha = 2  for GF(2**mm)  */
   gg[1] = 1 ;    /* g(x) = (X+alpha) initially */
   for (i=2; i<=nn-kk; i++)
    { gg[i] = 1 ;
      for (j=i-1; j>0; j--)
        if (gg[j] != 0)  gg[j] = gg[j-1]^ alpha_to[(index_of[gg[j]]+i)%nn] ;
        else gg[j] = gg[j-1] ;
      gg[0] = alpha_to[(index_of[gg[0]]+i)%nn] ;     /* gg[0] can never be zero */
    }
   /* convert gg[] to index form for quicker encoding */
   for (i=0; i<=nn-kk; i++)  gg[i] = index_of[gg[i]] ;
 }


void encode_rs()

 {
   register int i,j ;
   int feedback ;
   //howlong_t *encoder_rs = howlong_register("encoder_rs");
   howlong[1].name = "encoder_rs";
   howlong_in(&howlong[1]);

   for (i=0; i<nn-kk; i++)
	   bb[i] = 0 ;
   for (i=kk-1; i>=0; i--)
    {  feedback = index_of[data[i]^bb[nn-kk-1]] ;
       if (feedback != -1)
        { for (j=nn-kk-1; j>0; j--)
            if (gg[j] != -1)
              bb[j] = bb[j-1]^alpha_to[(gg[j]+feedback)%nn] ;
            else
              bb[j] = bb[j-1] ;
          bb[0] = alpha_to[(gg[0]+feedback)%nn] ;
        }
       else
        { for (j=nn-kk-1; j>0; j--)
            bb[j] = bb[j-1] ;
          bb[0] = 0 ;
        }
    }
   howlong_out(&howlong[1]);
   howlong_print(&howlong[1]);
 }



void decode_rs()

 {
    //howlong_t *decoder_rs = howlong_register("decoder_rs");
	howlong[2].name = "decoder_rs";
	howlong_in(&howlong[2]);
   register int i,j,u,q ;
   int elp[nn-kk+2][nn-kk], d[nn-kk+2], l[nn-kk+2], u_lu[nn-kk+2], s[nn-kk+1] ;
   int count=0, syn_error=0, root[tt], loc[tt], z[tt+1], err[nn], reg[tt+1] ;

/* first form the syndromes */
   for (i=1; i<=nn-kk; i++)
    { s[i] = 0 ;
      for (j=0; j<nn; j++)
        if (recd[j]!=-1)
          s[i] ^= alpha_to[(recd[j]+i*j)%nn] ;      /* recd[j] in index form */
/* convert syndrome from polynomial form to index form  */
      if (s[i]!=0)  syn_error=1 ;        /* set flag if non-zero syndrome => error */
      s[i] = index_of[s[i]] ;
    } ;

   if (syn_error)       /* if errors, try and correct */
    {

/* initialise table entries */
      d[0] = 0 ;           /* index form */
      d[1] = s[1] ;        /* index form */
      elp[0][0] = 0 ;      /* index form */
      elp[1][0] = 1 ;      /* polynomial form */
      for (i=1; i<nn-kk; i++)
        { elp[0][i] = -1 ;   /* index form */
          elp[1][i] = 0 ;   /* polynomial form */
        }
      l[0] = 0 ;
      l[1] = 0 ;
      u_lu[0] = -1 ;
      u_lu[1] = 0 ;
      u = 0 ;

      do
      {
        u++ ;
        if (d[u]==-1)
          { l[u+1] = l[u] ;
            for (i=0; i<=l[u]; i++)
             {  elp[u+1][i] = elp[u][i] ;
                elp[u][i] = index_of[elp[u][i]] ;
             }
          }
        else
/* search for words with greatest u_lu[q] for which d[q]!=0 */
          { q = u-1 ;
            while ((d[q]==-1) && (q>0)) q-- ;
/* have found first non-zero d[q]  */
            if (q>0)
             { j=q ;
               do
               { j-- ;
                 if ((d[j]!=-1) && (u_lu[q]<u_lu[j]))
                   q = j ;
               }while (j>0) ;
             } ;

/* have now found q such that d[u]!=0 and u_lu[q] is maximum */
/* store degree of new elp polynomial */
            if (l[u]>l[q]+u-q)  l[u+1] = l[u] ;
            else  l[u+1] = l[q]+u-q ;

/* form new elp(x) */
            for (i=0; i<nn-kk; i++)    elp[u+1][i] = 0 ;
            for (i=0; i<=l[q]; i++)
              if (elp[q][i]!=-1)
                elp[u+1][i+u-q] = alpha_to[(d[u]+nn-d[q]+elp[q][i])%nn] ;
            for (i=0; i<=l[u]; i++)
              { elp[u+1][i] ^= elp[u][i] ;
                elp[u][i] = index_of[elp[u][i]] ;  /*convert old elp value to index*/
              }
          }
        u_lu[u+1] = u-l[u+1] ;

/* form (u+1)th discrepancy */
        if (u<nn-kk)    /* no discrepancy computed on last iteration */
          {
            if (s[u+1]!=-1)
                   d[u+1] = alpha_to[s[u+1]] ;
            else
              d[u+1] = 0 ;
            for (i=1; i<=l[u+1]; i++)
              if ((s[u+1-i]!=-1) && (elp[u+1][i]!=0))
                d[u+1] ^= alpha_to[(s[u+1-i]+index_of[elp[u+1][i]])%nn] ;
            d[u+1] = index_of[d[u+1]] ;    /* put d[u+1] into index form */
          }
      } while ((u<nn-kk) && (l[u+1]<=tt)) ;

      u++ ;
      if (l[u]<=tt)         /* can correct error */
       {
/* put elp into index form */
         for (i=0; i<=l[u]; i++)   elp[u][i] = index_of[elp[u][i]] ;

/* find roots of the error location polynomial */
         for (i=1; i<=l[u]; i++)
           reg[i] = elp[u][i] ;
         count = 0 ;
         for (i=1; i<=nn; i++)
          {  q = 1 ;
             for (j=1; j<=l[u]; j++)
              if (reg[j]!=-1)
                { reg[j] = (reg[j]+j)%nn ;
                  q ^= alpha_to[reg[j]] ;
                } ;
             if (!q)        /* store root and error location number indices */
              { root[count] = i;
                loc[count] = nn-i ;
                count++ ;
              };
          } ;
         if (count==l[u])    /* no. roots = degree of elp hence <= tt errors */
          {
/* form polynomial z(x) */
           for (i=1; i<=l[u]; i++)        /* Z[0] = 1 always - do not need */
            { if ((s[i]!=-1) && (elp[u][i]!=-1))
                 z[i] = alpha_to[s[i]] ^ alpha_to[elp[u][i]] ;
              else if ((s[i]!=-1) && (elp[u][i]==-1))
                      z[i] = alpha_to[s[i]] ;
                   else if ((s[i]==-1) && (elp[u][i]!=-1))
                          z[i] = alpha_to[elp[u][i]] ;
                        else
                          z[i] = 0 ;
              for (j=1; j<i; j++)
                if ((s[j]!=-1) && (elp[u][i-j]!=-1))
                   z[i] ^= alpha_to[(elp[u][i-j] + s[j])%nn] ;
              z[i] = index_of[z[i]] ;         /* put into index form */
            } ;

  /* evaluate errors at locations given by error location numbers loc[i] */
           for (i=0; i<nn; i++)
             { err[i] = 0 ;
               if (recd[i]!=-1)        /* convert recd[] to polynomial form */
                 recd[i] = alpha_to[recd[i]] ;
               else  recd[i] = 0 ;
             }
           for (i=0; i<l[u]; i++)    /* compute numerator of error term first */
            { err[loc[i]] = 1;       /* accounts for z[0] */
              for (j=1; j<=l[u]; j++)
                if (z[j]!=-1)
                  err[loc[i]] ^= alpha_to[(z[j]+j*root[i])%nn] ;
              if (err[loc[i]]!=0)
               { err[loc[i]] = index_of[err[loc[i]]] ;
                 q = 0 ;     /* form denominator of error term */
                 for (j=0; j<l[u]; j++)
                   if (j!=i)
                     q += index_of[1^alpha_to[(loc[j]+root[i])%nn]] ;
                 q = q % nn ;
                 err[loc[i]] = alpha_to[(err[loc[i]]-q+nn)%nn] ;
                 recd[loc[i]] ^= err[loc[i]] ;  /*recd[i] must be in polynomial form */
               }
            }
          }
         else    /* no. roots != degree of elp => >tt errors and cannot solve */
           for (i=0; i<nn; i++)        /* could return error flag if desired */
               if (recd[i]!=-1)        /* convert recd[] to polynomial form */
                 recd[i] = alpha_to[recd[i]] ;
               else  recd[i] = 0 ;     /* just output received codeword as is */
       }
     else         /* elp has degree has degree >tt hence cannot solve */
       for (i=0; i<nn; i++)       /* could return error flag if desired */
          if (recd[i]!=-1)        /* convert recd[] to polynomial form */
            recd[i] = alpha_to[recd[i]] ;
          else  recd[i] = 0 ;     /* just output received codeword as is */
    }
   else       /* no non-zero syndromes => no errors: output received codeword */
    for (i=0; i<nn; i++)
       if (recd[i]!=-1)        /* convert recd[] to polynomial form */
         recd[i] = alpha_to[recd[i]] ;
       else  recd[i] = 0 ;
   howlong_out(&howlong[2]);
   howlong_print(&howlong[2]);
 }
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
