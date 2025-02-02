/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pid.h"
#include "GPS.h"
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct {
	double Latitude;
	double Longitude;
	double Height;
	uint32_t timestamp;
 }position;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint8_t DataToSend[100]; // Tablica zawierajaca dane do wyslania
uint8_t MessageCounter = 0; // Licznik wyslanych wiadomosci
uint8_t MessageLength = 0; // Zawiera dlugosc wysylanej wiadomosci
uint8_t ReceivedData[100]; // Tablica przechowujaca odebrane dane
uint8_t ReceivedDataFlag = 0; // Flaga informujaca o odebraniu danych

double g_azimuth, g_altitude, g_distance;

volatile uint16_t pulse_count_azimuth = 1600; // Licznik impulsow
volatile double positions_azimuth; // Licznik przekreconych pozycji

volatile uint16_t pulse_count_height = 1240; // Licznik impulsow
volatile double positions_height; // Licznik przekreconych pozycji

volatile int flaga1ms = 0;
volatile int flaga1s = 0;

cpid_t pid_azimuth;
cpid_t pid_height;

//uint16_t feedback[2];

//inicjalizacja dane pozycji - rondo Regana Wrocław
position home_position = { 51.111534, 17.060227, 117.09};
position actual_position = { 51.111534, 17.060227, 117.09};
position old_position = { 51.111534, 17.060227, 117.09};

int setpoint_azimuth = 180;
int setpoint_height = 180;
int pwm_control_height = 0;
int pwm_control_azimuth = 0;

int pwm_control_azimuth_abs = 0;
int pwm_control_height_abs = 0;

int pwm_dir_height = 0;
int pwm_dir_azimuth = 0;

int timer_val = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
 GPS_CallBack();
 //HAL_UART_Transmit_IT(&huart1, "RAMKA", sizeof("RAMKA"));
// HAL_UART_Transmit_IT(&huart1, &Received, 1); // Rozpoczecie nadawania danych z wykorzystaniem przerwan
// HAL_UART_Receive_IT(&huart3, &Received, 1); // Ponowne włączenie nasłuchiwania

 //		GPS.GPGGA.LatitudeDecimal, GPS.GPGGA.LongitudeDecimal, GPS.GPGGA.MSL_Altitude
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim == &htim4){
			//obsługa przerwania co 1ms
		flaga1ms = 1;
	}
	if(htim == &htim5){
			//obsługa przerwania co 1ms
		flaga1s = 1;
	}
}

int _write(int file, char *ptr, int len){
    //HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, 50);
    CDC_Transmit_FS((uint8_t*)ptr, len);
    return len;

}

void parse(){
  	char header[1];
  	int32_t PWM1, PWM2, DIR1, DIR2;
  	//PWM1 - height, PWM2 - azimuth

  	sscanf(ReceivedData, "%s %d %d %d %d", &header, &PWM1, &PWM2, &DIR1, &DIR2);
  	if( header[0] == 'S' && PWM1 >= 0 && PWM1 < 65535 && PWM2 >= 0 && PWM2 < 65535 && (DIR1 == 1 || DIR1 == 0) && (DIR2 == 1 || DIR2 == 0) )
  	{
  		//send_json(PWM1, PWM2);
  		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWM1 );
  		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, PWM2 );

  		if(DIR1 == 1){
  			HAL_GPIO_WritePin(MOTOR11_GPIO_Port, MOTOR11_Pin, GPIO_PIN_SET);
  			HAL_GPIO_WritePin(MOTOR12_GPIO_Port, MOTOR12_Pin, GPIO_PIN_RESET);
  		}

  		if(DIR1 == 0){
  			HAL_GPIO_WritePin(MOTOR11_GPIO_Port, MOTOR11_Pin, GPIO_PIN_RESET);
  			HAL_GPIO_WritePin(MOTOR12_GPIO_Port, MOTOR12_Pin, GPIO_PIN_SET);
  		}

  		if(DIR2 == 1){
  			HAL_GPIO_WritePin(MOTOR21_GPIO_Port, MOTOR21_Pin, GPIO_PIN_SET);
  			HAL_GPIO_WritePin(MOTOR22_GPIO_Port, MOTOR22_Pin, GPIO_PIN_RESET);
  		}

  		if(DIR2 == 0){
  			HAL_GPIO_WritePin(MOTOR21_GPIO_Port, MOTOR21_Pin, GPIO_PIN_RESET);
  			HAL_GPIO_WritePin(MOTOR22_GPIO_Port, MOTOR22_Pin, GPIO_PIN_SET);
  		}

  		if( PWM1 == 0 && PWM2 ==0 ){
  			HAL_GPIO_WritePin(MOTOR11_GPIO_Port, MOTOR11_Pin, GPIO_PIN_RESET);
  			HAL_GPIO_WritePin(MOTOR12_GPIO_Port, MOTOR12_Pin, GPIO_PIN_RESET);
  			HAL_GPIO_WritePin(MOTOR21_GPIO_Port, MOTOR21_Pin, GPIO_PIN_RESET);
  			HAL_GPIO_WritePin(MOTOR22_GPIO_Port, MOTOR22_Pin, GPIO_PIN_RESET);
  		}
  		//send_json(PWM1, PWM2);
  		int a = 0, b = 0, c = 0, d = 0;
  		if(HAL_GPIO_ReadPin(MOTOR11_GPIO_Port, MOTOR11_Pin) == GPIO_PIN_SET) a = 1;
  		if(HAL_GPIO_ReadPin(MOTOR12_GPIO_Port, MOTOR12_Pin) == GPIO_PIN_SET) b = 1;
  		if(HAL_GPIO_ReadPin(MOTOR21_GPIO_Port, MOTOR21_Pin) == GPIO_PIN_SET) c = 1;
  		if(HAL_GPIO_ReadPin(MOTOR22_GPIO_Port, MOTOR22_Pin) == GPIO_PIN_SET) d = 1;

  		printf("MOTOR11:%d MOTOR12:%d MOTOR21:%d MOTOR22:%d PWM1:%d PWM2:%d\r\n",a,b,c,d, PWM1, PWM2 );
  		//send_json_error(message);

//	  	sprintf(DataToSend, "%d %d %d %d \r\n", PWM1, PWM2, DIR1, DIR2);
//	  	printf(DataToSend);
  	}else printf("error - zle dane \r\n");
}

void send_json(int32_t Encoder1, int32_t Encoder2){
	printf("{\"enkoder1\":%d,\"enkoder2\":%d}\r\n", Encoder1, Encoder2);
}

void send_json_ada(double azimuth, double altitude, double distance){
	printf("{\"azimuth\":%f,\"altitude\":%f,\"distance\":%f}\r\n", azimuth, altitude, distance);
	g_azimuth = azimuth;
	g_altitude = altitude;
	g_distance = distance;
}

void send_json_error( char *error){
	printf("{\"error\":\"%s\"}\r\n", error);
}

void send_json_position(position actual, position predicted){
	printf("{\"PositionActual\":{\"Lat\":%f,\"Lon\":%f,\"Height\":%f},\"PositionPredicted\":{\"Lat\":%f,\"Lon\":%f,\"Height\":%f}}\r\n", actual.Latitude, actual.Longitude, actual.Height, predicted.Latitude, predicted.Longitude, predicted.Height );
}

void calc_azimuth(double Latitude1, double Longitude1, double Height1, double Latitude2, double Longitude2, double Height2, double *azimuth, double *distance, double *altitude){ //Latitude = φ Longitude = λ

	Latitude1 *= (M_PI/180);
	Longitude1 *= (M_PI/180);
	Latitude2 *= (M_PI/180);
	Longitude2 *= (M_PI/180);
	Height1 /= 1000;
	Height2 /= 1000;

	double delta_Latitude = (Latitude2 - Latitude1);
	double delta_Longitude = (Longitude2 - Longitude1);
	double delta_Height = Height2 - Height1;



	//θ = atan2 [(sin Δλ * cos φ₂), (cos φ�? * sin φ₂ �?� sin φ�? * cos φ₂ *  cos Δλ)]
	*azimuth = atan2( ( sin(delta_Longitude) * cos(Latitude2) ) , ( (cos(Latitude1) * sin(Latitude2)) - (sin(Latitude1) * cos(Latitude2) * cos(delta_Longitude)) ) ) * (180/ M_PI );
	//Haversine formula:
	//a = sin²(Δφ/2) + cos φ�? * cos φ₂ * sin²(Δλ/2)
	double a = pow( sin(delta_Latitude/2), 2.0 ) + (cos(Latitude1) * cos(Latitude2) * pow(sin(delta_Longitude/2), 2.0));
	//c = 2 * atan2 [�?�a, �?�(1�?�a)]
	double c = 2.0 * atan2( sqrt(a), sqrt(1.0-a));
	//d = R * c, R = 6371 km - radius of the Earth
	double sphere_distance = 6371.0 * c; // in km

	*distance = sqrt( pow(sphere_distance, 2.0) + pow(delta_Height, 2.0));// in km
	*altitude = acos(sphere_distance/ *distance)  * (180/M_PI);
}

void calc_azimuth_obj(position tracker, position object, double *azimuth, double *distance, double *altitude){ //Latitude = φ Longitude = λ

	tracker.Latitude *= (M_PI/180);
	tracker.Longitude *= (M_PI/180);
	object.Latitude *= (M_PI/180);
	object.Longitude *= (M_PI/180);
	tracker.Height /= 1000;
	object.Height /= 1000;

	double delta_Latitude = (object.Latitude - tracker.Latitude);
	double delta_Longitude = (object.Longitude - tracker.Longitude);
	double delta_Height = object.Height - tracker.Height;

	//θ = atan2 [(sin Δλ * cos φ₂), (cos φ�? * sin φ₂ �?� sin φ�? * cos φ₂ *  cos Δλ)]
	*azimuth = atan2( ( sin(delta_Longitude) * cos(object.Latitude) ) , ( (cos(tracker.Latitude) * sin(object.Latitude)) - (sin(tracker.Latitude) * cos(object.Latitude) * cos(delta_Longitude)) ) ) * (180/ M_PI );
	//Haversine formula:
	//a = sin²(Δφ/2) + cos φ�? * cos φ₂ * sin²(Δλ/2)
	double a = pow( sin(delta_Latitude/2), 2.0 ) + (cos(tracker.Latitude) * cos(object.Latitude) * pow(sin(delta_Longitude/2), 2.0));
	//c = 2 * atan2 [�?�a, �?�(1�?�a)]
	double c = 2.0 * atan2( sqrt(a), sqrt(1.0-a));
	//d = R * c, R = 6371 km - radius of the Earth
	double sphere_distance = 6371.0 * c; // in km

	*distance = sqrt( pow(sphere_distance, 2.0) + pow(delta_Height, 2.0));// in km
	*altitude = acos(sphere_distance/ *distance)  * (180/M_PI);
}

void parse_loc(){
  	char header[1];
  	double Latitude1, Longitude1, Height1, Latitude2, Longitude2, Height2, azimuth, distance, altitude;

  	sscanf(ReceivedData, "%s %lf %lf %lf %lf %lf %lf", &header, &Latitude1, &Longitude1, &Height1, &Latitude2, &Longitude2, &Height2);
  	if( header[0] == 'G' )
  	{
  		calc_azimuth( Latitude1,  Longitude1,  Height1,  Latitude2,  Longitude2,  Height2,  &azimuth,  &distance,  &altitude);
  		send_json_ada( azimuth, altitude, distance);
  	}else printf("error - zle dane \r\n");
}

void parse_home_pos(){
  	char header[1];
  	sscanf(ReceivedData, "%s %lf %lf %lf", &header, &home_position.Latitude, &home_position.Longitude, &home_position.Height);
}

void parse_actual_pos(){
  	char header[1];
  	double lat, lon,height;
  	sscanf(ReceivedData, "%s %lf %lf %lf", &header, &actual_position.Latitude, &actual_position.Longitude, &actual_position.Height);
//  	actual_position.Latitude = lat;
//	actual_position.Longitude = lon;
//	actual_position.Height = height;
//	printf("data: %lf,%lf,%lf", lat, lon, height);
}

//very simple prediction by linear approximation
position simple_predict(position actual, position old){
	position predicted;
	predicted.Latitude = 2.0 * actual.Latitude - old.Latitude;
	predicted.Longitude = 2.0 * actual.Longitude - old.Longitude;
	predicted.Height = 2.0 * actual.Height - old.Height;
	return predicted;
}


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_DMA_Init();
  MX_USB_DEVICE_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);

  TIM2->CNT = 1600;
  TIM3->CNT = 1240;

  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_Base_Start_IT(&htim5);

//  HAL_ADC_Start_DMA(&hadc1, feedback, 2);

  pid_init(&pid_azimuth, 80.0f, 0.0f, 50.0f, 10, 1);
  pid_azimuth.p_max = pid_scale(&pid_azimuth, 4095);
  pid_azimuth.p_min = pid_scale(&pid_azimuth, -4095);
  pid_azimuth.i_max = pid_scale(&pid_azimuth, 4095);
  pid_azimuth.i_min = pid_scale(&pid_azimuth, -4095);
  pid_azimuth.d_max = pid_scale(&pid_azimuth, 4095);
  pid_azimuth.d_min = pid_scale(&pid_azimuth, -4095);
  pid_azimuth.total_max = pid_scale(&pid_azimuth, 4095);
  pid_azimuth.total_min = pid_scale(&pid_azimuth, -4095);

  pid_init(&pid_height, 60.0f, 0.0f, 30.0f, 10, 1);//1ms - okres
  pid_height.p_max = pid_scale(&pid_height, 4095);
  pid_height.p_min = pid_scale(&pid_height, -4095);
  pid_height.i_max = pid_scale(&pid_height, 4095);
  pid_height.i_min = pid_scale(&pid_height, -4095);
  pid_height.d_max = pid_scale(&pid_height, 4095);
  pid_height.d_min = pid_scale(&pid_height, -4095);
  pid_height.total_max = pid_scale(&pid_height, 4095);
  pid_height.total_min = pid_scale(&pid_height, -4095);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
      HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

		HAL_GPIO_WritePin(MOTOR11_GPIO_Port, MOTOR11_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MOTOR12_GPIO_Port, MOTOR12_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MOTOR21_GPIO_Port, MOTOR21_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MOTOR22_GPIO_Port, MOTOR22_Pin, GPIO_PIN_RESET);
  while (1)
  {
	  timer_val = __HAL_TIM_GET_COUNTER(&htim4);

	  //send_json((int)feedback[0], (int)feedback[1] );
	  pulse_count_azimuth = TIM2->CNT; // przepisanie wartosci z rejestru timera
	  positions_azimuth = pulse_count_azimuth/8.88889; // zeskalowanie impulsow do stopni

	  pulse_count_height = TIM3->CNT; // przepisanie wartosci z rejestru timera
	  positions_height = pulse_count_height/6.88889; // zeskalowanie impulsow do stopni

	  //send_json( positions_height, positions_azimuth );

	  	  if( flaga1ms == 1 ){
	  		  flaga1ms = 0;

				  pwm_control_height = pid_calc(&pid_height, (int)positions_height, setpoint_height);
				  if( pwm_control_height >= 0 ) pwm_dir_height = 1;
				  else pwm_dir_height = 0;
				  pwm_control_height_abs = abs(pwm_control_height);

					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_control_height_abs );

					if(pwm_dir_height == 1){
						HAL_GPIO_WritePin(MOTOR11_GPIO_Port, MOTOR11_Pin, GPIO_PIN_SET);
						HAL_GPIO_WritePin(MOTOR12_GPIO_Port, MOTOR12_Pin, GPIO_PIN_RESET);
					}

					if(pwm_dir_height == 0){
						HAL_GPIO_WritePin(MOTOR11_GPIO_Port, MOTOR11_Pin, GPIO_PIN_RESET);
						HAL_GPIO_WritePin(MOTOR12_GPIO_Port, MOTOR12_Pin, GPIO_PIN_SET);
					}

		  		  pwm_control_azimuth = pid_calc(&pid_azimuth, (int)positions_azimuth, setpoint_azimuth);
		  		  if( pwm_control_azimuth >= 0 ) pwm_dir_azimuth = 1;
		  		  else pwm_dir_azimuth = 0;
		  		  pwm_control_azimuth_abs = abs(pwm_control_azimuth);

		    		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm_control_azimuth_abs );

		    		if(pwm_dir_azimuth == 1){
		      			HAL_GPIO_WritePin(MOTOR21_GPIO_Port, MOTOR21_Pin, GPIO_PIN_SET);
		      			HAL_GPIO_WritePin(MOTOR22_GPIO_Port, MOTOR22_Pin, GPIO_PIN_RESET);
		    		}

		    		if(pwm_dir_azimuth == 0){
		      			HAL_GPIO_WritePin(MOTOR21_GPIO_Port, MOTOR21_Pin, GPIO_PIN_RESET);
		      			HAL_GPIO_WritePin(MOTOR22_GPIO_Port, MOTOR22_Pin, GPIO_PIN_SET);
		    		}




	  	  }

	  	if( flaga1s == 1 ){
	  		flaga1s = 0;
	  		if(setpoint_height == 180) setpoint_height = 150;
	  		else setpoint_height = 180;
	  		if(setpoint_azimuth == 180) setpoint_azimuth = 150;
	  		else setpoint_azimuth = 180;
	  	}

//	  send_json( positions_height, positions_azimuth );
//
//	  pwm_control_height = pid_calc(&pid_height, (int)positions_height, setpoint_height)/10;



//	  while( positions_azimuth > 310 || positions_azimuth < 50 || positions_height > 310 || positions_height < 50 ){
//			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0 );
//			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0 );
//			HAL_GPIO_WritePin(MOTOR11_GPIO_Port, MOTOR11_Pin, GPIO_PIN_RESET);
//			HAL_GPIO_WritePin(MOTOR12_GPIO_Port, MOTOR12_Pin, GPIO_PIN_RESET);
//			HAL_GPIO_WritePin(MOTOR21_GPIO_Port, MOTOR21_Pin, GPIO_PIN_RESET);
//			HAL_GPIO_WritePin(MOTOR22_GPIO_Port, MOTOR22_Pin, GPIO_PIN_RESET);
//			HAL_Delay(100);
//			//send_json((int)feedback[0], (int)feedback[1] );
//			send_json_error("Poza zakresem");
//	  }

//	  if(ReceivedDataFlag == 1){
//	  	ReceivedDataFlag = 0;
//	  	//parse();
//	  	if(ReceivedData[0] == 'S') parse();
//	  	else if (ReceivedData[0] == 'G') parse_loc();
//	  	else if (ReceivedData[0] == 'H') parse_home_pos();
//	  	else if (ReceivedData[0] == 'A'){
//	  		parse_actual_pos();
//		  	send_json_position( actual_position , simple_predict( actual_position, old_position ) );
//		  	//HAL_Delay(5000);
//		  	old_position.Latitude = actual_position.Latitude;
//		  	old_position.Longitude = actual_position.Longitude;
//		  	old_position.Height = actual_position.Height;
//	  	}
//	  	else send_json_error( "Bad data frame construction!");
//	  }
	  //HAL_Delay(100);

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

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
