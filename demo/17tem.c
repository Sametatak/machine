/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_UART4_Init(void);
/* USER CODE BEGIN PFP */
uint16_t createModbusQuery(uint8_t slaveAddr, uint16_t startAddr, uint16_t registerCount, uint8_t *buffer);

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

void printBuffer(uint8_t *buffer, uint16_t length, const char *label);
void CDC_Transmit_String(char* string);
uint16_t calculateCRC(uint8_t *buffer, uint16_t length);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define MAX_PWM  300
#define CHIP_ID 0x00
#define ACCEL_DATA 0x08
#define BNO055_ID 0xa0
#define START_BYTE_WR 0xaa
#define START_BYTE_RESP 0xbb
#define READ 0x01
#define WRITE 0x00
#define OPER_MODE 0x3d
#define OPER_MODE_NDOF 0x0C
#define PWR_MODE 0x3e
#define PWR_MODE_NORMAL 0x00
#define PAGE_ID 0x07
#define SYS_TRIGGER 0x3f
#define UNIT_SEL 0x3b
#define AXIS_MAP_CONFIG 0x41
#define AXIS_MAP_SIGN 0x42

typedef struct {
    double kp;
    double ki;
    double kd;
    double integral;
    double prev_error;
} PIDController;

// Function to initialize the PIDController
void PIDController_init(PIDController *pid, double kp, double ki, double kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0;
    pid->prev_error = 0.0;
}

// Function to update the PIDController
double Controller_update(PIDController *pid, double actual, double target, double dt) {
    double error = target - actual;
    double derivative = (error - pid->prev_error) / dt;
    pid->integral += error * dt;
    pid->prev_error = error;

    // Calculate the control signal
    double control_signal = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;

    // Limit the change to a maximum adjustment per update for smooth transition
    double max_change = 1.0; // Define maximum change per update
    if (control_signal > max_change) {
        control_signal = max_change;
    } else if (control_signal < -max_change) {
        control_signal = -max_change;
    }

    // Update the actual value smoothly
    actual += control_signal;

    return actual;
}


static const uint16_t crcTable[256] = {
    // CRC16-Modbus table
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};



typedef struct {
    uint16_t data[128];
} ModbusDevice;

ModbusDevice modbusDevices[15];
#define ROM_START_ADDRESS 0x08010000
#define PAGE_SIZE ((uint32_t)FLASH_PAGE_SIZE)
#define PACKET_SIZE sizeof(ModbusDevice)

/* CRC Calculation Function */
uint16_t calculateCRC(uint8_t *buffer, uint16_t length) {
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < length; ++i) {
        crc = (crc >> 8) ^ crcTable[(crc & 0xFF) ^ buffer[i]];
    }
    return crc;
}


/* UART Receive Complete Callback */


/* Transmit String over USB CDC */
void CDC_Transmit_String(char* string) {
    CDC_Transmit_FS((uint8_t*)string, strlen(string));
}

uint16_t length = 0;
uint16_t function_code = 0;
uint8_t new_data_received = 0;
uint8_t fren = 0;
uint8_t motor = 0;
uint16_t value[4] = {0} ;
uint8_t number_of_bytes = 0;
uint16_t target_vel= 0;
uint16_t starting_address = 0;
uint8_t slave_address = 0;
uint16_t crc = 0xFFFF;
uint16_t default_pwm = 1001;
void CDC_ReceiveCallback(uint8_t *buf, uint32_t len) {
    // Callback function called when data is received over USB

    slave_address = buf[0];
    function_code = buf[1];
    starting_address = (buf[2] << 8) | buf[3];
    length = (buf[4] << 8) | buf[5];
    new_data_received = 1;
    crc = calculateCRC(buf, len);

    if (function_code == 16 && len >= 9 + length) {
    	number_of_bytes= buf[6];
        for (uint8_t i = 0; i < number_of_bytes; i+=2) {
            value[i/2] =(buf[7+i] << 8) | buf[8+i];
        }
    }
}

void initialize_device(uint8_t slave_address) {
    for (uint16_t i = 0; i < 300; i++) {
        modbusDevices[slave_address].data[i] = 0;
    }
}


void write_command(uint8_t slave_address, uint16_t starting_address, uint16_t length) {
	modbusDevices[slave_address].data[starting_address] = length;
	target_vel = length;
}
void write_multiple_command(uint8_t slave_address, uint16_t starting_address, uint16_t length, uint16_t* value) {
    for (uint16_t i = 0; i < number_of_bytes/2; i++) {
        modbusDevices[slave_address].data[starting_address + i] = value[i];
        //CDC_Transmit_FS(modbusDevices[slave_address].data, 10);
    }

}
void read_command(uint8_t slave_address, uint16_t starting_address, uint16_t length, uint16_t crc) {
    uint8_t response[128] = {0};
    uint8_t response_length = 3 + length * 2 + 2;
    if (starting_address + length <= 128) {
        response[0] = slave_address;
        response[1] = 0x03;
        response[2] = length * 2;

        for (uint8_t i = 0; i < length; i++) {
            response[3 + i * 2] = modbusDevices[slave_address].data[starting_address + i] >> 8;
            response[3 + i * 2 + 1] = modbusDevices[slave_address].data[starting_address + i] & 0xFF;
        }

        crc = calculateCRC(response, response_length - 2);
        response[response_length - 2] = crc & 0xFF;
        response[response_length - 1] = crc >> 8;

        CDC_Transmit_FS(response, response_length);
    }
}
uint16_t update_velocity(uint16_t current_velocity, uint16_t target_velocity, double alpha) {
    double current = (double)current_velocity;
    double target = (double)target_velocity;
    double updated = alpha * target + (1.0 - alpha) * current;

    // If the updated value is close enough to the target, set it to the target


    return (uint16_t)updated;
}

void user_pwm_setvalue_mosfet4(int16_t value,int16_t limit)
{
//tim 4 A L = Channel 3
//tim 4 B L = Channel 4

	//modbusDevices[01].data[0] = value;
    value = modbusDevices[01].data[0]*2-limit;
    value =  (value<limit)?value:limit;

    value =  (value>-limit)?value:-limit;
    if(modbusDevices[01].data[0] == 0){
    	value = 0;
    }
	//modbusDevices[01].data[0]= value;
/*

OUTS
mosfet_1_A_H
mosfet_1_B_H
mosfet_2_A_H
mosfet_2_B_H
mosfet_3_A_H
mosfet_3_B_H
mosfet_4_A_H
mosfet_4_B_H

PWMS
mosfet_1_A_L	 Timer 2 Channel 1
mosfet_1_B_L	 Timer 2 Channel 2
mosfet_2_A_L	 Timer 2 Channel 3
mosfet_2_B_L	 Timer 2 Channel 4
mosfet_3_A_L	 Timer 4 Channel 1
mosfet_3_B_L	 Timer 4 Channel 2
mosfet_4_A_L	 Timer 4 Channel 3
mosfet_4_B_L	 Timer 4 Channel 4
*/

	 if(value>0){
		 HAL_GPIO_WritePin(MOSFET_4_A_H_GPIO_Port, MOSFET_4_A_H_Pin, 1);
		 HAL_GPIO_WritePin(MOSFET_4_B_H_GPIO_Port, MOSFET_4_B_H_Pin, 0);
		 //HAL_GPIO_WritePin(TEST_OUT_GPIO_Port, TEST_OUT_Pin, 1);
	 }else if(value<0){
		 HAL_GPIO_WritePin(MOSFET_4_A_H_GPIO_Port, MOSFET_4_A_H_Pin, 0);
		 HAL_GPIO_WritePin(MOSFET_4_B_H_GPIO_Port, MOSFET_4_B_H_Pin, 1);
		 //HAL_GPIO_WritePin(TEST_OUT_GPIO_Port, TEST_OUT_Pin, 0);
	 }else  {
		 HAL_GPIO_WritePin(MOSFET_4_A_H_GPIO_Port, MOSFET_4_A_H_Pin, 0);
		 HAL_GPIO_WritePin(MOSFET_4_B_H_GPIO_Port, MOSFET_4_B_H_Pin, 0);
		 //HAL_GPIO_WritePin(TEST_OUT_GPIO_Port, TEST_OUT_Pin, 0);
	 }

	 TIM_OC_InitTypeDef sConfigOC;
	 sConfigOC.OCMode = TIM_OCMODE_PWM1;
	 sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	 sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	 sConfigOC.Pulse =(value>0)? default_pwm:value*-1;//mosfet_4_A_L
	 HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3);
	 HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);


	 TIM_OC_InitTypeDef sConfigOC2;
	 sConfigOC2.OCMode = TIM_OCMODE_PWM1;
	 sConfigOC2.OCPolarity = TIM_OCPOLARITY_HIGH;
	 sConfigOC2.OCFastMode = TIM_OCFAST_DISABLE;
	 sConfigOC2.Pulse = (value>=0)?value:default_pwm;//mosfet_4_B_L

	 HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC2, TIM_CHANNEL_4);
	 HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

}
void user_pwm_setvalue_mosfet3(int16_t value,int16_t limit)
{
    value = modbusDevices[01].data[1]*2-limit;
    value =  (value<limit)?value:limit;

    value =  (value>-limit)?value:-limit;
    if(modbusDevices[01].data[1] == 0){
    	value = 0;
    }
    if(value>0){
        HAL_GPIO_WritePin(MOSFET_3_A_H_GPIO_Port, MOSFET_3_A_H_Pin, 1);
        HAL_GPIO_WritePin(MOSFET_3_B_H_GPIO_Port, MOSFET_3_B_H_Pin, 0);
    }else if(value<0){
        HAL_GPIO_WritePin(MOSFET_3_A_H_GPIO_Port, MOSFET_3_A_H_Pin, 0);
        HAL_GPIO_WritePin(MOSFET_3_B_H_GPIO_Port, MOSFET_3_B_H_Pin, 1);
    }else{
        HAL_GPIO_WritePin(MOSFET_3_A_H_GPIO_Port, MOSFET_3_A_H_Pin, 0);
        HAL_GPIO_WritePin(MOSFET_3_B_H_GPIO_Port, MOSFET_3_B_H_Pin, 0);
    }

    TIM_OC_InitTypeDef sConfigOC;
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.Pulse = (value>0)? default_pwm:value*-1;

    HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);

    TIM_OC_InitTypeDef sConfigOC2;
    sConfigOC2.OCMode = TIM_OCMODE_PWM1;
    sConfigOC2.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC2.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC2.Pulse = (value>=0)?value:default_pwm;

    HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
}
void user_pwm_setvalue_mosfet2(int16_t value,int16_t limit)
{
    value = modbusDevices[01].data[2]*2-limit;
    value =  (value<limit)?value:limit;
    value =  (value>-limit)?value:-limit;
    if(modbusDevices[01].data[2] == 0){
    	value = 0;
    }
    if(value>0){
        HAL_GPIO_WritePin(MOSFET_2_A_H_GPIO_Port, MOSFET_2_A_H_Pin, 1);
        HAL_GPIO_WritePin(MOSFET_2_B_H_GPIO_Port, MOSFET_2_B_H_Pin, 0);
    }else if(value<0){
        HAL_GPIO_WritePin(MOSFET_2_A_H_GPIO_Port, MOSFET_2_A_H_Pin, 0);
        HAL_GPIO_WritePin(MOSFET_2_B_H_GPIO_Port, MOSFET_2_B_H_Pin, 1);
    }else{
        HAL_GPIO_WritePin(MOSFET_2_A_H_GPIO_Port, MOSFET_2_A_H_Pin, 0);
        HAL_GPIO_WritePin(MOSFET_2_B_H_GPIO_Port, MOSFET_2_B_H_Pin, 0);
    }

    TIM_OC_InitTypeDef sConfigOC;
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.Pulse = (value>0)? default_pwm:value*-1;

    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

    TIM_OC_InitTypeDef sConfigOC2;
    sConfigOC2.OCMode = TIM_OCMODE_PWM1;
    sConfigOC2.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC2.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC2.Pulse = (value>=0)?value:default_pwm;

    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC2, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
}
void user_pwm_setvalue_mosfet1(int16_t value,int16_t limit)
{
    value = modbusDevices[01].data[3]*2-limit;
    value =  (value<limit)?value:limit;
    value =  (value>-limit)?value:-limit;
    if(modbusDevices[01].data[3] == 0){
    	value = 0;
    }
    if(value>0){
        HAL_GPIO_WritePin(MOSFET_1_A_H_GPIO_Port, MOSFET_1_A_H_Pin, 1);
        HAL_GPIO_WritePin(MOSFET_1_B_H_GPIO_Port, MOSFET_1_B_H_Pin, 0);
    }else if(value<0){
        HAL_GPIO_WritePin(MOSFET_1_A_H_GPIO_Port, MOSFET_1_A_H_Pin, 0);
        HAL_GPIO_WritePin(MOSFET_1_B_H_GPIO_Port, MOSFET_1_B_H_Pin, 1);
    }else{
        HAL_GPIO_WritePin(MOSFET_1_A_H_GPIO_Port, MOSFET_1_A_H_Pin, 0);
        HAL_GPIO_WritePin(MOSFET_1_B_H_GPIO_Port, MOSFET_1_B_H_Pin, 0);
    }

    TIM_OC_InitTypeDef sConfigOC;
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.Pulse = (value>0)? default_pwm:value*-1;

    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

    TIM_OC_InitTypeDef sConfigOC2;
    sConfigOC2.OCMode = TIM_OCMODE_PWM1;
    sConfigOC2.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC2.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC2.Pulse = (value>=0)?value:default_pwm;

    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */
  initialize_device(1);
  PIDController controller;
  PIDController_init(&controller, 2.5, 0.1, 0.01);
  uint8_t velocity= 0;
  uint16_t limit= 300;
  HAL_GPIO_WritePin(FREN_GPIO_Port, FREN_Pin, 1);
  fren = 1;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1) {//loopp

	  //velocity = update_vexlocity(velocity, target_vel, alpha);
	  //HAL_Delay(100);


	  HAL_Delay(10);
      if (new_data_received) {
    	  if(function_code == 6) {
    		  write_command(slave_address, starting_address, length);
          } else if(function_code == 16) {
        	  write_multiple_command(slave_address, starting_address, length,value);
        	  user_pwm_setvalue_mosfet4(velocity,limit);
        	  user_pwm_setvalue_mosfet3(velocity,limit);
        	  user_pwm_setvalue_mosfet2(velocity,limit);
        	  user_pwm_setvalue_mosfet1(velocity,limit);
          }else {
        	  read_command(slave_address, starting_address, length, crc);
    	  }
          new_data_received = 0;
      }

  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.Prediv1Source = RCC_PREDIV1_SOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV3;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the Systick interrupt time
  */
  __HAL_RCC_PLLI2S_ENABLE();
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 72;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, MOSFET_2_B_H_Pin|MOSFET_2_A_H_Pin|MOSFET_1_B_H_Pin|MOSFET_1_A_H_Pin
                          |BUZZER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MOSFET_4_B_H_Pin|MOSFET_4_A_H_Pin|MOSFET_3_B_H_Pin|MOSFET_3_A_H_Pin
                          |FREN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : MOSFET_2_B_H_Pin MOSFET_2_A_H_Pin MOSFET_1_B_H_Pin MOSFET_1_A_H_Pin
                           BUZZER_Pin */
  GPIO_InitStruct.Pin = MOSFET_2_B_H_Pin|MOSFET_2_A_H_Pin|MOSFET_1_B_H_Pin|MOSFET_1_A_H_Pin
                          |BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : JOYSTICK_BTN_Pin SONAR_DATA_Pin */
  GPIO_InitStruct.Pin = JOYSTICK_BTN_Pin|SONAR_DATA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : MOSFET_4_B_H_Pin MOSFET_4_A_H_Pin MOSFET_3_B_H_Pin MOSFET_3_A_H_Pin
                           FREN_Pin */
  GPIO_InitStruct.Pin = MOSFET_4_B_H_Pin|MOSFET_4_A_H_Pin|MOSFET_3_B_H_Pin|MOSFET_3_A_H_Pin
                          |FREN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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

