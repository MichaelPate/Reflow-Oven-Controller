/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "ssd1306_font5x7.h"
#include "ssd1306_font_num16x24.h"
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
	STATE_INIT = 0,
	STATE_IDLE,
	STATE_CONFIG_PREHEAT,
	STATE_CONFIG_SOAK,
	STATE_CONFIG_REFLOW,
	STATE_CONFIG_COOLDOWN,
	STATE_CYCLE,
	STATE_STOP
} deviceState_t;
static deviceState_t currentDeviceState = STATE_INIT;

typedef enum
{
	STATE_CYCLE_INIT = 0,
	STATE_CYCLE_PREHEAT,
	STATE_CYCLE_SOAK,
	STATE_CYCLE_REFLOW,
	STATE_CYCLE_COOLDOWN
} cycleState_t;
static cycleState_t currentCycleState = STATE_CYCLE_INIT;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;

/* USER CODE BEGIN PV */
#define SSD1306_ADDR (0x3C << 1)	// I2C address for the OLED display controller

// Parameters for default curve
int preheat_target_c = 150;
int preheat_rate = 3;
int preheat_time_s = 100;
int soak_target_c = 150;
int soak_time_s = 100;
int reflow_peak_c = 240;
int reflow_peak_time_s = 10;
int reflow_total_time_s = 50;
int cooldown_rate = 4;
bool cooldown_door = true;

/**
uint32_t lastMillis_LED = 0;
#define DELAY_FAST_MS 50
#define DELAY_SLOW_MS 600

// While we do our scan we need to know if nothing ever came up
bool buttonPressed = false;

// This stores the last button presses detected
#define BUFFER_SIZE 5
int buffer[BUFFER_SIZE];
int finalBuffer[BUFFER_SIZE];
int bufferIdx = 0;
bool addedToBuffer = false;

uint32_t lastMillis_DisplayBuffer = 0;
#define BUF_DISP_INT_MS 10
**/
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
void float_to_string_1dp(float value, char *buffer);
uint16_t MAX6675_ReadRaw(void);
float MAX6675_ReadTempC(void);
void ssd1306_draw_big_digit(uint8_t x, uint8_t page, uint8_t digit);
void ssd1306_draw_big_number(uint8_t x, uint8_t page, uint32_t value);
void ssd1306_draw_char(uint8_t x, uint8_t page, char c);
void ssd1306_draw_string(uint8_t x, uint8_t page, const char *str);
void ssd1306_cmd(uint8_t cmd);
void ssd1306_data(uint8_t *data, size_t size);
void ssd1306_init(void);
void ssd1306_clear(void);
void ssd1306_char(uint8_t x, uint8_t page, const uint8_t *glyph);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void float_to_string_1dp(float value, char *buffer)
{
    int scaled = (int)(value * 10.0f);

    int int_part = scaled / 10;
    int frac_part = scaled % 10;

    if (frac_part < 0) frac_part = -frac_part;

    itoa(int_part, buffer, 10);

    int len = strlen(buffer);
    buffer[len] = '.';
    buffer[len + 1] = '0' + frac_part;
    buffer[len + 2] = '\0';
}


uint16_t MAX6675_ReadRaw(void)
{
    uint8_t rx[2] = {0};

    HAL_GPIO_WritePin(MAX6675_CS_GPIO_Port, MAX6675_CS_Pin, false);
    HAL_Delay(10);

    HAL_SPI_Receive(&hspi2, rx, 2, 100);

    HAL_GPIO_WritePin(MAX6675_CS_GPIO_Port, MAX6675_CS_Pin, true);

    printf((rx[0] << 8) | rx[1]);
    return (rx[0] << 8) | rx[1];
}

float MAX6675_ReadTempC(void)
{
    uint16_t raw = MAX6675_ReadRaw();

    if (raw & 0x4)
        return -1.0f;   // thermocouple open

    raw >>= 3;

    return raw * 0.25f;
}

void ssd1306_draw_big_digit(uint8_t x, uint8_t page, uint8_t digit)
{
    if (digit > 9)
        return;

    for (uint8_t p = 0; p < NUM_FONT_PAGES; p++)
    {
        ssd1306_cmd(0xB0 + page + p);
        ssd1306_cmd(0x00 + (x & 0x0F));
        ssd1306_cmd(0x10 + (x >> 4));

        ssd1306_data(
            (uint8_t *)font_num_16x24[digit][p],
            NUM_FONT_WIDTH
        );
    }
}

void ssd1306_draw_big_number(uint8_t x, uint8_t page, uint32_t value)
{
    char buf[10];
    snprintf(buf, sizeof(buf), "%lu", value);

    while (*buf)
    {
        ssd1306_draw_big_digit(x, page, *buf - '0');
        x += NUM_FONT_WIDTH + 2;  // spacing
        //buf++;
    }
}


void ssd1306_draw_string(uint8_t x, uint8_t page, const char *str)
{
    while (*str)
    {
        ssd1306_draw_char(x, page, *str++);
        x += 6;  // 5px glyph + 1px space

        if (x > 122)
            break;
    }
}


void ssd1306_draw_char(uint8_t x, uint8_t page, char c)
{
    if (c < 0x20 || c > 0x7E)
        c = '?';

    const uint8_t *glyph = font5x7[c - 0x20];

    ssd1306_cmd(0xB0 + page);
    ssd1306_cmd(0x00 + (x & 0x0F));
    ssd1306_cmd(0x10 + (x >> 4));

    ssd1306_data((uint8_t *)glyph, 5);

    uint8_t space = 0x00;
    ssd1306_data(&space, 1); // 1px spacing
}

void ssd1306_cmd(uint8_t cmd)
{
    uint8_t data[2] = {0x00, cmd};
    HAL_I2C_Master_Transmit(&hi2c1, SSD1306_ADDR, data, 2, HAL_MAX_DELAY);
}

void ssd1306_data(uint8_t *data, size_t size)
{
    uint8_t buf[129];
    buf[0] = 0x40;
    memcpy(&buf[1], data, size);
    HAL_I2C_Master_Transmit(&hi2c1, SSD1306_ADDR, buf, size + 1, HAL_MAX_DELAY);
}

void ssd1306_init(void)
{
    HAL_Delay(100);

    ssd1306_cmd(0xAE); // display off
    ssd1306_cmd(0x20); // memory addressing mode
    ssd1306_cmd(0x00); // horizontal addressing
    ssd1306_cmd(0xB0); // page start address
    ssd1306_cmd(0xC8); // COM scan direction
    ssd1306_cmd(0x00); // low column
    ssd1306_cmd(0x10); // high column
    ssd1306_cmd(0x40); // start line
    ssd1306_cmd(0x81); // contrast
    ssd1306_cmd(0x7F);
    ssd1306_cmd(0xA1); // segment remap
    ssd1306_cmd(0xA6); // normal display
    ssd1306_cmd(0xA8); // multiplex
    ssd1306_cmd(0x3F);
    ssd1306_cmd(0xA4); // display follows RAM
    ssd1306_cmd(0xD3); // display offset
    ssd1306_cmd(0x00);
    ssd1306_cmd(0xD5); // clock divide
    ssd1306_cmd(0x80);
    ssd1306_cmd(0xD9); // pre-charge
    ssd1306_cmd(0xF1);
    ssd1306_cmd(0xDA); // COM pins
    ssd1306_cmd(0x12);
    ssd1306_cmd(0xDB); // vcom detect
    ssd1306_cmd(0x40);
    ssd1306_cmd(0x8D); // charge pump
    ssd1306_cmd(0x14);
    ssd1306_cmd(0xAF); // display ON
}

void ssd1306_clear(void)
{
    uint8_t zero[128] = {0};

    for (uint8_t page = 0; page < 8; page++)
    {
        ssd1306_cmd(0xB0 + page);
        ssd1306_cmd(0x00);
        ssd1306_cmd(0x10);
        ssd1306_data(zero, 128);
    }
}

void ssd1306_char(uint8_t x, uint8_t page, const uint8_t *glyph)
{
    ssd1306_cmd(0xB0 + page);
    ssd1306_cmd(0x00 + (x & 0x0F));
    ssd1306_cmd(0x10 + (x >> 4));
    ssd1306_data((uint8_t *)glyph, 5);
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
  MX_I2C1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  ssd1306_init();
  ssd1306_clear();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // Main device state machine
	  switch (currentDeviceState)
	  {
	  case STATE_INIT:					// Init, first time tasks
		  ssd1306_clear();
		  ssd1306_draw_string(0,0,"REFLOW CONTROLLER");
		  ssd1306_draw_string(0,1,"MICHAEL PATE V2.0");
		  ssd1306_draw_string(0,3,"PRESS START");
		  currentDeviceState = STATE_IDLE;
		  break;
	  case STATE_IDLE:					// Waiting for user to do something
		  // testing, display temperature
		  float temp;
		  temp = 215.7f;
		  //temp = MAX6675_ReadTempC();


		  char text[5];

		  float_to_string_1dp(temp, text);
		  ssd1306_draw_string(0, 4, "      ");
		  ssd1306_draw_string(0, 4, text);
		  HAL_Delay(500);


		  //C0, R3 = START
		  HAL_GPIO_WritePin(C0_Output_GPIO_Port, C0_Output_Pin, true);
		  if (HAL_GPIO_ReadPin(R3_Input_GPIO_Port, R3_Input_Pin))
		  {
			  ssd1306_clear();
			  currentDeviceState = STATE_CONFIG_PREHEAT;
		  }
		  break;
	  case STATE_CONFIG_PREHEAT:				// Configure the reflow cycle, choose curves, times, temps
		  //We need to either choose a default curve or make our own

		  // Default curve settings
		  // Preheat: 150*C, <3*C/sec, 100sec
		  // Soak: 150*C, 120sec
		  // Reflow: Peak 240*C, >45sec above 217*C minimum, peak temp only 10 seconds
		  // Cooldown: 2-4*C/sec, recommend door slightly open

		  // Editing curves come in later edition
		  ssd1306_draw_string(0,0,"CONFIGURE PREHEAT");
		  ssd1306_draw_string(0,1,"------------------");
		  ssd1306_draw_string(0,2,"TARGET: 150*C");
		  ssd1306_draw_string(0,3,"RATE:     3");
		  ssd1306_draw_string(0,4,"TIME:   100SEC");
		  ssd1306_draw_string(0,7,"PRESS START OR STOP");

		  // Scan for either the start or stop button to go forward or backward
		  HAL_GPIO_WritePin(C0_Output_GPIO_Port, C0_Output_Pin, true);
		  HAL_GPIO_WritePin(C2_Output_GPIO_Port, C2_Output_Pin, false);
		  if (HAL_GPIO_ReadPin(R3_Input_GPIO_Port, R3_Input_Pin))
		  {
			  ssd1306_clear();
			  currentDeviceState = STATE_CONFIG_SOAK;
		  }
		  HAL_GPIO_WritePin(C0_Output_GPIO_Port, C0_Output_Pin, false);
		  HAL_GPIO_WritePin(C2_Output_GPIO_Port, C2_Output_Pin, true);
		  if (HAL_GPIO_ReadPin(R3_Input_GPIO_Port, R3_Input_Pin))
		  {
			  ssd1306_clear();
			  currentDeviceState = STATE_INIT;
		  }
		  break;
	  case STATE_CONFIG_SOAK:
		  ssd1306_draw_string(0,0,"CONFIGURE SOAK");
		  ssd1306_draw_string(0,1,"------------------");
		  ssd1306_draw_string(0,2,"TARGET: 150*C");
		  ssd1306_draw_string(0,3,"TIME:   100SEC");
		  ssd1306_draw_string(0,7,"PRESS START OR STOP");
		  HAL_GPIO_WritePin(C0_Output_GPIO_Port, C0_Output_Pin, true);
		  HAL_GPIO_WritePin(C2_Output_GPIO_Port, C2_Output_Pin, false);
		  if (HAL_GPIO_ReadPin(R3_Input_GPIO_Port, R3_Input_Pin))
		  {
			  ssd1306_clear();
			  currentDeviceState = STATE_CONFIG_REFLOW;
		  }
		  HAL_GPIO_WritePin(C0_Output_GPIO_Port, C0_Output_Pin, false);
		  HAL_GPIO_WritePin(C2_Output_GPIO_Port, C2_Output_Pin, true);
		  if (HAL_GPIO_ReadPin(R3_Input_GPIO_Port, R3_Input_Pin))
		  {
			  ssd1306_clear();
			  currentDeviceState = STATE_CONFIG_PREHEAT;
		  }
		  break;
	  case STATE_CONFIG_REFLOW:
		  ssd1306_draw_string(0,0,"CONFIGURE REFLOW");
		  ssd1306_draw_string(0,1,"------------------");
		  ssd1306_draw_string(0,2,"TARGET: 240*C");
		  ssd1306_draw_string(0,3,"PEAK:    10SEC");
		  ssd1306_draw_string(0,4,"TOTAL:   50SEC");
		  ssd1306_draw_string(0,7,"PRESS START OR STOP");
		  HAL_GPIO_WritePin(C0_Output_GPIO_Port, C0_Output_Pin, true);
		  HAL_GPIO_WritePin(C2_Output_GPIO_Port, C2_Output_Pin, false);
		  if (HAL_GPIO_ReadPin(R3_Input_GPIO_Port, R3_Input_Pin))
		  {
			  ssd1306_clear();
			  currentDeviceState = STATE_CONFIG_COOLDOWN;
		  }
		  HAL_GPIO_WritePin(C0_Output_GPIO_Port, C0_Output_Pin, false);
		  HAL_GPIO_WritePin(C2_Output_GPIO_Port, C2_Output_Pin, true);
		  if (HAL_GPIO_ReadPin(R3_Input_GPIO_Port, R3_Input_Pin))
		  {
			  ssd1306_clear();
			  currentDeviceState = STATE_CONFIG_SOAK;
		  }
		  break;
	  case STATE_CONFIG_COOLDOWN:
		  ssd1306_draw_string(0,0,"CONFIGURE COOLDOWN");
		  ssd1306_draw_string(0,1,"------------------");
		  ssd1306_draw_string(0,2,"RATE:     4");
		  ssd1306_draw_string(0,3,"DOOR:  OPEN");
		  ssd1306_draw_string(0,5,"READY TO REFLOW");
		  ssd1306_draw_string(0,6,"CYCLE WILL START NEXT");
		  ssd1306_draw_string(0,7,"PRESS START OR STOP");
		  HAL_GPIO_WritePin(C0_Output_GPIO_Port, C0_Output_Pin, true);
		  HAL_GPIO_WritePin(C2_Output_GPIO_Port, C2_Output_Pin, false);
		  if (HAL_GPIO_ReadPin(R3_Input_GPIO_Port, R3_Input_Pin))
		  {
			  ssd1306_clear();
			  currentDeviceState = STATE_CYCLE;
		  }
		  HAL_GPIO_WritePin(C0_Output_GPIO_Port, C0_Output_Pin, false);
		  HAL_GPIO_WritePin(C2_Output_GPIO_Port, C2_Output_Pin, true);
		  if (HAL_GPIO_ReadPin(R3_Input_GPIO_Port, R3_Input_Pin))
		  {
			  ssd1306_clear();
			  currentDeviceState = STATE_CONFIG_REFLOW;
		  }
		  break;
	  case STATE_CYCLE:					// Run the cycle, listen for stop input, manage PID loop
		  // We enter the next state machine
		  switch (currentCycleState)
		  {
		  case STATE_CYCLE_INIT:
			  currentCycleState = STATE_CYCLE_PREHEAT;
			  break;
		  case STATE_CYCLE_PREHEAT:
			  break;
		  case STATE_CYCLE_SOAK:
			  break;
		  case STATE_CYCLE_REFLOW:
			  break;
		  case STATE_CYCLE_COOLDOWN:

			  // At the end of the cycle
			  // if temp < thresh, set currentCycleState to STATE_CYCLE_INIT
			  //    and set currentDeviceState to STATE_STOP
			  break;
		  }
		  break;
	  case STATE_STOP:					// Cleanup tasks before returning to idle state
		  break;
	  }

	  /**
	  // Program loop 1, button state changes blink rate
	  // If the button is pressed, blink faster. Otherwise, blink slowly
	  if (!HAL_GPIO_ReadPin(Button_GPIO_Port, Button_Pin))
	  {
		  if (HAL_GetTick() - lastMillis_LED >= DELAY_FAST_MS)
		  {
			  lastMillis_LED = HAL_GetTick();
			  HAL_GPIO_TogglePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin);
		  }
		  //ssd1306_draw_string(0,0,"BUTTON PRESSED    ");
	  }
	  else
	  {
		  if (HAL_GetTick() - lastMillis_LED >= DELAY_SLOW_MS)
		  {
			  lastMillis_LED = HAL_GetTick();
			  HAL_GPIO_TogglePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin);
		  }
		  //ssd1306_draw_string(0,0,"                         ");
	  }



	  // Program loop 2, scan through the keypad to read for pressed keys
	  // For each output, read RB3-RB6 to determine which key is pressed
	  buttonPressed = false;
	  HAL_GPIO_WritePin(C0_Output_GPIO_Port, C0_Output_Pin, true);
	  HAL_GPIO_WritePin(C1_Output_GPIO_Port, C1_Output_Pin, false);
	  HAL_GPIO_WritePin(C2_Output_GPIO_Port, C2_Output_Pin, false);
	  if (HAL_GPIO_ReadPin(R0_Input_GPIO_Port, R0_Input_Pin))			// C0, R0 = 7
	  {
		  if (bufferIdx >= BUFFER_SIZE)
		  {
			  bufferIdx = 0;
		  }
		  if (addedToBuffer == false)
		  {
			  buffer[bufferIdx] = 7;
			  bufferIdx++;
			  addedToBuffer = true;
		  }

		  ssd1306_draw_string(0, 1, "7");
		  buttonPressed = true;
	  }
	  if (HAL_GPIO_ReadPin(R1_Input_GPIO_Port, R1_Input_Pin))			// C0, R1 = 4
	  {
		  if (bufferIdx >= BUFFER_SIZE)
		  {
			  bufferIdx = 0;
		  }
		  if (addedToBuffer == false)
		  {
			  buffer[bufferIdx] = 4;
			  bufferIdx++;
			  addedToBuffer = true;
		  }

		  ssd1306_draw_string(0, 1, "4");
		  buttonPressed = true;
	  }
	  if (HAL_GPIO_ReadPin(R2_Input_GPIO_Port, R2_Input_Pin))			// C0, R2 = 1
	  {
		  if (bufferIdx >= BUFFER_SIZE)
		  {
			  bufferIdx = 0;
		  }
		  if (addedToBuffer == false)
		  {
			  buffer[bufferIdx] = 1;
			  bufferIdx++;
			  addedToBuffer = true;
		  }

		  ssd1306_draw_string(0, 1, "1");
		  buttonPressed = true;
	  }
	  if (HAL_GPIO_ReadPin(R3_Input_GPIO_Port, R3_Input_Pin))			// C0, R3 = START
	  {
		  // The "START" button transfers the input buffer to the final buffer
		  if (addedToBuffer == false)
		  {
			  for (int idx=0; idx<BUFFER_SIZE; idx++)
			  {
				  finalBuffer[idx]=buffer[idx];
			  }

			  // After transferring, clear out the input buffer and reset
			  for (int idx=0; idx<BUFFER_SIZE; idx++)
			  {
				  buffer[idx]=0;
			  }
			  bufferIdx = 0;
			  addedToBuffer = true;
		  }


		  ssd1306_draw_string(0, 1, "START");
		  buttonPressed = true;
	  }
	  HAL_GPIO_WritePin(C0_Output_GPIO_Port, C0_Output_Pin, false);
	  HAL_GPIO_WritePin(C1_Output_GPIO_Port, C1_Output_Pin, true);
	  HAL_GPIO_WritePin(C2_Output_GPIO_Port, C2_Output_Pin, false);
	  if (HAL_GPIO_ReadPin(R0_Input_GPIO_Port, R0_Input_Pin))			// C1, R0 = 8
	  {
		  if (bufferIdx >= BUFFER_SIZE)
		  {
			  bufferIdx = 0;
		  }
		  if (addedToBuffer == false)
		  {
			  buffer[bufferIdx] = 8;
			  bufferIdx++;
			  addedToBuffer = true;
		  }

		  ssd1306_draw_string(0, 1, "8");
		  buttonPressed = true;
	  }
	  if (HAL_GPIO_ReadPin(R1_Input_GPIO_Port, R1_Input_Pin))			// C1, R1 = 5
	  {
		  if (bufferIdx >= BUFFER_SIZE)
		  {
			  bufferIdx = 0;
		  }
		  if (addedToBuffer == false)
		  {
			  buffer[bufferIdx] = 5;
			  bufferIdx++;
			  addedToBuffer = true;
		  }

		  ssd1306_draw_string(0, 1, "5");
		  buttonPressed = true;
	  }
	  if (HAL_GPIO_ReadPin(R2_Input_GPIO_Port, R2_Input_Pin))			// C1, R2 = 2
	  {
		  if (bufferIdx >= BUFFER_SIZE)
		  {
			  bufferIdx = 0;
		  }
		  if (addedToBuffer == false)
		  {
			  buffer[bufferIdx] = 2;
			  bufferIdx++;
			  addedToBuffer = true;
		  }

		  ssd1306_draw_string(0, 1, "2");
		  buttonPressed = true;
	  }
	  if (HAL_GPIO_ReadPin(R3_Input_GPIO_Port, R3_Input_Pin))			// C1, R3 = 0
	  {
		  if (bufferIdx >= BUFFER_SIZE)
		  {
			  bufferIdx = 0;
		  }
		  if (addedToBuffer == false)
		  {
			  buffer[bufferIdx] = 0;
			  bufferIdx++;
			  addedToBuffer = true;
		  }

		  ssd1306_draw_string(0, 1, "0");
		  buttonPressed = true;
	  }
	  HAL_GPIO_WritePin(C0_Output_GPIO_Port, C0_Output_Pin, false);
	  HAL_GPIO_WritePin(C1_Output_GPIO_Port, C1_Output_Pin, false);
	  HAL_GPIO_WritePin(C2_Output_GPIO_Port, C2_Output_Pin, true);
	  if (HAL_GPIO_ReadPin(R0_Input_GPIO_Port, R0_Input_Pin))			// C2, R0 = 9
	  {
		  if (bufferIdx >= BUFFER_SIZE)
		  {
			  bufferIdx = 0;
		  }
		  if (addedToBuffer == false)
		  {
			  buffer[bufferIdx] = 9;
			  bufferIdx++;
			  addedToBuffer = true;
		  }

		  ssd1306_draw_string(0, 1, "9");
		  buttonPressed = true;
	  }
	  if (HAL_GPIO_ReadPin(R1_Input_GPIO_Port, R1_Input_Pin))			// C2, R1 = 6
	  {
		  if (bufferIdx >= BUFFER_SIZE)
		  {
			  bufferIdx = 0;
		  }
		  if (addedToBuffer == false)
		  {
			  buffer[bufferIdx] = 6;
			  bufferIdx++;
			  addedToBuffer = true;
		  }

		  ssd1306_draw_string(0, 1, "6");
		  buttonPressed = true;
	  }
	  if (HAL_GPIO_ReadPin(R2_Input_GPIO_Port, R2_Input_Pin))			// C2, R2 = 3
	  {
		  if (bufferIdx >= BUFFER_SIZE)
		  {
			  bufferIdx = 0;
		  }
		  if (addedToBuffer == false)
		  {
			  buffer[bufferIdx] = 3;
			  bufferIdx++;
			  addedToBuffer = true;
		  }

		  ssd1306_draw_string(0, 1, "3");
		  buttonPressed = true;
	  }
	  if (HAL_GPIO_ReadPin(R3_Input_GPIO_Port, R3_Input_Pin))			// C2, R3 = STOP
	  {
		  // The "STOP" button clears the buffer and erases all inputted numbers
		  if (addedToBuffer == false)
		  {
			  for (int idx=0; idx<BUFFER_SIZE; idx++)
			  {
				  buffer[idx]=0;
			  }
			  bufferIdx = 0;
			  addedToBuffer = true;
		  }


		  ssd1306_draw_string(0, 1, "STOP");
		  buttonPressed = true;
	  }


	  // We never caught a button press during that scan cycle, so nothing was pressed
	  if (buttonPressed == false)
	  {
		  ssd1306_draw_string(0, 1, "      ");
		  addedToBuffer = false;
	  }


	  // We want to display the buffer of stored digits, but only on an interval
	  if (HAL_GetTick() - lastMillis_DisplayBuffer >= BUF_DISP_INT_MS)
	  {
		  // We also want to print a "-" at the current buffer index
		  ssd1306_draw_string(0,4,"        ");
		  ssd1306_draw_char(bufferIdx*7, 4, '-');

		  for (int idx=0; idx<BUFFER_SIZE; idx++)
		  {
			  ssd1306_draw_char(idx*7, 3, (char)((int)buffer[idx] + '0'));
			  ssd1306_draw_char(idx*7, 5, (char)((int)finalBuffer[idx] + '0'));
		  }
		  lastMillis_DisplayBuffer = HAL_GetTick();
	  }
	  **/

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

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
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, C0_Output_Pin|C1_Output_Pin|C2_Output_Pin|LED_BUILTIN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MAX6675_CS_GPIO_Port, MAX6675_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : Button_Pin */
  GPIO_InitStruct.Pin = Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : C0_Output_Pin C1_Output_Pin C2_Output_Pin LED_BUILTIN_Pin
                           MAX6675_CS_Pin */
  GPIO_InitStruct.Pin = C0_Output_Pin|C1_Output_Pin|C2_Output_Pin|LED_BUILTIN_Pin
                          |MAX6675_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Buzzer_Pin */
  GPIO_InitStruct.Pin = Buzzer_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Buzzer_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : R0_Input_Pin R1_Input_Pin R2_Input_Pin R3_Input_Pin */
  GPIO_InitStruct.Pin = R0_Input_Pin|R1_Input_Pin|R2_Input_Pin|R3_Input_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
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
#ifdef USE_FULL_ASSERT
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
