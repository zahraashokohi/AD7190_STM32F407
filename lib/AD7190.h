#ifndef AD7190_H
#define AD7190_H
                  
/*****************************/
  //  CALIBRATE PARAM
/*****************************/
#define   WEIGHT_SENSOR_TONE_KG             1000//Kg
#define   CALIBRATE_FACTOR                  2 
#define   VOLTAGE_SENSOR                    5
/*****************************/
/*****************************/
// Debug prints levels:
#include "stm32f4xx_hal.h"

#define AD7190_DOUT_TIMEOUT 0xFFFFF     //  TODO: This is arbitrary. Copied from Analog Devices generic driver.

#define AD7190_CS_CHANGE    1
#define AD7190_CS_NO_CHANGE 0

/* AD7190 Register Map */
#define AD7190_REG_COMM         0 // Communications Register (WO, 8-bit) 
#define AD7190_REG_STAT         0 // Status Register         (RO, 8-bit) 
#define AD7190_REG_MODE         1 // Mode Register           (RW, 24-bit) 
#define AD7190_REG_CONF         2 // Configuration Register  (RW, 24-bit)
#define AD7190_REG_DATA         3 // Data Register           (RO, 24/32-bit) 
#define AD7190_REG_ID           4 // ID Register             (RO, 8-bit) 
#define AD7190_REG_GPOCON       5 // GPOCON Register         (RW, 8-bit) 
#define AD7190_REG_OFFSET       6 // Offset Register         (RW, 24-bit 
#define AD7190_REG_FULLSCALE    7 // Full-Scale Register     (RW, 24-bit)

/* Communications Register Bit Designations (AD7190_REG_COMM) */
#define AD7190_COMM_WEN         (1 << 7)           // Write Enable. 
#define AD7190_COMM_WRITE       (0 << 6)           // Write Operation.
#define AD7190_COMM_READ        (1 << 6)           // Read Operation. 
#define AD7190_COMM_ADDR(x)     (((x) & 0x7) << 3) // Register Address. 
#define AD7190_COMM_CREAD       (1 << 2)           // Continuous Read of Data Register.

/* Status Register Bit Designations (AD7190_REG_STAT) */
#define AD7190_STAT_RDY         (1 << 7) // Ready.
#define AD7190_STAT_ERR         (1 << 6) // ADC error bit.
#define AD7190_STAT_NOREF       (1 << 5) // Error no external reference. 
#define AD7190_STAT_PARITY      (1 << 4) // Parity check of the data register. 
#define AD7190_STAT_CH2         (1 << 2) // Channel 2. 
#define AD7190_STAT_CH1         (1 << 1) // Channel 1. 
#define AD7190_STAT_CH0         (1 << 0) // Channel 0. 

/* Mode Register Bit Designations (AD7190_REG_MODE) */
#define AD7190_MODE_SEL(x)      (((x) & 0x7) << 21) // Operation Mode Select.
#define AD7190_MODE_DAT_STA     (1 << 20)           // Status Register transmission.
#define AD7190_MODE_CLKSRC(x)   (((x) & 0x3) << 18)  // Clock Source Select.
#define AD7190_MODE_SINC3       (1 << 15)           // SINC3 Filter Select.
#define AD7190_MODE_ENPAR       (1 << 13)           // Parity Enable.
#define AD7190_MODE_SCYCLE      (1 << 11)           // Single cycle conversion.
#define AD7190_MODE_REJ60       (1 << 10)           // 50/60Hz notch filter.
#define AD7190_MODE_RATE(x)     ((x) & 0x3FF)       // Filter Update Rate Select.

/* Mode Register: AD7190_MODE_SEL(x) options */
#define AD7190_MODE_CONT                0 // Continuous Conversion Mode.
#define AD7190_MODE_SINGLE              1 // Single Conversion Mode.
#define AD7190_MODE_IDLE                2 // Idle Mode.
#define AD7190_MODE_PWRDN               3 // Power-Down Mode.
#define AD7190_MODE_CAL_INT_ZERO        4 // Internal Zero-Scale Calibration.
#define AD7190_MODE_CAL_INT_FULL        5 // Internal Full-Scale Calibration.
#define AD7190_MODE_CAL_SYS_ZERO        6 // System Zero-Scale Calibration.
#define AD7190_MODE_CAL_SYS_FULL        7 // System Full-Scale Calibration.

/* Mode Register: AD7190_MODE_CLKSRC(x) options */
#define AD7190_CLK_EXT_MCLK1_2          0 // External crystal. The external crystal is connected from MCLK1 to MCLK2.
#define AD7190_CLK_EXT_MCLK2            1 // External Clock applied to MCLK2 
#define AD7190_CLK_INT                  2 // Internal 4.92 MHz clock. Pin MCLK2 is tristated.
#define AD7190_CLK_INT_CO               3 // Internal 4.92 MHz clock. The internal clock is available on MCLK2.

/* Filter Rate (AD7190_MODE_RATE)*/
/* Output Data Rate = (fmod/64)/FS */
#define AD7190_FILTER_RATE_1023   1023
#define AD7190_FILTER_RATE_640    640
#define AD7190_FILTER_RATE_480    480
#define AD7190_FILTER_RATE_96     96
#define AD7190_FILTER_RATE_80     80
#define AD7190_FILTER_RATE_32     32
#define AD7190_FILTER_RATE_16     16
#define AD7190_FILTER_RATE_5      5
#define AD7190_FILTER_RATE_2      2
#define AD7190_FILTER_RATE_1      1

/* Configuration Register Bit Designations (AD7190_REG_CONF) */
#define AD7190_CONF_CHOP        (1 << 23)            // CHOP enable.
#define AD7190_CONF_REFSEL      (1 << 20)            // REFIN1/REFIN2 Reference Select.
#define AD7190_CONF_CHAN(x)     (((x) & 0xFF) << 8)  // Channel select.
#define AD7190_CONF_BURN        (1 << 7)             // Burnout current enable.
#define AD7190_CONF_REFDET      (1 << 6)             // Reference detect enable.
#define AD7190_CONF_BUF         (1 << 4)             // Buffered Mode Enable.
#define AD7190_CONF_UNIPOLAR    (1 << 3)             // Unipolar/Bipolar Enable.
#define AD7190_CONF_GAIN(x)     ((x) & 0x7)          // Gain Select.

/* Configuration Register: AD7190_CONF_CHAN(x) options */
#define AD7190_CH_AIN1P_AIN2M      (1 << 0) // AIN1(+) - AIN2(-)       
#define AD7190_CH_AIN3P_AIN4M      (1 << 1) // AIN3(+) - AIN4(-)       
#define AD7190_CH_TEMP_SENSOR      (1 << 2) // Temperature sensor       
#define AD7190_CH_AIN2P_AIN2M      (1 << 3) // AIN2(+) - AIN2(-)       
#define AD7190_CH_AIN1P_AINCOM     (1 << 4) // AIN1(+) - AINCOM       
#define AD7190_CH_AIN2P_AINCOM     (1 << 5) // AIN2(+) - AINCOM       
#define AD7190_CH_AIN3P_AINCOM     (1 << 6) // AIN3(+) - AINCOM       
#define AD7190_CH_AIN4P_AINCOM     (1 << 7) // AIN4(+) - AINCOM

/* Configuration Register: AD7190_CONF_GAIN(x) options */
                                // ADC Input Range (5 V Reference)
#define AD7190_CONF_GAIN_1    0 // Gain 1    +-5 V
#define AD7190_CONF_GAIN_8    3 // Gain 8    +-625 mV
#define AD7190_CONF_GAIN_16   4 // Gain 16   +-312.5 mV
#define AD7190_CONF_GAIN_32   5 // Gain 32   +-156.2 mV
#define AD7190_CONF_GAIN_64   6 // Gain 64   +-78.125 mV
#define AD7190_CONF_GAIN_128  7 // Gain 128  +-39.06 mV
#define AD7190_CONF_GAIN_MASK 7 // To read gain config


#define AD7190_CONF_GAIN_1_FACTOR       +5 //V
#define AD7190_CONF_GAIN_8_FACTOR       625 //mV
#define AD7190_CONF_GAIN_16_FACTOR      312.5 //mV
#define AD7190_CONF_GAIN_32_FACTOR      156.2 //mV
#define AD7190_CONF_GAIN_64_FACTOR      78.125 //mV
#define AD7190_CONF_GAIN_128_FACTOR     39.06 //mV

#define AD7190_BUFF_ACTIVE     1
#define AD7190_BUFF_DISABLE    0

#define AD7190_UNIPOLAR        1
#define AD7190_BIPOLAR         0

/* ID Register Bit Designations (AD7190_REG_ID) */
#define ID_AD7190               0x4
#define AD7190_ID_MASK          0x07

/* GPOCON Register Bit Designations (AD7190_REG_GPOCON) */
#define AD7190_GPOCON_BPDSW     (1 << 6) // Bridge power-down switch enable
#define AD7190_GPOCON_GP32EN    (1 << 5) // Digital Output P3 and P2 enable
#define AD7190_GPOCON_GP10EN    (1 << 4) // Digital Output P1 and P0 enable
#define AD7190_GPOCON_P3DAT     (1 << 3) // P3 state
#define AD7190_GPOCON_P2DAT     (1 << 2) // P2 state
#define AD7190_GPOCON_P1DAT     (1 << 1) // P1 state
#define AD7190_GPOCON_P0DAT     (1 << 0) // P0 state


//     AD7190 Settings:
//     SPI frequency: MAX AD7190 f is 5000000 = 5 MHz = (100 + 100) ns
//     SPI_MODE3 <-- CPOL = 1 CPHA = 1
#define  DATA_SAMPLE_AD7190       30  
#define  DATA_BYTE_AD7190                  4

#define  LEN_BUF_AD7190                    (DATA_SAMPLE_AD7190*DATA_BYTE_AD7190)*2
 
#define  LEN_BUF_REAL_AD7190               DATA_SAMPLE_AD7190*DATA_BYTE_AD7190  

typedef struct
{
		
  GPIO_TypeDef      	*cs_gpio;
  uint16_t          	cs_pin;
  SPI_HandleTypeDef 	*spi;   
	GPIO_TypeDef      	*rdy_gpio;
	uint16_t          	rdy_pin;
  uint8_t 						pin_rdy;	
	uint8_t 						activeState;
	uint32_t 						spiErrorCount;
	uint8_t  					  id;
	float               temp;
	uint32_t            timeOutMilis;
}AD7190_Config;

typedef struct
{

	uint32_t indexRead;
	uint32_t indexWrite;
	uint32_t sensorData;
	uint16_t indexBuf;
  uint8_t sensorBuf[LEN_BUF_AD7190];
	uint32_t sampleData;
	float    readData;
}AT7190_Handler;

typedef struct
{
		uint32_t   toneSensor;
	  uint8_t    voltage;
	  uint8_t    caliFactor;
	  float      gainFactor;
}AD7190_Params;

/******************* ready pin config ************************/
	/*
		GPIO_InitStruct.Pin = DRDY_AD7190_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
		HAL_GPIO_Init(DRDY_AD7190_GPIO_Port, &GPIO_InitStruct);
	*/
/************************************************************/

/************************ spi config ************************/
/*

		SPI2 GPIO Configuration
    PB10     ------> SPI2_SCK
    PB14     ------> SPI2_MISO
    PB15     ------> SPI2_MOSI
    
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
*/
/************************************************************/
/********************* system clk config ********************/
/*
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  Initializes the CPU, AHB and APB buses clocks
  
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
*/
/************************************************************/
extern AD7190_Config AD7190_1;
extern AD7190_Config AD7190_2;
extern AD7190_Config AD7190_3;

extern AT7190_Handler  loadCell1;
extern AT7190_Handler  loadCell2;
extern AT7190_Handler  loadCell3;

extern AD7190_Params  load_1;
extern AD7190_Params  load_2;
extern AD7190_Params  load_3;

static const int AD7190_SPI_CLK = 5000000;




uint8_t  AD7190_begin(AD7190_Config* ad7190);

// Generic read/write functions:
uint32_t AD7190_getRegisterValue(AD7190_Config* ad7190,uint8_t registerAddress, uint8_t bytesNumber, uint8_t modifyCS);
void AD7190_setRegisterValue(AD7190_Config* ad7190,uint8_t registerAddress, uint32_t registerValue, uint8_t bytesNumber, uint8_t modifyCS);

void  AD7190_setCS(AD7190_Config* ad7190);
void  AD7190_releaseCS(AD7190_Config* ad7190);

void  AD7190_reset(AD7190_Config* ad7190);
uint8_t  AD7190_checkId(AD7190_Config* ad7190);


void  AD7190_setChannel(AD7190_Config* ad7190,uint8_t channel);
void  AD7190_setConfigurationRegister(AD7190_Config* ad7190,uint8_t channel, uint8_t buff, uint8_t polarity, uint8_t range);
void  AD7190_setPower(AD7190_Config* ad7190,uint8_t pwrMode);
void  AD7190_calibrate(AD7190_Config* ad7190,uint8_t calMode, uint8_t calChannel);

uint8_t  AD7190_waitRdyGoLow(AD7190_Config* ad7190);

void  AD7190_setModeContinuousRead(AD7190_Config* ad7190, uint8_t commRegValue);
void  AD7190_endModeContinuousRead(AD7190_Config* ad7190);
uint32_t  AD7190_getDataContinuousRead(AD7190_Config* ad7190,uint8_t bytesNumber);

// Custom read/write functions:
uint8_t   AD7190_getStatusRegister(AD7190_Config* ad7190);
uint32_t  AD7190_getDataRegister(AD7190_Config* ad7190, uint8_t sampleNumber);           // Checks status register and increas error counter
uint32_t  AD7190_getDataRegisterAvg(AD7190_Config* ad7190, uint8_t sampleNumber); 
float AD7190_getTemperature(AD7190_Config* ad7190);
GPIO_PinState AD7190_readPinReady (AD7190_Config* ad7190);
uint32_t AD7190_readRegisterData (AD7190_Config* ad7190, AD7190_Params* param, AT7190_Handler* loadCell);
#endif
