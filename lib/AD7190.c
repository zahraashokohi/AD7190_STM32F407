#include "AD7190.h"
#include "spi.h"
#include <stdlib.h>
#include <string.h>
#include "main.h"

#define   true   1
#define   false  0
#define   BYTE_3       16777215// 0XFFFFFF
#define   SPI_TIMOUT    1
#define   TIMEOUTPINRDY      100//ms
uint32_t timeOutMilis = AD7190_DOUT_TIMEOUT;


AD7190_Config AD7190_1 = 
{
	.activeState = false,
	.cs_gpio = CS1_AD7190_GPIO_Port,
	.cs_pin = CS1_AD7190_Pin,
	.pin_rdy = true,
	.rdy_gpio = DRDY_AD7190_GPIO_Port,
	.rdy_pin = DRDY_AD7190_Pin,
	.spi = &hspi2,
	.spiErrorCount = 0,
};
AD7190_Config AD7190_2 = {0};
AD7190_Config AD7190_3 = {0};

AT7190_Handler  loadCell1 = {0};
AT7190_Handler  loadCell2 = {0};
AT7190_Handler  loadCell3 = {0};

AD7190_Params  load_1 = 
{
.caliFactor = CALIBRATE_FACTOR ,
.toneSensor = WEIGHT_SENSOR_TONE_KG,
.voltage = VOLTAGE_SENSOR,
	.gainFactor = AD7190_CONF_GAIN_128_FACTOR,
};
AD7190_Params  load_2;
AD7190_Params  load_3;

uint8_t AD7190_begin(AD7190_Config* ad7190)
{
  AD7190_reset(ad7190);
  AD7190_setCS(ad7190);
  AD7190_waitRdyGoLow(ad7190);
  AD7190_releaseCS(ad7190);
  if(AD7190_checkId(ad7190))
	{
		ad7190->activeState = true;
	}
		else
	{
			ad7190->activeState = false;
  }
  return ad7190->activeState;
}

uint32_t AD7190_getRegisterValue(AD7190_Config* ad7190, uint8_t registerAddress, uint8_t bytesNumber, uint8_t modifyCS) 
{
		uint8_t data = 0;
		uint8_t inByte = 0;           // incoming byte from the SPI
		uint32_t result = 0;   
		uint8_t readData = 0;	// result to return
		uint8_t address = AD7190_COMM_READ | AD7190_COMM_ADDR(registerAddress);
		if (modifyCS) AD7190_setCS(ad7190);                                          // Pull SS slow to prep other end for transfer
	 
		HAL_SPI_Transmit(ad7190->spi, &address, 1, SPI_TIMOUT);                                    // send the device the register you want to read:
		data = 0x00;
		HAL_SPI_TransmitReceive(ad7190->spi, &data, &readData, 1, SPI_TIMOUT);                              // Send a value of 0 to read the first byte returned:
		result = readData;
		bytesNumber--;                                                  // decrement the number of bytes left to read:
		while (bytesNumber > 0) 
		{                                       // if you still have another byte to read:
			result = result << 8;                                         // shift the first byte left,
				data = 0x00;
				HAL_SPI_TransmitReceive(ad7190->spi, &data, &inByte, 1, SPI_TIMOUT);                             // then get the second byte:
			result = result | inByte;                                     // combine the byte you just got with the previous ones
			bytesNumber--;                                                // decrement the number of bytes left to read
		}
		if (modifyCS) AD7190_releaseCS(ad7190);                                     // Pull SS pin HIGH to signify end of data transfer
		return (result);
}

void AD7190_setRegisterValue(AD7190_Config* ad7190, uint8_t registerAddress, uint32_t registerValue, uint8_t bytesNumber, uint8_t modifyCS)
{

  uint8_t writeCommand[5] = {0, 0, 0, 0, 0};
  uint8_t* dataPointer    = (uint8_t*)&registerValue;
  uint8_t bytesNr         = bytesNumber;
  writeCommand[0] = AD7190_COMM_WRITE | AD7190_COMM_ADDR(registerAddress);
  while (bytesNr > 0) {
    writeCommand[bytesNr] = *dataPointer;
    dataPointer ++;
    bytesNr --;
  }
  if (modifyCS) AD7190_setCS(ad7190);                                                 // Pull SS slow to prep other end for transfer
  HAL_SPI_Transmit(ad7190->spi, writeCommand, bytesNumber + 1, 10);
  if (modifyCS) AD7190_releaseCS(ad7190);                                             // Pull SS high to signify end of data transfer
}

void AD7190_setCS(AD7190_Config* ad7190)
{
	HAL_GPIO_WritePin(ad7190->cs_gpio, ad7190->cs_pin, GPIO_PIN_RESET);
}

void AD7190_releaseCS(AD7190_Config* ad7190)
{
	HAL_GPIO_WritePin(ad7190->cs_gpio, ad7190->cs_pin, GPIO_PIN_SET);
}

void AD7190_reset(AD7190_Config* ad7190)
{

  uint8_t register_word[7];
  register_word[0] = 0x01;
  register_word[1] = 0xFF;
  register_word[2] = 0xFF;
  register_word[3] = 0xFF;
  register_word[4] = 0xFF;
  register_word[5] = 0xFF;
  register_word[6] = 0xFF;

	HAL_GPIO_WritePin(ad7190->cs_gpio, ad7190->cs_pin, GPIO_PIN_RESET);	//  Pull CS low to prep other end for transfer  
  HAL_SPI_Transmit(ad7190->spi,register_word, sizeof(register_word), SPI_TIMOUT);
  HAL_GPIO_WritePin(ad7190->cs_gpio, ad7190->cs_pin, GPIO_PIN_SET);
  
}

uint32_t returnValue = 0;
uint8_t AD7190_checkId(AD7190_Config* ad7190)
{
    returnValue = AD7190_getRegisterValue(ad7190,AD7190_REG_ID, 1, AD7190_CS_CHANGE);
		ad7190->id  = returnValue & AD7190_ID_MASK;
		if (ad7190->id  == ID_AD7190){
			return true;
		}
		return false;
}

float AD7190_getTemperature(AD7190_Config* ad7190) 
{
  // Get current Configuration Register value
  uint32_t currentRegConf = AD7190_getRegisterValue(ad7190, AD7190_REG_CONF, 3, AD7190_CS_CHANGE);

  currentRegConf &= ~(AD7190_CONF_CHAN(0xff) |   //  Clear channel bits
                      AD7190_CONF_UNIPOLAR |     //  Clear unipolar bits
                      AD7190_CONF_GAIN(0x7));    //  Clear gains bits

  // Change only bipolar and gain bits from Configuration register
  uint32_t newRegConf = currentRegConf | 
                        AD7190_CONF_CHAN(AD7190_CH_TEMP_SENSOR) | 
                        AD7190_BIPOLAR | 
                        AD7190_CONF_GAIN(AD7190_CONF_GAIN_1); 
  
  AD7190_setRegisterValue(ad7190, AD7190_REG_CONF, newRegConf, 3, AD7190_CS_CHANGE);
 
  uint32_t newRegMode = 0x0;
  uint32_t regDataReturned = 0x0;

  newRegMode = AD7190_MODE_SEL(AD7190_MODE_SINGLE) | 
            AD7190_MODE_CLKSRC(AD7190_CLK_INT) | 
            AD7190_MODE_RATE(AD7190_FILTER_RATE_96);                          // No idea why to set to AD7190_FILTER_RATE_96
                                                                              // Demo code use AD7190_FILTER_RATE_96 but can be set to AD7190_FILTER_RATE_1
                                                                              // Temperature can be read in 1 ms instead of 80 ms         
  AD7190_setCS(ad7190);                                                                    //
  AD7190_setRegisterValue(ad7190, AD7190_REG_MODE, newRegMode, 3, AD7190_CS_NO_CHANGE);      // NO Change on chip select as we wait for RDY/CIPO pin    
  AD7190_waitRdyGoLow(ad7190);                                                            // Whait CIPO/RDY to go LOW (Normally ~80ms)
  regDataReturned = AD7190_getRegisterValue(ad7190, AD7190_REG_DATA, 3, AD7190_CS_CHANGE);   // Get temperature
  AD7190_releaseCS(ad7190);                                                                // Release CS
  float temperature = 0x0;
  temperature = regDataReturned - 0x800000;
  temperature /= 2815;            // Kelvin Temperature
  temperature -= 273;             //Celsius Temperature
  ad7190->temp = temperature;
	return temperature;
}

void AD7190_setPower(AD7190_Config* ad7190,uint8_t pwrMode)
{
     uint32_t oldPwrMode = 0x0;
     uint32_t newPwrMode = 0x0; 
 
     oldPwrMode = AD7190_getRegisterValue(ad7190, AD7190_REG_MODE, 3, AD7190_CS_CHANGE);
     oldPwrMode &= ~(AD7190_MODE_SEL(0x7));
     
     newPwrMode = oldPwrMode | AD7190_MODE_SEL(pwrMode);
                                  
     AD7190_setRegisterValue(ad7190, AD7190_REG_MODE, newPwrMode, 3, AD7190_CS_CHANGE);
}

void AD7190_setChannel(AD7190_Config* ad7190, uint8_t channel) 
{
    uint32_t oldRegValue = 0x0;
    uint32_t newRegValue = 0x0;   
    
    oldRegValue = AD7190_getRegisterValue(ad7190,AD7190_REG_CONF, 3, AD7190_CS_CHANGE);              // Read current Configuration Register value
    
    oldRegValue &= ~(AD7190_CONF_CHAN(0xFF));                                           //  Clear channel bits
    newRegValue = oldRegValue | AD7190_CONF_CHAN(1 << channel);                         //  Generate new channel register value
    
    AD7190_setRegisterValue(ad7190,AD7190_REG_CONF, newRegValue, 3, AD7190_CS_CHANGE);                //  Write Configuration Register
}

void AD7190_calibrate(AD7190_Config* ad7190, uint8_t calMode, uint8_t calChannel) 
{
    uint32_t oldRegValue = 0x0;
    uint32_t newRegValue = 0x0;
    
    AD7190_setChannel(ad7190,calChannel);
    
    oldRegValue = AD7190_getRegisterValue(ad7190, AD7190_REG_MODE, 3, AD7190_CS_CHANGE);
    oldRegValue &= ~AD7190_MODE_SEL(0x7);
    
    newRegValue = oldRegValue | AD7190_MODE_SEL(calMode);

    AD7190_setCS(ad7190);
    AD7190_setRegisterValue(ad7190, AD7190_REG_MODE, newRegValue, 3, AD7190_CS_NO_CHANGE);
    AD7190_waitRdyGoLow(ad7190);
    AD7190_releaseCS(ad7190);
}

void AD7190_setConfigurationRegister(AD7190_Config* ad7190,uint8_t channel, uint8_t buff, uint8_t polarity, uint8_t range) 
{
  uint32_t oldRegValue = 0x0;
  uint32_t newRegValue = 0x0;

  // Get current register value of Configuration Register:
  oldRegValue = AD7190_getRegisterValue(ad7190,AD7190_REG_CONF, 3, AD7190_CS_CHANGE);
  
  // Mask to modify ONLY these 3 bits
  oldRegValue &= ~(AD7190_CONF_UNIPOLAR | AD7190_CONF_GAIN(AD7190_CONF_GAIN_MASK) | AD7190_CONF_BUF | AD7190_CONF_CHAN(0xFF));

  newRegValue = oldRegValue | (polarity * AD7190_CONF_UNIPOLAR) | AD7190_CONF_GAIN(range) | (buff * AD7190_CONF_BUF | AD7190_CONF_CHAN(channel));
  AD7190_setRegisterValue(ad7190, AD7190_REG_CONF, newRegValue, 3, AD7190_CS_CHANGE);

  AD7190_waitRdyGoLow(ad7190);        // Whait CIPO/RDY to go LOW
}

uint8_t AD7190_getStatusRegister(AD7190_Config* ad7190) 
{

  uint32_t retValue = AD7190_getRegisterValue(ad7190, AD7190_REG_STAT, 1, AD7190_CS_CHANGE);
  uint8_t ad7190Status = (uint8_t)(retValue & 0xFF);
  return ad7190Status;
  
}

uint32_t AD7190_getDataRegisterAvg(AD7190_Config* ad7190, uint8_t sampleNumber) 
{
  uint32_t samplesAverage = 0;
  for (uint8_t i = 0; i < sampleNumber; i++) {
    AD7190_waitRdyGoLow(ad7190);                                                  //  TODO: Check if this can be removed
    uint32_t v = AD7190_getRegisterValue(ad7190, AD7190_REG_DATA, 3, AD7190_CS_CHANGE);
    
    uint8_t channel = v & 0x7;   
    samplesAverage += (v >> 4);
  }
  
  samplesAverage = samplesAverage / sampleNumber;
  return samplesAverage ;

}

uint32_t AD7190_getDataRegister(AD7190_Config* ad7190,uint8_t sampleNumber) 
{
  if (!ad7190->activeState) 
	{
    ad7190->spiErrorCount =  ad7190->spiErrorCount + 1;
    
    if ( ad7190->spiErrorCount % 100 == 0) 
		{
      AD7190_begin(ad7190);
      return 0;
    }
  }
  
  uint8_t adStatus = AD7190_getStatusRegister(ad7190);
  if ((adStatus & 0x80) == 0x80) {
    AD7190_waitRdyGoLow(ad7190);                    // AD7190 data not ready.
  }
  
  uint32_t rawRead = AD7190_getDataRegisterAvg(ad7190, sampleNumber);

  return rawRead;
}

uint8_t AD7190_waitRdyGoLow(AD7190_Config* ad7190) 
{
  timeOutMilis = AD7190_DOUT_TIMEOUT;
	ad7190->pin_rdy = HAL_GPIO_ReadPin(ad7190->rdy_gpio, ad7190->rdy_pin);
  while (ad7190->pin_rdy  && timeOutMilis) {
    ad7190->pin_rdy  = HAL_GPIO_ReadPin(ad7190->rdy_gpio, ad7190->rdy_pin);
    timeOutMilis--;
  }
  if (!timeOutMilis) return true;
  return false;
}

void AD7190_setModeContinuousRead(AD7190_Config* ad7190,uint8_t commRegValue) 
{
  AD7190_setCS(ad7190);                                          // Pull SS slow to prep other end for transfer
  HAL_SPI_Transmit(ad7190->spi,&commRegValue, 1, SPI_TIMOUT);                 // send the device the register you want to read:

}

uint32_t AD7190_getDataContinuousRead(AD7190_Config* ad7190,uint8_t bytesNumber) 
{
  uint8_t inByte = 0;                                  // incoming byte from the SPI
  uint32_t result = 0;
  uint8_t data = 0;
	// result to return
	data = 0x00;
  result = HAL_SPI_TransmitReceive(ad7190->spi, &data, &inByte, 1, SPI_TIMOUT);                // Send a value of 0 to read the first byte returned

  uint8_t i = bytesNumber;                          // decrement the number of bytes left to read:
  while (i > 1) {                                   // if you still have another byte to read:
    result = result << 8;   	// shift the first byte left,
		data = 0x00;
    HAL_SPI_TransmitReceive(ad7190->spi, &data, &inByte, 1, SPI_TIMOUT);              // then get the second byte:
    result = result | inByte;                       // combine the byte you just got with the previous ones
    i--;                                            // decrement the number of bytes left to read
  }
  return (result);
}

void AD7190_endModeContinuousRead(AD7190_Config* ad7190) 
{
  AD7190_releaseCS(ad7190);          // Pull SS pin HIGH to signify end of data transfer
}

GPIO_PinState AD7190_readPinReady (AD7190_Config* ad7190)
{
	return HAL_GPIO_ReadPin(ad7190->rdy_gpio, ad7190->rdy_pin);
}

uint32_t regModeSettings = 0;
uint32_t regConfigSettings = 0;
uint8_t regGPIOSettings = 0;
uint32_t rawAd7190Data = 0;
float rawAd7190Data_float;
uint32_t  TONE_SENSOR = WEIGHT_SENSOR_TONE_KG; //1000Kg
float   kSensor = CALIBRATE_FACTOR;
uint8_t vSensor = VOLTAGE_SENSOR;
uint32_t AD7190_readRegisterData (AD7190_Config* ad7190, AD7190_Params* param, AT7190_Handler* loadCell)
{
			uint8_t modifyCS = AD7190_CS_CHANGE;
			regGPIOSettings =  AD7190_GPOCON_BPDSW;
			AD7190_setRegisterValue(ad7190, AD7190_REG_GPOCON, regGPIOSettings, 1,  AD7190_CS_CHANGE);
			
			 regConfigSettings = ( AD7190_CONF_CHAN(AD7190_CH_AIN1P_AIN2M) | AD7190_CONF_GAIN(AD7190_CONF_GAIN_128) | AD7190_CONF_BUF | AD7190_UNIPOLAR << 3 );   
			AD7190_setRegisterValue(ad7190, AD7190_REG_CONF, regConfigSettings, 3,  AD7190_CS_CHANGE);

			 regModeSettings = (AD7190_MODE_SEL(AD7190_MODE_SINGLE) | AD7190_MODE_CLKSRC(AD7190_CLK_INT) | AD7190_MODE_RATE(AD7190_FILTER_RATE_80));
			AD7190_setRegisterValue(ad7190, AD7190_REG_MODE, regModeSettings, 3,  AD7190_CS_CHANGE);
			ad7190->timeOutMilis = TIMEOUTPINRDY;
			while (AD7190_readPinReady(ad7190) && ad7190->timeOutMilis ) {
			}
			uint8_t data = 0x00;        // incoming byte from the SPI
			uint8_t readData = 0;
			uint8_t bytesNumber = 3;	// result to return
			uint8_t address = AD7190_COMM_READ | AD7190_COMM_ADDR(AD7190_REG_DATA);
			if(AD7190_readPinReady(ad7190) == GPIO_PIN_RESET)
			{
					if (modifyCS) AD7190_setCS(ad7190); 
				
					HAL_SPI_Transmit(ad7190->spi, &address, 1, SPI_TIMOUT);                                    
					HAL_SPI_TransmitReceive(ad7190->spi, &data, &readData, 1, SPI_TIMOUT);   
					loadCell->sensorBuf[loadCell->indexWrite++] = readData;
					loadCell->sensorData = readData;
					bytesNumber--;                                                  
					while (bytesNumber > 0) 
					{                                       
							loadCell->sensorData = loadCell->sensorData << 8;                                         
							HAL_SPI_TransmitReceive(ad7190->spi, &data, &readData, 1, SPI_TIMOUT); 
							loadCell->sensorBuf[loadCell->indexWrite++] = readData;					
							loadCell->sensorData = loadCell->sensorData | readData;                                     
							bytesNumber--;                                                
					}
					if (modifyCS) AD7190_releaseCS(ad7190);
					if(loadCell->indexWrite >= LEN_BUF_AD7190)
						loadCell->indexWrite = 0;
					
					loadCell->readData = ((loadCell->sensorData * param->gainFactor)/ BYTE_3)*(param->toneSensor/ (param->caliFactor*param->voltage));
			}
		return loadCell->sensorData;
}
