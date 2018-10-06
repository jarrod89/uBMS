
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : uBMS bring-up code
  ******************************************************************************
  * Hacky bring-up code for uBMS hardware: https://workspace.circuitmaker.com/Projects/Details/Jarrod-Tuma/EVBMS
  * LTC6813 functions are loosely based on the Arduino library from ADI: https://github.com/analogdevicesinc/Linduino/blob/master/LTSketchbook/Part%20Number/6000/6811/DC2259/DC2259.ino
  * Not suitable for anything! No responsibility taken for any use of this code!
  * Always monitor your batteries while charging. It's your own damn fault if YOU burn down your house while running this code!
  * Seriously though, no testing has been done on this code/hardware. Think about this before leaving it alone in a room with your batteries.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_hal.h"
#include "fatfs.h"
#include "usb_device.h"

const uint16_t crc15Table[256] = {0x0,0xc599, 0xceab, 0xb32, 0xd8cf, 0x1d56, 0x1664, 0xd3fd, 0xf407, 0x319e, 0x3aac,  //!<precomputed CRC15 Table
                                0xff35, 0x2cc8, 0xe951, 0xe263, 0x27fa, 0xad97, 0x680e, 0x633c, 0xa6a5, 0x7558, 0xb0c1,
                                0xbbf3, 0x7e6a, 0x5990, 0x9c09, 0x973b, 0x52a2, 0x815f, 0x44c6, 0x4ff4, 0x8a6d, 0x5b2e,
                                0x9eb7, 0x9585, 0x501c, 0x83e1, 0x4678, 0x4d4a, 0x88d3, 0xaf29, 0x6ab0, 0x6182, 0xa41b,
                                0x77e6, 0xb27f, 0xb94d, 0x7cd4, 0xf6b9, 0x3320, 0x3812, 0xfd8b, 0x2e76, 0xebef, 0xe0dd,
                                0x2544, 0x2be, 0xc727, 0xcc15, 0x98c, 0xda71, 0x1fe8, 0x14da, 0xd143, 0xf3c5, 0x365c,
                                0x3d6e, 0xf8f7,0x2b0a, 0xee93, 0xe5a1, 0x2038, 0x7c2, 0xc25b, 0xc969, 0xcf0, 0xdf0d,
                                0x1a94, 0x11a6, 0xd43f, 0x5e52, 0x9bcb, 0x90f9, 0x5560, 0x869d, 0x4304, 0x4836, 0x8daf,
                                0xaa55, 0x6fcc, 0x64fe, 0xa167, 0x729a, 0xb703, 0xbc31, 0x79a8, 0xa8eb, 0x6d72, 0x6640,
                                0xa3d9, 0x7024, 0xb5bd, 0xbe8f, 0x7b16, 0x5cec, 0x9975, 0x9247, 0x57de, 0x8423, 0x41ba,
                                0x4a88, 0x8f11, 0x57c, 0xc0e5, 0xcbd7, 0xe4e, 0xddb3, 0x182a, 0x1318, 0xd681, 0xf17b,
                                0x34e2, 0x3fd0, 0xfa49, 0x29b4, 0xec2d, 0xe71f, 0x2286, 0xa213, 0x678a, 0x6cb8, 0xa921,
                                0x7adc, 0xbf45, 0xb477, 0x71ee, 0x5614, 0x938d, 0x98bf, 0x5d26, 0x8edb, 0x4b42, 0x4070,
                                0x85e9, 0xf84, 0xca1d, 0xc12f, 0x4b6, 0xd74b, 0x12d2, 0x19e0, 0xdc79, 0xfb83, 0x3e1a, 0x3528,
                                0xf0b1, 0x234c, 0xe6d5, 0xede7, 0x287e, 0xf93d, 0x3ca4, 0x3796, 0xf20f, 0x21f2, 0xe46b, 0xef59,
                                0x2ac0, 0xd3a, 0xc8a3, 0xc391, 0x608, 0xd5f5, 0x106c, 0x1b5e, 0xdec7, 0x54aa, 0x9133, 0x9a01,
                                0x5f98, 0x8c65, 0x49fc, 0x42ce, 0x8757, 0xa0ad, 0x6534, 0x6e06, 0xab9f, 0x7862, 0xbdfb, 0xb6c9,
                                0x7350, 0x51d6, 0x944f, 0x9f7d, 0x5ae4, 0x8919, 0x4c80, 0x47b2, 0x822b, 0xa5d1, 0x6048, 0x6b7a,
                                0xaee3, 0x7d1e, 0xb887, 0xb3b5, 0x762c, 0xfc41, 0x39d8, 0x32ea, 0xf773, 0x248e, 0xe117, 0xea25,
                                0x2fbc, 0x846, 0xcddf, 0xc6ed, 0x374, 0xd089, 0x1510, 0x1e22, 0xdbbb, 0xaf8, 0xcf61, 0xc453,
                                0x1ca, 0xd237, 0x17ae, 0x1c9c, 0xd905, 0xfeff, 0x3b66, 0x3054, 0xf5cd, 0x2630, 0xe3a9, 0xe89b,
                                0x2d02, 0xa76f, 0x62f6, 0x69c4, 0xac5d, 0x7fa0, 0xba39, 0xb10b, 0x7492, 0x5368, 0x96f1, 0x9dc3,
                                0x585a, 0x8ba7, 0x4e3e, 0x450c, 0x8095
                               };

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

LPTIM_HandleTypeDef hlptim1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;

enum {
	TRANSFER_WAIT,
	TRANSFER_COMPLETE,
	TRANSFER_ERROR
};
/* Buffer used for transmission */
uint8_t aTxBuffer[13];
/* Buffer used for reception */
uint8_t aRxBuffer[13];

//Globals for debug
uint16_t voltages[18];

uint16_t auxVoltages[12];
float auxVoltagesFloat[12];
float current=0;

uint32_t tickstart = 0;
uint32_t main_period = 1000; //tick is 100us, set period for 10hz, 100ms
float maxBrickV=3.5;
float minBrickV=3.5;
uint8_t cells[] = ACTIVE_CELLS;
float voltagesFloat[SIZE_OF_ARRAY(cells)];
uint8_t state=2;
uint8_t minVctr=3;
uint8_t maxVctr=3;

/* transfer state */
__IO uint32_t wTransferState = TRANSFER_WAIT;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_CAN1_Init(void);
void MX_SPI1_Init(void);
void MX_USART1_UART_Init(void);
void MX_SPI2_Init(void);
void MX_RTC_Init(void);
void MX_LPTIM1_Init(void);
void SYSCLKConfig_STOP(void);
uint16_t pec15_calc(uint8_t len,uint8_t *data );
void LTC_wake(uint16_t numChips);
void LTC_Send(uint16_t cmd16, uint8_t poll);
void LTC_Send_Recieve(uint16_t cmd16, uint8_t *outputRxData, uint16_t rxBytes);
void LTC_Write(uint16_t cmd16, uint8_t total_ic, uint8_t *data);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
  {
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* Enable Power Clock */
  //__HAL_RCC_PWR_CLK_ENABLE();

  /* Ensure that MSI is wake-up system clock */
  //__HAL_RCC_WAKEUPSTOP_CLK_CONFIG(RCC_STOP_WAKEUPCLOCK_MSI);

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  //MX_FATFS_Init();
  //MX_CAN1_Init();
  MX_SPI1_Init();
  //MX_USART1_UART_Init();
  //MX_USB_DEVICE_Init();
  //MX_SPI2_Init();
  //MX_RTC_Init();
  //MX_LPTIM1_Init();
  /* USER CODE BEGIN 2 */
  //Disable external chips
  HAL_GPIO_WritePin(BMB_CS_GPIO_PORT, BMB_CS_PIN, HIGH);
  HAL_GPIO_WritePin(CAN_STBY_GPIO_PORT, CAN_STBY_PIN, HIGH);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //Write the config register to stop Vref from going down, this makes the aux measurements more accurate
  LTC_wake(1);
  delay_u(300);
  LTC_wake(1);
  uint8_t CRGA[] = {0xFC, 0x00, 0x00, 0x00, 0x00, 0x00};
  uint8_t CRGB[] = {0x0F, 0x00, 0x00, 0x00, 0x00, 0x00};
  //CRGA[5]= (CRGA[5] | 0x01);
  LTC_Write(WRCFGA, 1, (uint8_t *) CRGA);

  while(0) //this loop cycles through the cell discharge channels for testing, ie flashes all the pretty lights!
  {
		uint32_t dcc= 0xaaaa;
		CRGA[4]=(uint8_t)(dcc);
		CRGA[5]= (CRGA[5] & 0xF0) | ((dcc >> 8) & 0x0F);
		CRGB[0]= (CRGB[0] & 0x0F) | ((dcc >> 8) & 0xF0);
		CRGB[1]= (CRGB[1] & 0xFC) | ((dcc >> 16) & 0x03);
		LTC_wake(1);
		LTC_Write(WRCFGA, 1, (uint8_t *) CRGA);
		LTC_Write(WRCFGB, 1, (uint8_t *) CRGB);
		delay_m(100);

		dcc= 0x5555;
		CRGA[4]=(uint8_t)(dcc);
		CRGA[5]= (CRGA[5] & 0xF0) | ((dcc >> 8) & 0x0F);
		CRGB[0]= (CRGB[0] & 0x0F) | ((dcc >> 8) & 0xF0);
		CRGB[1]= (CRGB[1] & 0xFC) | ((dcc >> 16) & 0x03);
		LTC_wake(1);
		LTC_Write(WRCFGA, 1, (uint8_t *) CRGA);
		LTC_Write(WRCFGB, 1, (uint8_t *) CRGB);
		delay_m(100);

		dcc= 0xffff;
		CRGA[4]=(uint8_t)(dcc);
		CRGA[5]= (CRGA[5] & 0xF0) | ((dcc >> 8) & 0x0F);
		CRGB[0]= (CRGB[0] & 0x0F) | ((dcc >> 8) & 0xF0);
		CRGB[1]= (CRGB[1] & 0xFC) | ((dcc >> 16) & 0x03);
		LTC_wake(1);
		LTC_Write(WRCFGA, 1, (uint8_t *) CRGA);
		LTC_Write(WRCFGB, 1, (uint8_t *) CRGB);
		delay_m(100);
  }
  tickstart = HAL_GetTick();
  while (1)
  {
	  /* Main loop, testing for board bring-up.
	   * start ADCV command, use 422hz
	   * Wait 12.816ms for conversion
	   * read out cell voltages
	   *
	   */

	  /*
	   * Run main loop at 10hz
	   * get a low power mode working and use an interrupt instead of polling the sysTick
	   */
	  while((HAL_GetTick() - tickstart) < main_period)
	  {
	  }
	  tickstart = HAL_GetTick();

	  LTC_wake(1);
	  delay_u(300);
	  //Measure cell voltages ################################################################
      HAL_GPIO_TogglePin(LED2_GPIO_PORT, LED2_PIN);
	  LTC_wake(1);
	  //cmd=ADCVSC | MD=0x0 | DCP=1 | CH=0x0 = Measure all cell voltages + stack at 422hz, discharge permitted
	  uint16_t cmd=ADCVSC|(MD<<7)|(0x1<<4);
	  //wait (poll ltc5813) for conversion to complete
	  LTC_Send(cmd, 1);

	  //LTC_wake(1);
	  //Measure aux voltages ################################################################
	  //cmd=ADAX | MD=0x0 | CHG=0x0 = Measure GPIO and Vref at 422hz
	  cmd=ADAX|(MD<<7)|0x0;
	  //wait (poll ltc5813) for conversion to complete
	  LTC_Send(cmd, 1);

	  //LTC_wake(1);
	  uint8_t rxBytes=6; //(3x2x8b)

	  //Read out all cell voltages, save raw ADC values in voltages array
	  uint8_t RDCVAcmds[6]={RDCVA,RDCVB,RDCVC,RDCVD,RDCVE,RDCVF}; //read all cell voltage register groups
	  for(int c=0;c<6;c++)
	  {
		  LTC_Send_Recieve(RDCVAcmds[c], (uint8_t *)aRxBuffer, rxBytes);
		  for(int i=0;i<3;i++)
		  {
			  voltages[i+c*3] = (aRxBuffer[(i<<1)+1] << 8) | aRxBuffer[(i<<1)];
		  }
	  }

	  //Read out all cell voltages, save raw ADC values in voltages array
	  uint8_t RDAUXcmds[4]={RDAUXA,RDAUXB,RDAUXC,RDAUXD}; //read all cell voltage register groups
	  for(int c=0;c<4;c++)
	  {
		  LTC_Send_Recieve(RDAUXcmds[c], (uint8_t *)aRxBuffer, rxBytes);
		  for(int i=0;i<3;i++)
		  {
			  auxVoltages[i+c*3] = (aRxBuffer[(i<<1)+1] << 8) | aRxBuffer[(i<<1)];
		  }
	  }

	  //Convert to float for debug
	  maxBrickV=(float)(voltages[0])*ADC_RESOLUTION;
	  minBrickV=(float)(voltages[0])*ADC_RESOLUTION;
	  for(int i=0; i<SIZE_OF_ARRAY(cells); i++)
	  {
		  //only convert the cells in cells[] array
		  voltagesFloat[i]=(float)(voltages[cells[i]])*0.0001;
		  //get max and min
		  if(voltagesFloat[i] > maxBrickV)
		  			  maxBrickV=voltagesFloat[i];
		  if(voltagesFloat[i] < minBrickV)
		  			  minBrickV=voltagesFloat[i];
	  }

	  //Convert to float for debug
	  for(int i=0; i<12; i++)
	  {
		  auxVoltagesFloat[i]=(float)(auxVoltages[i])*ADC_RESOLUTION;
	  }

	  current=(auxVoltagesFloat[0] - auxVoltagesFloat[1] - AMP_OFFSET_ERROR) * AMPS_PER_VOLT;


	  //Check that we are good to ride!
	  if(minBrickV > MIN_V)
	  {
		  minVctr=3;
	  }
	  else
	  {
		  if(minVctr>1)
			  minVctr--;
		  else
			  state = 0;
	  }

	  //Check that we are good to charge!
	  if(maxBrickV < MAX_V)
	  {
		  maxVctr=3;
	  }
	  else
	  {
		  if(maxVctr)
			  maxVctr--;
		  else
			  state = 0;
	  }
	  //state=0;
	  switch(state){
	  case 0:
		  // off state.. wait for reset
		  //turn off load FET
		  CRGB[0]= (CRGB[0] | 0x01);
		  //turn off charge FET
		  CRGB[0]= (CRGB[0] | 0x02);
		  break;
	  case 1:
		  //check buttons
/*		  if(resetbutton)
			  state=2;
		  else if(chargebutton)
			  state=3;
*/
		  break;
	  case 2:
		  //turn on load FET
		  CRGB[0]= (CRGB[0] & ~0x01);
		  state=1;
		  break;
	  case 3:
		  //turn on charge FET
		  CRGB[0]= (CRGB[0] & ~0x02);
		  state=1;
		  break;
	  default:
		  state=0;
		  break;
	  }

	  LTC_wake(1);
	  delay_u(300);
	  LTC_Write(WRCFGB, 1, (uint8_t *) CRGB);
	  LTC_Write(WRCFGA, 1, (uint8_t *) CRGA);
  }
}

void LTC_wake(uint16_t numChips)
{
	for(int i=0; i<numChips; i++)
	{
		HAL_GPIO_WritePin(BMB_CS_GPIO_PORT, BMB_CS_PIN, LOW);
		//short delay
		//HAL_Delay(1);
		uint8_t cmd[]={0x00, 0x00};
		//for(int x=0;x<10;x++)
		//{
			HAL_GPIO_WritePin(BMB_CS_GPIO_PORT, BMB_CS_PIN, LOW);
			HAL_SPI_Transmit(&hspi1, (uint8_t*)cmd, 1, 100);
			HAL_GPIO_WritePin(BMB_CS_GPIO_PORT, BMB_CS_PIN, HIGH);
	//}
	}
}

void LTC_Send(uint16_t cmd16, uint8_t poll)
{
	/*
	 * Sends command+PEC over SPI
	 * If poll != 0 then it also waits for SDO to be asserted, this is used for polling
	 * the LTC6813 for end of ADC conversion.
	 */
	//Construct 4 byte command structure, 2 bytes for CMD, 2 bytes for PEC
	uint8_t cmd[4];
	cmd[0] = (uint8_t)(cmd16 >> 8);
	cmd[1] = (uint8_t)(cmd16);
	uint16_t cmd_pec = pec15_calc(2, cmd);
	cmd[2] = (uint8_t)(cmd_pec >> 8);
	cmd[3] = (uint8_t)(cmd_pec);

    uint8_t pRxData[]={0x00, 0x00};
	//Send command
	HAL_GPIO_WritePin(BMB_CS_GPIO_PORT, BMB_CS_PIN, LOW);
	switch(HAL_SPI_Transmit(&hspi1, (uint8_t*)cmd, 4, 100))
	  {
	    case HAL_OK:
	      //Toggle LED on: Transfer in transmission/Reception process is correct
	      HAL_GPIO_TogglePin(LED1_GPIO_PORT, LED1_PIN);
	      break;

	    case HAL_TIMEOUT:
	      /* An Error Occur ______________________________________________________ */
	    case HAL_ERROR:
	      /* Call Timeout Handler */
	      Error_Handler();
	      break;
	    default:
	      break;
	  }

	cmd[0] = 0x00;
	//poll for end of ADC conversion
	while(pRxData[0]==0x00)// & poll)
	{
	  switch(HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)cmd, (uint8_t*)pRxData, 1, 100))
		  {
			case HAL_OK:
				HAL_GPIO_TogglePin(busContactor_GPIO_PORT, busContactor_PIN);
			break;
			default:
				Error_Handler();
			break;
		  }
	}

	HAL_GPIO_WritePin(BMB_CS_GPIO_PORT, BMB_CS_PIN, HIGH);
}

void LTC_Write(uint16_t cmd16, uint8_t total_ic, uint8_t *data)
{
	/*
	* Sends command+PEC and data+PEC over SPI for all ICs in chain
	*/
	const uint8_t BYTES_IN_REG = 6;
	const uint8_t CMD_LEN = 4+(8*total_ic);
	uint8_t *cmd;
	uint16_t data_pec;
	uint16_t cmd_pec;
	uint8_t cmd_index;
	//allocate memory for entire data packet
	cmd = (uint8_t *)malloc(CMD_LEN*sizeof(uint8_t));
	//Construct 4 byte command structure, 2 bytes for CMD, 2 bytes for PEC
	cmd[0] = (uint8_t)(cmd16 >> 8);
	cmd[1] = (uint8_t)(cmd16);
	cmd_pec = pec15_calc(2, cmd);
	cmd[2] = (uint8_t)(cmd_pec >> 8);
	cmd[3] = (uint8_t)(cmd_pec);

	cmd_index = 4;
	for (uint8_t current_ic = total_ic; current_ic > 0; current_ic--)       // executes for each LTC681x in daisy chain, this loops starts with
	{
		// the last IC on the stack. The first configuration written is
		// received by the last IC in the daisy chain

		for (uint8_t current_byte = 0; current_byte < BYTES_IN_REG; current_byte++)
		{
			cmd[cmd_index] = data[((current_ic-1)*6)+current_byte];
			cmd_index = cmd_index + 1;
		}

		data_pec = (uint16_t)pec15_calc(BYTES_IN_REG, &data[(current_ic-1)*6]);    // calculating the PEC for each Iss configuration register data
		cmd[cmd_index] = (uint8_t)(data_pec >> 8);
		cmd[cmd_index + 1] = (uint8_t)data_pec;
		cmd_index = cmd_index + 2;
	}


	cs_low();
	switch(HAL_SPI_Transmit(&hspi1, (uint8_t*)cmd, CMD_LEN, 100))
	  {
	    case HAL_OK:
	      //Toggle LED on: Transfer in transmission/Reception process is correct
	      HAL_GPIO_TogglePin(LED1_GPIO_PORT, LED1_PIN);
	      break;

	    case HAL_TIMEOUT:
	      /* An Error Occur ______________________________________________________ */
	    case HAL_ERROR:
	      /* Call Timeout Handler */
	      Error_Handler();
	      break;
	    default:
	      break;
	  }
	cs_high();
	free(cmd);
}

void LTC_Send_Recieve(uint16_t cmd16, uint8_t *outputRxData, uint16_t rxBytes)
{
	/*
	 * Sends command+PEC over SPI, receives reply payload, checks PEC reply
	 */
	uint8_t pRxData[12];
	uint8_t cmd[12]; //bigger than any protocol packet we expect
	cmd[0] = (uint8_t)(cmd16 >> 8);
	cmd[1] = (uint8_t)(cmd16);
	uint16_t cmd_pec = pec15_calc(2, cmd);
	cmd[2] = (uint8_t)(cmd_pec >> 8);
	cmd[3] = (uint8_t)(cmd_pec);
	//pack the rest of the tx packet with zeros
	for(int i=4;i<12;i++)
		cmd[i]=0x00;

	HAL_GPIO_WritePin(BMB_CS_GPIO_PORT, BMB_CS_PIN, LOW);
	switch(HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)cmd, (uint8_t*)pRxData, rxBytes+6, 100))
	  {
	    case HAL_OK:
	      //Toggle LED on: Transfer in transmission/Reception process is correct
	      HAL_GPIO_TogglePin(LED1_GPIO_PORT, LED1_PIN);

	      for(int i=0; i<rxBytes; i++)
	      {
	    	  outputRxData[i]=pRxData[i+4];
	      }


	      break;

	    case HAL_TIMEOUT:
	      /* An Error Occur ______________________________________________________ */
	    case HAL_ERROR:
	      /* Call Timeout Handler */
	      Error_Handler();
	      break;
	    default:
	      break;
	  }
	HAL_GPIO_WritePin(BMB_CS_GPIO_PORT, BMB_CS_PIN, HIGH);

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Configure LSE Drive Capability
    */
  HAL_PWR_EnableBkUpAccess();

  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_LPTIM1|RCC_PERIPHCLK_USB;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Lptim1ClockSelection = RCC_LPTIM1CLKSOURCE_PCLK;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the main internal regulator output voltage
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/10000);//100us

    /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /**Enable MSI Auto calibration
    */
  HAL_RCCEx_EnableMSIPLLMode();

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
  * @brief  Configures system clock after wake-up from STOP: enable MSI, PLL
  *         and select PLL as system clock source.
  * @param  None
  * @retval None
  */
void SYSCLKConfig_STOP(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  uint32_t pFLatency = 0;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* Get the Oscillators configuration according to the internal RCC registers */
  HAL_RCC_GetOscConfig(&RCC_OscInitStruct);

  /* Enable PLL */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* Get the Clocks configuration according to the internal RCC registers */
  HAL_RCC_GetClockConfig(&RCC_ClkInitStruct, &pFLatency);

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource  = RCC_SYSCLKSOURCE_PLLCLK;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, pFLatency) != HAL_OK)
  {
    Error_Handler();
  }
}

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 16;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* LPTIM1 init function */
void MX_LPTIM1_Init(void)
{

  hlptim1.Instance = LPTIM1;
  hlptim1.Init.Clock.Source = LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC;
  hlptim1.Init.Clock.Prescaler = LPTIM_PRESCALER_DIV1;
  hlptim1.Init.Trigger.Source = LPTIM_TRIGSOURCE_SOFTWARE;
  hlptim1.Init.OutputPolarity = LPTIM_OUTPUTPOLARITY_HIGH;
  hlptim1.Init.UpdateMode = LPTIM_UPDATE_IMMEDIATE;
  hlptim1.Init.CounterSource = LPTIM_COUNTERSOURCE_INTERNAL;
  hlptim1.Init.Input1Source = LPTIM_INPUT1SOURCE_GPIO;
  hlptim1.Init.Input2Source = LPTIM_INPUT2SOURCE_GPIO;
  if (HAL_LPTIM_Init(&hlptim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* RTC init function */
void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

    /**Initialize RTC Only 
    */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 0x7F;//127;
  hrtc.Init.SynchPrediv = 0xF9;//255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  //hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/* SPI1 init function */
void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;//SPI_DATASIZE_4BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;//SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI2 init function */
void MX_SPI2_Init(void)
{

  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA2 PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA3 PA4 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB10 PB12 PB3 
                           PB4 PB5 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/**
  * @brief  TxRx Transfer completed callback.
  * @param  hspi: SPI handle
  * @note   This example shows a simple way to report end of DMA TxRx transfer, and
  *         you can add your own implementation.
  * @retval None
  */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi1)
{
  wTransferState = TRANSFER_COMPLETE;
}

/**
  * @brief  SPI error callbacks.
  * @param  hspi: SPI handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi1)
{
  wTransferState = TRANSFER_ERROR;
}

/*
  Calculates  and returns the CRC15
  */
uint16_t pec15_calc(uint8_t len, //Number of bytes that will be used to calculate a PEC
                    uint8_t *data //Array of data that will be used to calculate  a PEC
                   )
{
  uint16_t remainder,addr;

  remainder = 16;//initialize the PEC
  for (uint8_t i = 0; i<len; i++) // loops for each byte in data array
  {
    addr = ((remainder>>7)^data[i])&0xff;//calculate PEC table address
    remainder = (remainder<<8)^crc15Table[addr];
  }
  return(remainder*2);//The CRC15 has a 0 in the LSB so the remainder must be multiplied by 2
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
	 HAL_GPIO_TogglePin(LED3_GPIO_PORT, LED3_PIN);
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
