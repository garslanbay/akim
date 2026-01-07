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
#include <stdio.h>
#include <stdlib.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* =====================================================
   FLASH KAYIT YAPISI (16 BYTE – DOĞRU HİZALAMA)
   ===================================================== */

typedef struct
{
    uint32_t timestamp;    // saniye sayacı veya RTC epoch
    float    total_energy_Wh;    // 10 dk enerji
    uint32_t crc;          // ilk 8 byte CRC
    uint32_t reserved;     // hizalama / gelecek
} energy_record_t;

typedef struct __attribute__((packed))
{
    uint32_t total_seconds;     // zaman
    float    total_energy_Wh;   // TOPLAM enerji
    uint32_t crc;
    uint32_t reserved;
} log_10min_t; // 16 byte

typedef struct __attribute__((packed))
{
    uint32_t day_index;         // total_seconds / 86400
    float    daily_energy_Wh;
    uint32_t crc;
    uint32_t reserved;
} log_daily_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

extern uint8_t kontrol;
extern void NRF24L01_SLEEP_Mode(void);
extern void NRF24L01_WAKEUP_Mode(void);
extern uint8_t NRF24L01_TxPacket(uint8_t *txbuf);
extern void NRF24L01_TX_Mode(void);
extern uint8_t NRF24L01_Read_Reg(uint8_t reg, uint8_t *bufp,uint16_t len);
extern void NRF24L01_SPI_Init(void);
uint32_t energy_crc32(const uint8_t *data, uint32_t len);
#define MyAdress		0x05

/* USER CODE END PD */

#define ADC_BUF_SIZE          2000        // 2 kHz * 1 saniye
#define RMS_FACTOR            1.11f
#define MAINS_VOLTAGE         230.0f

#define ENERGY_LOG_INTERVAL_S 600         // 10 dakika
//#define FLASH_PAGE_SIZE       2048

#define SECONDS_PER_DAY   86400UL
#define DAYS_PER_MONTH    30UL
#define SECONDS_PER_MONTH (SECONDS_PER_DAY * DAYS_PER_MONTH)


#define FLASH_BASE_ADDR2   0x08000000
#define FLASH_SIZE_BYTES  (64 * 1024)   // 64 KB

#define FLASH_END_ADDR    (FLASH_BASE_ADDR2 + FLASH_SIZE_BYTES)
/* Son iki page */
#define PAGE_10MIN_ADDR      0x0800F800
#define PAGE_DAILY_ADDR      0x0800F000

#define PAGE_10MIN_END       0x08010000
#define PAGE_DAILY_END       0x0800F800
#define FLASH_LOG_START_ADDR 0x0800F800
#define FLASH_LOG_END_ADDR   0x08010000
#define FLASH_BASE_ADDR       FLASH_LOG_START_ADDR   // ÖRNEK: son page

#define SMPS_RIPPLE_TH        300
#define FLASH_EMPTY_DW        0xFFFFFFFFFFFFFFFFULL
/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define FLASH_IS_VALID(addr) \
    ((addr) >= 0x08000000 && (addr) < 0x08010000)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
TIM_HandleTypeDef htim14;

/* USER CODE BEGIN PV */

__attribute__((section(".flash_log")))
uint8_t flash_log_area[2048];

TIM_HandleTypeDef htim3;
static uint16_t adc_dma_buf[ADC_BUF_SIZE];
volatile uint8_t  adc_buf_ready = 0;
volatile uint8_t  one_second_tick = 0;

static float energy_acc_Wh = 0.0f;
static float energy_total_Wh;   // kalıcı (toplam)
static uint32_t total_seconds  = 0;
energy_record_t last;

uint32_t flash_write_addr = FLASH_BASE_ADDR;

uint8_t adc_factor=0;

float Vcc=0.0,temp=0.0,adc_volt=0.0,current=0.0,sum_volt=0,sum_current=0.0;
int32_t ort_vref=0,ort_temp=0,ort_volt=0;
uint16_t sayac=0,sayac2=0;
float daily_energy_Wh=0.0f;

uint8_t reg,return_reg=0;

uint8_t giden[30];
volatile uint8_t nrf_tx_request = 0;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */
void MX_TIM3_Init(void);
void Flash_Write_Energy(float energy);
void MX_TIM14_Init(void);
/* USER CODE END PFP */


static void Flash_Unlock(void)
{
    HAL_FLASH_Unlock();
}

static void Flash_Lock(void)
{
    HAL_FLASH_Lock();
}
static inline int Flash_Addr_Valid(uint32_t addr)
{
    return (addr >= FLASH_BASE_ADDR2 &&
            addr < FLASH_END_ADDR);
}


static uint32_t Flash_Find_Next_Addr(uint32_t start, uint32_t end, uint32_t rec_size)
{
    uint32_t addr = start;

    /* 4-byte alignment garanti */
    if (addr & 0x3)
        addr = (addr + 3) & ~0x3;

    while ((addr + rec_size) <= end)
    {
        /* SADECE word oku */
    	//if (!FLASH_IS_VALID(addr)) return addr;
        if (!Flash_Addr_Valid(addr))
            break;   // FLASH DIŞI → DUR
        uint32_t v = *(volatile uint32_t*)addr;

        if (v == 0xFFFFFFFF)
            return addr;

        addr += rec_size;
    }

    /* Page dolu → yazan fonksiyon erase edecek */
    return start;
}

void Flash_Write_10min(uint32_t total_seconds, float total_energy_Wh)
{
    log_10min_t rec;
    rec.total_seconds   = total_seconds;
    rec.total_energy_Wh = total_energy_Wh;
    rec.reserved        = 0xFFFFFFFF;
    rec.crc = energy_crc32((uint8_t*)&rec, 8);

    uint32_t addr = Flash_Find_Next_Addr(PAGE_10MIN_ADDR, PAGE_10MIN_END, sizeof(rec));

    if (addr + sizeof(energy_record_t) > FLASH_LOG_END_ADDR)
    {
        Flash_Erase_Log_Page();
        addr = FLASH_LOG_START_ADDR;
    }
    Flash_Unlock();
    uint32_t *p = (uint32_t*)&rec;
    for (uint32_t i = 0; i < sizeof(rec)/4; i++)
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr + i*4, p[i]);
    Flash_Lock();
}

void Flash_Write_Daily(uint32_t day_index, float daily_energy_Wh)
{
    log_daily_t rec;
    rec.day_index       = day_index;
    rec.daily_energy_Wh = daily_energy_Wh;
    rec.reserved        = 0xFFFFFFFF;
    rec.crc = energy_crc32((uint8_t*)&rec, 8);

    uint32_t addr = Flash_Find_Next_Addr(PAGE_DAILY_ADDR, PAGE_DAILY_END, sizeof(rec));

    Flash_Unlock();
    uint32_t *p = (uint32_t*)&rec;
    for (uint32_t i = 0; i < sizeof(rec)/4; i++)
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr + i*4, p[i]);
    Flash_Lock();
}

uint8_t Flash_Read_Last_10min(float *total_energy_Wh)
{
    uint32_t addr = PAGE_10MIN_ADDR;
    log_10min_t *rec, last = {0};

    while (addr + sizeof(log_10min_t) <= PAGE_10MIN_END)
    {
        rec = (log_10min_t*)addr;
        if (rec->total_seconds == 0xFFFFFFFF) break;

        uint32_t crc = energy_crc32((uint8_t*)rec, 8);
        if (crc == rec->crc) last = *rec;

        addr += sizeof(log_10min_t);
    }

    if (last.total_seconds != 0)
    {
        *total_energy_Wh = last.total_energy_Wh;
        return 1;
    }
    return 0;
}

uint8_t Flash_Read_Last_Daily(float *daily_energy_Wh)
{
    uint32_t addr = PAGE_DAILY_ADDR;
    log_daily_t *rec, last = {0};

    while (addr + sizeof(log_daily_t) <= PAGE_DAILY_END)
    {
        rec = (log_daily_t*)addr;
        if (rec->day_index == 0xFFFFFFFF) break;

        uint32_t crc = energy_crc32((uint8_t*)rec, 8);
        if (crc == rec->crc) last = *rec;

        addr += sizeof(log_daily_t);
    }

    if (last.day_index != 0)
    {
        *daily_energy_Wh = last.daily_energy_Wh;
        return 1;
    }
    return 0;
}



/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
 uint32_t energy_crc32(const uint8_t *data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFF;
    for (uint32_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++)
            crc = (crc & 1) ? (crc >> 1) ^ 0xEDB88320UL : (crc >> 1);
    }
    return ~crc;
}

void Flash_Erase_Page(void)
{
    FLASH_EraseInitTypeDef erase;
    uint32_t err;

    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Page = (FLASH_LOG_START_ADDR - 0x08000000) / FLASH_PAGE_SIZE;
    erase.NbPages = 1;

    HAL_FLASH_Unlock();
    HAL_FLASHEx_Erase(&erase, &err);
    HAL_FLASH_Lock();
}

void Flash_Save_State(uint32_t sec, float energy)
{
    energy_record_t rec;

    rec.timestamp   = sec;
    rec.total_energy_Wh = energy;
    rec.reserved        = 0xFFFFFFFF;
    rec.crc = energy_crc32((uint8_t*)&rec, 8);

    HAL_FLASH_Unlock();

    uint64_t *p = (uint64_t*)&rec;

    HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                      FLASH_LOG_START_ADDR, p[0]);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                      FLASH_LOG_START_ADDR + 8, p[1]);

    HAL_FLASH_Lock();
}
uint8_t Flash_Load_State(uint32_t *sec, float *energy)
{
    energy_record_t rec;

    memcpy(&rec, (void*)FLASH_LOG_START_ADDR, sizeof(rec));

    if (rec.timestamp == 0xFFFFFFFF)
        return 0;

    if (energy_crc32((uint8_t*)&rec, 8) != rec.crc)
        return 0;

    *sec    = rec.timestamp;
    *energy = rec.total_energy_Wh;
    return 1;
}

void Energy_Init(void)
{
    if (!Flash_Load_State(&total_seconds, &energy_total_Wh))
    {
        total_seconds   = 0;
        energy_total_Wh = 0.0f;
    }
}

void Flash_Erase_Log_Page(void)
{
    FLASH_EraseInitTypeDef erase;
    uint32_t error;

    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Page = (FLASH_LOG_START_ADDR - 0x08000000) / FLASH_PAGE_SIZE;
    erase.NbPages = 1;

    HAL_FLASH_Unlock();
    HAL_FLASHEx_Erase(&erase, &error);
    HAL_FLASH_Lock();

    flash_write_addr = FLASH_LOG_START_ADDR;
}
void Flash_Write_Total_Energy(void)
{
    energy_record_t rec;

    rec.timestamp = total_seconds;
    rec.total_energy_Wh  = energy_total_Wh;
    rec.reserved  = 0xFFFFFFFF;
    rec.crc       = energy_crc32((uint8_t*)&rec, 8);

    if (flash_write_addr + sizeof(energy_record_t) > FLASH_LOG_END_ADDR)
    {
        Flash_Erase_Log_Page();
    }

    HAL_FLASH_Unlock();

    uint64_t *p = (uint64_t*)&rec;

    HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                      flash_write_addr,
                      p[0]);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                      flash_write_addr + 8,
                      p[1]);

    HAL_FLASH_Lock();

    flash_write_addr += sizeof(energy_record_t);
}



/* =====================================================
   SMPS YAKLAŞTIRMA
   ===================================================== */

static float SMPS_Correction(float Irms, uint16_t ripple)
{
    float c = 1.0f;

    if (Irms < 0.3f)       c = 1.30f;
    else if (Irms < 0.6f)  c = 1.15f;
    else if (Irms < 1.0f)  c = 1.05f;

    if (ripple > SMPS_RIPPLE_TH)
        c *= 1.10f;

    return c;
}

/* =====================================================
   1 SANİYELİK ADC VERİ İŞLEME
   ===================================================== */
volatile uint8_t flash_write_req = 0;
float Irms=0.0f,isum=0.0f,i_1dakika=0.0f;
static void Process_1s(uint16_t *buf)
{
    uint32_t sum = 0;
    uint16_t min = 0xFFFF, max = 0;
    float current_gain;

    for (uint16_t i = 0; i < ADC_BUF_SIZE; i++)
    {
        uint16_t v = buf[i];
        sum += v;
        if (v < min) min = v;
        if (v > max) max = v;
    }

    float avg = (float)sum / ADC_BUF_SIZE;

    // ---- KALİBRASYON ----
    float adc_offset = 6.0f;
    current_gain = 0.0142f;


    float I_avg = (avg - adc_offset) * current_gain;
    if (I_avg < 0.02f) I_avg = 0;


     Irms = I_avg * 2.22f;

     if(Irms>3.0f)
    	 Irms=Irms*0.91f;

    float power_W = Irms * 230.0f;
    isum+=Irms;

    energy_acc_Wh += power_W / 3600.0f;
    energy_total_Wh+=power_W / 3600.0f;

}

void TIM14_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&htim14, TIM_FLAG_UPDATE))
    {
        __HAL_TIM_CLEAR_IT(&htim14, TIM_IT_UPDATE);
        one_second_tick = 1;
    }
}

/* =====================================================
   DMA CALLBACK → TAM 1 SANİYE
   ===================================================== */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    	 adc_buf_ready = 1;
        //Process_1s(adc_dma_buf);
}

/* =====================================================
   FLASH YAZMA (GÜVENLİ + WEAR LEVELING)
   ===================================================== */

uint8_t Flash_Slot_Empty(uint32_t addr)
{
    uint32_t *p = (uint32_t*)addr;
    return (p[0] == 0xFFFFFFFF && p[1] == 0xFFFFFFFF);
}
void Monthly_Reset(void)
{
    Flash_Erase_Page();
    energy_total_Wh = 0.0f;
    total_seconds  = 0;
}

static uint8_t Record_Is_Valid(const energy_record_t *rec)
{
    uint32_t crc_calc = energy_crc32((uint8_t*)&rec->timestamp, 8);
    return (crc_calc == rec->crc);
}
uint32_t Flash_Find_Last_Timestamp(void)
{
    uint32_t last_ts = 0;

    uint32_t addr = FLASH_LOG_START_ADDR;

    while (addr + 8 <= FLASH_LOG_END_ADDR)
    {
        uint32_t *p = (uint32_t*)addr;

        if (p[0] == 0xFFFFFFFF && p[1] == 0xFFFFFFFF)
        {
            break; // boş slot
        }

        last_ts = p[0];   // timestamp'i sakla
        addr += sizeof(energy_record_t);
    }

    /* Yazma adresini de güncelle */
    flash_write_addr = addr;

    return last_ts;
}

uint8_t Flash_Read_Last_Record(energy_record_t *out)
{
    uint32_t addr = FLASH_LOG_START_ADDR;
    energy_record_t tmp;
    uint8_t found = 0;

    while (addr + sizeof(energy_record_t) <= FLASH_LOG_END_ADDR)
    {
        memcpy(&tmp, (void*)addr, sizeof(energy_record_t));

        if (tmp.timestamp == 0xFFFFFFFF)
            break;

        if (Record_Is_Valid(&tmp))
        {
            *out = tmp;
            found = 1;
        }

        addr += sizeof(energy_record_t);
    }

    flash_write_addr = addr;
    return found;
}

void Flash_Write_Energy(float energy_Wh)
{
    energy_record_t rec;

    rec.timestamp = total_seconds;
    rec.total_energy_Wh = energy_Wh;
    rec.reserved  = 0xFFFFFFFF;
    rec.crc       = energy_crc32((uint8_t*)&rec, 8);

    /* Page doluysa sil */
    if (flash_write_addr + sizeof(energy_record_t) > FLASH_LOG_END_ADDR)
    {
        Flash_Erase_Log_Page();
    }

    HAL_FLASH_Unlock();

    uint64_t *p = (uint64_t*)&rec;

    HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                      flash_write_addr,
                      p[0]);

    HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                      flash_write_addr + 8,
                      p[1]);

    HAL_FLASH_Lock();

    flash_write_addr += sizeof(energy_record_t);
}
/* =====================================================
   BAŞLATMA
   ===================================================== */

void EnergyMeter_Start(void)
{
    HAL_TIM_Base_Start(&htim3);  // 2 kHz TRGO
    HAL_TIM_Base_Start_IT(&htim14); // 1 Hz zaman
    HAL_ADC_Start_DMA(&hadc1,
                      (uint32_t*)adc_dma_buf,
                      ADC_BUF_SIZE);
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

  __HAL_RCC_PWR_CLK_ENABLE();
  /* Check if the system was resumed from Standby mode */
  if (__HAL_PWR_GET_FLAG(PWR_FLAG_SB) != RESET)
  {
    /* Clear Standby flag */
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUFI);
    /* Check and Clear the Wakeup flag */
    if (__HAL_PWR_GET_FLAG(PWR_FLAG_WUF2) != RESET)
    {
      __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF2);
    }

  }

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  HAL_Delay(1);   // <<< KRİTİK
  MX_TIM3_Init();
  MX_TIM14_Init();
  NRF24L01_SPI_Init();
  HAL_ADCEx_Calibration_Start(&hadc1);
  adc_factor=HAL_ADCEx_Calibration_GetValue(&hadc1);

  if (Flash_Read_Last_Record(&last))
  {
	    total_seconds   = last.timestamp;
	    energy_total_Wh = last.total_energy_Wh;
  }
  else
  {
      total_seconds = 0;
      energy_acc_Wh = 0.0f;
  }

  Flash_Read_Last_10min(&energy_total_Wh);
  EnergyMeter_Start();

  uint32_t test = *(uint32_t*)FLASH_LOG_START_ADDR;
  /* USER CODE BEGIN 2 */
  //BSP_SPI1_Init();


 // __HAL_RCC_PWR_CLK_ENABLE();



  //return_reg=NRF24L01_Read_Reg(0x07,&reg,1);

  //Flash_Erase_Log_Page();//bir kere cagir

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


	  if (one_second_tick && adc_buf_ready)
	     {
	         one_second_tick = 0;
	         adc_buf_ready = 0;

	         Process_1s(adc_dma_buf);
	         total_seconds++;

	         if (total_seconds % 600 == 0)
	         {
	        	 //Flash_Write_10min(total_seconds, energy_total_Wh);
	        	 Flash_Write_Energy(energy_total_Wh);
	         }
	         if (total_seconds % 86400 == 0)
	         {
	             //Flash_Write_Daily(total_seconds/86400, daily_energy_Wh);
	             //daily_energy_Wh = 0.0f;
	         }
	         if (total_seconds % 60 == 0)
	         {
	        	 i_1dakika=isum/60.0f;
	        	 isum=0.0f;
	        	 sprintf((char *)giden, "%x %08.2f %05.2f %05d", MyAdress, energy_total_Wh,i_1dakika,total_seconds);
	        	 nrf_tx_request = 1; // gönderim isteği
	         }
	     }
	  if(nrf_tx_request)
	  {
	      nrf_tx_request = 0; // işaret sıfırla

	      NRF24L01_WAKEUP_Mode();
	      NRF24L01_TX_Mode();
	      NRF24L01_TxPacket(giden);
	      NRF24L01_SLEEP_Mode();
	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  /*
	  kontrol=0;
	  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_data, 150);
	  HAL_Delay(50);
	  while(!kontrol);
	  HAL_ADC_Stop_DMA(&hadc1);

	  HAL_Delay(100);

	  sum_volt=sum_vref=sum_temp=0;

	  for(sayac=0;sayac<150;sayac+=3)
	  {

		  sum_volt+=adc_data[sayac];
		  sum_vref+=adc_data[sayac+1]-adc_factor;
		  sum_temp+=adc_data[sayac+2]-adc_factor;
	  }

	  ort_volt=(sum_volt/50);
	  if(ort_volt>=1)
		  ort_volt-=1;
	  ort_vref=sum_vref/50;
	  ort_temp=sum_temp/50;
	  Vcc=__HAL_ADC_CALC_VREFANALOG_VOLTAGE(ort_vref>>3,ADC_RESOLUTION_12B);
	  temp=__HAL_ADC_CALC_TEMPERATURE(Vcc,ort_temp>>3,ADC_RESOLUTION_12B);
	  Vcc/=1000.0;


	  adc_volt=(Vcc/32768.0)*(ort_volt);

	  if(sum_current<0)
		  sum_current=0;
	  //adc_volt=(Vcc/4095.0)*(ort_volt>>3);
	  sum_current+=adc_volt;///ma, A icin /100

	  if(++sayac2>9)
	  {
		  sayac2=0;
		  current=sum_current*3.33;   //A //*1000 ma icin
		  sum_current=0;

		  sprintf((char *)giden,"%x %.2f",MyAdress,current); // @suppress("Float formatting support")

		  NRF24L01_WAKEUP_Mode();
		  NRF24L01_TX_Mode();
		  NRF24L01_TxPacket(giden);
		  NRF24L01_SLEEP_Mode();

		  HAL_Delay(60000);
	  }*/
/*
	  HAL_SPI_DeInit(&hspi1);


		  HAL_PWREx_EnablePullUpPullDownConfig();

		  HAL_PWREx_EnableGPIOPullUp(PWR_GPIO_C, GPIO_PIN_All);
		  HAL_PWREx_EnableGPIOPullUp(PWR_GPIO_B, GPIO_PIN_All);
		  HAL_PWREx_EnableGPIOPullUp(PWR_GPIO_A, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4
	 	  	  	   |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12);


	  HAL_ADCEx_DisableVoltageRegulator(&hadc1);
	  HAL_SuspendTick();


	  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUFI);

	  __HAL_PWR_INTERNALWAKEUP_DISABLE();
	  HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN2_LOW);
	  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF2);*/
	  //HAL_PWR_EnterSTANDBYMode();


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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
void MX_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
    hadc1.Init.LowPowerAutoWait = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;   // KRİTİK
    hadc1.Init.NbrOfConversion = 1;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T3_TRGO;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
    hadc1.Init.DMAContinuousRequests = ENABLE;
    hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
    hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_39CYCLES_5;

    HAL_ADC_Init(&hadc1);

    /* ADC KANALI */
    sConfig.Channel = ADC_CHANNEL_0;     // KULLANDIĞIN PIN
    sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;

    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

	HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
	__HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(&hrtc, RTC_FLAG_WUTF);

  /** Enable the WakeUp
  */
  if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 60, RTC_WAKEUPCLOCK_CK_SPRE_16BITS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */

void MX_TIM14_Init(void)
{
    __HAL_RCC_TIM14_CLK_ENABLE();

    htim14.Instance = TIM14;
    htim14.Init.Prescaler = 64000 - 1;   // 64 MHz / 64000 = 1 kHz
    htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim14.Init.Period = 1000 - 1;       // 1 Hz
    htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&htim14);

    HAL_NVIC_SetPriority(TIM14_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM14_IRQn);
}
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
void MX_DMA_Init(void)
{
    __HAL_RCC_DMA1_CLK_ENABLE();

    hdma_adc1.Instance = DMA1_Channel1;
    hdma_adc1.Init.Request = DMA_REQUEST_ADC1;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode = DMA_CIRCULAR;     // SÜREKLİ ÖLÇÜM
    hdma_adc1.Init.Priority = DMA_PRIORITY_HIGH;

    HAL_DMA_Init(&hdma_adc1);

    __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);

    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(NRF_CS_GPIO_Port, NRF_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB9 PB2 PB3 PB4
                           PB5 PB6 PB7 PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA2 PA3 PA4
                           PA8 PA9 PA10 PA11
                           PA12 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : NRF_CS_Pin */
  GPIO_InitStruct.Pin = NRF_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(NRF_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : NRF_IRQ_Pin */
  GPIO_InitStruct.Pin = NRF_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(NRF_IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : NRF_CE_Pin */
  GPIO_InitStruct.Pin = NRF_CE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(NRF_CE_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void MX_TIM3_Init(void)
{

	__HAL_RCC_TIM3_CLK_ENABLE();
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 6399;          // 64MHz / (63+1) = 1 MHz
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 4;            // 1MHz / (499+1) = 2000 Hz
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    HAL_TIM_Base_Init(&htim3);

    /* ADC TRIGGER */
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

    HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);
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
