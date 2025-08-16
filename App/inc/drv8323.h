#ifndef __DRV8323_H__
#define __DRV8323_H__
		 	 	  
//#include "main.h"
//#include "spi.h"

#include "n32g45x.h"
#include "n32g45x_conf.h"


//SPI 
#define DRC8323_SPI   				SPI1
#define DRC8323_SPI_CLK             RCC_APB2_PERIPH_SPI1
//SCK 引脚
#define DRV8323_SPI_SCK_PIN   		GPIO_PIN_5
#define DRV8323_SPI_SCK_PORT      	GPIOA
//MISO 引脚
#define DRV8323_SPI_MISO_PIN        GPIO_PIN_6
#define DRV8323_SPI_MISO_PORT  		GPIOA
//MOSI 引脚
#define DRV8323_SPI_MOSI_PIN        GPIO_PIN_7
#define DRV8323_SPI_MOSI_PORT  		GPIOA
//CS(NSS) 引脚
#define DRV8323_SPI_NSS_PIN    		GPIO_PIN_4
#define DRV8323_SPI_NSS_PORT   		GPIOA
//DRV8323使能引脚
#define DRV8323_GATE_EN_PIN    		GPIO_PIN_12
#define DRV8323_GATE_EN_PORT   		GPIOB
//DRV8323G故障引脚
#define DRV8323_nFAULT_PIN    		GPIO_PIN_7
#define DRV8323_nFAULT_PORT   		GPIOB
#define nFAULT_EXTI_PORT_SOURCE   GPIOB_PORT_SOURCE
#define nFAULT_EXTI_PIN_SOURCE    GPIO_PIN_SOURCE7
#define nFAULT_EXTI_LINE          EXTI_LINE7
#define nFAULT_EXTI_IRQn          EXTI9_5_IRQn
#define nFAULT_EXTI_IRQHandler    EXTI9_5_IRQHandler
//DRV8323G校准引脚
#define DRV8323_CAL_EN_PIN    		GPIO_PIN_6
#define DRV8323_CAL_EN_PORT   		GPIOB


//SPI片选
#define DRV8323_SPI_NSS_LOW()  	GPIO_WriteBit(DRV8323_SPI_NSS_PORT, DRV8323_SPI_NSS_PIN, Bit_RESET);
#define DRV8323_SPI_NSS_HIGH() 	GPIO_WriteBit(DRV8323_SPI_NSS_PORT, DRV8323_SPI_NSS_PIN, Bit_SET);
//芯片使能
#define DRV8323_GATE_EN_LOW()  	GPIO_WriteBit(DRV8323_GATE_EN_PORT, DRV8323_GATE_EN_PIN, Bit_RESET);
#define DRV8323_GATE_EN_HIGH() 	GPIO_WriteBit(DRV8323_GATE_EN_PORT, DRV8323_GATE_EN_PIN, Bit_SET);


void DRV8323_GPIO_Init(void);
void DRV8323_SPI_Init(void);  //初始化SPI
void Set_DRV8323(void);       // 用SPI配置DRV8323
uint16_t SPI_WRITE_DRV8323(uint16_t RegAddr,uint16_t Data);// DRV8323写入函数 16位
uint16_t SPI_READ_DRV8323(uint16_t RegAddr,uint16_t Data);  // DRV8323读取函数 16位
uint16_t DRV8323_do_checks(void);
//uint16_t SPI_ReadWrite16bit(uint16_t ReadAddr);// SPI 读 & 写16bit函数

//! \brief 读或写
//!
typedef enum
{
  CtrlMode_Read = 1 << 15,   //!< Read Mode
  CtrlMode_Write = 0 << 15   //!< Write Mode
} DRV8323_CtrlMode_e;


//! \brief 只读状态寄存器0，地址0x00
//!
typedef enum
{
  VDS_LC      = (1 << 0),    //!< VDS overcurrent fault on C low-side MOSFET
  VDS_HC      = (1 << 1),    //!< VDS overcurrent fault on C high-side MOSFET
  VDS_LB      = (1 << 2),    //!< VDS overcurrent fault on B low-side MOSFET
  VDS_HB      = (1 << 3),    //!< VDS overcurrent fault on B high-side MOSFET
  VDS_LA      = (1 << 4),    //!< VDS overcurrent fault on A low-side MOSFET
  VDS_HA      = (1 << 5),    //!< VDS overcurrent fault on A high-side MOSFET
  OTSD        = (1 << 6),    //!< Overtemperature shutdown
  UVLO        = (1 << 7),    //!< Undervoltage lockout fault condition
  GDF         = (1 << 8),    //!< Gate driver fault condition
  VDS_OCP     = (1 << 9),    //!< VDS monitor overcurrent fault condition
  FAULT       = (1 << 10)    //!< FAULT type, 0-Warning, 1-Latched
} DRV8323_STATUS00_WarningWatchdog_e;


//! \brief 只读状态寄存器1，地址0x01
//!
typedef enum
{
  VGS_LC      = (1 << 0),    //!< VGS gate drive fault on C low-side MOSFET
  VGS_HC      = (1 << 1),    //!< VGS gate drive fault on C high-side MOSFET
  VGS_LB      = (1 << 2),    //!< VGS gate drive fault on B low-side MOSFET
  VGS_HB      = (1 << 3),    //!< VGS gate drive fault on B high-side MOSFET
  VGS_LA      = (1 << 4),    //!< VGS gate drive fault on A low-side MOSFET
  VGS_HA      = (1 << 5),    //!< VGS gate drive fault on A high-side MOSFET
  CPUV        = (1 << 6),    //!< charge pump undervoltage fault
  OTW         = (1 << 7),    //!< overtemperature warning
  SC_OC       = (1 << 8),    //!< overcurrent on phase C
  SB_OC       = (1 << 9),    //!< overcurrent on phase B
  SA_OC       = (1 << 10)    //!< overcurrent on phase A
} DRV8323_STATUS01_OvVdsFaults_e;


//! \brief PWM模式选择
//!
typedef enum 
{
  PwmMode_6 = (0 << 5),     //!< PWM_MODE = 6 inputs
  PwmMode_3 = (1 << 5),     //!< PWM_MODE = 3 inputs
  PwmMode_1 = (2 << 5)      //!< PWM_MODE = 1 input
} DRV8323_CTRL02_PwmMode_e;


//! \brief Enumeration for the high side gate drive peak source current; TODO gate currents not consistent with DS
//!
typedef enum 
{
  ISour_HS_0p010_A = (0 << 4),  //!< IDRIVEP_HS = 0.010A
  ISour_HS_0p020_A = (1 << 4),  //!< IDRIVEP_HS = 0.020A
  ISour_HS_0p030_A = (2 << 4),  //!< IDRIVEP_HS = 0.030A
  ISour_HS_0p040_A = (3 << 4),  //!< IDRIVEP_HS = 0.040A
  ISour_HS_0p050_A = (4 << 4),  //!< IDRIVEP_HS = 0.050A
  ISour_HS_0p060_A = (5 << 4),  //!< IDRIVEP_HS = 0.060A
  ISour_HS_0p070_A = (6 << 4),  //!< IDRIVEP_HS = 0.070A
  ISour_HS_0p125_A = (7 << 4),  //!< IDRIVEP_HS = 0.125A
  ISour_HS_0p250_A = (8 << 4),  //!< IDRIVEP_HS = 0.250A
  ISour_HS_0p500_A = (9 << 4),  //!< IDRIVEP_HS = 0.500A
  ISour_HS_0p750_A = (10 << 4), //!< IDRIVEP_HS = 0.750A
  ISour_HS_1p000_A = (11 << 4)  //!< IDRIVEP_HS = 1.000A
} DRV8323_CTRL03_PeakSourCurHS_e;


//! \brief Enumeration for the high side gate drive peak sink current; TODO gate currents not consistent with DS
//!
typedef enum 
{
  ISink_HS_0p020_A = (0 << 0),  //!< IDRIVEN_HS = 0.020A
  ISink_HS_0p030_A = (1 << 0),  //!< IDRIVEN_HS = 0.030A
  ISink_HS_0p040_A = (2 << 0),  //!< IDRIVEN_HS = 0.040A
  ISink_HS_0p050_A = (3 << 0),  //!< IDRIVEN_HS = 0.050A
  ISink_HS_0p060_A = (4 << 0),  //!< IDRIVEN_HS = 0.060A
  ISink_HS_0p070_A = (5 << 0),  //!< IDRIVEN_HS = 0.070A
  ISink_HS_0p125_A = (6 << 0),  //!< IDRIVEN_HS = 0.125A
  ISink_HS_0p250_A = (7 << 0),  //!< IDRIVEN_HS = 0.250A
  ISink_HS_0p500_A = (8 << 0),  //!< IDRIVEN_HS = 0.500A
  ISink_HS_0p750_A = (9 << 0),  //!< IDRIVEN_HS = 0.750A
  ISink_HS_1p000_A = (10 << 0), //!< IDRIVEN_HS = 1.000A
  ISink_HS_1p250_A = (11 << 0)  //!< IDRIVEN_HS = 1.250A
} DRV8323_CTRL03_PeakSinkCurHS_e;


//! \brief Enumeration for the high side and low side gate drive peak source time; TODO adapt timings to DRV8323
//!
typedef enum 
{
  Lock_lock     = (6 << 8),     //!< Lock settings
  Lock_unlock   = (3 << 8)      //!< Unlock settings
} DRV8323_CTRL03_Lock_e;


//! \brief Enumeration for the high side and low side gate drive peak source time; TODO adapt timings to DRV8323
//!
typedef enum 
{
  TSour_250_ns  = (0 << 8),     //!< TDRIVEN = 250ns
  TSour_500_ns  = (1 << 8),     //!< TDRIVEN = 500ns
  TSour_1000_ns = (2 << 8),     //!< TDRIVEN = 1000ns
  TSour_2000_ns = (3 << 8)      //!< TDRIVEN = 2000ns
} DRV8323_CTRL04_PeakTime_e;


//! \brief Enumeration for the low side gate drive peak source current; TODO adapt current ratings
//!
typedef enum 
{
  ISour_LS_0p010_A = (0 << 4),  //!< IDRIVEP_LS = 0.010A
  ISour_LS_0p020_A = (1 << 4),  //!< IDRIVEP_LS = 0.020A
  ISour_LS_0p030_A = (2 << 4),  //!< IDRIVEP_LS = 0.030A
  ISour_LS_0p040_A = (3 << 4),  //!< IDRIVEP_LS = 0.040A
  ISour_LS_0p050_A = (4 << 4),  //!< IDRIVEP_LS = 0.050A
  ISour_LS_0p060_A = (5 << 4),  //!< IDRIVEP_LS = 0.060A
  ISour_LS_0p070_A = (6 << 4),  //!< IDRIVEP_LS = 0.070A
  ISour_LS_0p125_A = (7 << 4),  //!< IDRIVEP_LS = 0.125A
  ISour_LS_0p250_A = (8 << 4),  //!< IDRIVEP_LS = 0.250A
  ISour_LS_0p500_A = (9 << 4),  //!< IDRIVEP_LS = 0.500A
  ISour_LS_0p750_A = (10 << 4), //!< IDRIVEP_LS = 0.750A
  ISour_LS_1p000_A = (11 << 4)  //!< IDRIVEP_LS = 1.000A
} DRV8323_CTRL04_PeakSourCurLS_e;


//! \brief Enumeration for the low side gate drive peak sink current; TODO adapt current ratings
//!
typedef enum 
{
  ISink_LS_0p020_A = (0 << 0),  //!< IDRIVEN_LS = 0.020A
  ISink_LS_0p030_A = (1 << 0),  //!< IDRIVEN_LS = 0.030A
  ISink_LS_0p040_A = (2 << 0),  //!< IDRIVEN_LS = 0.040A
  ISink_LS_0p050_A = (3 << 0),  //!< IDRIVEN_LS = 0.050A
  ISink_LS_0p060_A = (4 << 0),  //!< IDRIVEN_LS = 0.060A
  ISink_LS_0p070_A = (5 << 0),  //!< IDRIVEN_LS = 0.070A
  ISink_LS_0p125_A = (6 << 0),  //!< IDRIVEN_LS = 0.125A
  ISink_LS_0p250_A = (7 << 0),  //!< IDRIVEN_LS = 0.250A
  ISink_LS_0p500_A = (8 << 0),  //!< IDRIVEN_LS = 0.500A
  ISink_LS_0p750_A = (9 << 0),  //!< IDRIVEN_LS = 0.750A
  ISink_LS_1p000_A = (10 << 0), //!< IDRIVEN_LS = 1.000A
  ISink_LS_1p250_A = (11 << 0)  //!< IDRIVEN_LS = 1.250A
} DRV8323_CTRL04_PeakSinkCurLS_e;


//! \brief Enumeration for the VDS comparator threshold
//!
typedef enum 
{
  VDS_Level_0p060_V = (0 << 0),    //!< VDS_LEVEL = 0.060V
  VDS_Level_0p130_V = (1 << 0),    //!< VDS_LEVEL = 0.130V
  VDS_Level_0p200_V = (2 << 0),    //!< VDS_LEVEL = 0.200V
  VDS_Level_0p260_V = (3 << 0),    //!< VDS_LEVEL = 0.260V
  VDS_Level_0p310_V = (4 << 0),    //!< VDS_LEVEL = 0.310V
  VDS_Level_0p450_V = (5 << 0),    //!< VDS_LEVEL = 0.450V
  VDS_Level_0p530_V = (6 << 0),    //!< VDS_LEVEL = 0.530V
  VDS_Level_0p600_V = (7 << 0),    //!< VDS_LEVEL = 0.600V
  VDS_Level_0p680_V = (8 << 0),    //!< VDS_LEVEL = 0.680V
  VDS_Level_0p750_V = (9 << 0),    //!< VDS_LEVEL = 0.750V
  VDS_Level_0p940_V = (10 << 0),   //!< VDS_LEVEL = 0.940V
  VDS_Level_1p130_V = (11 << 0),   //!< VDS_LEVEL = 1.130V
  VDS_Level_1p300_V = (12 << 0),   //!< VDS_LEVEL = 1.300V
  VDS_Level_1p500_V = (13 << 0),   //!< VDS_LEVEL = 1.500V
  VDS_Level_1p700_V = (14 << 0),   //!< VDS_LEVEL = 1.700V
  VDS_Level_1p880_V = (15 << 0)    //!< VDS_LEVEL = 1.880V
} DRV8323_CTRL05_VDSLVL_e;


//! \brief Enumeration for the OCP/VDS sense deglitch time; TODO adapt deglitch time comments
//!
typedef enum 
{
  VDSDeg_0_us = (0 << 4),       //!< TVDS = 0us
  VDSDeg_2_us = (1 << 4),       //!< TVDS = 2us
  VDSDeg_4_us = (2 << 4),       //!< TVDS = 4us
  VDSDeg_8_us = (3 << 4)        //!< TVDS = 8us
} DRV8323_CTRL05_OcpDeg_e;


//! \brief Enumeration for the OCP report mode
//!
typedef enum 
{
  Latched_Shutdown = (0 << 6),  //!< OCP_MODE = Latched fault
  Automatic_Retry = (1 << 6),   //!< OCP_MODE = Automatic Retry
  Report_Only  = (2 << 6),      //!< OCP_MODE = Report only
  Disable_OCP = (3 << 6)        //!< OCP_MODE = Disabled
} DRV8323_CTRL05_OcpMode_e;


//! \brief Enumeration for the driver dead time
//!
typedef enum 
{
  DeadTime_50_ns = (0 << 8),    //!< DEAD_TIME = 50ns
  DeadTime_100_ns = (1 << 8),   //!< DEAD_TIME = 100ns
  DeadTime_200_ns = (2 << 8),   //!< DEAD_TIME = 200ns
  DeadTime_400_ns = (3 << 8)    //!< DEAD_TIME = 400ns
} DRV8323_CTRL05_DeadTime_e;


//! \brief Enumeration for the Sense OCP level
//!
typedef enum 
{
  SEN_Lvl_Ocp_0p25 = (0 << 0),  //!< SEN_LVL = 0.25V
  SEN_Lvl_Ocp_0p50 = (1 << 0),  //!< SEN_LVL = 0.50V
  SEN_Lvl_Ocp_0p75 = (2 << 0),  //!< SEN_LVL = 0.75V
  SEN_Lvl_Ocp_1p00 = (3 << 0)   //!< SEN_LVL = 1.00V
} DRV8323_CTRL06_SENLevel_e;


//! \brief Enumeration for the gain of shunt amplifier
//!
typedef enum 
{
  Gain_5VpV =  (0 << 6),   //!< GAIN_CSA = 5V/V
  Gain_10VpV = (1 << 6),   //!< GAIN_CSA = 10V/V
  Gain_20VpV = (2 << 6),   //!< GAIN_CSA = 20V/V
  Gain_40VpV = (3 << 6)    //!< GAIN_CSA = 40V/V
} DRV8323_CTRL06_CSAGain_e;


//! \brief Enumeration for the register addresses
//!
typedef enum 
{
  Address_Status_0  = 0 << 11,   //!< Status Register 0
  Address_Status_1  = 1 << 11,   //!< Status Register 1
  Address_Control_2 = 2 << 11,   //!< Control Register 2
  Address_Control_3 = 3 << 11,   //!< Control Register 3
  Address_Control_4 = 4 << 11,   //!< Control Register 4
  Address_Control_5 = 5 << 11,   //!< Control Register 5
  Address_Control_6 = 6 << 11    //!< Control Register 6
} DRV8323_Address_e;

//! \brief Defines the address mask
//!
#define DRV8323_ADDR_MASK                   (0x7800)


//! \brief Defines the data mask
//!
#define DRV8323_DATA_MASK                   (0x07FF)


//! \brief Defines the R/W mask
//!
#define DRV8323_RW_MASK                     (0x8000)


//! \brief Defines the R/W mask
//!
#define DRV8323_FAULT_TYPE_MASK             (0x07FF)

#define DRV8323_STATUS00_VDS_LC_BITS        (1 << 0)
#define DRV8323_STATUS00_VDS_HC_BITS        (1 << 1)
#define DRV8323_STATUS00_VDS_LB_BITS        (1 << 2)
#define DRV8323_STATUS00_VDS_HB_BITS        (1 << 3)
#define DRV8323_STATUS00_VDS_LA_BITS        (1 << 4)
#define DRV8323_STATUS00_VDS_HA_BITS        (1 << 5)

//! \brief Defines the location of the OTSD (Over temperature shutdown) bits in the Status 1 register
//!
#define DRV8323_STATUS00_OTSD_BITS          (1 << 6)
#define DRV8323_STATUS00_UVLO_BITS          (1 << 7)
#define DRV8323_STATUS00_GDF_BITS           (1 << 8)
#define DRV8323_STATUS00_VDS_OCP_BITS       (1 << 9)
#define DRV8323_STATUS00_FAULT_BITS         (1 << 10)

static inline uint16_t DRV8323_buildCtrlWord(const DRV8323_CtrlMode_e ctrlMode,
                                             const DRV8323_Address_e regAddr,
                                             const uint16_t data)
{
  uint16_t ctrlWord = ctrlMode | regAddr | (data & DRV8323_DATA_MASK);

  return(ctrlWord);
} // end of DRV8323_buildCtrlWord() function

#endif  /* __DRV8323_H */
















//#endif
