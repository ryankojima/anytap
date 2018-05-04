/* SPI related configuration */
#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 18
#define PIN_NUM_CLK  22
#define PIN_NUM_CS   5

typedef enum
{
  DEVID = 0,
  THRESH_TAP = 0x1d,
  DUR = 0x21,
  Latent = 0x22,
  Window = 0x23,
  THRESH_ACT = 0x24,
  THRESH_INACT = 0x25,
  TIME_INACT = 0x26,
  ACT_INACT_CTL = 0x27,
  THRESH_FF = 0x28,
  TIME_FF = 0x29,
  TAP_AXES = 0x2a,
  ACT_TAP_STATUS = 0x2b,
  BW_RATE = 0x2c,
  POWER_CTL = 0x2d,
  INT_ENABLE = 0x2e,
  INT_MAP = 0x2f,
  INT_SOURCE = 0x30,
  DATA_FORMAT = 0x31,
  DATAX0 = 0x32,
  DATAX1 = 0x33,
  DATAY0 = 0x34,
  DATAY1 = 0x35,
  DATAZ0 = 0x36,
  DATAZ1 = 0x37,
  FIFO_CTL = 0x38,
  FIFO_STATUS = 0x39
  
} ADXL345_Address;

typedef enum
{
  CLICK = 0x1,
  BATTERY = 0x2
} dev_event_type;

typedef enum
  {
	INACTIVE = 0x1
  } sleep_cause;

typedef struct{
  dev_event_type event_type;
  uint32_t battery_mv;
} device_event_param;


void lockDeepSleep(void);
void unlockDeepSleep(void);
bool isLockDeepSleep(void);


uint8_t acc_spi_read(spi_device_handle_t spi, const uint8_t addr);
uint16_t acc_spi_read16(spi_device_handle_t spi, const uint8_t addr);
void acc_spi_read48(spi_device_handle_t spi, const uint8_t addr, uint16_t *rxBuf);

void acc_spi_write(spi_device_handle_t spi, const uint8_t addr, const uint8_t data);

void adxl_write(spi_device_handle_t spi, const ADXL345_Address addr, const uint8_t data);

uint8_t adxl_read(spi_device_handle_t spi, const ADXL345_Address addr);
void adxl_read48(spi_device_handle_t spi, const ADXL345_Address addr, uint16_t *rxBuf);


void init_acc(spi_device_handle_t spi);

void monitor_acc_task(void *pvParameters);

void acc_interrupt_task(void* arg);


void set_click_callback(void *callback);
void set_acc_wake_count(int wc);
