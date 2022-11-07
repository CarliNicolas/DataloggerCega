#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include "asm330lhhx_reg.h"
#include "SPI.h"
#include "AS5048A.h"

#ifdef IMU_USE_I2C
#include "Wire.h"
#endif

// WIFI
const char *wifiSSID = "Cega-2GHz";
const char *wifiPassword = "emulador4";
const char *dnsHostname = "scalextric"; // Sets the domain name - if set to "abc", access via: abc.local
const int serverPort = 23;
WiFiServer ipServer(serverPort);
WiFiClient ipClient;
static bool newClient;
//--------------------------------------------------

// ANGLE SENSOR AS5048
#define ANGLE_SENSOR_SPI_CS SS
AS5048A angleSensor(ANGLE_SENSOR_SPI_CS, false);

float giradoSensorMag = 0;
float valorAnguloAnterior = 0; // Utilizado para poder calcular la variación de angulo;
const int diametroRueda = 27;  // en mm
unsigned long tiempoMuestra = 0;
//--------------------------------------------------

// HALL EFECT SENSOR
#define PIN_LED_INDICADOR 15
const int SensorMagnetico = 34;
bool banderaCambioVuelta = false;
bool nuevaVuelta = false;
int vueltas = 0;
unsigned long tiempoVueltaActual = 0;
unsigned long tiempoVueltaAnterior = 0;
unsigned long tiempoVueltaDelta = 0;

long analizarVueltaHall();
//--------------------------------------------------

// MUESTREO
#define PIN_LED_INDICADOR 15
#define MUESTRAS_MAXIMAS 2500
long muestraActual = 0;
long muestraAlcanzada = 0;
bool finDeMuestreo;

typedef struct
{
  float g_acceleration[3];
  float mdps_angular[3];
  unsigned long estampaDeTiempo;
  float velocidad;
} carData;
static carData datosRecorrido[MUESTRAS_MAXIMAS];
carData *pMuestraActual = datosRecorrido;

enum statusLectura
{
  estadoInicial,
  estadoLectura,
  estadoTransfiriendo,
  estadoFinalizado
};
statusLectura estadoSistema;
bool listoParaLeer;
//--------------------------------------------------

// IMU asm330lhhx
#define INTERRUPT1_PIN 16
#define INTERRUPT2_PIN 17

#ifdef IMU_USE_SPI
// SPI PINS DEFINITIONS
#ifdef ALTERNATE_PINS
#define VSPI_MISO 2
#define VSPI_MOSI 4
#define VSPI_SCLK 0
#define VSPI_SS 33

#else
#define VSPI_MISO MISO
#define VSPI_MOSI MOSI
#define VSPI_SCLK SCK
#define VSPI_SS SS
#endif

#if CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3
#define VSPI FSPI
#endif

// uninitalised pointers to SPI objects
SPIClass *dev_spi = NULL;

// SPI settings
static const int spiClk = 1000000; // 1 MHz
SPISettings asm330lhhSPISettings(spiClk, MSBFIRST, SPI_MODE0);
#endif

#ifdef IMU_USE_I2C
uint8_t i2c_address = ASM330LHHX_I2C_ADD_L;
#endif

stmdev_ctx_t dev_ctx;

static int16_t data_raw_acceleration[3];
static int16_t data_raw_angular_rate[3];
static int16_t data_raw_temperature;
static float angular_rate_mdps[3];
static float acceleration_mg[3];
static float temperature_degC;
static uint8_t whoamI, rst;

#if defined(IMU_SELF_TEST)
#define MIN_ST_LIMIT_mg 90.0f
#define MAX_ST_LIMIT_mg 1700.0f
#define MIN_ST_LIMIT_mdps 150000.0f
#define MAX_ST_LIMIT_mdps 700000.0f

/* Self test results. */
#define ST_PASS 1U
#define ST_FAIL 0U
float val_st_off[3];
float val_st_on[3];
float test_val[3];
int16_t data_raw[3];
uint8_t st_result;
uint8_t drdy;
uint8_t i;
uint8_t j;
#endif // IMU_SELF_TEST

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
static void platform_delay(uint32_t ms);
static void platform_init(void);

void imu_begin();
bool imu_get_data();

bool imu_int1_active = false;
bool imu_int2_active = false;
void IRAM_ATTR IMU_INT1()
{
  imu_int1_active = true;
}
void IRAM_ATTR IMU_INT2()
{
  imu_int2_active = true;
}
//--------------------------------------------------

// void setup()
{
  estadoSistema = estadoInicial;

  Serial.begin(2000000);

  // WIFI INIT
  Serial.println();
  Serial.print(F("WiFi...."));
  WiFi.mode(WIFI_STA);
  WiFi.begin(wifiSSID, wifiPassword);
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(500);
  }
  Serial.print(F("connected: "));
  Serial.println(WiFi.localIP());

  if (!MDNS.begin(dnsHostname))
  {
    Serial.println("Error setting up MDNS responder.");
    while (1)
    {
      delay(1000);
    }
  }
  ipServer.begin();
  Serial.print(F("Server started: "));
  Serial.print(dnsHostname);
  Serial.print(F(".local:"));
  Serial.println(serverPort);
  //--------------------------------------------------

  // ANGLE SENSOR AS5048
  angleSensor.begin();
  angleSensor.setZeroPosition(angleSensor.getRawRotation());
  valorAnguloAnterior = angleSensor.getRotationInRadians();
  tiempoMuestra = micros();
  //--------------------------------------------------

  // HALL EFECT SENSOR
  pinMode(SensorMagnetico, INPUT);
  pinMode(PIN_LED_INDICADOR, OUTPUT);
  //--------------------------------------------------

  // IMU asm330lhhx
#ifdef IMU_USE_SPI
  // initialise instance of the SPIClass attached to SPI
  dev_spi = new SPIClass(VSPI);
  Serial.println("SPI created");
#ifndef ALTERNATE_PINS
  // initialise vspi with default pins
  // SCLK = 18, MISO = 19, MOSI = 23, SS = 5
  dev_spi->begin();
  Serial.println("SPI begin");
#else
  // alternatively route through GPIO pins of your choice
  dev_spi->begin(VSPI_SCLK, VSPI_MISO, VSPI_MOSI, VSPI_SS); // SCLK, MISO, MOSI, SS
  Serial.println("SPI begin");
#endif
  // set up slave select pins as outputs as the Arduino API
  // doesn't handle automatically pulling SS low
  pinMode(dev_spi->pinSS(), OUTPUT); // VSPI SS
#endif                               // IMU_USE_SPI
#ifdef IMU_USE_I2C
  Wire.begin();
  Wire.setClock(400000L);
  Serial.println("I2C begin");
#endif // IMU_USE_I2C
  delay(100);

  imu_begin();
  pinMode(INTERRUPT1_PIN, INPUT_PULLDOWN);
  pinMode(INTERRUPT2_PIN, INPUT_PULLDOWN);
  attachInterrupt(INTERRUPT1_PIN, IMU_INT1, RISING);
  attachInterrupt(INTERRUPT2_PIN, IMU_INT2, RISING);
  //--------------------------------------------------

  digitalWrite(PIN_LED_INDICADOR, LOW);
}
// void loop()
{
  ipClient = ipServer.available();
  if (ipClient)
  {
    static bool newClient = true;
    while (ipClient.connected())
    {
      // Once client is connected, tell it is connected (once!)
      if (newClient)
      {
        Serial.println("Client connected");
        ipClient.printf("Connected to IMU\r\n");
        newClient = false;
        muestraActual = 0;
        digitalWrite(PIN_LED_INDICADOR, HIGH);
      }
      // Reads from IP input and writes to the Keybus as a virtual keypad
      if (ipClient.available() > 0)
      {
        if (ipClient.peek() == 0xFF)
        { // Checks for Telnet options negotiation data
          for (byte i = 0; i < 3; i++)
            ipClient.read();
        }
        else
        {
          char c = static_cast<char>(ipClient.read());
          Serial.print(c);
        }
      }

      // estado del sistema
      if ((vueltas % 2 == 1) && !finDeMuestreo && (muestraActual < MUESTRAS_MAXIMAS) && listoParaLeer)
      {
        estadoSistema = estadoLectura;
        Serial.println("Estado Lectura");
        digitalWrite(PIN_LED_INDICADOR, (muestraActual % 50 == 0) ? !bool(digitalRead(PIN_LED_INDICADOR)) : bool(digitalRead(PIN_LED_INDICADOR)));
      }
      else if (((vueltas % 2 == 0) || (muestraActual >= (MUESTRAS_MAXIMAS - 1))) && listoParaLeer)
      {
        estadoSistema = estadoFinalizado;
        Serial.println("Estado Finalizada lectura");
        digitalWrite(PIN_LED_INDICADOR, LOW);
      }
      else if (finDeMuestreo)
      {
        estadoSistema = estadoTransfiriendo;
        Serial.println("Estado transfiriendo");
        digitalWrite(PIN_LED_INDICADOR, (muestraActual % 100 == 0) ? !bool(digitalRead(PIN_LED_INDICADOR)) : bool(digitalRead(PIN_LED_INDICADOR)));
      }
      else
      {
        Serial.println("Estado inicial");
        estadoSistema = estadoInicial;
        digitalWrite(PIN_LED_INDICADOR, HIGH);
      }

      switch (estadoSistema)
      {
      case estadoInicial:
        if (banderaCambioVuelta)
        {
          muestraActual = 0;
          listoParaLeer = true;
          finDeMuestreo = false;
          banderaCambioVuelta = false;
        }
        break;
      case estadoLectura:
        // if (imu_int1_active)
        // {
        if (imu_get_data())
        {
          muestraActual++;
        }
        // imu_int1_active = false;
        // }
        break;
      case estadoFinalizado:
        muestraAlcanzada = muestraActual;
        muestraActual = 0;
        finDeMuestreo = true;
        listoParaLeer = false;
        break;
      case estadoTransfiriendo:
        if (muestraActual <= muestraAlcanzada)
        {
          if ((datosRecorrido[muestraActual].g_acceleration[0] != 0) || (datosRecorrido[muestraActual].g_acceleration[1] != 0) || (datosRecorrido[muestraActual].g_acceleration[2] != 0) || (datosRecorrido[muestraActual].mdps_angular[0] != 0) || (datosRecorrido[muestraActual].mdps_angular[1] != 0) || (datosRecorrido[muestraActual].mdps_angular[2] != 0) || (datosRecorrido[muestraActual].estampaDeTiempo != 0) || (datosRecorrido[muestraActual].velocidad != 0))
          {
            ipClient.printf("%f,", datosRecorrido[muestraActual].g_acceleration[0]);
            ipClient.printf("%f,", datosRecorrido[muestraActual].g_acceleration[1]);
            ipClient.printf("%f,", datosRecorrido[muestraActual].g_acceleration[2]);

            ipClient.printf("%f,", datosRecorrido[muestraActual].mdps_angular[0]);
            ipClient.printf("%f,", datosRecorrido[muestraActual].mdps_angular[1]);
            ipClient.printf("%f,", datosRecorrido[muestraActual].mdps_angular[2]);

            ipClient.printf("%ld,", datosRecorrido[muestraActual].estampaDeTiempo);
            ipClient.printf("%f,", datosRecorrido[muestraActual].velocidad);
            ipClient.printf("%d\n", vueltas);
            //ipClient.printf("%d\n", muestraActual);
          }
          muestraActual++;
        }
        else
        {
          finDeMuestreo = false;
          imu_int1_active = false;
        }

        break;
      default:
        // Nunca deberia entrar aca !
        Serial.print("Hasta las manos");
        break;
      }
      analizarVueltaHall();
    }
    ipClient.stop();
    newClient = true;
    Serial.println("Client disconnected");
  }
}

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len)
{
#if defined(NUCLEO_F411RE)
  HAL_I2C_Mem_Write(handle, ASM330LHHX_I2C_ADD_L, reg,
                    I2C_MEMADD_SIZE_8BIT, (uint8_t *)bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Transmit(handle, (uint8_t *)bufp, len, 1000);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  i2c_lld_write(handle, ASM330LHHX_I2C_ADD_L & 0xFE, reg, (uint8_t *)bufp, len);
#elif defined(ESP32)
#ifdef IMU_USE_SPI
  dev_spi->beginTransaction(asm330lhhSPISettings);
  digitalWrite(dev_spi->pinSS(), LOW); // pull SS low
  dev_spi->transfer(reg);
  for (int i = 0; i < len; i++)
  {
    dev_spi->transfer(bufp[i]);
  }
  digitalWrite(dev_spi->pinSS(), HIGH); // pull ss high
  dev_spi->endTransaction();
#endif /*IMU_USE_SPI*/
#ifdef IMU_USE_I2C
  Wire.beginTransmission(((uint8_t)(((i2c_address) >> 1) & 0x7F)));
  Wire.write(reg);
  for (int i = 0; i < len; i++)
  {
    Wire.write(bufp[i]);
  }
  Wire.endTransmission(true);
#endif /*IMU_USE_I2C*/
#endif
  return 0;
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
#if defined(NUCLEO_F411RE)
  HAL_I2C_Mem_Read(handle, ASM330LHHX_I2C_ADD_L, reg,
                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
  reg |= 0x80;
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Receive(handle, bufp, len, 1000);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  i2c_lld_read(handle, ASM330LHHX_I2C_ADD_L & 0xFE, reg, bufp, len);
#elif defined(ESP32)
#ifdef IMU_USE_SPI
  dev_spi->beginTransaction(asm330lhhSPISettings);
  digitalWrite(dev_spi->pinSS(), LOW); // pull SS low
  reg = reg | 0b10000000;
  dev_spi->transfer(reg);
  for (int i = 0; i < len; i++)
  {
    *(bufp + i) = dev_spi->transfer(0);
  }
  digitalWrite(dev_spi->pinSS(), HIGH); // pull ss high to signify end of data transfer
  dev_spi->endTransaction();
#endif /*IMU_USE_SPI*/
#ifdef IMU_USE_I2C
  Wire.beginTransmission(((uint8_t)(((i2c_address) >> 1) & 0x7F)));
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(((uint8_t)(((i2c_address) >> 1) & 0x7F)), (uint8_t)len);
  int i = 0;
  while (Wire.available())
  {
    bufp[i] = Wire.read();
    i++;
  }
#endif /*IMU_USE_I2C*/
#endif
  return 0;
}

/*
 * @brief  platform specific delay (platform dependent)
 *
 * @param  ms        delay in ms
 *
 */
static void platform_delay(uint32_t ms)
{
#if defined(NUCLEO_F411RE) | defined(STEVAL_MKI109V3)
  HAL_Delay(ms);
#elif defined(SPC584B_DIS)
  osalThreadDelayMilliseconds(ms);
#elif defined(ESP32)
  delay(ms);
#endif
}

/*
 * @brief  platform specific initialization (platform dependent)
 */
static void platform_init(void)
{
#if defined(STEVAL_MKI109V3)
  TIM3->CCR1 = PWM_3V3;
  TIM3->CCR2 = PWM_3V3;
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_Delay(1000);
#endif
}

void imu_begin()
{
  /* Initialize mems driver interface */
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;

  /* Check device ID */
  do
  {
    asm330lhhx_device_id_get(&dev_ctx, &whoamI);
    if (whoamI != ASM330LHHX_ID)
    {
      Serial.print("ASM330lhhx not found, whoamI: ");
      Serial.println(whoamI, HEX);
    }
  } while (whoamI != ASM330LHHX_ID);
  Serial.println("ASM330lhhx found!");

  /* Restore default configuration */
  asm330lhhx_reset_set(&dev_ctx, PROPERTY_ENABLE);
  do
  {
    asm330lhhx_reset_get(&dev_ctx, &rst);
  } while (rst);

  /* Disable I3C interface */
  asm330lhhx_i3c_disable_set(&dev_ctx, ASM330LHHX_I3C_DISABLE);

  /* Enable Block Data Update */
  asm330lhhx_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
#ifdef IMU_SELF_TEST
  /* Accelerometer Self Test */

  /* Set Output Data Rate */
  asm330lhhx_xl_data_rate_set(&dev_ctx, ASM330LHHX_XL_ODR_52Hz);

  /* Set full scale */
  asm330lhhx_xl_full_scale_set(&dev_ctx, ASM330LHHX_4g);

  /* Wait stable output */
  platform_delay(100);
  /* Check if new value available */
  do
  {
    asm330lhhx_xl_flag_data_ready_get(&dev_ctx, &drdy);
  } while (!drdy);
  /* Read dummy data and discard it */
  asm330lhhx_acceleration_raw_get(&dev_ctx, data_raw);
  Serial.println("Acc self test");

  /* Read 5 sample and get the average vale for each axis */
  memset(val_st_off, 0x00, 3 * sizeof(float));
  for (i = 0; i < 5; i++)
  {
    /* Check if new value available */
    do
    {
      asm330lhhx_xl_flag_data_ready_get(&dev_ctx, &drdy);
    } while (!drdy);
    /* Read data and accumulate the mg value */
    asm330lhhx_acceleration_raw_get(&dev_ctx, data_raw);
    for (j = 0; j < 3; j++)
    {
      val_st_off[j] += asm330lhhx_from_fs4g_to_mg(data_raw[j]);
    }
  }

  /* Calculate the mg average values */
  for (i = 0; i < 3; i++)
  {
    val_st_off[i] /= 5.0f;
  }

  /* Enable Self Test positive (or negative) */
  asm330lhhx_xl_self_test_set(&dev_ctx, ASM330LHHX_XL_ST_POSITIVE);
  // asm330lhhx_xl_self_test_set(&dev_ctx, LIS2DH12_XL_ST_NEGATIVE);

  /* Wait stable output */
  platform_delay(100);

  /* Check if new value available */
  do
  {
    asm330lhhx_xl_flag_data_ready_get(&dev_ctx, &drdy);
  } while (!drdy);
  /* Read dummy data and discard it */
  asm330lhhx_acceleration_raw_get(&dev_ctx, data_raw);

  /* Read 5 sample and get the average vale for each axis */
  memset(val_st_on, 0x00, 3 * sizeof(float));
  for (i = 0; i < 5; i++)
  {
    /* Check if new value available */
    do
    {
      asm330lhhx_xl_flag_data_ready_get(&dev_ctx, &drdy);
    } while (!drdy);
    /* Read data and accumulate the mg value */
    asm330lhhx_acceleration_raw_get(&dev_ctx, data_raw);
    for (j = 0; j < 3; j++)
    {
      val_st_on[j] += asm330lhhx_from_fs4g_to_mg(data_raw[j]);
    }
  }

  /* Calculate the mg average values */
  for (i = 0; i < 3; i++)
  {
    val_st_on[i] /= 5.0f;
  }

  /* Calculate the mg values for self test */
  for (i = 0; i < 3; i++)
  {
    test_val[i] = fabs((val_st_on[i] - val_st_off[i]));
  }
  /* Check self test limit */
  st_result = ST_PASS;
  for (i = 0; i < 3; i++)
  {
    Serial.print(test_val[i]);
    Serial.print(", ");
    if ((MIN_ST_LIMIT_mg > test_val[i]) ||
        (test_val[i] > MAX_ST_LIMIT_mg))
    {
      Serial.print(" Fail ");
      st_result = ST_FAIL;
    }
  }
  if (st_result == ST_PASS)
  {
    Serial.println(" Self test - PASS");
  }
  else
  {
    Serial.println(" Self test - FAIL");
  }

  /* Disable Self Test */
  asm330lhhx_xl_self_test_set(&dev_ctx, ASM330LHHX_XL_ST_DISABLE);
  /* Disable sensor. */
  asm330lhhx_xl_data_rate_set(&dev_ctx, ASM330LHHX_XL_ODR_OFF);
  /* Gyroscope Self Test */

  /* Set Output Data Rate */
  asm330lhhx_gy_data_rate_set(&dev_ctx, ASM330LHHX_GY_ODR_208Hz);
  /* Set full scale */
  asm330lhhx_gy_full_scale_set(&dev_ctx, ASM330LHHX_2000dps);

  /* Wait stable output */
  platform_delay(100);

  /* Check if new value available */
  do
  {
    asm330lhhx_gy_flag_data_ready_get(&dev_ctx, &drdy);
  } while (!drdy);
  /* Read dummy data and discard it */
  asm330lhhx_angular_rate_raw_get(&dev_ctx, data_raw);
  Serial.println("Gyro selftest");

  /* Read 5 sample and get the average vale for each axis */
  memset(val_st_off, 0x00, 3 * sizeof(float));
  for (i = 0; i < 5; i++)
  {
    /* Check if new value available */
    do
    {
      asm330lhhx_gy_flag_data_ready_get(&dev_ctx, &drdy);
    } while (!drdy);
    /* Read data and accumulate the mg value */
    asm330lhhx_angular_rate_raw_get(&dev_ctx, data_raw);
    for (j = 0; j < 3; j++)
    {
      val_st_off[j] += asm330lhhx_from_fs2000dps_to_mdps(data_raw[j]);
    }
  }
  /* Calculate the mg average values */
  for (i = 0; i < 3; i++)
  {
    val_st_off[i] /= 5.0f;
  }

  /* Enable Self Test positive (or negative) */
  asm330lhhx_gy_self_test_set(&dev_ctx, ASM330LHHX_GY_ST_POSITIVE);
  // asm330lhhx_gy_self_test_set(&dev_ctx, LIS2DH12_GY_ST_NEGATIVE);

  /* Wait stable output */
  platform_delay(100);

  /* Read 5 sample and get the average vale for each axis */
  memset(val_st_on, 0x00, 3 * sizeof(float));
  for (i = 0; i < 5; i++)
  {
    /* Check if new value available */
    do
    {
      asm330lhhx_gy_flag_data_ready_get(&dev_ctx, &drdy);
    } while (!drdy);
    /* Read data and accumulate the mg value */
    asm330lhhx_angular_rate_raw_get(&dev_ctx, data_raw);
    for (j = 0; j < 3; j++)
    {
      val_st_on[j] += asm330lhhx_from_fs2000dps_to_mdps(data_raw[j]);
    }
  }

  /* Calculate the mg average values */
  for (i = 0; i < 3; i++)
  {
    val_st_on[i] /= 5.0f;
  }

  /* Calculate the mg values for self test */
  for (i = 0; i < 3; i++)
  {
    test_val[i] = fabs((val_st_on[i] - val_st_off[i]));
  }
  /* Check self test limit */

  for (i = 0; i < 3; i++)
  {
    Serial.print(test_val[i]);
    Serial.print(", ");
    if ((MIN_ST_LIMIT_mdps > test_val[i]) ||
        (test_val[i] > MAX_ST_LIMIT_mdps))
    {
      Serial.print(" Fail ");
      st_result = ST_FAIL;
    }
  }

  if (st_result == ST_PASS)
  {
    Serial.println("Self test - PASS");
  }
  else
  {
    Serial.println("Self test - FAIL");
  }
  Serial.println("Selftest ended");
  /* Disable Self Test */
  asm330lhhx_gy_self_test_set(&dev_ctx, ASM330LHHX_GY_ST_DISABLE);
  /* Disable sensor. */
  asm330lhhx_gy_data_rate_set(&dev_ctx, ASM330LHHX_GY_ODR_OFF);

  while (1)
    ;
#endif /*IMU_SELF_TEST*/
  /* Set FIFO watermark (number of unread sensor data TAG + 6 bytes
   * stored in FIFO) to 10 samples
   */
  // asm330lhhx_fifo_watermark_set(&dev_ctx, 10);

  /* Set FIFO batch XL/Gyro ODR to 417Hz */
  // asm330lhhx_fifo_xl_batch_set(&dev_ctx, ASM330LHHX_XL_BATCHED_AT_417Hz);
  // asm330lhhx_fifo_gy_batch_set(&dev_ctx, ASM330LHHX_GY_BATCHED_AT_417Hz);

  /* Set FIFO mode to Stream mode (aka Continuous Mode) */
  // asm330lhhx_fifo_mode_set(&dev_ctx, ASM330LHHX_STREAM_MODE);

  /* Enable drdy 75 μs pulse: uncomment if interrupt must be pulsed */
  asm330lhhx_data_ready_mode_set(&dev_ctx, ASM330LHHX_DRDY_PULSED);

  /* Set Output Data Rate. */
  asm330lhhx_xl_data_rate_set(&dev_ctx, ASM330LHHX_XL_ODR_208Hz);
  // asm330lhhx_xl_data_rate_set(&dev_ctx, ASM330LHHX_XL_ODR_104Hz);
  asm330lhhx_gy_data_rate_set(&dev_ctx, ASM330LHHX_GY_ODR_208Hz);
  // asm330lhhx_gy_data_rate_set(&dev_ctx, ASM330LHHX_GY_ODR_104Hz);

  /* Set full scale. */
  asm330lhhx_xl_full_scale_set(&dev_ctx, ASM330LHHX_2g);
  asm330lhhx_gy_full_scale_set(&dev_ctx, ASM330LHHX_1000dps);
  // asm330lhhx_gy_full_scale_set(&dev_ctx, ASM330LHHX_2000dps);

  /* Enable timestamp. */
  asm330lhhx_timestamp_set(&dev_ctx, PROPERTY_ENABLE);

  /* Configure filtering chain(No aux interface)
   * Accelerometer - LPF1 + LPF2 path
   * Gyro - LPF1
   */
  asm330lhhx_xl_power_mode_set(&dev_ctx, ASM330LHHX_HIGH_PERFORMANCE_MD);
  asm330lhhx_xl_fast_settling_set(&dev_ctx, PROPERTY_ENABLE);
  asm330lhhx_xl_filter_lp2_set(&dev_ctx, PROPERTY_ENABLE);
  asm330lhhx_xl_hp_path_on_out_set(&dev_ctx, ASM330LHHX_LP_ODR_DIV_100);

  asm330lhhx_gy_hp_path_internal_set(&dev_ctx, ASM330LHHX_HP_FILTER_NONE);
  asm330lhhx_gy_filter_lp1_set(&dev_ctx, PROPERTY_ENABLE);
  asm330lhhx_gy_lp1_bandwidth_set(&dev_ctx, ASM330LHHX_LIGHT);

  /*  Uncomment to configure INT 1 */
  asm330lhhx_pin_int1_route_t int1_route;
  /* Uncomment to configure INT 2 */
  asm330lhhx_pin_int2_route_t int2_route;

  /* Interrupt generation on INT1 pin */
  asm330lhhx_pin_int1_route_get(&dev_ctx, &int1_route);
  int1_route.int1_ctrl.int1_drdy_g = PROPERTY_ENABLE;
  asm330lhhx_pin_int1_route_set(&dev_ctx, &int1_route);

  /* Interrupt generation on INT2 pin */
  asm330lhhx_pin_int2_route_get(&dev_ctx, &int2_route);
  int2_route.int2_ctrl.int2_drdy_xl = PROPERTY_ENABLE;
  asm330lhhx_pin_int2_route_set(&dev_ctx, &int2_route);
}

bool imu_get_data()
{

  // Prints data
  bool return_value = false;

  asm330lhhx_reg_t reg;
  uint32_t timestamp;

  asm330lhhx_status_reg_get(&dev_ctx, &reg.status_reg); // Read output only if new value is available.

  if (reg.status_reg.xlda && reg.status_reg.gda)
  {
    asm330lhhx_timestamp_raw_get(&dev_ctx, &timestamp);
    return_value = true;
  }
  if (return_value)
  {
    // IMU GET ACC
    if (reg.status_reg.xlda) // Read acceleration field data
    {
      memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
      asm330lhhx_acceleration_raw_get(&dev_ctx, data_raw_acceleration);

#ifdef IMU_RAW_DATA
      datosRecorrido[muestraActual].g_acceleration[0] = data_raw_acceleration[0];
      datosRecorrido[muestraActual].g_acceleration[1] = data_raw_acceleration[1];
      datosRecorrido[muestraActual].g_acceleration[2] = data_raw_acceleration[2];
#else
      datosRecorrido[muestraActual].g_acceleration[0] = asm330lhhx_from_fs2g_to_mg(data_raw_acceleration[0]);
      datosRecorrido[muestraActual].g_acceleration[1] = asm330lhhx_from_fs2g_to_mg(data_raw_acceleration[1]);
      datosRecorrido[muestraActual].g_acceleration[2] = asm330lhhx_from_fs2g_to_mg(data_raw_acceleration[2]);
#endif
    }
    // IMU GET GYRO
    if (reg.status_reg.gda) // Read angular rate field data
    {
      memset(data_raw_angular_rate, 0x00, 3 * sizeof(int16_t));
      asm330lhhx_angular_rate_raw_get(&dev_ctx, data_raw_angular_rate);

#ifdef IMU_RAW_DATA
      datosRecorrido[muestraActual].mdps_angular[0] = data_raw_angular_rate[0];
      datosRecorrido[muestraActual].mdps_angular[1] = data_raw_angular_rate[1];
      datosRecorrido[muestraActual].mdps_angular[2] = data_raw_angular_rate[2];
#else
      datosRecorrido[muestraActual].mdps_angular[0] = asm330lhhx_from_fs1000dps_to_mdps(data_raw_angular_rate[0]);
      datosRecorrido[muestraActual].mdps_angular[1] = asm330lhhx_from_fs1000dps_to_mdps(data_raw_angular_rate[1]);
      datosRecorrido[muestraActual].mdps_angular[2] = asm330lhhx_from_fs1000dps_to_mdps(data_raw_angular_rate[2]);
#endif
    }

    // AS5048 GET SPEED
    float magSensorAngle = angleSensor.getRotationInRadians();
    float deltaAngMagnetico = magSensorAngle - valorAnguloAnterior;
    unsigned long tiempoNuevaMuestra = micros();

    if (deltaAngMagnetico < -PI)
      deltaAngMagnetico += 2 * PI;

    if (deltaAngMagnetico > PI)
      deltaAngMagnetico -= 2 * PI;
    giradoSensorMag += deltaAngMagnetico;
    valorAnguloAnterior = magSensorAngle;

    float velocidadLineal = deltaAngMagnetico * (diametroRueda / 2) * 1000 / (tiempoNuevaMuestra - tiempoMuestra);
    tiempoMuestra = tiempoNuevaMuestra;

    datosRecorrido[muestraActual].velocidad = velocidadLineal;

    // datosRecorrido[muestraActual].estampaDeTiempo = timestamp;
    datosRecorrido[muestraActual].estampaDeTiempo = tiempoMuestra;
  }
  return return_value;
}

long analizarVueltaHall()
{
  long hallValue = analogRead(SensorMagnetico);
  hallValue += analogRead(SensorMagnetico);
  hallValue += analogRead(SensorMagnetico);
  hallValue += analogRead(SensorMagnetico);
  hallValue += analogRead(SensorMagnetico);
  hallValue /= 5;
  if (hallValue > 2200 && !nuevaVuelta)
  {
    nuevaVuelta = true;
  }
  if (hallValue < 2000 && nuevaVuelta)
  {
    tiempoVueltaAnterior = tiempoVueltaActual;
    tiempoVueltaActual = millis();
    tiempoVueltaDelta = tiempoVueltaActual - tiempoVueltaAnterior;
    nuevaVuelta = false;
    vueltas++;
    banderaCambioVuelta = true;
  }
  return hallValue;
}