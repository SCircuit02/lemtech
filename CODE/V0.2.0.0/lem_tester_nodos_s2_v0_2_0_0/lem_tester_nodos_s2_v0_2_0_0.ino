/*
 * LEM Tester Nodos - ESP32-S2
 * Version: 0.2.0.0
 *
 * Portal WiFi abierto "LemTESTER_Nodos_XXXX" (XXXX = 2 ultimos bytes de la MAC base,
 * la que muestra esptool). La pagina tiene un perfil por tipo de nodo; al abrir uno,
 * el ESP reconfigura sus pines para ese nodo:
 *
 *   LemWeather : SIM7600X (GNSS y hora), BME280, veleta 0..1,12 V, pulsos de viento
 *                y registro cada 1 min en RAM con descarga CSV (igual que v0.1.0.0)
 *   LemSense   : SHT30 / BME280, voltajes de P0_0..P0_6 (ADC_BAT), pulsos de
 *                P1_6..P2_1 con voltaje y HIGH/LOW, reloj DS1338 (leer y escribir)
 *   SDI-12     : buscar sensores, comando aI!, mediciones aC!/aM! (+ aD0!..aD9!),
 *                cambio de ID (aAb!) y comando manual (basado en la pestana del LemPDA)
 *
 * GPIO14 (reguladores SIM7600) y GPIO15 (regulador de sensores) siempre encendidos.
 *
 * Core: esp32 by Espressif 3.x (compilado con 3.3.12, placa "ESP32S2 Dev Module")
 * Librerias: Adafruit BME280 Library, Adafruit SHT31 Library
 *            (+ Adafruit Unified Sensor, Adafruit BusIO)
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <Adafruit_SHT31.h>
#include <math.h>
#include <stdarg.h>
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "driver/gpio.h"
#include "soc/gpio_reg.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

// ============================================
// PIN DEFINITIONS
// ============================================
// --- Comunes ---
#define SIM_EN_PIN    14   // Reguladores SIM7600: SIEMPRE encendido
#define SENS_EN_PIN   15   // Regulador de sensores: siempre encendido
#define SDA_PIN       8    // P1_5
#define SCL_PIN       9    // P1_3

// --- LemWeather ---
#define SIM_RX_PIN    3    // RX del ESP <- TX del SIM7600
#define SIM_TX_PIN    4    // TX del ESP -> RX del SIM7600
#define VELETA_PIN    2    // P0_1, veleta 0..1,12 V
#define VIENTO_CANAL  0    // pulsos de viento = canal 0 de la tabla de pulsos (GPIO10, P1_6)

// --- LemSense: entradas analogicas ---
#define N_ANALOG 5
const uint8_t     ANALOG_GPIO[N_ANALOG]   = {1, 2, 5, 6, 7};
const char* const ANALOG_NOMBRE[N_ANALOG] = {"P0_0", "P0_1", "P0_4", "P0_5", "P0_6 ADC_BAT"};

// --- Pulsos (LemSense usa los 4, LemWeather solo el canal 0) ---
#define N_PULSOS 4
const uint8_t     PULSO_GPIO[N_PULSOS]   = {10, 11, 12, 13};
const char* const PULSO_NOMBRE[N_PULSOS] = {"P1_6", "P1_7", "P2_0", "P2_1"};

// --- SDI-12 ---
// Front-end de 2 pines como el LemPDA (TX -> driver inversor, RX <- Schmitt inversor):
// el MCU ve un UART normal, 1200 baud 7E1. Mismos pines que el UART del SIM7600.
#define SDI12_TX_PIN  4
#define SDI12_RX_PIN  3

// ============================================
// CONFIGURACION
// ============================================
#define FW_VERSION        "0.2.0.0"

// --- WiFi / portal cautivo (red abierta) ---
#define AP_PREFIJO        "LemTESTER_Nodos_"
#define AP_CANAL          6
#define AP_MAX_CLIENTES   4

// --- ADC (ESP32-S2: 13 bits, ADC1 = GPIO1..10, ADC2 = GPIO11..20) ---
#define ADC_RAW_MAX       8191
#define ADC_SAT_MARGEN    16       // raw >= 8175 = ADC saturado (fuera de rango)
#define ADC_CNT_MAX       2047     // N de ADC que se muestra: escala de 11 bits
#define ADC2_REINTENTOS   20       // ADC2 lo arbitra el WiFi: la lectura puede fallar
#define ADC_MUESTRAS      16

// --- LemSense ---
#define UMBRAL_HIGH_MV    3000     // HIGH si el pin esta sobre 3,0 V
#define SENSE_PULLUP      0        // 1 = pull-up interno en P1_6..P2_1 (0 = medir el nodo tal cual)
#define SENSE_ADC_MS      1000
#define SENSE_I2C_MS      2000
#define SENSE_REINTENTO_MS 5000
#define DS1338_ADDR       0x68

// --- SDI-12 ---
#define SDI12_BAUD        1200
#define SDI12_TX_INV      0        // 1 si el front-end NO invierte TX
#define SDI12_RX_INV      0        // 1 si el front-end NO invierte RX
#define SDI12_BREAK_MS    13       // break >= 12 ms
#define SDI12_MARCA_MS    9        // marca >= 8,33 ms
#define SDI12_PROBE_MS    120      // espera del primer caracter en "a!"
#define SDI12_RESP_MS     400
#define SDI12_DATOS_MS    900
#define SDI12_GAP_MS      30       // silencio que cierra una respuesta
#define SDI12_ESPERA_MAX_S 120     // tope de espera de una medicion (ttt)
#define SDI12_NUM_DIR     62       // 0-9, a-z, A-Z
#define SDI12_MAX_VALORES 30
#define SDI12_MONITOR     8

// --- SIM7600 ---
#define SIM_BAUD          115200
#define SIM_T_SYNC_MAX_MS 40000   // sin respuesta a "AT": reabrir el UART (sin cortar GPIO14)
#define SIM_T_RESYNC_MS   2000
#define SIM_FALLOS_MAX    5
#define GPS_POLL_MS       5000
#define GPS_FIX_VIGENTE_S 120
#define GNSS_ANT_ACTIVA   0       // 1 = alimentar antena activa por VDD_AUX (AT+CVAUXV/CVAUXS)

// --- Veleta ---
#define VELETA_VMAX_MV    1120
#define VELETA_MV_FALLA   (VELETA_VMAX_MV + VELETA_VMAX_MV / 10)
#define VELETA_OFFSET_DEG 0.0f
#define VELETA_INVERTIDA  0
#define VELETA_MUESTRAS   32
#define VELETA_PERIODO_MS 1000
#define VELETA_EMA        0.3f

// --- Pulsos (antirrebote) ---
#define PULSO_ALTO_MIN_US 2000    // la linea debe estar en ALTO >= esto antes de aceptar un flanco
#define PULSO_PER_MIN_US  4000    // periodo minimo aceptado -> max 250 Hz

// --- BME280 ---
#define BME_PERIODO_MS    2000
#define BME_REINTENTO_MS  30000

// --- Registro LemWeather en RAM ---
// Buffer estatico en .noinit: sobrevive reset por software, watchdog, panic y brownout.
// Limite fisico: la RAM estatica del S2 termina en 0x3FFDE000 y la comparte con el
// codigo IRAM. Si se pide mas de lo que cabe, el enlazador falla con
// "region dram0_0_seg overflowed": nunca compila algo que no quepa.
#define LOG_PERIODO_S     60
#if ARDUINO_USB_CDC_ON_BOOT || ARDUINO_USB_MSC_ON_BOOT || ARDUINO_USB_DFU_ON_BOOT
#define LOG_CAP_INTERNA   480      // con TinyUSB: 480 x 32 B = 15 KB = 8 h
#else
#define LOG_CAP_INTERNA   1440     // 1440 x 32 B = 45 KB = 24 h
#endif
#define LOG_MAGIC         0x4C454D31UL   // "LEM1"
#define LOG_VERSION       1

#define EPOCH_MIN_VALIDO  1704067200UL   // 2024-01-01

// ============================================
// PERFILES
// ============================================
enum Perfil : uint8_t { PERFIL_NINGUNO = 0, PERFIL_WEATHER = 1, PERFIL_SENSE = 2, PERFIL_SDI12 = 3 };
Perfil perfil = PERFIL_NINGUNO;
#define PERFIL_MAGIC  0x4C454DUL
RTC_NOINIT_ATTR uint32_t perfilNoinit;   // (magic << 8) | perfil: vuelve al mismo perfil tras un reset

// ============================================
// REGISTRO: FORMATO BINARIO (32 bytes)
// ============================================
#define FLG_TS_EPOCH   0x01
#define FLG_GPS_VALIDO 0x02
#define FLG_GPS_FIX    0x04
#define FLG_BME_OK     0x08
#define FLG_DIR_OK     0x10

typedef struct __attribute__((packed)) {
  uint32_t ts;          // epoch UTC [s] o segundos desde arranque (ver FLG_TS_EPOCH)
  int32_t  lat_e6;      // latitud  [1e-6 grados]
  int32_t  lon_e6;      // longitud [1e-6 grados]
  uint32_t pulsosTot;   // contador total de pulsos
  uint32_t periodoUs;   // periodo medio de pulsos en la ventana [us], 0 = sin dato
  int16_t  temp_c;      // temperatura [0,01 C]
  uint16_t hum_c;       // humedad     [0,01 %HR]
  uint16_t pres_2pa;    // presion     [2 Pa]
  uint16_t dir_d;       // direccion   [0,1 grados]
  uint16_t pulsosVen;   // pulsos en la ventana
  uint8_t  flags;
  uint8_t  arranque;    // nro de arranque (mod 256)
} LogRec;
static_assert(sizeof(LogRec) == 32, "LogRec debe medir 32 bytes");

typedef struct {
  uint32_t magic;
  uint32_t version;
  uint32_t cap;
  uint32_t head;
  uint32_t count;
  uint32_t escritos;
  uint32_t arranques;
  uint32_t suma;        // suma de CRC32 de los registros validos
  uint32_t crc;         // CRC32 de los campos anteriores
} LogHdr;

__NOINIT_ATTR LogHdr logHdr;
__NOINIT_ATTR LogRec logInterno[LOG_CAP_INTERNA];

LogRec*  logBuf         = nullptr;
uint32_t logCap         = 0;
uint32_t logRecuperados = 0;
uint8_t  arranqueActual = 0;
LogRec   ultimoReg;
bool     hayUltimoReg   = false;

// ============================================
// HARDWARE OBJECTS
// ============================================
Adafruit_BME280 bme;
Adafruit_SHT31  sht;
HardwareSerial& uart1 = Serial1;   // SIM7600 (LemWeather) o SDI-12
WebServer server(80);
DNSServer dnsServer;

// ============================================
// ADC
// ============================================
typedef struct {
  int  raw;     // promedio, 13 bits
  int  mv;      // calibrado (eFuse) o aproximado si no hay calibracion
  bool ok;      // false: ADC2 ocupado por el WiFi en todos los reintentos
  bool sat;     // raw al tope: el voltaje real es >= mv
  bool cal;
} LecturaAdc;

adc_oneshot_unit_handle_t adcUnidad[2] = {nullptr, nullptr};
adc_cali_handle_t         adcCal[2][2] = {{nullptr, nullptr}, {nullptr, nullptr}};  // [unidad][0=6dB,1=12dB]

// ============================================
// RELOJ (UTC)
// ============================================
enum FuenteReloj : uint8_t { RELOJ_NO = 0, RELOJ_NAVEGADOR = 1, RELOJ_GPS = 2 };
uint8_t relojFuente   = RELOJ_NO;
int64_t relojOffsetUs = 0;      // epoch[us] - esp_timer[us]

// ============================================
// PULSOS (variables compartidas con la ISR)
// ============================================
typedef struct {
  uint32_t tot;        // pulsos desde que se armo el canal
  uint32_t perUs;      // periodo entre los dos ultimos pulsos
  int64_t  ultUs;      // ultimo pulso aceptado
  int64_t  subidaUs;   // ultima subida (contacto abierto)
  uint32_t venN;       // ventana de registro (LemWeather)
  int64_t  venPrimUs;
  int64_t  venUltUs;
} CanalPulso;

portMUX_TYPE        muxPulsos = portMUX_INITIALIZER_UNLOCKED;
volatile CanalPulso pul[N_PULSOS];
bool                pulArmado[N_PULSOS] = {false, false, false, false};
bool                isrServicio = false;
uint32_t            pulsosBase  = 0;   // total de viento acumulado (LemWeather)

// ============================================
// VELETA
// ============================================
uint32_t veletaMv    = 0;
bool     veletaOk    = false;
float    dirInstDeg  = 0.0f;
float    dirEmaX     = 0.0f;
float    dirEmaY     = 0.0f;
bool     dirEmaIni   = false;
float    venDirSin   = 0.0f;
float    venDirCos   = 0.0f;
uint32_t venDirN     = 0;

// ============================================
// SENSORES I2C
// ============================================
bool     bmeOk   = false;
uint8_t  bmeAddr = 0;
float    bmeT = NAN, bmeH = NAN, bmeP = NAN;
float    venSumT = 0, venSumH = 0, venSumP = 0;
uint32_t venBmeN = 0;
unsigned long bmeUltIntento = 0;

bool     shtOk   = false;
uint8_t  shtAddr = 0;
float    shtT = NAN, shtH = NAN;
unsigned long shtUltIntento = 0;

// ============================================
// LEMSENSE
// ============================================
LecturaAdc anLect[N_ANALOG];
LecturaAdc puLect[N_PULSOS];
uint8_t    puDigital[N_PULSOS];
unsigned long tSenseAdc = 0;
unsigned long tSenseI2c = 0;

bool    rtcOk = false, rtcValido = false, rtcOsf = false, rtcCh = false;
int     rtcAnio = 0;
uint8_t rtcMes = 0, rtcDia = 0, rtcHora = 0, rtcMin = 0, rtcSeg = 0;

// ============================================
// SIM7600 / GNSS
// ============================================
enum SimEstado { SIM_APAGADO, SIM_ARRANCANDO, SIM_CONFIG, SIM_OPERANDO, SIM_RESYNC };
SimEstado simEstado   = SIM_APAGADO;
unsigned long simT0   = 0;
uint8_t  simPaso      = 0;
uint8_t  simFallos    = 0;
uint8_t  gpsErrores   = 0;
uint32_t simResyncs   = 0;
unsigned long gpsUltConsulta = 0;

enum AtRes { AT_LIBRE, AT_ESPERANDO, AT_OK, AT_ERROR, AT_TIMEOUT };
AtRes    atRes      = AT_LIBRE;
unsigned long atT0  = 0;
unsigned long atTmo = 0;
const char* atPrefijo = nullptr;
char     atResp[128]   = {0};
char     simLinea[160] = {0};
uint16_t simLineaLen   = 0;

bool     gpsValido  = false;
int32_t  gpsLatE6   = 0;
int32_t  gpsLonE6   = 0;
float    gpsAlt     = NAN;
unsigned long gpsUltFixMs = 0;

// ============================================
// SDI-12
// ============================================
enum SdiEstado : uint8_t { SDI_LIBRE, SDI_ESCANEANDO, SDI_MED_ID, SDI_MED_CMD, SDI_MED_ESPERA,
                           SDI_MED_DATOS, SDI_CAMBIO, SDI_MANUAL };

typedef struct {
  char a;
  char id[40];         // respuesta a aI! sin la direccion
} SdiSensor;

typedef struct {
  SdiSensor sens[SDI12_NUM_DIR];
  uint8_t   n;
  uint8_t   scanIdx;   // 0..61 sondeo con "a!", luego lectura de aI!
  uint8_t   idIdx;
  bool      scanHecho;

  char      medA;      // direccion medida
  char      medTipo;   // 'C' o 'M'
  uint8_t   medIdx;    // 0 = aC!/aM!, 1..9 = aC1!..aM9!
  char      medCmd[8];
  char      medResp[24];
  char      medId[40];
  uint16_t  ttt;
  uint16_t  nEsp;
  int64_t   listoUs;
  uint8_t   dIdx;
  char      vals[SDI12_MAX_VALORES][12];
  uint8_t   nVals;
  char      dResp[10][84];
  uint8_t   nD;
  char      medErr[48];
  bool      medHecho;
  char      srBuf[8];  // service request de aM! ("a\r\n")
  uint8_t   srLen;

  char      chgDe, chgA;
  bool      chgOk, chgHecho;
  char      chgMsg[48];

  char      manTx[24];
  char      manRx[84];
  bool      manHecho;

  char      monTx[SDI12_MONITOR][24];
  char      monRx[SDI12_MONITOR][84];
  uint8_t   monN, monCab;
} SdiDatos;

SdiDatos* sdi       = nullptr;   // en heap (no consume la RAM estatica del registro)
SdiEstado sdiEstado = SDI_LIBRE;

// ============================================
// SISTEMA
// ============================================
char     apSsid[32];
int64_t  proxLogUs    = 0;
unsigned long tVeleta = 0;
unsigned long tBme    = 0;
char*    jsonBuf      = nullptr;   // en heap
size_t   jsonCap      = 0;
size_t   jsonLen      = 0;

// ============================================
// FORWARD DECLARATIONS
// ============================================
void isrPulso(void* arg);
void pulsoArmar(uint8_t i, bool pullup);
void pulsoDesarmar(uint8_t i);
void pulsoLeer(uint8_t i, uint32_t& tot, uint32_t& perUs, int64_t& ultUs);

void perfilCambiar(Perfil nuevo);
void perfilSalir(Perfil p);
void perfilEntrar(Perfil p);
const char* perfilTxt(Perfil p);

uint32_t crc32Calc(const void* d, size_t n);
int  fmtFijo(char* o, size_t n, int64_t v, uint8_t dec);
uint32_t diasDesdeCivil(int y, unsigned m, unsigned d);
void civilDesdeEpoch(uint32_t t, int& y, unsigned& mo, unsigned& d, unsigned& h, unsigned& mi, unsigned& s);
float mvAGrados(uint32_t mv);
const char* cardinal(float deg);
bool parsearCGPSINFO(const char* l, int32_t& latE6, int32_t& lonE6, float& alt, int64_t& epochUs);
uint8_t bin2bcd(uint8_t v);
bool bcdValido(uint8_t b);
uint8_t bcd2bin(uint8_t b);
bool fechaValida(int y, unsigned mo, unsigned d, unsigned h, unsigned mi, unsigned s);
char sdiDirEn(int i);
int  sdiIndice(char a);
uint8_t sdiParsearValores(const char* r, char vals[][12], uint8_t nVals, uint8_t maxVals);
bool sdiParsearMedicion(const char* r, char a, char tipo, uint16_t& ttt, uint16_t& n);

uint32_t uptimeS();
uint32_t epochAhora();
void relojAjustar(int64_t epochUs, uint8_t fuente);

void logInit();
bool logHdrValido();
void logHdrSellar();
void logBorrar();
void logAgregar(const LogRec& r);
void logFecharArranque(uint32_t epochArranque);
void registrar();
void ventanaWeatherReiniciar();
int  formatearLineaCSV(char* o, size_t n, const LogRec& r, uint32_t num);

void adcIniciar();
bool adcConfigCanal(uint8_t gpio, adc_atten_t att);
LecturaAdc adcLeer(uint8_t gpio, adc_atten_t att, uint8_t n);
int  adcCuenta11(int raw);

void muestrearVeleta();
bool bmeIniciar();
void tareaBME();
bool shtIniciar();
void tareaSHT();

bool rtcLeer();
bool rtcEscribir(int y, unsigned mo, unsigned d, unsigned h, unsigned mi, unsigned s);
void senseTarea();

void simTarea();
void simLeerUart();
void simProcesarLinea(const char* l);
void atEnviar(const char* cmd, const char* prefijo, unsigned long tmo);
void simEncender();
void simDetener();
void simResincronizar();
bool gpsFixVigente();
const char* simEstadoTxt();

void sdiIniciar();
void sdiDetener();
size_t sdiLeerLinea(char* resp, size_t n, uint16_t primerMs);
size_t sdiTransaccion(const char* cmd, char* resp, size_t n, uint16_t primerMs);
bool sdiProbar(char a);
bool sdiEnLista(char a);
void sdiTarea();
void sdiCambiarId();
const char* sdiEstadoTxt();

void iniciarWiFi();
void jReset();
void jAdd(const char* fmt, ...);
void jNum(const char* k, float v, int dec, bool coma);
void jStr(const char* k, const char* s, bool coma);
void jSis();
void jEnviar();
bool exigirPerfil(Perfil p);
void responder(int code, bool ok, const char* msg);
void webRaiz();
void webPortal();
void webPerfil();
void webApiSis();
void webApiWeather();
void webCSV();
void webBorrar();
void webHora();
void webApiSense();
void webSenseRtc();
void webSenseReset();
void webApiSdi();
bool sdiLibreOResponder();
void webSdiScan();
void webSdiMedir();
void webSdiId();
void webSdiCmd();
const char* resetTxt(esp_reset_reason_t r);

// ============================================
// PAGINA WEB (portal cautivo)
// ============================================
const char PAGINA[] PROGMEM = R"rawliteral(<!DOCTYPE html><html lang="es"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>LEM Tester Nodos</title>
<style>
:root{--bg:#f3f5f7;--cd:#fff;--tx:#16202a;--mu:#5b6b7b;--ac:#0b7a55;--bd:#dce2e8;--wa:#a86500;--er:#c0392b}
@media(prefers-color-scheme:dark){:root{--bg:#0e1318;--cd:#17202a;--tx:#e6ecf1;--mu:#93a3b3;--ac:#3fd69a;--bd:#283542;--wa:#f2b14c;--er:#ef7468}}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--tx);font:15px/1.45 system-ui,-apple-system,"Segoe UI",Roboto,sans-serif}
header{display:flex;justify-content:space-between;align-items:center;gap:8px;padding:12px 16px;background:var(--cd)}
h1{font-size:17px;margin:0}h1 small{color:var(--mu);font-weight:400;font-size:12px;margin-left:6px}
nav{display:flex;gap:4px;padding:6px 16px 8px;background:var(--cd);border-bottom:1px solid var(--bd);overflow-x:auto;position:sticky;top:0;z-index:2}
nav a{padding:7px 12px;border-radius:8px;color:var(--mu);text-decoration:none;font-weight:600;white-space:nowrap}
nav a.on{background:var(--ac);color:var(--cd)}
main{display:none;gap:12px;padding:12px 16px;max-width:1100px;margin:auto;grid-template-columns:repeat(auto-fit,minmax(280px,1fr))}
main.on{display:grid}
.c{background:var(--cd);border:1px solid var(--bd);border-radius:12px;padding:14px;min-width:0}.w2{grid-column:1/-1}
.c h2{font-size:12px;letter-spacing:.08em;text-transform:uppercase;color:var(--mu);margin:0 0 8px;font-weight:600}
.big{font-size:28px;font-weight:600;font-variant-numeric:tabular-nums;line-height:1.15}
.big u{text-decoration:none;font-size:15px;color:var(--mu);font-weight:400;margin-left:3px}
.r{display:flex;justify-content:space-between;gap:10px;padding:5px 0;border-bottom:1px dashed var(--bd)}.r:last-child{border:0}
.r span{color:var(--mu)}.r b{font-weight:600;font-variant-numeric:tabular-nums;text-align:right;overflow-wrap:anywhere}
.tg{display:inline-block;padding:1px 9px;border-radius:99px;font-size:12px;border:1px solid currentColor;white-space:nowrap}
.ok{color:var(--ac)}.wa{color:var(--wa)}.er{color:var(--er)}.mu{color:var(--mu)}
.bar{height:10px;background:var(--bd);border-radius:99px;overflow:hidden;margin:8px 0}.bar i{display:block;height:100%;width:0;background:var(--ac)}
.bt{display:flex;gap:8px;flex-wrap:wrap;margin-top:10px}
.b{padding:9px 14px;border-radius:8px;border:1px solid var(--ac);background:var(--ac);color:var(--cd);font:inherit;font-weight:600;text-decoration:none;cursor:pointer}
.b.s{background:none;color:var(--ac)}.b.x{background:none;color:var(--er);border-color:var(--er)}.b:disabled{opacity:.45;cursor:default}
.w{display:flex;gap:14px;align-items:center}svg{flex:none}
.nt{font-size:12px;color:var(--mu);margin:8px 0 0}a{color:var(--ac)}
.op{display:block;text-decoration:none;color:inherit}.op h3{margin:0 0 4px;font-size:19px;color:var(--ac)}.op p{margin:0;color:var(--mu)}
.tw{overflow-x:auto}table{width:100%;border-collapse:collapse;font-size:14px;font-variant-numeric:tabular-nums}
th,td{padding:6px 5px;text-align:right;border-bottom:1px dashed var(--bd);white-space:nowrap}th:first-child,td:first-child{text-align:left}
th{font-size:12px;color:var(--mu);font-weight:600}td small{display:block;color:var(--mu);font-size:12px}
.fr{display:flex;gap:8px;flex-wrap:wrap;align-items:end;margin-top:8px}.fr label{display:flex;flex-direction:column;font-size:12px;color:var(--mu);gap:3px}
select,input{font:inherit;padding:8px;border-radius:8px;border:1px solid var(--bd);background:var(--bg);color:var(--tx);min-width:0}
#q_cmd{flex:1;min-width:120px}
.lst div{display:flex;justify-content:space-between;gap:8px;padding:8px;border:1px solid var(--bd);border-radius:8px;margin-top:6px;cursor:pointer}
.lst div.on{border-color:var(--ac);box-shadow:inset 3px 0 0 var(--ac)}.lst b{font-size:18px}
pre{margin:6px 0 0;font-size:12px;white-space:pre-wrap;overflow-wrap:anywhere;color:var(--mu)}
#aviso{display:none;gap:10px;align-items:center;justify-content:space-between;margin:12px 16px 0;padding:10px 12px;border:1px solid var(--wa);border-radius:10px;color:var(--wa)}
</style></head><body>
<header><h1>LEM &middot; Tester de nodos<small id="id"></small></h1><span id="con" class="tg mu">conectando</span></header>
<nav><a href="#inicio" id="n-inicio">Inicio</a><a href="#weather" id="n-weather">LemWeather</a><a href="#sense" id="n-sense">LemSense</a><a href="#sdi" id="n-sdi">SDI-12</a></nav>
<div id="aviso"><span id="avt"></span><button class="b s" id="avb">Activar aqu&iacute;</button></div>

<main id="v-inicio">
<a class="c op" href="#weather"><h3>LemWeather</h3><p>GPS del SIM7600, BME280, veleta, pulsos de viento y registro CSV cada 1 min.</p></a>
<a class="c op" href="#sense"><h3>LemSense</h3><p>SHT30 / BME280, voltajes P0_0&hellip;P0_6, pulsos P1_6&hellip;P2_1 y reloj DS1338.</p></a>
<a class="c op" href="#sdi"><h3>SDI-12</h3><p>Buscar sensores, identificaci&oacute;n (aI!), mediciones aC!/aM! y cambio de ID.</p></a>
<section class="c"><h2>Sistema</h2>
<div class="r"><span>Perfil activo</span><b id="i_pf">--</b></div>
<div class="r"><span>Encendido hace</span><b id="i_up">--</b></div>
<div class="r"><span>&Uacute;ltimo reset</span><b id="i_rst">--</b></div>
<div class="r"><span>Heap libre / m&iacute;nimo</span><b id="i_hp">--</b></div>
<div class="r"><span>Firmware</span><b id="i_fw">--</b></div>
</section>
</main>

<main id="v-weather">
<section class="c"><h2>Ubicaci&oacute;n GPS</h2>
<div class="r"><span>Estado</span><b id="gst">--</b></div>
<div class="r"><span>Latitud</span><b id="lat">--</b></div>
<div class="r"><span>Longitud</span><b id="lon">--</b></div>
<div class="r"><span>Altitud</span><b id="alt">--</b></div>
<div class="r"><span>M&oacute;dulo SIM7600</span><b id="sim">--</b></div>
<div class="bt"><a class="b s" id="geo" href="#">Abrir en mapa</a><a class="b s" id="gm" href="#" target="_blank">Google Maps</a></div>
</section>
<section class="c"><h2>Veleta</h2>
<div class="w"><svg width="112" height="112" viewBox="0 0 100 100" aria-label="Br&uacute;jula">
<circle cx="50" cy="50" r="46" fill="none" stroke="currentColor" stroke-opacity=".25" stroke-width="2"/>
<g fill="currentColor" font-size="10" font-weight="600" text-anchor="middle"><text x="50" y="15">N</text><text x="88" y="54">E</text><text x="50" y="94">S</text><text x="12" y="54">O</text></g>
<g id="ag" style="transition:transform .6s;transform-origin:50px 50px"><polygon points="50,14 56,50 50,45 44,50" fill="var(--ac)"/><polygon points="50,86 56,50 50,55 44,50" fill="currentColor" fill-opacity=".3"/></g>
<circle cx="50" cy="50" r="3" fill="currentColor"/></svg>
<div><div class="big" id="dir">--</div><div class="mu" id="dirc">--</div></div></div>
<div class="r"><span>Media &uacute;ltimo minuto</span><b id="dirm">--</b></div>
<div class="r"><span>Tensi&oacute;n</span><b id="mv">--</b></div>
</section>
<section class="c"><h2>Pulsos de viento (P1_6)</h2>
<div class="big" id="ptot">--<u>pulsos</u></div>
<div class="r"><span>Per&iacute;odo</span><b id="per">--</b></div>
<div class="r"><span>Frecuencia</span><b id="hz">--</b></div>
<div class="r"><span>Pulsos &uacute;ltimo minuto</span><b id="pmin">--</b></div>
<div class="r"><span>Per&iacute;odo medio &uacute;ltimo minuto</span><b id="pmed">--</b></div>
</section>
<section class="c"><h2>Ambiente (BME280)</h2>
<div class="big" id="t">--</div>
<div class="r"><span>Humedad relativa</span><b id="h">--</b></div>
<div class="r"><span>Presi&oacute;n atmosf&eacute;rica</span><b id="p">--</b></div>
<div class="r"><span>Sensor</span><b id="bst">--</b></div>
</section>
<section class="c"><h2>Registro en RAM (1 min)</h2>
<div class="big" id="ln">--</div>
<div class="bar"><i id="lb"></i></div>
<div class="r"><span>Capacidad</span><b id="lcap">--</b></div>
<div class="r"><span>Tiempo hasta sobrescribir</span><b id="lrest">--</b></div>
<div class="r"><span>Memoria</span><b id="lmem">--</b></div>
<div class="bt"><a class="b" href="/log.csv" download>Descargar CSV</a><button class="b x" id="del">Borrar</button></div>
<p class="nt">El registro solo avanza con este perfil activo. Si la descarga no inicia desde la ventana del portal, abre <b>http://192.168.4.1</b> en el navegador.</p>
</section>
<section class="c"><h2>Hora</h2>
<div class="r"><span>Hora local</span><b id="hl">--</b></div>
<div class="r"><span>Fuente de hora</span><b id="hf">--</b></div>
</section>
</main>

<main id="v-sense">
<section class="c"><h2>Sensores I2C (P1_5 SDA &middot; P1_3 SCL)</h2>
<div class="r"><span>SHT30</span><b id="s_sst">--</b></div>
<div class="r"><span>Temperatura</span><b id="s_st">--</b></div>
<div class="r"><span>Humedad relativa</span><b id="s_sh">--</b></div>
<div class="r"><span>BME280</span><b id="s_bst">--</b></div>
<div class="r"><span>Temperatura</span><b id="s_bt">--</b></div>
<div class="r"><span>Humedad relativa</span><b id="s_bh">--</b></div>
<div class="r"><span>Presi&oacute;n</span><b id="s_bp">--</b></div>
</section>
<section class="c"><h2>Reloj DS1338</h2>
<div class="r"><span>Estado</span><b id="s_rst">--</b></div>
<div class="big" id="s_rf">--</div>
<div class="mu" id="s_rd">--</div>
<div class="r"><span>Oscilador</span><b id="s_rosc">--</b></div>
<div class="fr"><label style="flex:1">Fecha y hora a escribir<input type="datetime-local" step="1" id="s_rin"></label></div>
<div class="bt"><button class="b" id="s_rw">Escribir esta fecha</button><button class="b s" id="s_rtel">Escribir hora del tel&eacute;fono</button></div>
<p class="nt" id="s_rmsg"></p>
</section>
<section class="c w2"><h2>Entradas anal&oacute;gicas</h2>
<div class="tw"><table><thead><tr><th>Pin</th><th>Voltaje</th><th>N&deg; ADC</th></tr></thead><tbody id="s_an"></tbody></table></div>
<p class="nt">N&deg; ADC en escala 0-2047. ADC del ESP32-S2 a 12 dB: rango calibrado 0 a ~2,5 V; "&ge;" indica ADC saturado (el voltaje real es mayor).</p>
</section>
<section class="c w2"><h2>Pulsos</h2>
<div class="tw"><table><thead><tr><th>Pin</th><th>Voltaje / ADC</th><th>Estado</th><th>Pulsos / Hz</th></tr></thead><tbody id="s_pu"></tbody></table></div>
<div class="bt"><button class="b s" id="s_prst">Reiniciar contadores</button></div>
<p class="nt" id="s_pnt">HIGH si el pin est&aacute; sobre 3,0 V. Los pulsos cuentan flancos de bajada.</p>
</section>
</main>

<main id="v-sdi">
<section class="c"><h2>Bus SDI-12</h2>
<div class="r"><span>Estado</span><b id="q_est">--</b></div>
<div class="bar"><i id="q_bar"></i></div>
<div class="r"><span>Sensores encontrados</span><b id="q_n">--</b></div>
<div class="bt"><button class="b" id="q_scan">Buscar sensores</button></div>
<div class="lst" id="q_lst"></div>
<p class="nt" id="q_pins"></p>
</section>
<section class="c"><h2>Sensor <span id="q_sa">--</span></h2>
<div class="r"><span>Versi&oacute;n SDI-12</span><b id="q_iv">--</b></div>
<div class="r"><span>Fabricante</span><b id="q_if">--</b></div>
<div class="r"><span>Modelo</span><b id="q_im">--</b></div>
<div class="r"><span>Versi&oacute;n sensor</span><b id="q_iw">--</b></div>
<div class="r"><span>Serie / opcional</span><b id="q_is">--</b></div>
<div class="fr"><label>Medici&oacute;n<select id="q_m"><option value="C">aC! concurrente</option><option value="M">aM!</option></select></label>
<label>&Iacute;ndice<select id="q_i"><option value="0">&mdash;</option><option>1</option><option>2</option><option>3</option><option>4</option><option>5</option><option>6</option><option>7</option><option>8</option><option>9</option></select></label>
<button class="b" id="q_med">Medir</button></div>
<div class="r"><span>Comando / respuesta</span><b id="q_mc">--</b></div>
<div class="r"><span>Estado</span><b id="q_ms">--</b></div>
<div class="tw"><table><thead><tr><th>#</th><th>Valor</th></tr></thead><tbody id="q_vals"></tbody></table></div>
<pre id="q_d"></pre>
</section>
<section class="c"><h2>Cambiar ID</h2>
<div class="fr"><label>Sensor<select id="q_de"></select></label><label>Nuevo ID (libres)<select id="q_a"></select></label><button class="b" id="q_chg">Cambiar</button></div>
<p class="nt" id="q_chgm"></p>
<h2 style="margin-top:16px">Comando manual</h2>
<div class="fr"><input id="q_cmd" placeholder="0I!" maxlength="20" autocapitalize="off" autocomplete="off"><button class="b" id="q_tx">Enviar</button></div>
<div class="r"><span>Respuesta</span><b id="q_rx">--</b></div>
</section>
<section class="c w2"><h2>Monitor del bus</h2><pre id="q_mon">--</pre></section>
</main>

<script>
const $=i=>document.getElementById(i);
const f=(v,d)=>v==null?'--':Number(v).toFixed(d);
function dur(s){s=Math.max(0,Math.round(s));const d=Math.floor(s/86400),h=Math.floor(s%86400/3600),m=Math.floor(s%3600/60);return(d?d+' d ':'')+(d||h?h+' h ':'')+m+' min';}
function kb(b){return(b/1024).toFixed(1)+' KB';}
function tag(e,t,c){e.textContent=t;e.className='tg '+c;}
function esc(s){return String(s).replace(/[&<>"]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]));}
const post=u=>fetch(u,{method:'POST'}).then(r=>r.json()).catch(()=>({ok:0,msg:'sin conexión'}));
const V=['inicio','weather','sense','sdi'],PN={weather:1,sense:2,sdi:3},PT=['ninguno','LemWeather','LemSense','SDI-12'];
const API={inicio:'/api/sis',weather:'/api/w',sense:'/api/s',sdi:'/api/sdi'};
let vista='inicio',timer=null,busy=false;
function ir(){
 vista=location.hash.slice(1);if(!V.includes(vista))vista='inicio';
 V.forEach(v=>{$('v-'+v).classList.toggle('on',v==vista);$('n-'+v).classList.toggle('on',v==vista);});
 if(PN[vista])post('/perfil?p='+PN[vista]).then(tick);else tick();
}
function aviso(p){const m=!!PN[vista]&&p!=PN[vista];$('aviso').style.display=m?'flex':'none';if(m)$('avt').textContent='Perfil activo: '+PT[p]+' (otro teléfono lo cambió).';}
$('avb').onclick=()=>post('/perfil?p='+PN[vista]).then(tick);
async function tick(){
 clearTimeout(timer);
 try{const r=await fetch(API[vista],{cache:'no-store'});const d=await r.json();
  $('id').textContent=d.id;aviso(d.perfil);R[vista](d);tag($('con'),'en línea','ok');}
 catch(e){tag($('con'),'sin conexión','er');}
 timer=setTimeout(tick,vista=='sdi'&&busy?700:2000);
}
function rIni(d){const s=d.sis;$('i_pf').textContent=PT[d.perfil];$('i_up').textContent=dur(s.up);$('i_rst').textContent=s.rst;
 $('i_hp').textContent=kb(s.heap)+' / '+kb(s.heapMin);$('i_fw').textContent=s.fw;}
let horaOk=false;
function rW(d){
 const g=d.gps;
 if(g.val){$('lat').textContent=f(g.lat,6)+'°';$('lon').textContent=f(g.lon,6)+'°';
  $('geo').href='geo:'+g.lat+','+g.lon+'?q='+g.lat+','+g.lon;$('gm').href='https://maps.google.com/?q='+g.lat+','+g.lon;}
 $('alt').textContent=g.alt==null?'--':f(g.alt,1)+' m';
 if(g.fix)tag($('gst'),'Fix vigente','ok');else if(g.val)tag($('gst'),'última hace '+dur(g.edad),'wa');else tag($('gst'),'buscando satélites','wa');
 $('sim').textContent=d.sim+(d.simRst?' ('+d.simRst+' resinc.)':'');
 const v=d.dir;
 $('dir').textContent=v.ok?f(v.deg,0)+'°':'--';$('dirc').textContent=v.ok?'desde '+v.card:'veleta en falla';
 $('ag').style.transform='rotate('+(v.ok?v.deg:0)+'deg)';
 $('dirm').textContent=v.med==null?'--':f(v.med,1)+'° '+v.medc;$('mv').textContent=v.mv+' mV';
 const p=d.pul;$('ptot').innerHTML=p.tot+'<u>pulsos</u>';
 if(p.edad==null||p.edad>=60000||p.per==null){$('per').textContent=p.edad==null?'sin pulsos':(p.edad>=60000?'detenido ('+dur(p.edad/1000)+')':'--');$('hz').textContent='0 Hz';}
 else{const e=Math.max(p.per,p.edad);$('per').textContent=(p.edad>p.per?'≥ ':'')+f(e,1)+' ms';$('hz').textContent=(p.edad>p.per?'≤ ':'')+f(1000/e,3)+' Hz';}
 $('pmin').textContent=p.min==null?'--':p.min;$('pmed').textContent=p.med==null?'--':f(p.med,1)+' ms';
 const b=d.bme;$('t').innerHTML=b.ok?f(b.t,2)+'<u>°C</u>':'--';
 $('h').textContent=b.ok?f(b.h,2)+' %':'--';$('p').textContent=b.ok?f(b.p,2)+' hPa':'--';
 if(b.ok)tag($('bst'),'OK 0x'+b.addr.toString(16),'ok');else tag($('bst'),'sin respuesta','er');
 const l=d.log;$('ln').innerHTML=l.n+'<u>/ '+l.cap+' registros</u>';
 $('lb').style.width=(l.cap?100*l.n/l.cap:0).toFixed(1)+'%';
 $('lcap').textContent=dur(l.cap*l.per)+' ('+l.rb+' B/reg)';
 $('lrest').textContent=l.n>=l.cap?'sobrescribiendo el más antiguo':dur((l.cap-l.n)*l.per);
 $('lmem').textContent=kb(l.cap*l.rb)+' RAM interna, sobrevive resets'+(l.rec?', '+l.rec+' recuperados':'');
 $('hl').textContent=d.epoch?new Date(d.epoch*1000).toLocaleString('es'):'sin hora';
 $('hf').textContent=['sin hora','navegador','GPS'][d.reloj];
 if(!d.reloj&&!horaOk){horaOk=true;post('/hora?e='+Math.floor(Date.now()/1000));}
}
$('del').onclick=()=>{if(confirm('¿Borrar todo el registro en RAM?'))post('/borrar');};
function vt(a){return !a.ok?'<span class="er">ADC2 ocupado</span>':(a.sat?'≥ ':'')+(a.mv/1000).toFixed(3)+' V';}
function fz(p){if(p.per==null||p.edad==null||p.edad>=60000)return '--';const e=Math.max(p.per,p.edad);return(p.edad>p.per?'≤ ':'')+(1000/e).toFixed(2)+' Hz';}
const DIAS=['domingo','lunes','martes','miércoles','jueves','viernes','sábado'];
let rtcIni=false;
function rS(d){
 const s=d.sht,b=d.bme;
 tag($('s_sst'),s.ok?'detectado 0x'+s.addr.toString(16):'no detectado',s.ok?'ok':'mu');
 $('s_st').textContent=s.ok&&s.t!=null?f(s.t,2)+' °C':'--';$('s_sh').textContent=s.ok&&s.h!=null?f(s.h,2)+' %':'--';
 tag($('s_bst'),b.ok?'detectado 0x'+b.addr.toString(16):'no detectado',b.ok?'ok':'mu');
 $('s_bt').textContent=b.ok&&b.t!=null?f(b.t,2)+' °C':'--';$('s_bh').textContent=b.ok&&b.h!=null?f(b.h,2)+' %':'--';
 $('s_bp').textContent=b.ok&&b.p!=null?f(b.p,2)+' hPa':'--';
 $('s_an').innerHTML=d.an.map(a=>'<tr><td>'+a.n+'<small>GPIO'+a.g+'</small></td><td>'+vt(a)+'</td><td>'+(a.ok?a.cnt:'--')+'</td></tr>').join('');
 $('s_pu').innerHTML=d.pu.map(p=>'<tr><td>'+p.n+'<small>GPIO'+p.g+'</small></td><td>'+vt(p)+'<small>ADC '+(p.ok?p.cnt:'--')+'</small></td><td>'+(p.ok?'<span class="tg '+(p.hi?'ok':'mu')+'">'+(p.hi?'HIGH':'LOW')+'</span>':'--')+'</td><td>'+p.tot+'<small>'+fz(p)+'</small></td></tr>').join('');
 const c=d.rtc;
 if(!c.ok){tag($('s_rst'),'no detectado (0x68)','er');$('s_rf').textContent='--';$('s_rd').textContent='--';$('s_rosc').textContent='--';}
 else if(!c.val){tag($('s_rst'),'en 0x68, fecha inválida','wa');$('s_rf').textContent='--';$('s_rd').textContent='escribe una fecha para iniciarlo';}
 else{tag($('s_rst'),'detectado 0x68','ok');$('s_rf').textContent=c.f.slice(11);
  const p=c.f.split(/[- :]/);$('s_rd').textContent=DIAS[new Date(+p[0],p[1]-1,+p[2]).getDay()]+' '+c.f.slice(0,10);}
 if(c.ok)$('s_rosc').innerHTML=c.ch?'<span class="er">detenido (CH=1)</span>':(c.osf?'<span class="wa">se detuvo alguna vez (OSF=1)</span>':'<span class="ok">en marcha</span>');
 if(!rtcIni){rtcIni=true;telefono();}
}
function dos(n){return String(n).padStart(2,'0');}
function fmtTel(t){return t.getFullYear()+'-'+dos(t.getMonth()+1)+'-'+dos(t.getDate())+'T'+dos(t.getHours())+':'+dos(t.getMinutes())+':'+dos(t.getSeconds());}
function telefono(){$('s_rin').value=fmtTel(new Date());}
function escribirRtc(v){const m=v.match(/^(\d{4})-(\d\d)-(\d\d)T(\d\d):(\d\d)(?::(\d\d))?/);
 if(!m){$('s_rmsg').textContent='Fecha incompleta';return;}
 $('s_rmsg').textContent='Escribiendo…';
 post('/s/rtc?f='+m[1]+m[2]+m[3]+m[4]+m[5]+(m[6]||'00')).then(r=>{$('s_rmsg').textContent=r.msg||'';tick();});}
$('s_rw').onclick=()=>escribirRtc($('s_rin').value);
$('s_rtel').onclick=()=>{telefono();escribirRtc($('s_rin').value);};
$('s_prst').onclick=()=>post('/s/reset').then(tick);
let selA=null,qLista='';
const QE={libre:'libre',escaneando:'buscando sensores…',midiendo:'midiendo…',esperando:'esperando datos…',leyendo:'leyendo datos…',cambiando:'cambiando ID…',comando:'enviando comando…'};
function pid(s){return{v:s.length>1?s[0]+'.'+s[1]:'--',f:s.substr(2,8).trim()||'--',m:s.substr(10,6).trim()||'--',w:s.substr(16,3).trim()||'--',s:s.substr(19).trim()||'--'};}
function dirs(){const a=[];for(let i=0;i<62;i++)a.push(i<10?String(i):String.fromCharCode(i<36?87+i:29+i));return a;}
function rQ(d){
 busy=d.est!='libre';
 $('q_est').textContent=QE[d.est]||d.est;
 $('q_bar').style.width=(d.est=='escaneando'?100*d.scan.i/d.scan.n:(d.scan.hecho?100:0))+'%';
 $('q_n').textContent=d.scan.hecho||d.sens.length?d.sens.length:'--';
 $('q_pins').textContent='TX GPIO'+d.pins[0]+', RX GPIO'+d.pins[1]+' (UART 1200 7E1)';
 ['q_scan','q_med','q_chg','q_tx'].forEach(i=>$(i).disabled=busy);
 const key=d.sens.map(s=>s.a+s.id).join('|');
 if(key!=qLista){qLista=key;
  if(!d.sens.some(s=>s.a==selA))selA=d.sens.length?d.sens[0].a:null;
  $('q_lst').innerHTML=d.sens.map(s=>{const p=pid(s.id);return '<div data-a="'+esc(s.a)+'"><b>'+esc(s.a)+'</b><span>'+esc(p.f)+' '+esc(p.m)+'</span></div>';}).join('');
  $('q_de').innerHTML=d.sens.map(s=>'<option>'+esc(s.a)+'</option>').join('');
  const usadas=d.sens.map(s=>s.a);$('q_a').innerHTML=dirs().filter(x=>!usadas.includes(x)).map(x=>'<option>'+x+'</option>').join('');
 }
 [...$('q_lst').children].forEach(e=>{e.classList.toggle('on',e.dataset.a==selA);e.onclick=()=>{selA=e.dataset.a;qLista='';tick();};});
 const s=d.sens.find(x=>x.a==selA);$('q_sa').textContent=selA==null?'--':selA;
 const p=pid(s?s.id:'');$('q_iv').textContent=p.v;$('q_if').textContent=p.f;$('q_im').textContent=p.m;$('q_iw').textContent=p.w;$('q_is').textContent=p.s;
 const m=d.med;
 if(m.a!=selA){$('q_mc').textContent='--';$('q_ms').textContent='sin medici\u00f3n';$('q_vals').innerHTML='';$('q_d').textContent='';}
 else{$('q_mc').textContent=m.cmd+(m.resp?' → '+m.resp:'');
  $('q_ms').textContent=d.est=='esperando'?'datos en '+f(m.resta,1)+' s':(m.err?m.err:(m.hecho?m.vals.length+' valores de '+(m.n||'?')+' esperados':(QE[d.est]||'')));
  $('q_vals').innerHTML=m.vals.map((v,i)=>'<tr><td>'+(i+1)+'</td><td>'+esc(v.replace(/^\+/,''))+'</td></tr>').join('');
  $('q_d').textContent=m.d.join('\n');}
 if(d.chg.hecho){$('q_chgm').textContent=(d.chg.ok?'OK: ':'Fallo: ')+d.chg.msg;$('q_chgm').className='nt '+(d.chg.ok?'ok':'er');}
 if(d.man.tx)$('q_rx').textContent=d.man.tx+' → '+(d.man.hecho?(d.man.rx||'(sin respuesta)'):'…');
 $('q_mon').textContent=d.mon.length?d.mon.map(x=>'> '+x[0]+'\n< '+(x[1]||'(sin respuesta)')).join('\n'):'--';
}
function qPost(u){post(u).then(r=>{if(!r.ok&&r.msg)alert(r.msg);busy=true;tick();});}
$('q_scan').onclick=()=>qPost('/sdi/scan');
$('q_med').onclick=()=>{if(selA==null){alert('Primero busca y elige un sensor');return;}qPost('/sdi/medir?a='+encodeURIComponent(selA)+'&m='+$('q_m').value+'&i='+$('q_i').value);};
$('q_chg').onclick=()=>{const de=$('q_de').value,a=$('q_a').value;if(!de||!a)return;if(confirm('¿Cambiar el ID '+de+' a '+a+'?'))qPost('/sdi/id?de='+encodeURIComponent(de)+'&a='+encodeURIComponent(a));};
$('q_tx').onclick=()=>{const c=$('q_cmd').value.trim();if(c)qPost('/sdi/cmd?c='+encodeURIComponent(c));};
const R={inicio:rIni,weather:rW,sense:rS,sdi:rQ};
window.onhashchange=ir;ir();
</script></body></html>)rawliteral";

// ============================================
// ISR PULSOS
// ============================================
// Antirrebote: se acepta un flanco de bajada solo si la linea estuvo en ALTO al
// menos PULSO_ALTO_MIN_US y si paso PULSO_PER_MIN_US desde el pulso anterior.
// arg = (canal << 8) | gpio. El nivel se lee del registro (seguro en ISR).
void IRAM_ATTR isrPulso(void* arg) {
  uint32_t a     = (uint32_t)(uintptr_t)arg;
  uint32_t i     = a >> 8;
  uint32_t g     = a & 0xFF;
  int64_t  t     = esp_timer_get_time();
  uint32_t nivel = (REG_READ(GPIO_IN_REG) >> g) & 1UL;
  portENTER_CRITICAL_ISR(&muxPulsos);
  volatile CanalPulso& c = pul[i];
  if (nivel) {
    c.subidaUs = t;
  } else if ((t - c.subidaUs) >= PULSO_ALTO_MIN_US &&
             (c.ultUs == 0 || (t - c.ultUs) >= PULSO_PER_MIN_US)) {
    if (c.ultUs != 0) {
      int64_t p = t - c.ultUs;
      c.perUs = (p > 0xFFFFFFFFLL) ? 0xFFFFFFFFUL : (uint32_t)p;
    }
    c.ultUs = t;
    c.tot   = c.tot + 1;
    if (c.venN == 0) c.venPrimUs = t;
    c.venUltUs = t;
    c.venN     = c.venN + 1;
  }
  portEXIT_CRITICAL_ISR(&muxPulsos);
}

// ============================================
// SETUP
// ============================================
void setup() {
  // Reguladores siempre encendidos
  pinMode(SIM_EN_PIN, OUTPUT);
  digitalWrite(SIM_EN_PIN, HIGH);
  pinMode(SENS_EN_PIN, OUTPUT);
  digitalWrite(SENS_EN_PIN, HIGH);
  pinMode(SIM_TX_PIN, INPUT);
  pinMode(SIM_RX_PIN, INPUT);

  Serial.begin(115200);
  delay(20);

  Wire.begin(SDA_PIN, SCL_PIN, 100000);
  bmeOk = bmeIniciar();
  bmeUltIntento = millis();

  adcIniciar();
  uart1.setRxBufferSize(1024);

  jsonCap = 8192;
  jsonBuf = (char*)malloc(jsonCap);

  iniciarWiFi();
  logInit();

  // Volver al perfil que estaba activo si fue un reset (no un encendido)
  Perfil p = PERFIL_NINGUNO;
  if (esp_reset_reason() != ESP_RST_POWERON && (perfilNoinit >> 8) == PERFIL_MAGIC &&
      (perfilNoinit & 0xFF) <= PERFIL_SDI12) {
    p = (Perfil)(perfilNoinit & 0xFF);
  }
  perfilCambiar(p);

  Serial.printf("LEM Tester Nodos v%s  SSID:%s  reset:%s  perfil:%s\n", FW_VERSION, apSsid,
                resetTxt(esp_reset_reason()), perfilTxt(perfil));
  Serial.printf("Registro LemWeather: %lu x %u B, recuperados %lu. Heap libre %lu B\n",
                (unsigned long)logCap, (unsigned)sizeof(LogRec), (unsigned long)logRecuperados,
                (unsigned long)ESP.getFreeHeap());
}

// ============================================
// MAIN LOOP
// ============================================
void loop() {
  dnsServer.processNextRequest();
  server.handleClient();

  if (perfil == PERFIL_WEATHER) {
    simTarea();
    unsigned long ahora = millis();
    if (ahora - tVeleta >= VELETA_PERIODO_MS) { tVeleta = ahora; muestrearVeleta(); }
    if (ahora - tBme >= BME_PERIODO_MS)       { tBme = ahora;    tareaBME(); }
    int64_t us = esp_timer_get_time();
    if (us >= proxLogUs) {
      registrar();
      proxLogUs += (int64_t)LOG_PERIODO_S * 1000000LL;
      if (proxLogUs <= us) proxLogUs = us + (int64_t)LOG_PERIODO_S * 1000000LL;
    }
  } else if (perfil == PERFIL_SENSE) {
    senseTarea();
  } else if (perfil == PERFIL_SDI12) {
    sdiTarea();
  }

  delay(2);
}

// ============================================
// PERFILES
// ============================================
const char* perfilTxt(Perfil p) {
  switch (p) {
    case PERFIL_WEATHER: return "LemWeather";
    case PERFIL_SENSE:   return "LemSense";
    case PERFIL_SDI12:   return "SDI-12";
    default:             return "ninguno";
  }
}

void perfilCambiar(Perfil nuevo) {
  if (nuevo == perfil && nuevo != PERFIL_NINGUNO) return;
  perfilSalir(perfil);
  perfil = nuevo;
  perfilEntrar(nuevo);
  perfilNoinit = (PERFIL_MAGIC << 8) | (uint32_t)nuevo;
}

// Deja los pines del perfil en entrada sin pull (alta impedancia)
void perfilSalir(Perfil p) {
  switch (p) {
    case PERFIL_WEATHER:
      portENTER_CRITICAL(&muxPulsos);
      pulsosBase += pul[VIENTO_CANAL].tot;      // el total de viento continua al volver
      portEXIT_CRITICAL(&muxPulsos);
      pulsoDesarmar(VIENTO_CANAL);
      simDetener();
      break;
    case PERFIL_SENSE:
      for (uint8_t i = 0; i < N_PULSOS; i++) pulsoDesarmar(i);
      break;
    case PERFIL_SDI12:
      sdiDetener();
      break;
    default:
      break;
  }
}

void perfilEntrar(Perfil p) {
  switch (p) {
    case PERFIL_WEATHER:
      adcConfigCanal(VELETA_PIN, ADC_ATTEN_DB_6);       // 0..~1,3 V cubre 1,12 V
      pulsoArmar(VIENTO_CANAL, true);                   // contacto a GND: pull-up
      ventanaWeatherReiniciar();
      dirEmaIni = false;
      simEstado = SIM_APAGADO;                          // simTarea abre el UART
      tVeleta = tBme = millis();
      proxLogUs = esp_timer_get_time() + (int64_t)LOG_PERIODO_S * 1000000LL;
      break;
    case PERFIL_SENSE:
      // Primero el canal ADC (deja el pin como analogico) y despues la entrada
      // digital: asi el mismo pin cuenta pulsos y se le mide el voltaje.
      for (uint8_t i = 0; i < N_ANALOG; i++) adcConfigCanal(ANALOG_GPIO[i], ADC_ATTEN_DB_12);
      for (uint8_t i = 0; i < N_PULSOS; i++) {
        adcConfigCanal(PULSO_GPIO[i], ADC_ATTEN_DB_12);
        pulsoArmar(i, SENSE_PULLUP);
      }
      for (uint8_t i = 0; i < N_ANALOG; i++) anLect[i] = {0, 0, false, false, false};
      for (uint8_t i = 0; i < N_PULSOS; i++) puLect[i] = {0, 0, false, false, false};
      tSenseAdc = millis() - SENSE_ADC_MS;
      tSenseI2c = millis() - SENSE_I2C_MS;
      shtUltIntento = millis() - SENSE_REINTENTO_MS;
      break;
    case PERFIL_SDI12:
      sdiIniciar();
      break;
    default:
      break;
  }
}

// ============================================
// UTILIDADES (sin dependencias de hardware)
// ============================================
uint32_t crc32Calc(const void* d, size_t n) {
  const uint8_t* p = (const uint8_t*)d;
  uint32_t c = 0xFFFFFFFFUL;
  while (n--) {
    c ^= *p++;
    for (int k = 0; k < 8; k++) c = (c >> 1) ^ (0xEDB88320UL & (0UL - (c & 1UL)));
  }
  return ~c;
}

// Escribe v / 10^dec con 'dec' decimales exactos (punto decimal). Sin float.
int fmtFijo(char* o, size_t n, int64_t v, uint8_t dec) {
  static const uint32_t p10[] = {1UL, 10UL, 100UL, 1000UL, 10000UL, 100000UL, 1000000UL};
  bool neg   = v < 0;
  uint64_t a = neg ? (uint64_t)(-v) : (uint64_t)v;
  if (dec == 0) return snprintf(o, n, "%s%lu", neg ? "-" : "", (unsigned long)a);
  return snprintf(o, n, "%s%lu.%0*lu", neg ? "-" : "",
                  (unsigned long)(a / p10[dec]), (int)dec, (unsigned long)(a % p10[dec]));
}

// Dias desde 1970-01-01 (algoritmo de H. Hinnant, calendario gregoriano)
uint32_t diasDesdeCivil(int y, unsigned m, unsigned d) {
  y -= (m <= 2);
  int32_t  era = y / 400;
  uint32_t yoe = (uint32_t)(y - era * 400);
  uint32_t doy = (153 * (m > 2 ? m - 3 : m + 9) + 2) / 5 + d - 1;
  uint32_t doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  return (uint32_t)(era * 146097 + (int32_t)doe - 719468);
}

void civilDesdeEpoch(uint32_t t, int& y, unsigned& mo, unsigned& d, unsigned& h, unsigned& mi, unsigned& s) {
  uint32_t dias = t / 86400UL, sd = t % 86400UL;
  h  = sd / 3600; mi = (sd % 3600) / 60; s = sd % 60;
  uint32_t z   = dias + 719468UL;
  uint32_t era = z / 146097UL;
  uint32_t doe = z - era * 146097UL;
  uint32_t yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
  uint32_t doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
  uint32_t mp  = (5 * doy + 2) / 153;
  d  = doy - (153 * mp + 2) / 5 + 1;
  mo = mp < 10 ? mp + 3 : mp - 9;
  y  = (int)(yoe + era * 400) + (mo <= 2);
}

// 0 mV = Norte, VMAX/4 = Este, VMAX/2 = Sur, 3*VMAX/4 = Oeste, VMAX = Norte
float mvAGrados(uint32_t mv) {
  if (mv > VELETA_VMAX_MV) mv = VELETA_VMAX_MV;
  float d = (float)mv * 360.0f / (float)VELETA_VMAX_MV;
#if VELETA_INVERTIDA
  d = 360.0f - d;
#endif
  d = fmodf(d + VELETA_OFFSET_DEG, 360.0f);
  if (d < 0.0f) d += 360.0f;
  if (d >= 360.0f) d = 0.0f;
  return d;
}

const char* cardinal(float deg) {
  static const char* const pts[16] = {"N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
                                      "S", "SSO", "SO", "OSO", "O", "ONO", "NO", "NNO"};
  int i = (int)((deg + 11.25f) / 22.5f);
  return pts[((i % 16) + 16) % 16];
}

// "+CGPSINFO: 3113.343286,N,12121.234064,E,250311,072809.3,44.1,0.0,0"
// Sin fix: "+CGPSINFO: ,,,,,,,,". Retorna true si hay posicion; epochUs = 0 sin fecha.
bool parsearCGPSINFO(const char* l, int32_t& latE6, int32_t& lonE6, float& alt, int64_t& epochUs) {
  epochUs = 0;
  const char* p = strchr(l, ':');
  if (!p) return false;
  p++;
  while (*p == ' ') p++;
  char buf[112];
  strncpy(buf, p, sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = 0;

  char* c[9];
  int   nc = 0;
  char* s  = buf;
  c[nc++]  = s;
  while (*s && nc < 9) {
    if (*s == ',') { *s = 0; c[nc++] = s + 1; }
    s++;
  }
  if (nc < 6 || c[0][0] == 0 || c[2][0] == 0) return false;

  double latR = atof(c[0]), lonR = atof(c[2]);
  double latG = floor(latR / 100.0), lonG = floor(lonR / 100.0);
  double lat  = latG + (latR - latG * 100.0) / 60.0;
  double lon  = lonG + (lonR - lonG * 100.0) / 60.0;
  if (c[1][0] == 'S') lat = -lat;
  if (c[3][0] == 'W') lon = -lon;
  if (fabs(lat) > 90.0 || fabs(lon) > 180.0) return false;
  latE6 = (int32_t)lround(lat * 1e6);
  lonE6 = (int32_t)lround(lon * 1e6);
  alt   = (nc > 6 && c[6][0]) ? (float)atof(c[6]) : NAN;

  if (strlen(c[4]) == 6 && strlen(c[5]) >= 6) {
    unsigned dd = (c[4][0] - '0') * 10 + (c[4][1] - '0');
    unsigned mm = (c[4][2] - '0') * 10 + (c[4][3] - '0');
    unsigned yy = (c[4][4] - '0') * 10 + (c[4][5] - '0');
    unsigned hh = (c[5][0] - '0') * 10 + (c[5][1] - '0');
    unsigned mi = (c[5][2] - '0') * 10 + (c[5][3] - '0');
    double   ss = atof(c[5] + 4);
    if (dd >= 1 && dd <= 31 && mm >= 1 && mm <= 12 && hh < 24 && mi < 60 && ss >= 0.0 && ss < 61.0) {
      int64_t ep = (int64_t)diasDesdeCivil(2000 + yy, mm, dd) * 86400LL + hh * 3600LL + mi * 60LL;
      epochUs = ep * 1000000LL + (int64_t)llround(ss * 1e6);
    }
  }
  return true;
}

uint8_t bin2bcd(uint8_t v) {
  return (uint8_t)(((v / 10) << 4) | (v % 10));
}

bool bcdValido(uint8_t b) {
  return (b & 0x0F) <= 9 && (b >> 4) <= 9;
}

uint8_t bcd2bin(uint8_t b) {
  return (uint8_t)((b >> 4) * 10 + (b & 0x0F));
}

// Fecha gregoriana real (incluye dias por mes y bisiestos), anio 2000..2099 (DS1338)
bool fechaValida(int y, unsigned mo, unsigned d, unsigned h, unsigned mi, unsigned s) {
  if (y < 2000 || y > 2099 || mo < 1 || mo > 12 || d < 1 || h > 23 || mi > 59 || s > 59) return false;
  uint32_t ini = diasDesdeCivil(y, mo, 1);
  uint32_t fin = (mo == 12) ? diasDesdeCivil(y + 1, 1, 1) : diasDesdeCivil(y, mo + 1, 1);
  return d <= fin - ini;
}

// Direcciones SDI-12: indice 0..61 -> '0'-'9', 'a'-'z', 'A'-'Z'
char sdiDirEn(int i) {
  if (i < 10) return (char)('0' + i);
  if (i < 36) return (char)('a' + (i - 10));
  return (char)('A' + (i - 36));
}

int sdiIndice(char a) {
  if (a >= '0' && a <= '9') return a - '0';
  if (a >= 'a' && a <= 'z') return 10 + (a - 'a');
  if (a >= 'A' && a <= 'Z') return 36 + (a - 'A');
  return -1;
}

// Separa los valores de una respuesta aDn! ("a+1.23-4.5+6") por su signo.
// Guarda el texto exacto de cada valor (sin pasar por float). Retorna cuantos agrego.
uint8_t sdiParsearValores(const char* r, char vals[][12], uint8_t nVals, uint8_t maxVals) {
  uint8_t agregados = 0;
  if (!r[0]) return 0;
  const char* p = r + 1;                       // saltar la direccion
  while (*p && nVals + agregados < maxVals) {
    if (*p != '+' && *p != '-') break;         // formato invalido
    char*  v = vals[nVals + agregados];
    size_t k = 0;
    v[k++] = *p++;
    while (*p && *p != '+' && *p != '-') {
      if (k < 11) v[k++] = *p;
      p++;
    }
    v[k] = 0;
    if (k < 2) break;                          // signo sin digitos
    agregados++;
  }
  return agregados;
}

// Respuesta a aM! = "atttn", a aC! = "atttnn" (ttt segundos, n valores)
bool sdiParsearMedicion(const char* r, char a, char tipo, uint16_t& ttt, uint16_t& n) {
  size_t len = strlen(r);
  size_t largo = (tipo == 'C') ? 6 : 5;
  if (len < largo || r[0] != a) return false;
  for (size_t i = 1; i < largo; i++) if (r[i] < '0' || r[i] > '9') return false;
  ttt = (uint16_t)((r[1] - '0') * 100 + (r[2] - '0') * 10 + (r[3] - '0'));
  n   = (uint16_t)atoi(r + 4);
  return true;
}

// ============================================
// RELOJ
// ============================================
uint32_t uptimeS() {
  return (uint32_t)(esp_timer_get_time() / 1000000LL);
}

uint32_t epochAhora() {
  if (relojFuente == RELOJ_NO) return 0;
  return (uint32_t)((esp_timer_get_time() + relojOffsetUs) / 1000000LL);
}

void relojAjustar(int64_t epochUs, uint8_t fuente) {
  if (epochUs < (int64_t)EPOCH_MIN_VALIDO * 1000000LL) return;
  if (fuente < relojFuente) return;
  bool primera  = (relojFuente == RELOJ_NO);
  relojOffsetUs = epochUs - esp_timer_get_time();
  relojFuente   = fuente;
  if (primera) logFecharArranque((uint32_t)(relojOffsetUs / 1000000LL));
}

// ============================================
// REGISTRO EN RAM (buffer circular, LemWeather)
// ============================================
void logHdrSellar() {
  logHdr.crc = crc32Calc(&logHdr, offsetof(LogHdr, crc));
}

bool logHdrValido() {
  if (logHdr.magic != LOG_MAGIC || logHdr.version != LOG_VERSION || logHdr.cap != logCap) return false;
  if (logHdr.head >= logCap || logHdr.count > logCap) return false;
  if (crc32Calc(&logHdr, offsetof(LogHdr, crc)) != logHdr.crc) return false;
  uint32_t suma = 0;
  uint32_t ini  = (logHdr.head + logCap - logHdr.count) % logCap;
  for (uint32_t i = 0; i < logHdr.count; i++) suma += crc32Calc(&logBuf[(ini + i) % logCap], sizeof(LogRec));
  return suma == logHdr.suma;
}

void logBorrar() {
  logHdr.head     = 0;
  logHdr.count    = 0;
  logHdr.escritos = 0;
  logHdr.suma     = 0;
  logHdrSellar();
  hayUltimoReg    = false;
}

void logInit() {
  logBuf = logInterno;
  logCap = LOG_CAP_INTERNA;
  if (esp_reset_reason() != ESP_RST_POWERON && logHdrValido()) {
    logHdr.arranques++;
    logRecuperados = logHdr.count;
    if (logHdr.count > 0) {
      const LogRec& u = logBuf[(logHdr.head + logCap - 1) % logCap];
      pulsosBase   = u.pulsosTot;
      ultimoReg    = u;
      hayUltimoReg = true;
    }
    logHdrSellar();
  } else {
    logHdr.magic = LOG_MAGIC; logHdr.version = LOG_VERSION; logHdr.cap = logCap; logHdr.arranques = 0;
    logBorrar();
  }
  arranqueActual = (uint8_t)(logHdr.arranques & 0xFF);
}

void logAgregar(const LogRec& r) {
  uint32_t i = logHdr.head;
  if (logHdr.count == logCap) logHdr.suma -= crc32Calc(&logBuf[i], sizeof(LogRec));
  else logHdr.count++;
  logBuf[i] = r;
  logHdr.suma += crc32Calc(&r, sizeof(LogRec));
  logHdr.head  = (i + 1) % logCap;
  logHdr.escritos++;
  logHdrSellar();
}

void logFecharArranque(uint32_t epochArranque) {
  uint32_t ini = (logHdr.head + logCap - logHdr.count) % logCap;
  for (uint32_t k = 0; k < logHdr.count; k++) {
    LogRec& r = logBuf[(ini + k) % logCap];
    if (r.arranque != arranqueActual || (r.flags & FLG_TS_EPOCH)) continue;
    logHdr.suma -= crc32Calc(&r, sizeof(LogRec));
    r.ts    += epochArranque;
    r.flags |= FLG_TS_EPOCH;
    logHdr.suma += crc32Calc(&r, sizeof(LogRec));
  }
  if (hayUltimoReg && ultimoReg.arranque == arranqueActual && !(ultimoReg.flags & FLG_TS_EPOCH)) {
    ultimoReg.ts    += epochArranque;
    ultimoReg.flags |= FLG_TS_EPOCH;
  }
  logHdrSellar();
}

// Reinicia los acumuladores de la ventana de 1 minuto (al entrar al perfil)
void ventanaWeatherReiniciar() {
  venSumT = venSumH = venSumP = 0;
  venBmeN = 0;
  venDirSin = venDirCos = 0.0f;
  venDirN = 0;
  portENTER_CRITICAL(&muxPulsos);
  pul[VIENTO_CANAL].venN = 0;
  portEXIT_CRITICAL(&muxPulsos);
}

// Cierra la ventana de 1 minuto y guarda un registro
void registrar() {
  LogRec r;
  memset(&r, 0, sizeof(r));
  r.arranque = arranqueActual;

  if (relojFuente != RELOJ_NO) { r.ts = epochAhora(); r.flags |= FLG_TS_EPOCH; }
  else r.ts = uptimeS();

  if (gpsValido) {
    r.lat_e6 = gpsLatE6;
    r.lon_e6 = gpsLonE6;
    r.flags |= FLG_GPS_VALIDO;
    if (gpsFixVigente()) r.flags |= FLG_GPS_FIX;
  }

  portENTER_CRITICAL(&muxPulsos);
  volatile CanalPulso& c = pul[VIENTO_CANAL];
  uint32_t n    = c.venN;
  int64_t  p0   = c.venPrimUs;
  int64_t  p1   = c.venUltUs;
  uint32_t tot  = c.tot;
  uint32_t ultP = c.perUs;
  c.venN        = 0;
  portEXIT_CRITICAL(&muxPulsos);
  r.pulsosTot = pulsosBase + tot;
  r.pulsosVen = (n > 0xFFFF) ? 0xFFFF : (uint16_t)n;
  if (n >= 2)      r.periodoUs = (uint32_t)((p1 - p0) / (int64_t)(n - 1));
  else if (n == 1) r.periodoUs = ultP;
  else             r.periodoUs = 0;

  if (venBmeN > 0) {
    r.temp_c   = (int16_t)lroundf(venSumT / venBmeN * 100.0f);
    r.hum_c    = (uint16_t)lroundf(venSumH / venBmeN * 100.0f);
    r.pres_2pa = (uint16_t)lroundf(venSumP / venBmeN * 50.0f);
    r.flags   |= FLG_BME_OK;
  }
  venSumT = venSumH = venSumP = 0;
  venBmeN = 0;

  if (venDirN > 0) {
    float d = atan2f(venDirSin, venDirCos) * RAD_TO_DEG;
    if (d < 0.0f) d += 360.0f;
    uint32_t dd = (uint32_t)lroundf(d * 10.0f);
    r.dir_d  = (uint16_t)(dd >= 3600 ? dd - 3600 : dd);
    r.flags |= FLG_DIR_OK;
  }
  venDirSin = venDirCos = 0.0f;
  venDirN   = 0;

  logAgregar(r);
  ultimoReg    = r;
  hayUltimoReg = true;
}

// n,fecha_utc,seg_desde_arranque,arranque,lat,lon,gps_fix,temp_C,hum_pct,pres_hPa,dir_deg,dir_card,pulsos_min,pulsos_total,periodo_ms
int formatearLineaCSV(char* o, size_t n, const LogRec& r, uint32_t num) {
  char fecha[24] = "", seg[12] = "", lat[16] = "", lon[16] = "";
  char t[12] = "", h[12] = "", p[12] = "", dir[10] = "", per[16] = "";
  const char* card = "";

  if (r.flags & FLG_TS_EPOCH) {
    int y; unsigned mo, d, hh, mi, ss;
    civilDesdeEpoch(r.ts, y, mo, d, hh, mi, ss);
    snprintf(fecha, sizeof(fecha), "%04d-%02u-%02u %02u:%02u:%02u", y, mo, d, hh, mi, ss);
  } else {
    snprintf(seg, sizeof(seg), "%lu", (unsigned long)r.ts);
  }
  if (r.flags & FLG_GPS_VALIDO) {
    fmtFijo(lat, sizeof(lat), r.lat_e6, 6);
    fmtFijo(lon, sizeof(lon), r.lon_e6, 6);
  }
  if (r.flags & FLG_BME_OK) {
    fmtFijo(t, sizeof(t), r.temp_c, 2);
    fmtFijo(h, sizeof(h), r.hum_c, 2);
    fmtFijo(p, sizeof(p), (int64_t)r.pres_2pa * 2, 2);
  }
  if (r.flags & FLG_DIR_OK) {
    fmtFijo(dir, sizeof(dir), r.dir_d, 1);
    card = cardinal(r.dir_d / 10.0f);
  }
  if (r.periodoUs) fmtFijo(per, sizeof(per), r.periodoUs, 3);

  return snprintf(o, n, "%lu,%s,%s,%u,%s,%s,%u,%s,%s,%s,%s,%s,%u,%lu,%s\r\n",
                  (unsigned long)num, fecha, seg, (unsigned)r.arranque, lat, lon,
                  (r.flags & FLG_GPS_FIX) ? 1u : 0u, t, h, p, dir, card,
                  (unsigned)r.pulsosVen, (unsigned long)r.pulsosTot, per);
}

// ============================================
// ADC (driver oneshot del ESP-IDF, sin analogRead)
// ============================================
// Se usa el driver directo para poder tener distinta atenuacion por pin (veleta a
// 6 dB, LemSense a 12 dB) con su propia calibracion, y para reintentar en ADC2.
void adcIniciar() {
  for (int u = 0; u < 2; u++) {
    adc_oneshot_unit_init_cfg_t c = {};
    c.unit_id  = (adc_unit_t)u;
    c.ulp_mode = ADC_ULP_MODE_DISABLE;
    if (adc_oneshot_new_unit(&c, &adcUnidad[u]) != ESP_OK) adcUnidad[u] = nullptr;
    for (int a = 0; a < 2; a++) {
      adc_cali_line_fitting_config_t k = {};
      k.unit_id  = (adc_unit_t)u;
      k.atten    = a ? ADC_ATTEN_DB_12 : ADC_ATTEN_DB_6;
      k.bitwidth = ADC_BITWIDTH_DEFAULT;
      if (adc_cali_create_scheme_line_fitting(&k, &adcCal[u][a]) != ESP_OK) adcCal[u][a] = nullptr;
    }
  }
}

// Configura atenuacion del canal. OJO: el IDF deja el pin como analogico (sin
// entrada digital ni pull-up). Las lecturas posteriores no tocan el pin.
bool adcConfigCanal(uint8_t gpio, adc_atten_t att) {
  adc_unit_t u;
  adc_channel_t ch;
  if (adc_oneshot_io_to_channel(gpio, &u, &ch) != ESP_OK || !adcUnidad[u]) return false;
  adc_oneshot_chan_cfg_t c = {};
  c.atten    = att;
  c.bitwidth = ADC_BITWIDTH_DEFAULT;
  return adc_oneshot_config_channel(adcUnidad[u], ch, &c) == ESP_OK;
}

LecturaAdc adcLeer(uint8_t gpio, adc_atten_t att, uint8_t n) {
  LecturaAdc r = {0, 0, false, false, false};
  adc_unit_t u;
  adc_channel_t ch;
  if (adc_oneshot_io_to_channel(gpio, &u, &ch) != ESP_OK || !adcUnidad[u]) return r;
  int32_t suma = 0;
  int     ok   = 0;
  for (uint8_t i = 0; i < n; i++) {
    int v = 0;
    esp_err_t e = ESP_FAIL;
    for (int k = 0; k < ADC2_REINTENTOS && e != ESP_OK; k++) {
      e = adc_oneshot_read(adcUnidad[u], ch, &v);
      if (e == ESP_ERR_TIMEOUT) delayMicroseconds(100);   // ADC2 tomado por el WiFi
    }
    if (e == ESP_OK) { suma += v; ok++; }
  }
  if (!ok) return r;
  r.ok  = true;
  r.raw = (int)((suma + ok / 2) / ok);
  r.sat = r.raw >= ADC_RAW_MAX - ADC_SAT_MARGEN;
  adc_cali_handle_t cal = adcCal[u][att == ADC_ATTEN_DB_12 ? 1 : 0];
  if (cal && adc_cali_raw_to_voltage(cal, r.raw, &r.mv) == ESP_OK) r.cal = true;
  else r.mv = (int)((int64_t)r.raw * (att == ADC_ATTEN_DB_12 ? 2500 : 1300) / ADC_RAW_MAX);  // sin eFuse: aproximado
  return r;
}

// N de ADC en escala 0..2047 (13 bits -> 11 bits, con redondeo)
int adcCuenta11(int raw) {
  int c = (raw + 2) >> 2;
  return c > ADC_CNT_MAX ? ADC_CNT_MAX : c;
}

// ============================================
// PULSOS
// ============================================
void pulsoArmar(uint8_t i, bool pullup) {
  gpio_num_t g = (gpio_num_t)PULSO_GPIO[i];
  if (!isrServicio) {
    esp_err_t e = gpio_install_isr_service(0);
    isrServicio = (e == ESP_OK || e == ESP_ERR_INVALID_STATE);
  }
  if (pulArmado[i]) pulsoDesarmar(i);

  portENTER_CRITICAL(&muxPulsos);
  pul[i].tot = 0; pul[i].perUs = 0; pul[i].ultUs = 0; pul[i].subidaUs = 0;
  pul[i].venN = 0; pul[i].venPrimUs = 0; pul[i].venUltUs = 0;
  portEXIT_CRITICAL(&muxPulsos);

  gpio_config_t c = {};
  c.pin_bit_mask = 1ULL << g;
  c.mode         = GPIO_MODE_INPUT;
  c.pull_up_en   = pullup ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
  c.pull_down_en = GPIO_PULLDOWN_DISABLE;
  c.intr_type    = GPIO_INTR_ANYEDGE;
  gpio_config(&c);
  gpio_isr_handler_add(g, isrPulso, (void*)(uintptr_t)(((uint32_t)i << 8) | (uint32_t)g));
  pulArmado[i] = true;
}

void pulsoDesarmar(uint8_t i) {
  gpio_num_t g = (gpio_num_t)PULSO_GPIO[i];
  if (pulArmado[i]) {
    gpio_intr_disable(g);
    gpio_isr_handler_remove(g);
  }
  gpio_config_t c = {};
  c.pin_bit_mask = 1ULL << g;
  c.mode         = GPIO_MODE_INPUT;
  c.pull_up_en   = GPIO_PULLUP_DISABLE;
  c.pull_down_en = GPIO_PULLDOWN_DISABLE;
  c.intr_type    = GPIO_INTR_DISABLE;
  gpio_config(&c);
  pulArmado[i] = false;
}

void pulsoLeer(uint8_t i, uint32_t& tot, uint32_t& perUs, int64_t& ultUs) {
  portENTER_CRITICAL(&muxPulsos);
  tot   = pul[i].tot;
  perUs = pul[i].perUs;
  ultUs = pul[i].ultUs;
  portEXIT_CRITICAL(&muxPulsos);
}

// ============================================
// VELETA
// ============================================
void muestrearVeleta() {
  LecturaAdc l = adcLeer(VELETA_PIN, ADC_ATTEN_DB_6, VELETA_MUESTRAS);
  if (!l.ok) return;
  veletaMv = (uint32_t)l.mv;
  veletaOk = (veletaMv <= VELETA_MV_FALLA);
  if (!veletaOk) return;

  dirInstDeg = mvAGrados(veletaMv);
  float rad = dirInstDeg * DEG_TO_RAD;
  float cx = cosf(rad), sy = sinf(rad);
  venDirCos += cx;
  venDirSin += sy;
  venDirN++;
  if (!dirEmaIni) { dirEmaX = cx; dirEmaY = sy; dirEmaIni = true; }
  else {
    dirEmaX += VELETA_EMA * (cx - dirEmaX);
    dirEmaY += VELETA_EMA * (sy - dirEmaY);
  }
}

// ============================================
// SENSORES I2C (BME280 / SHT30)
// ============================================
bool bmeIniciar() {
  const uint8_t dirs[2] = {0x76, 0x77};
  for (uint8_t i = 0; i < 2; i++) {
    if (bme.begin(dirs[i], &Wire)) {
      bmeAddr = dirs[i];
      bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                      Adafruit_BME280::SAMPLING_X2,    // temperatura
                      Adafruit_BME280::SAMPLING_X16,   // presion
                      Adafruit_BME280::SAMPLING_X1,    // humedad
                      Adafruit_BME280::FILTER_X4,
                      Adafruit_BME280::STANDBY_MS_500);
      return true;
    }
  }
  return false;
}

void tareaBME() {
  unsigned long reintento = (perfil == PERFIL_SENSE) ? SENSE_REINTENTO_MS : BME_REINTENTO_MS;
  if (!bmeOk) {
    if (millis() - bmeUltIntento >= reintento) {
      bmeUltIntento = millis();
      bmeOk = bmeIniciar();
    }
    if (!bmeOk) { bmeT = bmeH = bmeP = NAN; return; }
  }

  Wire.beginTransmission(bmeAddr);
  if (Wire.endTransmission() != 0) {
    bmeOk = false;
    bmeUltIntento = millis();
    bmeT = bmeH = bmeP = NAN;
    return;
  }

  float t = bme.readTemperature();
  float h = bme.readHumidity();
  float p = bme.readPressure() / 100.0f;
  if (isnan(t) || isnan(h) || isnan(p) || t < -40.0f || t > 85.0f ||
      h < 0.0f || h > 100.0f || p < 300.0f || p > 1100.0f) {
    bmeT = bmeH = bmeP = NAN;
    return;
  }
  bmeT = t; bmeH = h; bmeP = p;
  venSumT += t; venSumH += h; venSumP += p;
  venBmeN++;
}

bool shtIniciar() {
  const uint8_t dirs[2] = {0x44, 0x45};
  for (uint8_t i = 0; i < 2; i++) {
    Wire.beginTransmission(dirs[i]);
    if (Wire.endTransmission() != 0) continue;     // nadie en esa direccion
    if (sht.begin(dirs[i])) { shtAddr = dirs[i]; return true; }
  }
  return false;
}

void tareaSHT() {
  if (!shtOk) {
    if (millis() - shtUltIntento >= SENSE_REINTENTO_MS) {
      shtUltIntento = millis();
      shtOk = shtIniciar();
    }
    if (!shtOk) { shtT = shtH = NAN; return; }
  }
  Wire.beginTransmission(shtAddr);
  if (Wire.endTransmission() != 0) {
    shtOk = false;
    shtUltIntento = millis();
    shtT = shtH = NAN;
    return;
  }
  float t, h;
  if (!sht.readBoth(&t, &h) || t < -40.0f || t > 125.0f || h < 0.0f || h > 100.0f) {
    shtT = shtH = NAN;                               // CRC o rango invalido
    return;
  }
  shtT = t; shtH = h;
}

// ============================================
// LEMSENSE: RELOJ DS1338 + TAREA
// ============================================
// Registros 0x00..0x06 en BCD (seg, min, hora, dia semana, dia, mes, anio),
// 0x07 control (bit 5 = OSF: el oscilador se detuvo). Bit 7 de 0x00 = CH (reloj detenido).
bool rtcLeer() {
  uint8_t r[8];
  Wire.beginTransmission(DS1338_ADDR);
  Wire.write((uint8_t)0x00);
  if (Wire.endTransmission() != 0 || Wire.requestFrom((uint8_t)DS1338_ADDR, (uint8_t)8) != 8) {
    rtcOk = false;
    rtcValido = false;
    return false;
  }
  for (uint8_t i = 0; i < 8; i++) r[i] = (uint8_t)Wire.read();
  rtcOk  = true;
  rtcCh  = r[0] & 0x80;
  rtcOsf = r[7] & 0x20;

  uint8_t hBcd = r[2] & 0x3F;
  bool    h12  = r[2] & 0x40;
  if (h12) hBcd = r[2] & 0x1F;
  bool bcdOk = bcdValido(r[0] & 0x7F) && bcdValido(r[1] & 0x7F) && bcdValido(hBcd) &&
               bcdValido(r[4] & 0x3F) && bcdValido(r[5] & 0x1F) && bcdValido(r[6]);
  if (!bcdOk) { rtcValido = false; return true; }

  rtcSeg  = bcd2bin(r[0] & 0x7F);
  rtcMin  = bcd2bin(r[1] & 0x7F);
  rtcHora = bcd2bin(hBcd);
  if (h12) rtcHora = (uint8_t)(rtcHora % 12 + ((r[2] & 0x20) ? 12 : 0));
  rtcDia  = bcd2bin(r[4] & 0x3F);
  rtcMes  = bcd2bin(r[5] & 0x1F);
  rtcAnio = 2000 + bcd2bin(r[6]);
  rtcValido = fechaValida(rtcAnio, rtcMes, rtcDia, rtcHora, rtcMin, rtcSeg);
  return true;
}

bool rtcEscribir(int y, unsigned mo, unsigned d, unsigned h, unsigned mi, unsigned s) {
  uint8_t dow = (uint8_t)((diasDesdeCivil(y, mo, d) + 3) % 7 + 1);   // 1 = lunes ... 7 = domingo
  Wire.beginTransmission(DS1338_ADDR);
  Wire.write((uint8_t)0x00);
  Wire.write((uint8_t)(bin2bcd((uint8_t)s) & 0x7F));    // CH = 0: oscilador en marcha
  Wire.write(bin2bcd((uint8_t)mi));
  Wire.write(bin2bcd((uint8_t)h));                      // bit 6 = 0: formato 24 h
  Wire.write(dow);
  Wire.write(bin2bcd((uint8_t)d));
  Wire.write(bin2bcd((uint8_t)mo));
  Wire.write(bin2bcd((uint8_t)(y - 2000)));
  if (Wire.endTransmission() != 0) return false;

  // Limpiar OSF (bit 5 del control), conservando el resto
  Wire.beginTransmission(DS1338_ADDR);
  Wire.write((uint8_t)0x07);
  if (Wire.endTransmission() != 0 || Wire.requestFrom((uint8_t)DS1338_ADDR, (uint8_t)1) != 1) return false;
  uint8_t ctrl = (uint8_t)Wire.read();
  Wire.beginTransmission(DS1338_ADDR);
  Wire.write((uint8_t)0x07);
  Wire.write((uint8_t)(ctrl & ~0x20));
  return Wire.endTransmission() == 0;
}

void senseTarea() {
  unsigned long ahora = millis();
  if (ahora - tSenseAdc >= SENSE_ADC_MS) {
    tSenseAdc = ahora;
    for (uint8_t i = 0; i < N_ANALOG; i++) anLect[i] = adcLeer(ANALOG_GPIO[i], ADC_ATTEN_DB_12, ADC_MUESTRAS);
    for (uint8_t i = 0; i < N_PULSOS; i++) {
      puLect[i]    = adcLeer(PULSO_GPIO[i], ADC_ATTEN_DB_12, ADC_MUESTRAS);
      puDigital[i] = (uint8_t)gpio_get_level((gpio_num_t)PULSO_GPIO[i]);
    }
  }
  if (ahora - tSenseI2c >= SENSE_I2C_MS) {
    tSenseI2c = ahora;
    tareaBME();
    tareaSHT();
    rtcLeer();
  }
}

// ============================================
// SIM7600: DRIVER AT NO BLOQUEANTE (LemWeather)
// ============================================
void atEnviar(const char* cmd, const char* prefijo, unsigned long tmo) {
  atResp[0] = 0;
  atPrefijo = prefijo;
  atTmo     = tmo;
  atT0      = millis();
  atRes     = AT_ESPERANDO;
  uart1.print(cmd);
  uart1.print("\r");
}

void simProcesarLinea(const char* l) {
  if (atRes != AT_ESPERANDO) return;
  if (!strcmp(l, "OK")) atRes = AT_OK;
  else if (!strcmp(l, "ERROR") || !strncmp(l, "+CME ERROR", 10)) atRes = AT_ERROR;
  else if (atPrefijo && !strncmp(l, atPrefijo, strlen(atPrefijo))) {
    strncpy(atResp, l, sizeof(atResp) - 1);
    atResp[sizeof(atResp) - 1] = 0;
  }
}

void simLeerUart() {
  while (uart1.available()) {
    char c = (char)uart1.read();
    if (c == '\r') continue;
    if (c == '\n') {
      simLinea[simLineaLen] = 0;
      if (simLineaLen) simProcesarLinea(simLinea);
      simLineaLen = 0;
    } else if (simLineaLen < sizeof(simLinea) - 1) {
      simLinea[simLineaLen++] = c;
    }
  }
  if (atRes == AT_ESPERANDO && millis() - atT0 >= atTmo) atRes = AT_TIMEOUT;
}

// GPIO14 queda siempre en alto: aqui solo se abre el UART
void simEncender() {
  uart1.begin(SIM_BAUD, SERIAL_8N1, SIM_RX_PIN, SIM_TX_PIN);
  uart1.setTxInvert(false);
  uart1.setRxInvert(false);
  simLineaLen = 0;
  atRes       = AT_LIBRE;
  simFallos   = 0;
  simT0       = millis();
  simEstado   = SIM_ARRANCANDO;
}

void simDetener() {
  uart1.end();
  pinMode(SIM_TX_PIN, INPUT);
  pinMode(SIM_RX_PIN, INPUT);
  atRes     = AT_LIBRE;
  simEstado = SIM_APAGADO;
}

// Sin respuesta: se cierra y reabre el UART (los reguladores no se tocan)
void simResincronizar() {
  uart1.end();
  atRes     = AT_LIBRE;
  simFallos = 0;
  simT0     = millis();
  simResyncs++;
  simEstado = SIM_RESYNC;
}

void simTarea() {
  if (simEstado != SIM_APAGADO && simEstado != SIM_RESYNC) simLeerUart();
  unsigned long ahora = millis();

  switch (simEstado) {
    case SIM_APAGADO:
      simEncender();
      break;

    case SIM_ARRANCANDO:
      if (atRes == AT_ESPERANDO) break;
      if (atRes == AT_OK) { atRes = AT_LIBRE; simPaso = 0; simEstado = SIM_CONFIG; break; }
      if (ahora - simT0 >= SIM_T_SYNC_MAX_MS) { simResincronizar(); break; }
      if (atRes == AT_LIBRE || ahora - atT0 >= 1000) atEnviar("AT", nullptr, 500);
      break;

    case SIM_CONFIG:
      if (atRes == AT_ESPERANDO) break;
      if (atRes == AT_TIMEOUT) {
        if (++simFallos >= SIM_FALLOS_MAX) { simResincronizar(); break; }
        atRes = AT_LIBRE;
      } else if (atRes == AT_OK || atRes == AT_ERROR) {
        simFallos = 0;
        switch (simPaso) {
          case 0: simPaso = GNSS_ANT_ACTIVA ? 1 : 3; break;
          case 1: simPaso = 2; break;
          case 2: simPaso = 3; break;
          case 3: simPaso = (atRes == AT_OK && !strncmp(atResp, "+CGPS: 1", 8)) ? 5 : 4; break;
          default: simPaso = 5; break;
        }
        atRes = AT_LIBRE;
      }
      switch (simPaso) {
        case 0: atEnviar("ATE0", nullptr, 1000); break;
        case 1: atEnviar("AT+CVAUXV=3050", nullptr, 1000); break;
        case 2: atEnviar("AT+CVAUXS=1", nullptr, 1000); break;
        case 3: atEnviar("AT+CGPS?", "+CGPS:", 1000); break;
        case 4: atEnviar("AT+CGPS=1,1", nullptr, 5000); break;
        default:
          gpsErrores     = 0;
          gpsUltConsulta = ahora - GPS_POLL_MS;
          simEstado      = SIM_OPERANDO;
          break;
      }
      break;

    case SIM_OPERANDO:
      if (atRes == AT_ESPERANDO) break;
      if (atRes == AT_OK) {
        simFallos  = 0;
        gpsErrores = 0;
        int32_t la, lo; float al; int64_t ep;
        if (atResp[0] && parsearCGPSINFO(atResp, la, lo, al, ep)) {
          gpsLatE6 = la; gpsLonE6 = lo; gpsAlt = al;
          gpsValido   = true;
          gpsUltFixMs = ahora;
          if (ep) relojAjustar(ep, RELOJ_GPS);
        }
      } else if (atRes == AT_ERROR) {
        simFallos = 0;
        if (++gpsErrores >= 3) { atRes = AT_LIBRE; simPaso = 3; simEstado = SIM_CONFIG; break; }
      } else if (atRes == AT_TIMEOUT) {
        if (++simFallos >= SIM_FALLOS_MAX) { simResincronizar(); break; }
      }
      atRes = AT_LIBRE;
      if (ahora - gpsUltConsulta >= GPS_POLL_MS) {
        gpsUltConsulta = ahora;
        atEnviar("AT+CGPSINFO", "+CGPSINFO:", 1500);
      }
      break;

    case SIM_RESYNC:
      if (ahora - simT0 >= SIM_T_RESYNC_MS) simEstado = SIM_APAGADO;
      break;
  }
}

bool gpsFixVigente() {
  return gpsValido && (millis() - gpsUltFixMs) < (unsigned long)GPS_FIX_VIGENTE_S * 1000UL;
}

const char* simEstadoTxt() {
  switch (simEstado) {
    case SIM_APAGADO:    return "UART cerrado";
    case SIM_ARRANCANDO: return "esperando AT";
    case SIM_CONFIG:     return "configurando";
    case SIM_OPERANDO:   return "operando";
    case SIM_RESYNC:     return "resincronizando";
  }
  return "?";
}

// ============================================
// SDI-12 (UART1 a 1200 baud 7E1)
// ============================================
// Igual que el LemPDA: break + marca + comando y lectura hasta CR/LF. Aqui por UART
// por hardware (no bit-bang): el S2 tiene un solo nucleo y apagar interrupciones
// ~8 ms por caracter afectaria al WiFi.
void sdiIniciar() {
  if (!sdi) sdi = (SdiDatos*)calloc(1, sizeof(SdiDatos));
  sdiEstado = SDI_LIBRE;
  uart1.begin(SDI12_BAUD, SERIAL_7E1, SDI12_RX_PIN, SDI12_TX_PIN);
  uart1.setTxInvert(SDI12_TX_INV);
  uart1.setRxInvert(SDI12_RX_INV);
  uart1.setRxFIFOFull(1);          // cada byte llega de inmediato: separa eco y respuesta
  uart1.setRxTimeout(1);
}

void sdiDetener() {
  uart1.end();
  pinMode(SDI12_TX_PIN, INPUT);
  pinMode(SDI12_RX_PIN, INPUT);
  sdiEstado = SDI_LIBRE;
}

// Lee una respuesta hasta LF. primerMs: espera del primer caracter.
size_t sdiLeerLinea(char* resp, size_t n, uint16_t primerMs) {
  size_t len = 0;
  resp[0] = 0;
  unsigned long t0 = millis(), ult = 0;
  bool empezo = false;
  for (;;) {
    while (uart1.available()) {
      int c = uart1.read() & 0x7F;
      empezo = true;
      ult = millis();
      if (c == '\n') { resp[len] = 0; return len; }
      if (c >= ' ' && c < 127 && len < n - 1) resp[len++] = (char)c;
    }
    unsigned long ahora = millis();
    if (!empezo && ahora - t0 >= primerMs) break;
    if (empezo && ahora - ult >= SDI12_GAP_MS) break;
    delay(1);
  }
  resp[len] = 0;
  return len;
}

// Break (>= 12 ms) + marca (>= 8,33 ms) + comando. El break se genera invirtiendo
// la linea TX, igual que uart_send_break() del core. Luego se descarta el eco.
size_t sdiTransaccion(const char* cmd, char* resp, size_t n, uint16_t primerMs) {
  uart1.setTxInvert(!SDI12_TX_INV);      // linea en espacio = break
  delay(SDI12_BREAK_MS);
  uart1.setTxInvert(SDI12_TX_INV);       // marca
  delay(SDI12_MARCA_MS);
  while (uart1.available()) uart1.read();
  uart1.write((const uint8_t*)cmd, strlen(cmd));
  uart1.flush();                         // hasta el ultimo bit de stop
  delay(2);
  while (uart1.available()) uart1.read();   // eco del propio comando (el sensor espera >= 8,33 ms)
  size_t len = sdiLeerLinea(resp, n, primerMs);

  // Monitor: ultimas transacciones
  uint8_t k = sdi->monCab;
  strncpy(sdi->monTx[k], cmd, sizeof(sdi->monTx[k]) - 1);
  sdi->monTx[k][sizeof(sdi->monTx[k]) - 1] = 0;
  strncpy(sdi->monRx[k], resp, sizeof(sdi->monRx[k]) - 1);
  sdi->monRx[k][sizeof(sdi->monRx[k]) - 1] = 0;
  sdi->monCab = (uint8_t)((k + 1) % SDI12_MONITOR);
  if (sdi->monN < SDI12_MONITOR) sdi->monN++;
  return len;
}

bool sdiProbar(char a) {
  char cmd[3] = {a, '!', 0};
  char r[16];
  return sdiTransaccion(cmd, r, sizeof(r), SDI12_PROBE_MS) > 0 && r[0] == a;
}

bool sdiEnLista(char a) {
  for (uint8_t i = 0; i < sdi->n; i++) if (sdi->sens[i].a == a) return true;
  return false;
}

// Cambio de ID con verificacion (misma logica que el LemPDA)
void sdiCambiarId() {
  char de = sdi->chgDe, a = sdi->chgA;
  sdi->chgOk = false;
  if (!sdiProbar(de)) {
    snprintf(sdi->chgMsg, sizeof(sdi->chgMsg), "sin sensor en %c", de);
    return;
  }
  if (sdiProbar(a)) {
    snprintf(sdi->chgMsg, sizeof(sdi->chgMsg), "%c ya esta en uso", a);
    return;
  }
  char cmd[5] = {de, 'A', a, '!', 0};
  char r[16];
  sdiTransaccion(cmd, r, sizeof(r), SDI12_RESP_MS);
  delay(100);
  bool nuevoOk   = sdiProbar(a);
  bool viejoFuera = !sdiProbar(de);
  if (nuevoOk) {
    sdi->chgOk = true;
    if (viejoFuera) snprintf(sdi->chgMsg, sizeof(sdi->chgMsg), "%c -> %c", de, a);
    else            snprintf(sdi->chgMsg, sizeof(sdi->chgMsg), "responde en %c (y aun en %c)", a, de);
    for (uint8_t i = 0; i < sdi->n; i++) if (sdi->sens[i].a == de) sdi->sens[i].a = a;
  } else {
    snprintf(sdi->chgMsg, sizeof(sdi->chgMsg), "no se pudo escribir (resp: %s)", r[0] ? r : "ninguna");
  }
}

// Maquina de estados: a lo mas una transaccion por llamada (la web sigue atendiendo)
void sdiTarea() {
  if (!sdi) return;
  char r[84];
  switch (sdiEstado) {
    case SDI_ESCANEANDO:
      if (sdi->scanIdx < SDI12_NUM_DIR) {
        char a = sdiDirEn(sdi->scanIdx);
        if (sdiProbar(a)) { sdi->sens[sdi->n].a = a; sdi->sens[sdi->n].id[0] = 0; sdi->n++; }
        sdi->scanIdx++;
      } else if (sdi->idIdx < sdi->n) {
        SdiSensor& s = sdi->sens[sdi->idIdx++];
        char cmd[4] = {s.a, 'I', '!', 0};
        if (sdiTransaccion(cmd, r, sizeof(r), SDI12_RESP_MS) > 1 && r[0] == s.a) {
          strncpy(s.id, r + 1, sizeof(s.id) - 1);
          s.id[sizeof(s.id) - 1] = 0;
        }
      } else {
        sdi->scanHecho = true;
        sdiEstado = SDI_LIBRE;
      }
      break;

    case SDI_MED_ID: {
      char cmd[4] = {sdi->medA, 'I', '!', 0};
      if (sdiTransaccion(cmd, r, sizeof(r), SDI12_RESP_MS) > 1 && r[0] == sdi->medA) {
        strncpy(sdi->medId, r + 1, sizeof(sdi->medId) - 1);
        for (uint8_t i = 0; i < sdi->n; i++) if (sdi->sens[i].a == sdi->medA) strcpy(sdi->sens[i].id, sdi->medId);
      }
      sdiEstado = SDI_MED_CMD;
      break;
    }

    case SDI_MED_CMD: {
      uint16_t ttt = 0, n = 0;
      for (int intento = 0; intento < 2; intento++) {
        uint8_t k = 0;
        sdi->medCmd[k++] = sdi->medA;
        sdi->medCmd[k++] = sdi->medTipo;
        if (sdi->medIdx) sdi->medCmd[k++] = (char)('0' + sdi->medIdx);
        sdi->medCmd[k++] = '!';
        sdi->medCmd[k]   = 0;
        sdiTransaccion(sdi->medCmd, sdi->medResp, sizeof(sdi->medResp), SDI12_RESP_MS);
        if (sdiParsearMedicion(sdi->medResp, sdi->medA, sdi->medTipo, ttt, n)) break;
        if (sdi->medTipo != 'C') break;
        sdi->medTipo = 'M';                    // sin aC!: reintentar con aM! (como el LemPDA)
      }
      if (!sdiParsearMedicion(sdi->medResp, sdi->medA, sdi->medTipo, ttt, n)) {
        snprintf(sdi->medErr, sizeof(sdi->medErr), "sin respuesta valida a %s", sdi->medCmd);
        sdi->medHecho = true;
        sdiEstado = SDI_LIBRE;
        break;
      }
      sdi->ttt   = ttt;
      sdi->nEsp  = n;
      sdi->srLen = 0;
      uint16_t espera = ttt > SDI12_ESPERA_MAX_S ? SDI12_ESPERA_MAX_S : ttt;
      sdi->listoUs = esp_timer_get_time() + (int64_t)espera * 1000000LL;
      sdi->dIdx  = 0;
      sdiEstado  = (ttt == 0) ? SDI_MED_DATOS : SDI_MED_ESPERA;
      break;
    }

    case SDI_MED_ESPERA:
      // aM! avisa con un "service request" (la direccion sola) cuando termina antes
      while (uart1.available()) {
        int c = uart1.read() & 0x7F;
        if (c == '\n') {
          sdi->srBuf[sdi->srLen] = 0;
          if (sdi->medTipo == 'M' && sdi->srLen == 1 && sdi->srBuf[0] == sdi->medA) sdi->listoUs = 0;
          sdi->srLen = 0;
        } else if (c >= ' ' && sdi->srLen < sizeof(sdi->srBuf) - 1) {
          sdi->srBuf[sdi->srLen++] = (char)c;
        }
      }
      if (esp_timer_get_time() >= sdi->listoUs) sdiEstado = SDI_MED_DATOS;
      break;

    case SDI_MED_DATOS: {
      char cmd[5] = {sdi->medA, 'D', (char)('0' + sdi->dIdx), '!', 0};
      size_t len = sdiTransaccion(cmd, r, sizeof(r), SDI12_DATOS_MS);
      uint8_t agregados = 0;
      if (len > 1 && r[0] == sdi->medA) {
        strncpy(sdi->dResp[sdi->nD], r, sizeof(sdi->dResp[0]) - 1);
        sdi->dResp[sdi->nD][sizeof(sdi->dResp[0]) - 1] = 0;
        sdi->nD++;
        agregados = sdiParsearValores(r, sdi->vals, sdi->nVals, SDI12_MAX_VALORES);
        sdi->nVals += agregados;
      }
      sdi->dIdx++;
      bool fin = (agregados == 0) || sdi->dIdx > 9 || sdi->nVals >= SDI12_MAX_VALORES ||
                 (sdi->nEsp > 0 && sdi->nVals >= sdi->nEsp);
      if (fin) {
        if (sdi->nVals == 0) snprintf(sdi->medErr, sizeof(sdi->medErr), "el sensor no entrego datos");
        sdi->medHecho = true;
        sdiEstado = SDI_LIBRE;
      }
      break;
    }

    case SDI_CAMBIO:
      sdiCambiarId();
      sdi->chgHecho = true;
      sdiEstado = SDI_LIBRE;
      break;

    case SDI_MANUAL:
      sdiTransaccion(sdi->manTx, sdi->manRx, sizeof(sdi->manRx), SDI12_DATOS_MS);
      sdi->manHecho = true;
      sdiEstado = SDI_LIBRE;
      break;

    default:
      break;
  }
}

const char* sdiEstadoTxt() {
  switch (sdiEstado) {
    case SDI_ESCANEANDO: return "escaneando";
    case SDI_MED_ID:
    case SDI_MED_CMD:    return "midiendo";
    case SDI_MED_ESPERA: return "esperando";
    case SDI_MED_DATOS:  return "leyendo";
    case SDI_CAMBIO:     return "cambiando";
    case SDI_MANUAL:     return "comando";
    default:             return "libre";
  }
}

// ============================================
// WIFI + PORTAL CAUTIVO
// ============================================
void iniciarWiFi() {
  uint64_t mac = ESP.getEfuseMac();   // byte 0 de la MAC en los 8 bits bajos
  snprintf(apSsid, sizeof(apSsid), "%s%02X%02X", AP_PREFIJO,
           (unsigned)((mac >> 32) & 0xFF), (unsigned)((mac >> 40) & 0xFF));

  WiFi.persistent(false);
  WiFi.mode(WIFI_AP);
  WiFi.softAP(apSsid, nullptr, AP_CANAL, 0, AP_MAX_CLIENTES);   // red abierta
  delay(100);

  dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
  dnsServer.start(53, "*", WiFi.softAPIP());

  server.on("/",           HTTP_GET,  webRaiz);
  server.on("/perfil",     HTTP_POST, webPerfil);
  server.on("/api/sis",    HTTP_GET,  webApiSis);
  server.on("/api/w",      HTTP_GET,  webApiWeather);
  server.on("/log.csv",    HTTP_GET,  webCSV);
  server.on("/borrar",     HTTP_POST, webBorrar);
  server.on("/hora",       HTTP_POST, webHora);
  server.on("/api/s",      HTTP_GET,  webApiSense);
  server.on("/s/rtc",      HTTP_POST, webSenseRtc);
  server.on("/s/reset",    HTTP_POST, webSenseReset);
  server.on("/api/sdi",    HTTP_GET,  webApiSdi);
  server.on("/sdi/scan",   HTTP_POST, webSdiScan);
  server.on("/sdi/medir",  HTTP_POST, webSdiMedir);
  server.on("/sdi/id",     HTTP_POST, webSdiId);
  server.on("/sdi/cmd",    HTTP_POST, webSdiCmd);
  server.onNotFound(webPortal);
  server.begin();
}

void webRaiz() {
  server.sendHeader("Cache-Control", "no-store");
  server.send_P(200, "text/html; charset=utf-8", PAGINA);
}

// URL desconocida (generate_204, hotspot-detect.html...) -> portal
void webPortal() {
  server.sendHeader("Location", String("http://") + WiFi.softAPIP().toString() + "/", true);
  server.send(302, "text/plain", "");
}

// --- JSON con buffer en heap ---
void jReset() {
  jsonLen = 0;
  if (jsonBuf) jsonBuf[0] = 0;
}

void jAdd(const char* fmt, ...) {
  if (!jsonBuf || jsonLen >= jsonCap - 1) return;
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(jsonBuf + jsonLen, jsonCap - jsonLen, fmt, ap);
  va_end(ap);
  if (n > 0) jsonLen += min((size_t)n, jsonCap - 1 - jsonLen);
}

void jNum(const char* k, float v, int dec, bool coma) {
  if (isnan(v)) jAdd("\"%s\":null%s", k, coma ? "," : "");
  else jAdd("\"%s\":%.*f%s", k, dec, v, coma ? "," : "");
}

// Cadena con escape JSON (las respuestas SDI-12 pueden traer " o \)
void jStr(const char* k, const char* s, bool coma) {
  if (k) jAdd("\"%s\":", k);
  jAdd("\"");
  for (; *s; s++) {
    unsigned char c = (unsigned char)*s;
    if (c == '"' || c == '\\') jAdd("\\%c", c);
    else if (c < 0x20 || c > 0x7E) jAdd("\\u%04x", c);
    else jAdd("%c", c);
  }
  jAdd("\"%s", coma ? "," : "");
}

void jSis() {
  jAdd("\"sis\":{\"fw\":\"%s\",\"up\":%lu,\"heap\":%lu,\"heapMin\":%lu,\"rst\":\"%s\"}",
       FW_VERSION, (unsigned long)uptimeS(), (unsigned long)ESP.getFreeHeap(),
       (unsigned long)ESP.getMinFreeHeap(), resetTxt(esp_reset_reason()));
}

void jEnviar() {
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", jsonBuf ? jsonBuf : "{}");
}

void responder(int code, bool ok, const char* msg) {
  jReset();
  jAdd("{\"ok\":%d,", ok ? 1 : 0);
  jStr("msg", msg, false);
  jAdd("}");
  server.send(code, "application/json", jsonBuf ? jsonBuf : "{}");
}

bool exigirPerfil(Perfil p) {
  if (perfil == p) return true;
  char m[64];
  snprintf(m, sizeof(m), "el perfil activo es %s", perfilTxt(perfil));
  responder(409, false, m);
  return false;
}

void webPerfil() {
  long p = strtol(server.arg("p").c_str(), nullptr, 10);
  if (p < PERFIL_NINGUNO || p > PERFIL_SDI12) { responder(400, false, "perfil invalido"); return; }
  perfilCambiar((Perfil)p);
  responder(200, true, perfilTxt(perfil));
}

void webApiSis() {
  jReset();
  jAdd("{\"id\":\"%s\",\"perfil\":%u,", apSsid, (unsigned)perfil);
  jSis();
  jAdd("}");
  jEnviar();
}

void webApiWeather() {
  uint32_t tot, perUs; int64_t ultUs;
  pulsoLeer(VIENTO_CANAL, tot, perUs, ultUs);
  if (perfil != PERFIL_WEATHER) { tot = 0; perUs = 0; ultUs = 0; }
  int64_t ahoraUs = esp_timer_get_time();
  bool    gpsFix  = gpsFixVigente();

  jReset();
  jAdd("{\"id\":\"%s\",\"perfil\":%u,\"up\":%lu,\"reloj\":%u,\"epoch\":%lu,\"sim\":\"%s\",\"simRst\":%lu,",
       apSsid, (unsigned)perfil, (unsigned long)uptimeS(), (unsigned)relojFuente,
       (unsigned long)epochAhora(), simEstadoTxt(), (unsigned long)simResyncs);

  jAdd("\"gps\":{\"val\":%d,\"fix\":%d,", gpsValido ? 1 : 0, gpsFix ? 1 : 0);
  if (gpsValido) {
    char la[16], lo[16];
    fmtFijo(la, sizeof(la), gpsLatE6, 6);
    fmtFijo(lo, sizeof(lo), gpsLonE6, 6);
    jAdd("\"lat\":%s,\"lon\":%s,\"edad\":%lu,", la, lo, (unsigned long)((millis() - gpsUltFixMs) / 1000UL));
  }
  jNum("alt", gpsAlt, 1, false);
  jAdd("},");

  bool bOk = bmeOk && !isnan(bmeT);
  jAdd("\"bme\":{\"ok\":%d,\"addr\":%u,", bOk ? 1 : 0, (unsigned)bmeAddr);
  jNum("t", bmeT, 2, true); jNum("h", bmeH, 2, true); jNum("p", bmeP, 2, false);
  jAdd("},");

  float dirViva = atan2f(dirEmaY, dirEmaX) * RAD_TO_DEG;
  if (dirViva < 0.0f) dirViva += 360.0f;
  bool dOk = veletaOk && dirEmaIni;
  jAdd("\"dir\":{\"ok\":%d,\"mv\":%lu,\"card\":\"%s\",", dOk ? 1 : 0, (unsigned long)veletaMv, dOk ? cardinal(dirViva) : "");
  jNum("deg", dOk ? dirViva : NAN, 1, true);
  if (hayUltimoReg && (ultimoReg.flags & FLG_DIR_OK))
    jAdd("\"med\":%.1f,\"medc\":\"%s\"},", ultimoReg.dir_d / 10.0f, cardinal(ultimoReg.dir_d / 10.0f));
  else
    jAdd("\"med\":null},");

  jAdd("\"pul\":{\"tot\":%lu,", (unsigned long)(pulsosBase + tot));
  if (perUs != 0) jAdd("\"per\":%.3f,", perUs / 1000.0);
  else            jAdd("\"per\":null,");
  if (ultUs != 0) jAdd("\"edad\":%.0f,", (ahoraUs - ultUs) / 1000.0);
  else            jAdd("\"edad\":null,");
  if (hayUltimoReg) {
    jAdd("\"min\":%u,", (unsigned)ultimoReg.pulsosVen);
    if (ultimoReg.periodoUs) jAdd("\"med\":%.3f},", ultimoReg.periodoUs / 1000.0);
    else jAdd("\"med\":null},");
  } else {
    jAdd("\"min\":null,\"med\":null},");
  }

  jAdd("\"log\":{\"n\":%lu,\"cap\":%lu,\"rb\":%u,\"per\":%u,\"rec\":%lu},",
       (unsigned long)logHdr.count, (unsigned long)logCap, (unsigned)sizeof(LogRec), (unsigned)LOG_PERIODO_S,
       (unsigned long)logRecuperados);
  jSis();
  jAdd("}");
  jEnviar();
}

// CSV generado al vuelo en bloques
void webCSV() {
  char nombre[56];
  uint32_t ep = epochAhora();
  if (ep) {
    int y; unsigned mo, d, h, mi, s;
    civilDesdeEpoch(ep, y, mo, d, h, mi, s);
    snprintf(nombre, sizeof(nombre), "%s_%04d%02u%02u_%02u%02u.csv", apSsid, y, mo, d, h, mi);
  } else {
    snprintf(nombre, sizeof(nombre), "%s_registro.csv", apSsid);
  }

  server.sendHeader("Content-Disposition", String("attachment; filename=\"") + nombre + "\"");
  server.sendHeader("Cache-Control", "no-store");
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/csv; charset=utf-8", "");

  static char bloque[1460];
  size_t bl = snprintf(bloque, sizeof(bloque),
    "n,fecha_utc,seg_desde_arranque,arranque,lat,lon,gps_fix,temp_C,hum_pct,pres_hPa,"
    "dir_deg,dir_card,pulsos_min,pulsos_total,periodo_ms\r\n");

  uint32_t count = (logBuf && logCap) ? logHdr.count : 0;
  uint32_t escritos = logHdr.escritos;
  uint32_t ini   = count ? (logHdr.head + logCap - count) % logCap : 0;
  char linea[192];
  for (uint32_t i = 0; i < count; i++) {
    int n = formatearLineaCSV(linea, sizeof(linea), logBuf[(ini + i) % logCap], escritos - count + i + 1);
    if (n <= 0) continue;
    if (bl + (size_t)n > sizeof(bloque)) {
      server.sendContent(bloque, bl);
      bl = 0;
      if (!server.client().connected()) return;
    }
    memcpy(bloque + bl, linea, n);
    bl += n;
  }
  if (bl) server.sendContent(bloque, bl);
  server.sendContent("");
}

void webBorrar() {
  logBorrar();
  responder(200, true, "registro borrado");
}

void webHora() {
  uint32_t e = (uint32_t)strtoul(server.arg("e").c_str(), nullptr, 10);
  relojAjustar((int64_t)e * 1000000LL, RELOJ_NAVEGADOR);
  responder(200, relojFuente != RELOJ_NO, relojFuente ? "hora tomada" : "hora rechazada");
}

void webApiSense() {
  jReset();
  jAdd("{\"id\":\"%s\",\"perfil\":%u,\"umbral\":%u,", apSsid, (unsigned)perfil, (unsigned)UMBRAL_HIGH_MV);

  bool sOk = shtOk && !isnan(shtT);
  jAdd("\"sht\":{\"ok\":%d,\"addr\":%u,", shtOk ? 1 : 0, (unsigned)shtAddr);
  jNum("t", sOk ? shtT : NAN, 2, true); jNum("h", sOk ? shtH : NAN, 2, false);
  jAdd("},");
  bool bOk = bmeOk && !isnan(bmeT);
  jAdd("\"bme\":{\"ok\":%d,\"addr\":%u,", bmeOk ? 1 : 0, (unsigned)bmeAddr);
  jNum("t", bOk ? bmeT : NAN, 2, true); jNum("h", bOk ? bmeH : NAN, 2, true); jNum("p", bOk ? bmeP : NAN, 2, false);
  jAdd("},");

  jAdd("\"an\":[");
  for (uint8_t i = 0; i < N_ANALOG; i++) {
    const LecturaAdc& a = anLect[i];
    jAdd("%s{\"n\":\"%s\",\"g\":%u,\"ok\":%d,\"mv\":%d,\"cnt\":%d,\"raw\":%d,\"sat\":%d,\"cal\":%d}",
         i ? "," : "", ANALOG_NOMBRE[i], (unsigned)ANALOG_GPIO[i], a.ok ? 1 : 0, a.mv,
         adcCuenta11(a.raw), a.raw, a.sat ? 1 : 0, a.cal ? 1 : 0);
  }
  jAdd("],\"pu\":[");
  int64_t ahoraUs = esp_timer_get_time();
  for (uint8_t i = 0; i < N_PULSOS; i++) {
    const LecturaAdc& a = puLect[i];
    uint32_t tot, perUs; int64_t ultUs;
    pulsoLeer(i, tot, perUs, ultUs);
    // HIGH: sobre el umbral, o ADC saturado (el voltaje real es mayor que el maximo medible)
    bool hi = a.ok && (a.mv >= UMBRAL_HIGH_MV || a.sat);
    jAdd("%s{\"n\":\"%s\",\"g\":%u,\"ok\":%d,\"mv\":%d,\"cnt\":%d,\"raw\":%d,\"sat\":%d,\"hi\":%d,\"dig\":%u,\"tot\":%lu,",
         i ? "," : "", PULSO_NOMBRE[i], (unsigned)PULSO_GPIO[i], a.ok ? 1 : 0, a.mv, adcCuenta11(a.raw),
         a.raw, a.sat ? 1 : 0, hi ? 1 : 0, (unsigned)puDigital[i], (unsigned long)tot);
    if (perUs) jAdd("\"per\":%.3f,", perUs / 1000.0); else jAdd("\"per\":null,");
    if (ultUs) jAdd("\"edad\":%.0f}", (ahoraUs - ultUs) / 1000.0); else jAdd("\"edad\":null}");
  }
  jAdd("],");

  jAdd("\"rtc\":{\"ok\":%d,\"val\":%d,\"osf\":%d,\"ch\":%d,\"f\":\"", rtcOk ? 1 : 0, rtcValido ? 1 : 0,
       rtcOsf ? 1 : 0, rtcCh ? 1 : 0);
  if (rtcOk && rtcValido)
    jAdd("%04d-%02u-%02u %02u:%02u:%02u", rtcAnio, rtcMes, rtcDia, rtcHora, rtcMin, rtcSeg);
  jAdd("\"},");
  jSis();
  jAdd("}");
  jEnviar();
}

// f = AAAAMMDDhhmmss (hora del telefono o la que eligio el usuario)
void webSenseRtc() {
  if (!exigirPerfil(PERFIL_SENSE)) return;
  String f = server.arg("f");
  if (f.length() != 14) { responder(400, false, "formato AAAAMMDDhhmmss"); return; }
  for (uint8_t i = 0; i < 14; i++) if (f[i] < '0' || f[i] > '9') { responder(400, false, "solo digitos"); return; }
  int      y  = f.substring(0, 4).toInt();
  unsigned mo = f.substring(4, 6).toInt(),  d  = f.substring(6, 8).toInt();
  unsigned h  = f.substring(8, 10).toInt(), mi = f.substring(10, 12).toInt(), s = f.substring(12, 14).toInt();
  if (!fechaValida(y, mo, d, h, mi, s)) { responder(400, false, "fecha invalida (anio 2000-2099)"); return; }
  if (!rtcEscribir(y, mo, d, h, mi, s)) { responder(200, false, "el DS1338 no respondio"); return; }
  delay(5);
  rtcLeer();
  // Verificacion: lo leido debe coincidir con lo escrito (+-2 s)
  int64_t esc = (int64_t)diasDesdeCivil(y, mo, d) * 86400LL + h * 3600L + mi * 60L + s;
  int64_t lei = rtcValido ? (int64_t)diasDesdeCivil(rtcAnio, rtcMes, rtcDia) * 86400LL +
                            rtcHora * 3600L + rtcMin * 60L + rtcSeg : -1000;
  bool ok = rtcValido && !rtcCh && llabs(lei - esc) <= 2;
  responder(200, ok, ok ? "hora escrita y verificada" : "escrita, pero la lectura no coincide");
}

void webSenseReset() {
  if (!exigirPerfil(PERFIL_SENSE)) return;
  for (uint8_t i = 0; i < N_PULSOS; i++) {
    portENTER_CRITICAL(&muxPulsos);
    pul[i].tot = 0; pul[i].perUs = 0; pul[i].ultUs = 0;
    portEXIT_CRITICAL(&muxPulsos);
  }
  responder(200, true, "contadores en cero");
}

void webApiSdi() {
  jReset();
  jAdd("{\"id\":\"%s\",\"perfil\":%u,\"est\":\"%s\",\"pins\":[%u,%u],", apSsid, (unsigned)perfil,
       sdiEstadoTxt(), (unsigned)SDI12_TX_PIN, (unsigned)SDI12_RX_PIN);
  if (!sdi) {
    jAdd("\"scan\":{\"i\":0,\"n\":%u,\"hecho\":0},\"sens\":[],\"med\":{},\"chg\":{},\"man\":{},\"mon\":[],",
         (unsigned)SDI12_NUM_DIR);
  } else {
    jAdd("\"scan\":{\"i\":%u,\"n\":%u,\"hecho\":%d},\"sens\":[", (unsigned)sdi->scanIdx,
         (unsigned)SDI12_NUM_DIR, sdi->scanHecho ? 1 : 0);
    for (uint8_t i = 0; i < sdi->n; i++) {
      char a[2] = {sdi->sens[i].a, 0};
      jAdd("%s{", i ? "," : "");
      jStr("a", a, true);
      jStr("id", sdi->sens[i].id, false);
      jAdd("}");
    }
    jAdd("],\"med\":{");
    if (sdi->medA) {
      char a[2] = {sdi->medA, 0};
      jStr("a", a, true);
      jStr("cmd", sdi->medCmd, true);
      jStr("resp", sdi->medResp, true);
      jStr("err", sdi->medErr, true);
      float resta = 0.0f;
      if (sdiEstado == SDI_MED_ESPERA) {
        int64_t q = sdi->listoUs - esp_timer_get_time();
        resta = q > 0 ? q / 1e6f : 0.0f;
      }
      jAdd("\"ttt\":%u,\"n\":%u,\"resta\":%.1f,\"hecho\":%d,\"vals\":[", (unsigned)sdi->ttt,
           (unsigned)sdi->nEsp, resta, sdi->medHecho ? 1 : 0);
      for (uint8_t i = 0; i < sdi->nVals; i++) { if (i) jAdd(","); jStr(nullptr, sdi->vals[i], false); }
      jAdd("],\"d\":[");
      for (uint8_t i = 0; i < sdi->nD; i++) { if (i) jAdd(","); jStr(nullptr, sdi->dResp[i], false); }
      jAdd("]");
    }
    jAdd("},\"chg\":{\"hecho\":%d,\"ok\":%d,", sdi->chgHecho ? 1 : 0, sdi->chgOk ? 1 : 0);
    jStr("msg", sdi->chgMsg, false);
    jAdd("},\"man\":{\"hecho\":%d,", sdi->manHecho ? 1 : 0);
    jStr("tx", sdi->manTx, true);
    jStr("rx", sdi->manRx, false);
    jAdd("},\"mon\":[");
    for (uint8_t i = 0; i < sdi->monN; i++) {
      uint8_t k = (uint8_t)((sdi->monCab + SDI12_MONITOR - sdi->monN + i) % SDI12_MONITOR);
      jAdd("%s[", i ? "," : "");
      jStr(nullptr, sdi->monTx[k], true);
      jStr(nullptr, sdi->monRx[k], false);
      jAdd("]");
    }
    jAdd("],");
  }
  jSis();
  jAdd("}");
  jEnviar();
}

bool sdiLibreOResponder() {
  if (!exigirPerfil(PERFIL_SDI12)) return false;
  if (!sdi) { responder(500, false, "sin memoria para SDI-12"); return false; }
  if (sdiEstado != SDI_LIBRE) { responder(409, false, "el bus esta ocupado"); return false; }
  return true;
}

void webSdiScan() {
  if (!sdiLibreOResponder()) return;
  sdi->n = 0;
  sdi->scanIdx = 0;
  sdi->idIdx = 0;
  sdi->scanHecho = false;
  sdiEstado = SDI_ESCANEANDO;
  responder(200, true, "buscando");
}

void webSdiMedir() {
  if (!sdiLibreOResponder()) return;
  String a = server.arg("a"), m = server.arg("m"), i = server.arg("i");
  if (a.length() != 1 || sdiIndice(a[0]) < 0) { responder(400, false, "direccion invalida"); return; }
  if (m != "C" && m != "M") { responder(400, false, "modo C o M"); return; }
  long idx = i.length() ? strtol(i.c_str(), nullptr, 10) : 0;
  if (idx < 0 || idx > 9) { responder(400, false, "indice 0-9"); return; }
  sdi->medA = a[0];
  sdi->medTipo = m[0];
  sdi->medIdx = (uint8_t)idx;
  sdi->medCmd[0] = sdi->medResp[0] = sdi->medId[0] = sdi->medErr[0] = 0;
  sdi->ttt = sdi->nEsp = 0;
  sdi->nVals = sdi->nD = 0;
  sdi->medHecho = false;
  sdiEstado = SDI_MED_ID;
  responder(200, true, "midiendo");
}

void webSdiId() {
  if (!sdiLibreOResponder()) return;
  String de = server.arg("de"), a = server.arg("a");
  if (de.length() != 1 || a.length() != 1 || sdiIndice(de[0]) < 0 || sdiIndice(a[0]) < 0) {
    responder(400, false, "direccion invalida");
    return;
  }
  if (de[0] == a[0]) { responder(400, false, "el ID nuevo es igual al actual"); return; }
  if (sdiEnLista(a[0])) { responder(400, false, "ese ID ya lo usa otro sensor"); return; }
  sdi->chgDe = de[0];
  sdi->chgA = a[0];
  sdi->chgHecho = false;
  sdi->chgMsg[0] = 0;
  sdiEstado = SDI_CAMBIO;
  responder(200, true, "cambiando");
}

// Comando manual: imprimible, termina en '!', hasta 20 caracteres
void webSdiCmd() {
  if (!sdiLibreOResponder()) return;
  String c = server.arg("c");
  c.trim();
  if (c.length() < 2 || c.length() > 20 || c[c.length() - 1] != '!') {
    responder(400, false, "el comando debe terminar en ! (max 20)");
    return;
  }
  for (size_t i = 0; i < c.length(); i++) {
    if (c[i] < ' ' || c[i] > '~') { responder(400, false, "caracter invalido"); return; }
  }
  strncpy(sdi->manTx, c.c_str(), sizeof(sdi->manTx) - 1);
  sdi->manTx[sizeof(sdi->manTx) - 1] = 0;
  sdi->manRx[0] = 0;
  sdi->manHecho = false;
  sdiEstado = SDI_MANUAL;
  responder(200, true, "enviando");
}

const char* resetTxt(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON:  return "encendido";
    case ESP_RST_EXT:      return "pin EN";
    case ESP_RST_SW:       return "software";
    case ESP_RST_PANIC:    return "panic";
    case ESP_RST_INT_WDT:  return "WDT int";
    case ESP_RST_TASK_WDT: return "WDT tarea";
    case ESP_RST_WDT:      return "WDT";
    case ESP_RST_BROWNOUT: return "brownout";
    case ESP_RST_DEEPSLEEP:return "deep sleep";
    default:               return "otro";
  }
}
