/*
 * LEM Nodo Meteo - ESP32-S2
 * Version: 0.1.0.0
 *
 * - SIM7600X: posicion GNSS y hora UTC (AT+CGPSINFO), driver AT no bloqueante
 * - BME280: temperatura, humedad, presion (I2C)
 * - Veleta analogica 0..1,12 V  (0 V = 1,12 V = Norte, 0,56 V = Sur)
 * - Entrada de pulsos a GND (reed) con antirrebote por software
 * - Portal cautivo WiFi con datos en vivo
 * - Registro cada 1 min en RAM (buffer circular binario de 32 B/registro)
 *   descargable como CSV (separador ',' y decimal '.'). No escribe la flash.
 *
 * Core: esp32 by Espressif 3.x  (compilado con 3.3.12, placa "ESP32S2 Dev Module")
 * Librerias: Adafruit BME280 Library (+ Adafruit Unified Sensor, Adafruit BusIO)
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <math.h>
#include <stdarg.h>
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "driver/gpio.h"

// ============================================
// PIN DEFINITIONS
// ============================================
#define SIM_RX_PIN   3    // RX del ESP  <- TX del SIM7600
#define SIM_TX_PIN   4    // TX del ESP  -> RX del SIM7600
#define SIM_EN_PIN   14   // Habilita reguladores del SIM7600 (HIGH = ON)
#define SENS_EN_PIN  15   // Habilita regulador de sensores   (HIGH = ON)
#define SDA_PIN      8
#define SCL_PIN      9
#define VELETA_PIN   2    // ADC1_CH1, 0 .. 1,12 V
#define PULSOS_PIN   10   // Contacto a GND, cuenta flanco de bajada

// ============================================
// CONFIGURACION
// ============================================
#define FW_VERSION        "0.1.0.0"

// --- WiFi / portal cautivo ---
#define AP_PREFIJO        "LEM-WX-"   // SSID = prefijo + 4 hex de la MAC
#define AP_PASS           ""          // "" = red abierta. Si se usa: minimo 8 caracteres
#define AP_CANAL          6
#define AP_MAX_CLIENTES   4

// --- SIM7600 ---
#define SIM_BAUD          115200
#define SIM_T_BOOT_MAX_MS 40000   // espera maxima de respuesta a "AT" tras energizar
#define SIM_T_OFF_MS      5000    // reguladores apagados durante un reinicio del modulo
#define SIM_FALLOS_MAX    5       // timeouts AT seguidos antes de reiniciar el modulo
#define GPS_POLL_MS       5000    // periodo de AT+CGPSINFO
#define GPS_FIX_VIGENTE_S 120     // fix mas antiguo que esto = "ultima posicion conocida"
#define GNSS_ANT_ACTIVA   0       // 1 = alimentar antena activa por VDD_AUX (AT+CVAUXV/CVAUXS)

// --- Veleta ---
#define VELETA_VMAX_MV    1120    // tension a 360 grados (= Norte). Calibrar con "mV" del portal
#define VELETA_MV_FALLA   (VELETA_VMAX_MV + VELETA_VMAX_MV / 10)  // >110 % = cableado en falla
#define VELETA_OFFSET_DEG 0.0f    // correccion de montaje (se suma)
#define VELETA_INVERTIDA  0       // 1 si la tension crece en sentido antihorario
#define VELETA_MUESTRAS   32      // lecturas ADC promediadas por muestra
#define VELETA_PERIODO_MS 1000    // muestreo de direccion
#define VELETA_EMA        0.3f    // suavizado vectorial de la vista en vivo

// --- Pulsos ---
#define PULSO_ALTO_MIN_US 2000    // la linea debe estar en ALTO >= esto antes de aceptar un flanco
#define PULSO_PER_MIN_US  4000    // periodo minimo aceptado (antirrebote) -> max 250 Hz

// --- BME280 ---
#define BME_PERIODO_MS    2000
#define BME_REINTENTO_MS  30000

// --- Registro en RAM ---
// LOG_MODO 0 (persistente): buffer estatico en .noinit, sobrevive reset por software,
//   watchdog, panic y brownout. Limite fisico: la RAM estatica del S2 termina en
//   0x3FFDE000 (arriba esta el bootloader) y la comparte con el codigo IRAM.
//   Medido con core 3.3.12 (espacio estatico libre para el registro):
//     USB CDC On Boot = Disabled : 52248 B -> max 1632 reg. Se usan 1440 (24 h)
//     USB CDC On Boot = Enabled  : 19536 B -> max  609 reg. Se usan  480 ( 8 h)
//   (TinyUSB ocupa ~32 KB de RAM estatica). Si se pide mas de lo que cabe, el
//   enlazador falla con "region dram0_0_seg overflowed": nunca compila algo que no quepa.
// LOG_MODO 1 (PSRAM): modulos S2FN4R2 / WROVER, activar "PSRAM: Enabled".
//   1,75 MB = 57344 registros = 39,8 dias. Se pierde con cualquier reset.
//   Si no se detecta PSRAM usa el heap interno dejando LOG_RESERVA_HEAP libre.
#define LOG_PERIODO_S     60
#define LOG_MODO          0
#if ARDUINO_USB_CDC_ON_BOOT || ARDUINO_USB_MSC_ON_BOOT || ARDUINO_USB_DFU_ON_BOOT
#define LOG_CAP_INTERNA   480                 // modo 0 con TinyUSB: 480 x 32 B = 15 KB = 8 h
#else
#define LOG_CAP_INTERNA   1440                // modo 0: 1440 x 32 B = 45 KB = 24 h
#endif
#define LOG_PSRAM_BYTES   (1792UL * 1024UL)   // modo 1: 1,75 MB -> 57344 registros
#define LOG_RESERVA_HEAP  65536               // modo 1 sin PSRAM: heap que se deja a WiFi/web
#define LOG_MAGIC         0x4C454D31UL        // "LEM1"
#define LOG_VERSION       1

#define EPOCH_MIN_VALIDO  1704067200UL        // 2024-01-01: fechas anteriores se descartan

// ============================================
// REGISTRO: FORMATO BINARIO (32 bytes)
// ============================================
// Valores en punto fijo: el CSV se genera al descargar, asi en RAM
// cabe ~3,5x mas que guardando texto (~110 B por linea CSV).
#define FLG_TS_EPOCH   0x01   // ts = fecha UTC (epoch); si no, segundos desde el arranque
#define FLG_GPS_VALIDO 0x02   // lat/lon tienen una posicion conocida
#define FLG_GPS_FIX    0x04   // fix vigente al momento del registro
#define FLG_BME_OK     0x08   // temp/hum/pres validos
#define FLG_DIR_OK     0x10   // direccion valida

typedef struct __attribute__((packed)) {
  uint32_t ts;          // epoch UTC [s] o segundos desde arranque (ver FLG_TS_EPOCH)
  int32_t  lat_e6;      // latitud  [1e-6 grados]
  int32_t  lon_e6;      // longitud [1e-6 grados]
  uint32_t pulsosTot;   // contador total de pulsos
  uint32_t periodoUs;   // periodo medio de pulsos en la ventana [us], 0 = sin dato
  int16_t  temp_c;      // temperatura [0,01 C]   (promedio del minuto)
  uint16_t hum_c;       // humedad     [0,01 %HR] (promedio del minuto)
  uint16_t pres_2pa;    // presion     [2 Pa]     (promedio del minuto)
  uint16_t dir_d;       // direccion   [0,1 grados] (media vectorial del minuto)
  uint16_t pulsosVen;   // pulsos en la ventana de registro
  uint8_t  flags;
  uint8_t  arranque;    // nro de arranque (mod 256) en que se escribio
} LogRec;
static_assert(sizeof(LogRec) == 32, "LogRec debe medir 32 bytes");

typedef struct {
  uint32_t magic;
  uint32_t version;
  uint32_t cap;
  uint32_t head;        // proxima posicion a escribir
  uint32_t count;       // registros validos
  uint32_t escritos;    // registros escritos desde el ultimo power-on (numeracion)
  uint32_t arranques;   // arranques desde el ultimo power-on
  uint32_t suma;        // suma de CRC32 de los registros validos
  uint32_t crc;         // CRC32 de los campos anteriores
} LogHdr;

// .noinit: la RAM no se inicializa al arrancar -> sobrevive reset por software,
// watchdog, panic y (normalmente) brownout. Se valida con magic + CRC al arrancar.
__NOINIT_ATTR LogHdr logHdr;
#if LOG_MODO == 0
__NOINIT_ATTR LogRec logInterno[LOG_CAP_INTERNA];
#endif

LogRec*  logBuf         = nullptr;
uint32_t logCap         = 0;
bool     logPersistente = false;
bool     logEnPsram     = false;
uint32_t logRecuperados = 0;
uint8_t  arranqueActual = 0;
LogRec   ultimoReg;
bool     hayUltimoReg   = false;

// ============================================
// HARDWARE OBJECTS
// ============================================
Adafruit_BME280 bme;
HardwareSerial& simSerial = Serial1;
WebServer server(80);
DNSServer dnsServer;

// ============================================
// RELOJ (UTC)
// ============================================
enum FuenteReloj : uint8_t { RELOJ_NO = 0, RELOJ_NAVEGADOR = 1, RELOJ_GPS = 2 };
uint8_t relojFuente   = RELOJ_NO;
int64_t relojOffsetUs = 0;      // epoch[us] - esp_timer[us]

// ============================================
// PULSOS (variables compartidas con la ISR)
// ============================================
portMUX_TYPE muxPulsos = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t isrPulsosTot    = 0;  // pulsos desde el arranque
volatile uint32_t isrUltPeriodoUs = 0;  // periodo entre los dos ultimos pulsos
volatile int64_t  isrUltPulsoUs   = 0;  // instante del ultimo pulso aceptado
volatile int64_t  isrSubidaUs     = 0;  // instante de la ultima subida (contacto abierto)
volatile uint32_t isrVenPulsos    = 0;  // pulsos en la ventana de registro
volatile int64_t  isrVenPrimUs    = 0;  // primer pulso de la ventana
volatile int64_t  isrVenUltUs     = 0;  // ultimo pulso de la ventana
uint32_t pulsosBase = 0;                // total recuperado tras un reset

// ============================================
// VELETA
// ============================================
uint32_t veletaMv    = 0;
bool     veletaOk    = false;
float    dirInstDeg  = 0.0f;     // ultima muestra
float    dirEmaX     = 0.0f;     // suavizado vectorial (cos)
float    dirEmaY     = 0.0f;     // suavizado vectorial (sin)
bool     dirEmaIni   = false;
float    venDirSin   = 0.0f;     // acumuladores de la ventana de registro
float    venDirCos   = 0.0f;
uint32_t venDirN     = 0;

// ============================================
// BME280
// ============================================
bool     bmeOk       = false;
uint8_t  bmeAddr     = 0;
float    bmeT = NAN, bmeH = NAN, bmeP = NAN;   // C, %HR, hPa
float    venSumT = 0, venSumH = 0, venSumP = 0;
uint32_t venBmeN     = 0;
unsigned long bmeUltIntento = 0;

// ============================================
// SIM7600 / GNSS
// ============================================
enum SimEstado { SIM_APAGADO, SIM_ARRANCANDO, SIM_CONFIG, SIM_OPERANDO, SIM_REINICIANDO };
SimEstado simEstado   = SIM_APAGADO;
unsigned long simT0   = 0;
uint8_t  simPaso      = 0;
uint8_t  simFallos    = 0;
uint8_t  gpsErrores   = 0;
uint32_t simReinicios = 0;
unsigned long gpsUltConsulta = 0;

enum AtRes { AT_LIBRE, AT_ESPERANDO, AT_OK, AT_ERROR, AT_TIMEOUT };
AtRes    atRes      = AT_LIBRE;
unsigned long atT0  = 0;
unsigned long atTmo = 0;
const char* atPrefijo = nullptr;     // prefijo de la linea de respuesta a capturar
char     atResp[128]   = {0};
char     simLinea[160] = {0};
uint16_t simLineaLen   = 0;

bool     gpsValido  = false;   // hay posicion conocida
int32_t  gpsLatE6   = 0;
int32_t  gpsLonE6   = 0;
float    gpsAlt     = NAN;
unsigned long gpsUltFixMs = 0;

// ============================================
// SISTEMA
// ============================================
char     apSsid[24];
int64_t  proxLogUs    = 0;
unsigned long tVeleta = 0;
unsigned long tBme    = 0;

// ============================================
// FORWARD DECLARATIONS
// ============================================
void isrPulsos();
void leerPulsos(uint32_t& tot, uint32_t& perUs, int64_t& ultUs);

uint32_t crc32Calc(const void* d, size_t n);
int  fmtFijo(char* o, size_t n, int64_t v, uint8_t dec);
uint32_t diasDesdeCivil(int y, unsigned m, unsigned d);
void civilDesdeEpoch(uint32_t t, int& y, unsigned& mo, unsigned& d, unsigned& h, unsigned& mi, unsigned& s);
float mvAGrados(uint32_t mv);
const char* cardinal(float deg);
bool parsearCGPSINFO(const char* l, int32_t& latE6, int32_t& lonE6, float& alt, int64_t& epochUs);

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
int  formatearLineaCSV(char* o, size_t n, const LogRec& r, uint32_t num);

void muestrearVeleta();
bool bmeIniciar();
void tareaBME();

void simTarea();
void simLeerUart();
void simProcesarLinea(const char* l);
void atEnviar(const char* cmd, const char* prefijo, unsigned long tmo);
void simEncender();
void simReiniciar();
bool gpsFixVigente();
const char* simEstadoTxt();

void iniciarWiFi();
void webRaiz();
void webApi();
void webCSV();
void webBorrar();
void webHora();
void webPortal();
const char* resetTxt(esp_reset_reason_t r);

// ============================================
// PAGINA WEB (portal cautivo)
// ============================================
const char PAGINA[] PROGMEM = R"rawliteral(<!DOCTYPE html><html lang="es"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>LEM Nodo Meteo</title>
<style>
:root{--bg:#f3f5f7;--cd:#fff;--tx:#16202a;--mu:#5b6b7b;--ac:#0b7a55;--bd:#dce2e8;--wa:#a86500;--er:#c0392b}
@media(prefers-color-scheme:dark){:root{--bg:#0e1318;--cd:#17202a;--tx:#e6ecf1;--mu:#93a3b3;--ac:#3fd69a;--bd:#283542;--wa:#f2b14c;--er:#ef7468}}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--tx);font:15px/1.45 system-ui,-apple-system,"Segoe UI",Roboto,sans-serif}
header{display:flex;justify-content:space-between;align-items:center;gap:8px;padding:12px 16px;background:var(--cd);border-bottom:1px solid var(--bd)}
h1{font-size:17px;margin:0}h1 small{color:var(--mu);font-weight:400;font-size:13px;margin-left:6px}
main{display:grid;gap:12px;padding:12px 16px;max-width:1100px;margin:auto;grid-template-columns:repeat(auto-fit,minmax(270px,1fr))}
.c{background:var(--cd);border:1px solid var(--bd);border-radius:12px;padding:14px}
.c h2{font-size:12px;letter-spacing:.08em;text-transform:uppercase;color:var(--mu);margin:0 0 8px;font-weight:600}
.big{font-size:28px;font-weight:600;font-variant-numeric:tabular-nums;line-height:1.15}
.big u{text-decoration:none;font-size:15px;color:var(--mu);font-weight:400;margin-left:3px}
.r{display:flex;justify-content:space-between;gap:10px;padding:5px 0;border-bottom:1px dashed var(--bd)}.r:last-child{border:0}
.r span{color:var(--mu)}.r b{font-weight:600;font-variant-numeric:tabular-nums;text-align:right}
.tg{display:inline-block;padding:1px 9px;border-radius:99px;font-size:12px;border:1px solid currentColor;white-space:nowrap}
.ok{color:var(--ac)}.wa{color:var(--wa)}.er{color:var(--er)}.mu{color:var(--mu)}
.bar{height:10px;background:var(--bd);border-radius:99px;overflow:hidden;margin:8px 0}.bar i{display:block;height:100%;width:0;background:var(--ac)}
.bt{display:flex;gap:8px;flex-wrap:wrap;margin-top:10px}
.b{padding:9px 14px;border-radius:8px;border:1px solid var(--ac);background:var(--ac);color:var(--cd);font:inherit;font-weight:600;text-decoration:none;cursor:pointer}
.b.s{background:none;color:var(--ac)}.b.x{background:none;color:var(--er);border-color:var(--er)}
.w{display:flex;gap:14px;align-items:center}svg{flex:none}
.nt{font-size:12px;color:var(--mu);margin:8px 0 0}a{color:var(--ac)}
</style></head><body>
<header><h1>LEM &middot; Nodo Meteo<small id="id"></small></h1><span id="con" class="tg mu">conectando</span></header>
<main>
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
<section class="c"><h2>Pulsos</h2>
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
<p class="nt">Si la descarga no inicia desde la ventana del portal, abre <b>http://192.168.4.1</b> en el navegador.</p>
</section>
<section class="c"><h2>Sistema</h2>
<div class="r"><span>Hora local</span><b id="hl">--</b></div>
<div class="r"><span>Fuente de hora</span><b id="hf">--</b></div>
<div class="r"><span>Encendido hace</span><b id="up">--</b></div>
<div class="r"><span>&Uacute;ltimo reset</span><b id="rst">--</b></div>
<div class="r"><span>Heap libre / m&iacute;nimo</span><b id="hp">--</b></div>
<div class="r"><span>Firmware</span><b id="fw">--</b></div>
</section>
</main>
<script>
const $=i=>document.getElementById(i);
const f=(v,d)=>v==null?'--':Number(v).toFixed(d);
function dur(s){s=Math.max(0,Math.round(s));const d=Math.floor(s/86400),h=Math.floor(s%86400/3600),m=Math.floor(s%3600/60);return(d?d+' d ':'')+(d||h?h+' h ':'')+m+' min';}
function kb(b){return(b/1024).toFixed(1)+' KB';}
function tag(e,t,c){e.textContent=t;e.className='tg '+c;}
let horaOk=false;
function render(d){
 $('id').textContent=d.id;$('fw').textContent=d.fw;
 const g=d.gps;
 if(g.val){$('lat').textContent=f(g.lat,6)+'°';$('lon').textContent=f(g.lon,6)+'°';
  $('geo').href='geo:'+g.lat+','+g.lon+'?q='+g.lat+','+g.lon;$('gm').href='https://maps.google.com/?q='+g.lat+','+g.lon;}
 $('alt').textContent=g.alt==null?'--':f(g.alt,1)+' m';
 if(g.fix)tag($('gst'),'Fix vigente','ok');
 else if(g.val)tag($('gst'),'última hace '+dur(g.edad),'wa');
 else tag($('gst'),'buscando satélites','wa');
 $('sim').textContent=d.sim+(d.simRst?' ('+d.simRst+' reinicios)':'');
 const v=d.dir;
 $('dir').textContent=v.ok?f(v.deg,0)+'°':'--';$('dirc').textContent=v.ok?'desde '+v.card:'veleta en falla';
 $('ag').style.transform='rotate('+(v.ok?v.deg:0)+'deg)';
 $('dirm').textContent=v.med==null?'--':f(v.med,1)+'° '+v.medc;
 $('mv').textContent=v.mv+' mV';
 const p=d.pul;
 $('ptot').innerHTML=p.tot+'<u>pulsos</u>';
 if(p.edad==null||p.edad>=60000||p.per==null){
  $('per').textContent=p.edad==null?'sin pulsos':(p.edad>=60000?'detenido ('+dur(p.edad/1000)+')':'--');$('hz').textContent='0 Hz';
 }else{
  const e=Math.max(p.per,p.edad);
  $('per').textContent=(p.edad>p.per?'≥ ':'')+f(e,1)+' ms';$('hz').textContent=(p.edad>p.per?'≤ ':'')+f(1000/e,3)+' Hz';
 }
 $('pmin').textContent=p.min==null?'--':p.min;
 $('pmed').textContent=p.med==null?'--':f(p.med,1)+' ms';
 const b=d.bme;
 $('t').innerHTML=b.ok?f(b.t,2)+'<u>°C</u>':'--';
 $('h').textContent=b.ok?f(b.h,2)+' %':'--';$('p').textContent=b.ok?f(b.p,2)+' hPa':'--';
 if(b.ok)tag($('bst'),'OK 0x'+b.addr.toString(16),'ok');else tag($('bst'),'sin respuesta','er');
 const l=d.log;
 $('ln').innerHTML=l.n+'<u>/ '+l.cap+' registros</u>';
 $('lb').style.width=(l.cap?100*l.n/l.cap:0).toFixed(1)+'%';
 $('lcap').textContent=l.cap?dur(l.cap*l.per)+' ('+l.rb+' B/reg)':'sin memoria para el registro';
 $('lrest').textContent=!l.cap?'--':l.n>=l.cap?'sobrescribiendo el más antiguo':dur((l.cap-l.n)*l.per);
 $('lmem').textContent=kb(l.cap*l.rb)+(l.psram?' PSRAM':' RAM interna')+(l.pers?', sobrevive resets':'')+(l.rec?', '+l.rec+' recuperados':'');
 const s=d.sis;
 $('hl').textContent=d.epoch?new Date(d.epoch*1000).toLocaleString('es'):'sin hora';
 $('hf').textContent=['sin hora','navegador','GPS'][d.reloj];
 $('up').textContent=dur(d.up);$('rst').textContent=s.rst+' (arranque '+s.arr+')';
 $('hp').textContent=kb(s.heap)+' / '+kb(s.heapMin);
 if(!d.reloj&&!horaOk){horaOk=true;fetch('/hora?e='+Math.floor(Date.now()/1000),{method:'POST'});}
}
async function tick(){
 try{const r=await fetch('/api',{cache:'no-store'});render(await r.json());tag($('con'),'en línea','ok');}
 catch(e){tag($('con'),'sin conexión','er');}
 setTimeout(tick,2000);
}
$('del').onclick=()=>{if(confirm('¿Borrar todo el registro en RAM?'))fetch('/borrar',{method:'POST'});};
tick();
</script></body></html>)rawliteral";

// ============================================
// ISR PULSOS
// ============================================
// Antirrebote: se acepta un flanco de bajada solo si la linea estuvo en ALTO
// al menos PULSO_ALTO_MIN_US (filtra el rebote al abrir el contacto) y si
// paso PULSO_PER_MIN_US desde el pulso anterior (filtra el rebote al cerrar).
void IRAM_ATTR isrPulsos() {
  int64_t t   = esp_timer_get_time();
  int   nivel = gpio_get_level((gpio_num_t)PULSOS_PIN);
  portENTER_CRITICAL_ISR(&muxPulsos);
  if (nivel) {
    isrSubidaUs = t;
  } else if ((t - isrSubidaUs) >= PULSO_ALTO_MIN_US &&
             (isrUltPulsoUs == 0 || (t - isrUltPulsoUs) >= PULSO_PER_MIN_US)) {
    if (isrUltPulsoUs != 0) {
      int64_t p = t - isrUltPulsoUs;
      isrUltPeriodoUs = (p > 0xFFFFFFFFLL) ? 0xFFFFFFFFUL : (uint32_t)p;
    }
    isrUltPulsoUs = t;
    isrPulsosTot  = isrPulsosTot + 1;
    if (isrVenPulsos == 0) isrVenPrimUs = t;
    isrVenUltUs   = t;
    isrVenPulsos  = isrVenPulsos + 1;
  }
  portEXIT_CRITICAL_ISR(&muxPulsos);
}

void leerPulsos(uint32_t& tot, uint32_t& perUs, int64_t& ultUs) {
  portENTER_CRITICAL(&muxPulsos);
  tot   = isrPulsosTot;
  perUs = isrUltPeriodoUs;
  ultUs = isrUltPulsoUs;
  portEXIT_CRITICAL(&muxPulsos);
}

// ============================================
// SETUP
// ============================================
void setup() {
  // Reguladores en estado definido desde el arranque
  pinMode(SIM_EN_PIN, OUTPUT);
  digitalWrite(SIM_EN_PIN, LOW);
  pinMode(SENS_EN_PIN, OUTPUT);
  digitalWrite(SENS_EN_PIN, HIGH);
  pinMode(SIM_TX_PIN, INPUT);   // sin UART activa hasta energizar el SIM (evita alimentarlo por TX)
  pinMode(SIM_RX_PIN, INPUT);

  Serial.begin(115200);
  delay(20);                    // arranque del BME280 tras energizar (datasheet: 2 ms)

  Wire.begin(SDA_PIN, SCL_PIN, 100000);
  bmeOk = bmeIniciar();
  bmeUltIntento = millis();

  // ESP32-S2 a 6 dB: rango util ~0..1300 mV (cubre 1,12 V con mejor resolucion que 11 dB).
  // Debe ser la atenuacion GLOBAL y antes de la primera lectura: en core 3.x
  // analogSetPinAttenuation() no tiene efecto si el pin aun no es canal ADC, y la
  // calibracion de analogReadMilliVolts() se crea con la atenuacion global.
  analogSetAttenuation(ADC_6db);
  pinMode(PULSOS_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PULSOS_PIN), isrPulsos, CHANGE);

  simSerial.setRxBufferSize(1024);

  iniciarWiFi();
  logInit();

  Serial.printf("LEM Nodo Meteo v%s  SSID:%s  reset:%s\n", FW_VERSION, apSsid, resetTxt(esp_reset_reason()));
  Serial.printf("Registro: %lu x %u B = %lu B (%s), recuperados %lu\n",
                (unsigned long)logCap, (unsigned)sizeof(LogRec), (unsigned long)(logCap * sizeof(LogRec)),
                logEnPsram ? "PSRAM" : "RAM interna", (unsigned long)logRecuperados);
  Serial.printf("Heap libre %lu B, bloque max %lu B\n", (unsigned long)ESP.getFreeHeap(), (unsigned long)ESP.getMaxAllocHeap());

  proxLogUs = esp_timer_get_time() + (int64_t)LOG_PERIODO_S * 1000000LL;
}

// ============================================
// MAIN LOOP
// ============================================
void loop() {
  dnsServer.processNextRequest();
  server.handleClient();
  simTarea();

  unsigned long ahora = millis();
  if (ahora - tVeleta >= VELETA_PERIODO_MS) {
    tVeleta = ahora;
    muestrearVeleta();
  }
  if (ahora - tBme >= BME_PERIODO_MS) {
    tBme = ahora;
    tareaBME();
  }

  int64_t us = esp_timer_get_time();
  if (us >= proxLogUs) {
    registrar();
    proxLogUs += (int64_t)LOG_PERIODO_S * 1000000LL;
    if (proxLogUs <= us) proxLogUs = us + (int64_t)LOG_PERIODO_S * 1000000LL;  // se atraso (descarga larga)
  }

  delay(2);
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
// Sin fix: "+CGPSINFO: ,,,,,,,,". Retorna true si hay posicion.
// epochUs = 0 si la fecha/hora no es valida.
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

  double latR = atof(c[0]), lonR = atof(c[2]);           // ddmm.mmmmmm / dddmm.mmmmmm
  double latG = floor(latR / 100.0), lonG = floor(lonR / 100.0);
  double lat  = latG + (latR - latG * 100.0) / 60.0;
  double lon  = lonG + (lonR - lonG * 100.0) / 60.0;
  if (c[1][0] == 'S') lat = -lat;
  if (c[3][0] == 'W') lon = -lon;
  if (fabs(lat) > 90.0 || fabs(lon) > 180.0) return false;
  latE6 = (int32_t)lround(lat * 1e6);
  lonE6 = (int32_t)lround(lon * 1e6);
  alt   = (nc > 6 && c[6][0]) ? (float)atof(c[6]) : NAN;

  // Fecha ddmmyy + hora hhmmss.s (UTC)
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

// GPS tiene prioridad sobre la hora del navegador. La primera vez que hay hora
// en este arranque, se fechan los registros que solo tenian segundos desde arranque.
void relojAjustar(int64_t epochUs, uint8_t fuente) {
  if (epochUs < (int64_t)EPOCH_MIN_VALIDO * 1000000LL) return;
  if (fuente < relojFuente) return;
  bool primera  = (relojFuente == RELOJ_NO);
  relojOffsetUs = epochUs - esp_timer_get_time();
  relojFuente   = fuente;
  if (primera) logFecharArranque((uint32_t)(relojOffsetUs / 1000000LL));   // epoch con esp_timer = 0
}

// ============================================
// REGISTRO EN RAM (buffer circular)
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

// Se llama despues de iniciar WiFi/web para que el modo 1 vea el heap real que queda.
void logInit() {
#if LOG_MODO == 1
  if (psramFound()) {
    logBuf = (LogRec*)heap_caps_malloc(LOG_PSRAM_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (logBuf) { logCap = LOG_PSRAM_BYTES / sizeof(LogRec); logEnPsram = true; }
  }
  if (!logBuf) {
    size_t libre  = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    size_t bloque = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    size_t bytes  = (libre > LOG_RESERVA_HEAP) ? libre - LOG_RESERVA_HEAP : 0;
    if (bytes > bloque) bytes = bloque;
    logCap = bytes / sizeof(LogRec);
    if (logCap >= 64) logBuf = (LogRec*)heap_caps_malloc(logCap * sizeof(LogRec), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!logBuf) logCap = 0;
  }
  logPersistente = false;
  logHdr.magic = LOG_MAGIC; logHdr.version = LOG_VERSION; logHdr.cap = logCap; logHdr.arranques = 0;
  logBorrar();
#else
  logBuf         = logInterno;
  logCap         = LOG_CAP_INTERNA;
  logPersistente = true;
  if (esp_reset_reason() != ESP_RST_POWERON && logHdrValido()) {
    logHdr.arranques++;
    logRecuperados = logHdr.count;
    if (logHdr.count > 0) {
      const LogRec& u = logBuf[(logHdr.head + logCap - 1) % logCap];
      pulsosBase  = u.pulsosTot;          // el total continua tras el reset
      ultimoReg   = u;
      hayUltimoReg = true;
    }
    logHdrSellar();
  } else {
    logHdr.magic = LOG_MAGIC; logHdr.version = LOG_VERSION; logHdr.cap = logCap; logHdr.arranques = 0;
    logBorrar();
  }
#endif
  arranqueActual = (uint8_t)(logHdr.arranques & 0xFF);
}

void logAgregar(const LogRec& r) {
  if (!logBuf || logCap == 0) return;
  uint32_t i = logHdr.head;
  if (logHdr.count == logCap) logHdr.suma -= crc32Calc(&logBuf[i], sizeof(LogRec));   // pisa el mas antiguo
  else logHdr.count++;
  logBuf[i] = r;
  logHdr.suma += crc32Calc(&r, sizeof(LogRec));
  logHdr.head  = (i + 1) % logCap;
  logHdr.escritos++;
  logHdrSellar();
}

void logFecharArranque(uint32_t epochArranque) {
  if (!logBuf) return;
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

  // Pulsos de la ventana
  portENTER_CRITICAL(&muxPulsos);
  uint32_t n    = isrVenPulsos;
  int64_t  p0   = isrVenPrimUs;
  int64_t  p1   = isrVenUltUs;
  uint32_t tot  = isrPulsosTot;
  uint32_t ultP = isrUltPeriodoUs;
  isrVenPulsos  = 0;
  portEXIT_CRITICAL(&muxPulsos);
  r.pulsosTot = pulsosBase + tot;
  r.pulsosVen = (n > 0xFFFF) ? 0xFFFF : (uint16_t)n;
  if (n >= 2)      r.periodoUs = (uint32_t)((p1 - p0) / (int64_t)(n - 1));
  else if (n == 1) r.periodoUs = ultP;       // intervalo con el pulso anterior
  else             r.periodoUs = 0;

  // BME280: promedio de la ventana
  if (venBmeN > 0) {
    r.temp_c   = (int16_t)lroundf(venSumT / venBmeN * 100.0f);
    r.hum_c    = (uint16_t)lroundf(venSumH / venBmeN * 100.0f);
    r.pres_2pa = (uint16_t)lroundf(venSumP / venBmeN * 50.0f);   // hPa*100/2
    r.flags   |= FLG_BME_OK;
  }
  venSumT = venSumH = venSumP = 0;
  venBmeN = 0;

  // Veleta: media vectorial (promediar angulos directo falla en 359/1 grados)
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
// VELETA
// ============================================
void muestrearVeleta() {
  uint32_t acc = 0;
  for (int i = 0; i < VELETA_MUESTRAS; i++) acc += analogReadMilliVolts(VELETA_PIN);
  veletaMv = acc / VELETA_MUESTRAS;
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
// BME280
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
  if (!bmeOk) {
    if (millis() - bmeUltIntento >= BME_REINTENTO_MS) {
      bmeUltIntento = millis();
      bmeOk = bmeIniciar();
    }
    if (!bmeOk) { bmeT = bmeH = bmeP = NAN; return; }
  }

  // Verificar que sigue en el bus (la libreria no detecta desconexion)
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
    bmeT = bmeH = bmeP = NAN;     // fuera del rango del sensor: lectura invalida
    return;
  }
  bmeT = t; bmeH = h; bmeP = p;
  venSumT += t; venSumH += h; venSumP += p;
  venBmeN++;
}

// ============================================
// SIM7600: DRIVER AT NO BLOQUEANTE
// ============================================
void atEnviar(const char* cmd, const char* prefijo, unsigned long tmo) {
  atResp[0] = 0;
  atPrefijo = prefijo;
  atTmo     = tmo;
  atT0      = millis();
  atRes     = AT_ESPERANDO;
  simSerial.print(cmd);
  simSerial.print("\r");
}

void simProcesarLinea(const char* l) {
  if (atRes != AT_ESPERANDO) return;        // URC fuera de transaccion (RDY, +CPIN, SMS DONE...)
  if (!strcmp(l, "OK")) atRes = AT_OK;
  else if (!strcmp(l, "ERROR") || !strncmp(l, "+CME ERROR", 10)) atRes = AT_ERROR;
  else if (atPrefijo && !strncmp(l, atPrefijo, strlen(atPrefijo))) {
    strncpy(atResp, l, sizeof(atResp) - 1);
    atResp[sizeof(atResp) - 1] = 0;
  }
}

void simLeerUart() {
  while (simSerial.available()) {
    char c = (char)simSerial.read();
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

void simEncender() {
  digitalWrite(SIM_EN_PIN, HIGH);
  simSerial.begin(SIM_BAUD, SERIAL_8N1, SIM_RX_PIN, SIM_TX_PIN);
  simLineaLen = 0;
  atRes       = AT_LIBRE;
  simFallos   = 0;
  simT0       = millis();
  simEstado   = SIM_ARRANCANDO;
}

// Corta los reguladores. Se usa solo si el modulo no responde (no hay otra forma de recuperarlo).
void simReiniciar() {
  simSerial.end();
  pinMode(SIM_TX_PIN, INPUT);       // no alimentar el modulo apagado a traves de su RX
  pinMode(SIM_RX_PIN, INPUT);
  digitalWrite(SIM_EN_PIN, LOW);
  atRes     = AT_LIBRE;
  simFallos = 0;
  simT0     = millis();
  simReinicios++;
  simEstado = SIM_REINICIANDO;
}

void simTarea() {
  if (simEstado != SIM_APAGADO && simEstado != SIM_REINICIANDO) simLeerUart();
  unsigned long ahora = millis();

  switch (simEstado) {
    case SIM_APAGADO:
      simEncender();
      break;

    case SIM_ARRANCANDO:
      // Esperar a que el modulo responda "AT" (arranque tipico 10-20 s)
      if (atRes == AT_ESPERANDO) break;
      if (atRes == AT_OK) { atRes = AT_LIBRE; simPaso = 0; simEstado = SIM_CONFIG; break; }
      if (ahora - simT0 >= SIM_T_BOOT_MAX_MS) { simReiniciar(); break; }
      if (atRes == AT_LIBRE || ahora - atT0 >= 1000) atEnviar("AT", nullptr, 500);
      break;

    case SIM_CONFIG:
      if (atRes == AT_ESPERANDO) break;
      if (atRes == AT_TIMEOUT) {
        if (++simFallos >= SIM_FALLOS_MAX) { simReiniciar(); break; }
        atRes = AT_LIBRE;                                    // repetir el paso
      } else if (atRes == AT_OK || atRes == AT_ERROR) {      // paso terminado
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
        case 4: atEnviar("AT+CGPS=1,1", nullptr, 5000); break;   // GNSS autonomo
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
        if (++gpsErrores >= 3) { atRes = AT_LIBRE; simPaso = 3; simEstado = SIM_CONFIG; break; }  // GNSS apagado: reconfigurar
      } else if (atRes == AT_TIMEOUT) {
        if (++simFallos >= SIM_FALLOS_MAX) { simReiniciar(); break; }
      }
      atRes = AT_LIBRE;
      if (ahora - gpsUltConsulta >= GPS_POLL_MS) {
        gpsUltConsulta = ahora;
        atEnviar("AT+CGPSINFO", "+CGPSINFO:", 1500);
      }
      break;

    case SIM_REINICIANDO:
      if (ahora - simT0 >= SIM_T_OFF_MS) simEstado = SIM_APAGADO;
      break;
  }
}

bool gpsFixVigente() {
  return gpsValido && (millis() - gpsUltFixMs) < (unsigned long)GPS_FIX_VIGENTE_S * 1000UL;
}

const char* simEstadoTxt() {
  switch (simEstado) {
    case SIM_APAGADO:     return "apagado";
    case SIM_ARRANCANDO:  return "arrancando";
    case SIM_CONFIG:      return "configurando";
    case SIM_OPERANDO:    return "operando";
    case SIM_REINICIANDO: return "reiniciando";
  }
  return "?";
}

// ============================================
// WIFI + PORTAL CAUTIVO
// ============================================
void iniciarWiFi() {
  uint64_t mac = ESP.getEfuseMac();
  snprintf(apSsid, sizeof(apSsid), "%s%02X%02X", AP_PREFIJO,
           (unsigned)((mac >> 32) & 0xFF), (unsigned)((mac >> 40) & 0xFF));

  WiFi.persistent(false);           // no escribir credenciales en flash
  WiFi.mode(WIFI_AP);
  WiFi.softAP(apSsid, strlen(AP_PASS) ? AP_PASS : nullptr, AP_CANAL, 0, AP_MAX_CLIENTES);
  delay(100);

  dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
  dnsServer.start(53, "*", WiFi.softAPIP());     // todo nombre resuelve al nodo

  server.on("/",        HTTP_GET,  webRaiz);
  server.on("/api",     HTTP_GET,  webApi);
  server.on("/log.csv", HTTP_GET,  webCSV);
  server.on("/borrar",  HTTP_POST, webBorrar);
  server.on("/hora",    HTTP_POST, webHora);
  server.onNotFound(webPortal);
  server.begin();
}

void webRaiz() {
  server.sendHeader("Cache-Control", "no-store");
  server.send_P(200, "text/html; charset=utf-8", PAGINA);
}

// Cualquier URL desconocida (generate_204, hotspot-detect.html, connecttest.txt...)
// redirige al portal: asi Android/iOS/Windows abren la ventana de "iniciar sesion".
void webPortal() {
  server.sendHeader("Location", String("http://") + WiFi.softAPIP().toString() + "/", true);
  server.send(302, "text/plain", "");
}

// --- JSON con buffer fijo ---
char   jsonBuf[1400];
size_t jsonLen = 0;

void jAdd(const char* fmt, ...) {
  if (jsonLen >= sizeof(jsonBuf) - 1) return;
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(jsonBuf + jsonLen, sizeof(jsonBuf) - jsonLen, fmt, ap);
  va_end(ap);
  if (n > 0) jsonLen += min((size_t)n, sizeof(jsonBuf) - 1 - jsonLen);
}

void jNum(const char* k, float v, int dec, bool coma = true) {
  if (isnan(v)) jAdd("\"%s\":null%s", k, coma ? "," : "");
  else jAdd("\"%s\":%.*f%s", k, dec, v, coma ? "," : "");
}

void webApi() {
  uint32_t tot, perUs; int64_t ultUs;
  leerPulsos(tot, perUs, ultUs);
  int64_t ahoraUs = esp_timer_get_time();
  bool    gpsFix  = gpsFixVigente();

  jsonLen = 0;
  jAdd("{\"id\":\"%s\",\"fw\":\"%s\",\"up\":%lu,\"reloj\":%u,\"epoch\":%lu,\"sim\":\"%s\",\"simRst\":%lu,",
       apSsid, FW_VERSION, (unsigned long)uptimeS(), (unsigned)relojFuente,
       (unsigned long)epochAhora(), simEstadoTxt(), (unsigned long)simReinicios);

  // GPS
  jAdd("\"gps\":{\"val\":%d,\"fix\":%d,", gpsValido ? 1 : 0, gpsFix ? 1 : 0);
  if (gpsValido) {
    char la[16], lo[16];
    fmtFijo(la, sizeof(la), gpsLatE6, 6);
    fmtFijo(lo, sizeof(lo), gpsLonE6, 6);
    jAdd("\"lat\":%s,\"lon\":%s,\"edad\":%lu,", la, lo, (unsigned long)((millis() - gpsUltFixMs) / 1000UL));
  }
  jNum("alt", gpsAlt, 1, false);
  jAdd("},");

  // BME280
  bool bOk = bmeOk && !isnan(bmeT);
  jAdd("\"bme\":{\"ok\":%d,\"addr\":%u,", bOk ? 1 : 0, (unsigned)bmeAddr);
  jNum("t", bmeT, 2); jNum("h", bmeH, 2); jNum("p", bmeP, 2, false);
  jAdd("},");

  // Veleta: vista suavizada + media del ultimo registro
  float dirViva = atan2f(dirEmaY, dirEmaX) * RAD_TO_DEG;
  if (dirViva < 0.0f) dirViva += 360.0f;
  bool dOk = veletaOk && dirEmaIni;
  jAdd("\"dir\":{\"ok\":%d,\"mv\":%lu,\"card\":\"%s\",", dOk ? 1 : 0, (unsigned long)veletaMv, dOk ? cardinal(dirViva) : "");
  jNum("deg", dOk ? dirViva : NAN, 1);
  if (hayUltimoReg && (ultimoReg.flags & FLG_DIR_OK))
    jAdd("\"med\":%.1f,\"medc\":\"%s\"},", ultimoReg.dir_d / 10.0f, cardinal(ultimoReg.dir_d / 10.0f));
  else
    jAdd("\"med\":null},");

  // Pulsos: ultimo periodo medido y tiempo desde el ultimo pulso (el portal
  // muestra max(periodo, edad): si el sensor se detiene el periodo no queda congelado)
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

  // Registro
  jAdd("\"log\":{\"n\":%lu,\"cap\":%lu,\"rb\":%u,\"per\":%u,\"pers\":%d,\"psram\":%d,\"rec\":%lu},",
       (unsigned long)logHdr.count, (unsigned long)logCap, (unsigned)sizeof(LogRec), (unsigned)LOG_PERIODO_S,
       logPersistente ? 1 : 0, logEnPsram ? 1 : 0, (unsigned long)logRecuperados);

  // Sistema
  jAdd("\"sis\":{\"heap\":%lu,\"heapMin\":%lu,\"rst\":\"%s\",\"arr\":%lu}}",
       (unsigned long)ESP.getFreeHeap(), (unsigned long)ESP.getMinFreeHeap(),
       resetTxt(esp_reset_reason()), (unsigned long)logHdr.arranques);

  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", jsonBuf);
}

// CSV generado al vuelo en bloques (no se arma en RAM: 3072 lineas ~ 340 KB)
void webCSV() {
  char nombre[48];
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

  // Instantanea de la cabecera (el loop no corre durante este handler)
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
  server.sendContent("");             // fin de la respuesta chunked
}

void webBorrar() {
  logBorrar();
  server.send(200, "application/json", "{\"ok\":1}");
}

// El navegador entrega su hora si el nodo aun no tiene (sin fix GPS)
void webHora() {
  uint32_t e = (uint32_t)strtoul(server.arg("e").c_str(), nullptr, 10);
  relojAjustar((int64_t)e * 1000000LL, RELOJ_NAVEGADOR);
  server.send(200, "application/json", relojFuente ? "{\"ok\":1}" : "{\"ok\":0}");
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
