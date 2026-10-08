# LEM Nodo Meteo · ESP32-S2

Firmware: `CODE/V0.1.0.0/lem_meteo_s2_v0_1_0_0/lem_meteo_s2_v0_1_0_0.ino`

Nodo con SIM7600X (GNSS), BME280, veleta analógica y entrada de pulsos.
Publica un portal cautivo WiFi con los datos en vivo y guarda un registro
cada 1 minuto en RAM, descargable como CSV (separador `,`, decimal `.`).
El firmware no escribe la flash.

## Pines

| GPIO | Función |
|-----:|---------|
| 3  | RX del ESP ← TX del SIM7600 |
| 4  | TX del ESP → RX del SIM7600 |
| 14 | Habilita reguladores del SIM7600 (HIGH = encendido) |
| 15 | Habilita regulador de sensores (HIGH = encendido) |
| 8 / 9 | I2C SDA / SCL (BME280 en 0x76 o 0x77) |
| 2  | Veleta 0..1,12 V (ADC1_CH1, atenuación 6 dB) |
| 10 | Pulsos: contacto a GND, `INPUT_PULLUP`, cuenta el flanco de bajada |

## Compilar

- Core **esp32 by Espressif 3.x** (verificado con 3.3.12). Placa: *ESP32S2 Dev Module*.
- Librería: *Adafruit BME280 Library* (instala también *Adafruit Unified Sensor* y *Adafruit BusIO*).
- **USB CDC On Boot** cambia la capacidad del registro (ver abajo). El firmware ajusta la capacidad solo.
- **PSRAM: Enabled** solo si el módulo tiene PSRAM (ESP32-S2FN4R2, WROVER, LOLIN S2 Mini).

## Portal

1. Conectarse a la red WiFi `LEM-WX-XXXX` (abierta; la clave se cambia en `AP_PASS`).
2. El teléfono abre la ventana de "iniciar sesión" sola. Si no la abre, ir a `http://192.168.4.1`.
3. Para **descargar el CSV**, usar el navegador normal en `http://192.168.4.1`.
   Las ventanas de portal cautivo de iOS y Android muchas veces no permiten descargas.

El portal muestra lo siguiente:

- Posición GPS: latitud, longitud, altitud y edad del fix.
- Pulsos totales, período y frecuencia.
- Dirección del viento: brújula, grados y punto cardinal. También la media vectorial del último minuto.
- Temperatura, humedad y presión.
- Estado del registro y del sistema: heap libre, motivo del último reset y fuente de la hora.

Si el nodo todavía no tiene hora GPS, toma la hora del teléfono conectado.
Cuando llega la primera hora, los registros de este arranque que solo tenían
"segundos desde el arranque" reciben su fecha UTC.

## ¿Cuántos datos caben en la RAM?

Cada registro se guarda en binario de punto fijo: **32 bytes por minuto**.
El CSV se genera recién al descargar. Como texto CSV, un registro ocupa ~100 B
(122 B como máximo), así que en binario caben 3 a 4 veces más registros.

La SRAM del ESP32-S2 es de 320 KB, pero no toda queda libre. La caché usa 16 KB.
El código en IRAM usa ~70 KB y comparte la misma RAM. Luego están las
variables, FreeRTOS, WiFi, lwIP y el servidor web. Valores medidos con el
enlazador (core 3.3.12):

| Configuración | Registros | Duración a 1/min | ¿Sobrevive a un reset? |
|---|---:|---:|:--:|
| `LOG_MODO 0`, USB CDC On Boot **Disabled** | **1 440** (máx. 1 632) | **24 h** (máx. 27,2 h) | Sí |
| `LOG_MODO 0`, USB CDC On Boot **Enabled** | **480** (máx. ~609) | **8 h** (máx. ~10 h) | Sí |
| `LOG_MODO 1`, módulo con PSRAM de 2 MB | **57 344** | **39,8 días** | No |

- **Modo 0 (por defecto).** El buffer está en la sección `.noinit` de la RAM interna.
  El arranque no la borra, así que el registro sobrevive a resets por software,
  watchdog, panic y, normalmente, brownout (caídas del SIM7600). Al arrancar se
  valida con un número mágico, el CRC32 de la cabecera y la suma de los CRC32 de
  todos los registros. Si algo no calza, el registro empieza vacío. El límite es
  físico: la RAM estática termina en `0x3FFDE000`, porque arriba está la zona del
  bootloader. Si se pide más de lo que cabe, el enlazador falla con
  `region dram0_0_seg overflowed`. Así nunca se compila algo que no quepa.
  TinyUSB (USB CDC On Boot) ocupa ~32 KB de esa zona.
- **Modo 1.** Usa la PSRAM. Si no la encuentra, usa el heap interno dejando 64 KB
  libres para WiFi y web. Este registro se pierde con cualquier reset.
- **Al llenarse no se satura.** El buffer es circular: el registro nuevo
  reemplaza al más antiguo y siempre quedan las últimas N horas. La columna `n`
  (número correlativo) muestra si se perdieron registros antiguos.
- **Corte de energía.** Con un corte de energía se pierde todo el registro, porque está en RAM.
- **Heap.** Después de reservar el registro de 24 h quedan ~140 KB de heap al
  arrancar, antes de FreeRTOS y WiFi. El valor real se ve en el portal, en
  *Heap libre / mínimo*.

### Sobre el desgaste de la flash

El temor tiene fundamento solo si se escribe de forma ingenua. La flash NOR de
estos módulos aguanta ~100 000 borrados por sector de 4 KB. Si se reescribe el
mismo sector cada minuto, el sector muere en ~69 días. Con un registro circular
de 32 B en una partición de 1 MB, cada sector se borra una vez cada 22,7 días.
Así, la flash duraría miles de años. Si más adelante se necesita que el
registro sobreviva a cortes de energía, esa es la forma correcta de hacerlo.

## CSV

```
n,fecha_utc,seg_desde_arranque,arranque,lat,lon,gps_fix,temp_C,hum_pct,pres_hPa,dir_deg,dir_card,pulsos_min,pulsos_total,periodo_ms
7,2026-10-08 18:25:00,,3,-33.435391,-70.610905,1,-1.25,45.10,955.32,202.5,SSO,42,123456,1432.123
```

- `fecha_utc`: queda vacía si no había hora. En ese caso `seg_desde_arranque` indica los segundos desde el arranque `arranque`.
- `temp_C`, `hum_pct` y `pres_hPa`: promedio de las lecturas del minuto (una cada 2 s).
- `dir_deg`: media **vectorial** del minuto (una muestra por segundo). Un promedio aritmético da un error cerca del norte: 359° y 1° darían 180°.
- `periodo_ms`: período medio de los pulsos del minuto, igual a (último − primero) / (n − 1).
- Un campo vacío significa que no hay dato (sin fix, BME280 sin respuesta, sin pulsos o veleta en falla).

## Veleta

- La conversión es lineal: 0 V = N, 0,28 V = E, 0,56 V = S, 0,84 V = O y 1,12 V = N.
- Si la tensión crece en sentido antihorario, usar `VELETA_INVERTIDA 1`.
- Para alinear el montaje, usar `VELETA_OFFSET_DEG`.
- 1 mV de error equivale a 0,32° (360° / 1 120 mV). Para calibrar, leer *Tensión* en el portal y ajustar `VELETA_VMAX_MV`.
- Una tensión mayor que 110 % de VMAX se marca como falla de cableado.

## Notas de hardware

- **Niveles UART del SIM7600.** La UART del SIM7600 trabaja a 1,8 V. Verificar que la placa tenga un traductor de nivel hacia los 3,3 V del ESP32.
- **Encendido del SIM7600.** El firmware supone que el módulo arranca solo al habilitar sus reguladores (PWRKEY en auto-encendido). Si PWRKEY necesita un pulso, hace falta un GPIO extra.
- **Recuperación del SIM7600.** Si el módulo no responde a AT, el firmware le corta los reguladores 5 s y lo vuelve a encender. Mientras está apagado, los pines UART quedan como entrada para no alimentarlo por su RX.
- **Picos de corriente.** El SIM7600 tiene picos de hasta ~2 A. Con poca capacidad de respaldo, el ESP sufre brownout. El registro sobrevive a ese reset, pero conviene evitarlo.
- **Antirrebote de pulsos.** Un flanco se acepta si la línea estuvo en ALTO ≥ 2 ms y pasaron ≥ 4 ms desde el pulso anterior, con un máximo de 250 Hz.
- **Cables largos de pulsos.** Con cable largo se recomienda una resistencia externa de 10 kΩ a 3,3 V y un condensador de 100 nF a GND (τ = 1 ms).

## Verificación realizada

- Compilación sin warnings del sketch en estas configuraciones:
  - ESP32S2 Dev Module, con PSRAM y sin PSRAM, con USB CDC y sin USB CDC.
  - LOLIN S2 Mini.
  - Modo 0 y modo 1.
- Margen de RAM estática medido en cada configuración: entre 3 y 6 KB.
- 56 pruebas en PC, con AddressSanitizer y UBSan, de las funciones del sketch:
  - CRC32.
  - Punto fijo.
  - Fechas: ida y vuelta contra la libc, de 1970 a 2106.
  - Veleta y puntos cardinales.
  - Parser de `AT+CGPSINFO`.
  - Líneas CSV.
  - Buffer circular: detección de corrupción y fechado retroactivo.
- El JSON de `/api` es válido en 7 estados del nodo.
- El portal se renderizó en Chromium a 390 px de ancho, en tema claro y oscuro, sin errores de JS.
- **No verificado en hardware:** el heap real con WiFi activo, el diálogo con el SIM7600 y el portal cautivo en teléfonos reales.
