# ARBORIS - Monitoreo inteligente de estres y estabilidad en arboles (STM32F051)

## 1. Objetivo del proyecto
Implementar un nodo de monitoreo preventivo para arboles del campus usando STM32F051, con lectura de sensores, evaluacion de riesgo y envio de datos por Bluetooth.

Sensores integrados:
- Humedad de suelo (MOD-003, salida AO + ADC)
- Lluvia (YL-83: DO digital y AO analogico)
- Temperatura y humedad ambiental (DHT22)
- Vibracion/inercia del tronco (MPU6050 por I2C)
- Comunicacion Bluetooth (HC-05/HC-06 por UART)

---

## 2. Arquitectura implementada (firmware)
El ciclo principal (1 segundo) realiza:
1. Lectura ADC de humedad de suelo.
2. Lectura lluvia digital (DO) y analogica (AO).
3. Lectura DHT22 por protocolo one-wire por software (bit-banging).
4. Lectura MPU6050 por I2C (acelerometro + giroscopio).
5. Evaluacion de estado de riesgo (verde/amarillo/rojo).
6. Envio de trama por UART (Bluetooth).

---

## 3. Conexiones finales
### 3.1 MOD-003 (suelo)
- VCC -> 3.3V
- GND -> GND
- AO -> PA0 (ADC_IN0)

### 3.2 YL-83 (lluvia)
- VCC -> 3.3V
- GND -> GND
- DO -> PA1 (entrada digital)
- AO -> PA3 (ADC_IN3)

### 3.3 DHT22 (modulo)
- VCC -> 3.3V
- GND -> GND
- DATA -> PA2 (GPIO OD, conmutado a entrada por software durante lectura)

### 3.4 MPU6050 (GY-521)
- VCC -> 3.3V (segun modulo tambien puede aceptar 5V)
- GND -> GND
- SCL -> PB6 (I2C1_SCL)
- SDA -> PB7 (I2C1_SDA)
- AD0 -> GND (0x68) o VCC (0x69). El firmware autodetecta ambas.

### 3.5 Bluetooth HC-05/HC-06
- TXD (BT) -> PA10 (USART1_RX)
- RXD (BT) -> PA9 (USART1_TX)
- GND -> GND
- VCC -> segun modulo

---

## 4. Hallazgos importantes durante la integracion
## 4.1 Live Expressions y variables uint8_t
Se detecto error visual del debugger con algunas `uint8_t`:
- `error reading variable: Converting character sets: Invalid argument`

Solucion:
- Exponer variables espejo `uint32_t` para depuracion.
- En visualizacion, preferir variables `_dbg`.

## 4.2 DHT22: problema de checksum con retardos por NOP
Con retardos aproximados por bucles `NOP`, el sensor respondia pero fallaba checksum.

Solucion:
- Uso de TIM14 a 1 MHz (1 us por tick) para temporizacion real en microsegundos.
- Esto estabilizo la lectura (`lectura_valida=1` y `checksum_ok=1`).

## 4.3 MPU6050: direccion I2C variable
Dependiendo de AD0, el modulo responde en 0x68 o 0x69.

Solucion:
- Autodeteccion de ambas direcciones al inicializar.

## 4.4 YL-83 AO: variacion muy pequena
Se observo que la salida analogica AO tiene variaciones pequenas entre seco/mojado en algunos modulos.

Solucion:
- Calibracion por referencia real:
  - `lluvia_ao_adc_ref_seco`
  - `lluvia_ao_adc_ref_mojado`
- Normalizacion a porcentaje calibrado (`lluvia_ao_nivel_agua_calibrado_pct`).
- Uso de DO como bandera principal de lluvia activa.

## 4.5 Trama UART y floats vacios
En Termite aparecian campos vacios con `%.1f`.

Causa:
- `printf/snprintf` sin soporte de `float` en la configuracion/newlib de STM32.

Solucion aplicada:
- Enviar valores escalados x10 en enteros (`HS_X10`, `TA_X10`, etc.).
- Evita dependencia de soporte `float` en printf.

---

## 5. Lógica de riesgo implementada
Estados:
- 0 = Verde
- 1 = Amarillo
- 2 = Rojo

Reglas:
1. Rojo si:
- humedad de suelo >= `umbral_humedad_saturada_pct`
- y lluvia activa (DO)
- y vibracion anomala (gyro absoluto supera umbral)

2. Amarillo si cualquiera:
- humedad suelo >= `umbral_humedad_alta_pct`
- o lluvia activa
- o vibracion anomala

3. Verde en otro caso.

---

## 6. Precision y rendimiento: float vs entero en STM32F051
El STM32F051 (Cortex-M0) no tiene FPU de hardware. Esto implica:

1. Precision:
- `float` da buena representacion para porcentajes/temperatura.
- Para visualizacion y logica simple es suficiente.
- En sensores analogicos, el error dominante suele ser el sensor/calibracion, no `float`.

2. Costo computacional:
- Operaciones `float` son mas lentas que enteros (emulacion por software).
- `snprintf` con `float` puede ser costoso y aumentar uso de memoria.

3. Impacto real en este proyecto:
- Con periodo de 1 s, el rendimiento actual es suficiente.
- El cuello de botella no es el `float`; son tiempos de sensores (DHT22/I2C/UART).

4. Recomendacion practica:
- Mantener `float` para calculo interno si facilita lectura.
- Para transmision UART y cliente, enviar enteros escalados (x10 o x100).
- Si se requiere optimizacion futura: migrar toda la canalizacion a fixed-point.

---

## 7. Estructura de trama de datos (Bluetooth UART)
### 7.1 Trama enviada por UART (cada 1 s)
Formato real en firmware (CSV, 13 campos, termina en salto de linea `\n`):

`<HS_X10>,<HS_ADC>,<LLD>,<LLA_X10>,<LLA_ADC>,<TA_X10>,<HA_X10>,<AX>,<AY>,<AZ>,<GX>,<GY>,<GZ>\n`

Cadena `snprintf` real:

`"%d,%u,%lu,%d,%u,%d,%d,%d,%d,%d,%d,%d,%d\n"`

| Pos | Campo   | Tipo | Escala / rango esperado | Descripcion |
|---|---|---|---|---|
| 1 | `HS_X10` | `int` | `% suelo * 10` | Humedad de suelo escalada (ej. `610` = `61.0%`). |
| 2 | `HS_ADC` | `uint` | `0..4095` | ADC crudo de humedad de suelo. |
| 3 | `LLD` | `uint32` | `0` o `1` | Lluvia digital activa (`1` = lluvia detectada). |
| 4 | `LLA_X10` | `int` | `% agua calibrado * 10` | Nivel de agua de lluvia AO calibrado (ej. `125` = `12.5%`). |
| 5 | `LLA_ADC` | `uint` | `0..4095` | ADC crudo de lluvia AO. |
| 6 | `TA_X10` | `int` | `C * 10` (con signo) | Temperatura ambiente escalada (ej. `222` = `22.2C`, `-30` = `-3.0C`). |
| 7 | `HA_X10` | `int` | `%RH * 10` | Humedad ambiente escalada (ej. `585` = `58.5%`). |
| 8 | `AX` | `int` | `int16` | Aceleracion X del MPU6050 (crudo). |
| 9 | `AY` | `int` | `int16` | Aceleracion Y del MPU6050 (crudo). |
| 10 | `AZ` | `int` | `int16` | Aceleracion Z del MPU6050 (crudo). |
| 11 | `GX` | `int` | `int16` | Giroscopio X del MPU6050 (crudo). |
| 12 | `GY` | `int` | `int16` | Giroscopio Y del MPU6050 (crudo). |
| 13 | `GZ` | `int` | `int16` | Giroscopio Z del MPU6050 (crudo). |

Ejemplo de linea:

`610,3846,0,125,3842,-30,585,8500,1500,-13200,-50,140,-160`

Interpretacion rapida:
- `HS_X10=610` -> `61.0%` de humedad de suelo.
- `TA_X10=-30` -> `-3.0C`.
- `HA_X10=585` -> `58.5%` de humedad ambiente.

Nota:
- La trama UART actual no incluye encabezado, version, ni checksum; solo CSV + `\n`.

### 7.2 Trama cruda del DHT22 (40 bits = 5 bytes)
Antes de armar la trama UART, el DHT22 entrega 5 bytes:

| Byte | Contenido |
|---|---|
| `datos[0]` | Humedad alta |
| `datos[1]` | Humedad baja |
| `datos[2]` | Temperatura alta (7 bits) + bit de signo (bit 7) |
| `datos[3]` | Temperatura baja |
| `datos[4]` | Checksum (`datos[0] + datos[1] + datos[2] + datos[3]`, 8 bits) |

Decodificacion usada en firmware:
- `hum_cruda = (datos[0] << 8) | datos[1]`
- `humedad_ambiente_pct = hum_cruda / 10.0`
- `temp_cruda = ((datos[2] & 0x7F) << 8) | datos[3]`
- `temperatura_ambiente_c = temp_cruda / 10.0`
- Si `(datos[2] & 0x80) != 0`, la temperatura se vuelve negativa.

Ejemplo para `-3.0C`:
- Magnitud `3.0C` -> `30` en escala x10 (`0x001E`).
- Se activa bit de signo en `datos[2]`.
- Valores tipicos: `datos[2] = 0x80`, `datos[3] = 0x1E`.
- Resultado final en UART: `TA_X10 = -30`.

---

## 8. Parametros calibrables recomendados
### Lluvia AO
- `lluvia_ao_adc_ref_seco`
- `lluvia_ao_adc_ref_mojado`
- `umbral_lluvia_leve_pct`
- `umbral_lluvia_media_pct`
- `umbral_lluvia_fuerte_pct`

### Riesgo
- `umbral_humedad_alta_pct`
- `umbral_humedad_saturada_pct`
- `umbral_vibracion_gyro_abs`

---

## 9. Proximos pasos sugeridos
1. Ajustar umbrales con datos reales en campo (seco, llovizna, lluvia fuerte, viento).
2. Definir protocolo de aplicacion (JSON/CSV versionado).
3. Implementar checksum de trama o delimitador robusto.
4. Agregar timestamp y/o ID de nodo.
5. Mover logica final de alerta al cliente (app), manteniendo firmware como fuente de datos crudos + flags basicas.
