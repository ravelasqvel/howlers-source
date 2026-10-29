# Autosintonizador PID con Redes LSTM y Banco de Pruebas (`bench_pid_autotuning`)

**Plataforma:** ESP32-S3 (Dual-Core Xtensa LX7 @ 240 MHz, ESP-IDF + FreeRTOS @ 1 kHz)  
**Repositorio Monorepo:** [`ravelasqvel/howlers-source`](https://github.com/ravelasqvel/howlers-source) (`firmware/apps/bench_pid_autotuning/`)  
**Repositorio de Origen:** [`nparra-code/pid-autotuning`](https://github.com/nparra-code/pid-autotuning) · *Revisión de integración: 2026-10-27*

Este documento unifica la documentación general de la aplicación **`bench_pid_autotuning`** con la **Guía completa de integración del autosintonizador PID con LSTM** y el **análisis de arquitectura de tareas FreeRTOS**, pensado para cualquier desarrollador o estudiante que tenga un firmware de robot completo (como **Striker 1**, **Striker 2** o **Striker v3**) y quiera usar esta aplicación para sintonizar los controladores PID de las 3 ruedas omnidireccionales.

---

## Tabla de Contenidos

1. [¿Qué es y qué vas a integrar?](#1-qué-es-y-qué-vas-a-integrar)
2. [Flujo de una Sesión de Autosintonización](#2-flujo-de-una-sesión-de-autosintonización)
3. [Mapa del Repositorio: Qué Portar y Qué No](#3-mapa-del-repositorio-qué-portar-y-qué-no)
4. [Análisis de Diagramas de Tareas FreeRTOS y Concurrencia](#4-análisis-de-diagramas-de-tareas-freertos-y-concurrencia)
5. [Compatibilidad: Léelo Antes de Escribir Código](#5-compatibilidad-léelo-antes-de-escribir-código)
6. [Servidor Python de Inferencia (`python_server.py`)](#6-servidor-python-de-inferencia-python_serverpy)
7. [Integración Paso a Paso en tu Firmware](#7-integración-paso-a-paso-en-tu-firmware)
8. [Protocolo de Red TCP (Referencia Binaria)](#8-protocolo-de-red-tcp-referencia-binaria)
9. [Modelos LSTM Incluidos y Contratos de Entrada/Salida](#9-modelos-lstm-incluidos-y-contratos-de-entradasalida)
10. [Métricas de Desempeño del Servidor](#10-métricas-de-desempeño-del-servidor)
11. [Checklist de Puesta en Marcha (Paso a Paso)](#11-checklist-de-puesta-en-marcha-paso-a-paso)
12. [Problemas Conocidos del Código de Referencia](#12-problemas-conocidos-del-código-de-referencia)
13. [Solución de Problemas (Troubleshooting)](#13-solución-de-problemas-troubleshooting)
14. [Apéndices: Herramientas de Validación (`fake_robot.py` y `check_csv.py`)](#14-apéndices-herramientas-de-validación-fake_robotpy-y-check_csvpy)

---

## 1. ¿Qué es y qué vas a integrar?

El robot ejecuta una trayectoria de prueba, envía por **TCP** cómo la siguió a un PC, y una red neuronal **LSTM** devuelve las ganancias $(K_p, K_i, K_d)$ de cada una de las 3 ruedas (9 valores en total). El robot las aplica y repite la trayectoria para verificar el desempeño.

| Pieza | Dónde corre | Qué haces tú |
| :--- | :--- | :--- |
| **Cliente de telemetría (`telemetry.c`, `telemetry.h`)** | ESP32-S3 | Copiarlo a tu proyecto y alimentarlo con tus datos de rueda (`state`, `setpoint`). |
| **Capa de pegamento (*"hooks"*: `autotune_port.c`, `autotune_port.h`)** | ESP32-S3 | Escribir **4 funciones callback** que lo conectan a tu firmware (ver [§7.5](#75-capa-de-pegamento-código-listo-para-copiar)). |
| **Servidor de inferencia (`python_server.py` + `models/*.h5`)** | PC en la misma red WiFi | Instalar dependencias, configurar y ejecutar (ver [§6](#6-servidor-python-de-inferencia-python_serverpy)). |

> **Principio clave:** **No tocas tu lazo de control.** Solo lees `(estado, setpoint)` de cada rueda a 1 kHz, los envías en lotes por TCP y recibes 9 números para escribir en las estructuras de tu PID cuando el robot se detiene entre pasadas.

---

## 2. Flujo de una Sesión de Autosintonización

```text
Robot (ESP32-S3)                                     Servidor (PC, Python - Puerto 8888/TCP)
─────────────────                                    ───────────────────────────────────────
Run 1: trayectoria con tus ganancias iniciales
  ── DATA_BATCH (100 muestras) x N ────────────────► acumula muestras y verifica CRC32
  ── INFERENCE_REQUEST (0x06) ─────────────────────► construye ventanas (20x15), corre LSTM -> 9 ganancias
  ◄── PID_RESPONSE (0x02, converged = 0) ───────────
aplica las 9 ganancias en los 3 PIDs

Run 2: misma trayectoria (verificación "after")
  ── DATA_BATCH (100 muestras) x N ────────────────► guarda log "after", genera gráfica comparativa PNG
  ── INFERENCE_REQUEST (0x06) ─────────────────────►
  ◄── PID_RESPONSE (0x02, converged = 1) ───────────
aplica ganancias y termina (cierra socket TCP)
```

Con la configuración por defecto (`RUNS_TO_CONVERGE = 2`) hay **una sola inferencia real**. La segunda respuesta repite las mismas ganancias; el segundo *run* sirve para medir el *"después"* y generar la gráfica comparativa. Si subes `RUNS_TO_CONVERGE`, habrá más iteraciones (pero revisa el desfase de contabilidad detallado en [§12](#12-problemas-conocidos-del-código-de-referencia)).

---

## 3. Mapa del Repositorio: Qué Portar y Qué No

En el monorepo [`ravelasqvel/howlers-source`](https://github.com/ravelasqvel/howlers-source), los archivos originales de `pid-autotuning-firmware/` están distribuidos entre `firmware/apps/bench_pid_autotuning/`, `firmware/components/` y `software/ground_station/autotuning_servers/`:

| Ruta Original (`pid-autotuning-firmware/`) | Ubicación en Monorepo (`howlers-source`) | Rol | ¿Lo necesitas para sintonizar tu robot? |
| :--- | :--- | :--- | :--- |
| `main/telemetry.c`, `main/include/telemetry.h` | `firmware/apps/bench_pid_autotuning/main/` | Cliente TCP, protocolo binario, CRC32 | **Sí, tal cual** |
| `python_server.py` | `software/ground_station/autotuning_servers/python_server.py` | Servidor de inferencia TCP (puerto 8888) | **Sí** |
| `models/*.h5` | `firmware/apps/bench_pid_autotuning/models/` *(o `components/control/pid_lib/autotuning_models/`)* | 4 modelos LSTM entrenados en Keras | **Sí (al menos uno)** |
| `drivers/pid_lib/` | `firmware/components/control/pid_lib/autotuning_rnn/` | PID discreto (`pid_ext.c`) con el que se entrenó el modelo | **Solo si tu PID es distinto** y quieres adoptar el mismo ([§5](#5-compatibilidad-léelo-antes-de-escribir-código)) |
| `main/firmware_main.c` | `firmware/apps/bench_pid_autotuning/main/firmware_main.c` | Ejemplo: WiFi, `autotuning_task`, `apply_pid_constants` | **Referencia.** No copiar tal cual ([§12](#12-problemas-conocidos-del-código-de-referencia)) |
| `main/control_main.c`, `main/mov_calculation.c` | `firmware/apps/bench_pid_autotuning/main/` | Control y trayectoria de excitación del robot de referencia | **Referencia** ([§7.4](#74-trayectoria-de-excitación) y [§7.6](#76-qué-debe-pasar-en-tu-lado-en-cada-run)) |
| `drivers/bldc_pwm`, `sensors/AS5600_HAL_Library`, `platform`, `drivers/sensor_fusion` | `firmware/components/` | Hardware específico del robot de referencia | **No** (tu proyecto ya tiene sus propios drivers de rueda/PWM) |
| `data_logger_server.py` | `software/ground_station/autotuning_servers/data_logger_server.py` | Guarda telemetría en CSV sin correr inferencia | **Útil para validar la integración** ([§11](#11-checklist-de-puesta-en-marcha-paso-a-paso)) |
| `analyze_telemetry.py` | `software/ground_station/autotuning_servers/analyze_telemetry.py` | Genera gráficas a partir de un CSV | Opcional |
| `motor_ident_server.py`, `view_motor_ident.py` | `software/ground_station/autotuning_servers/` | Identificación abierta de motor (puerto 8889) | Opcional (genera constantes clásicas) |

---

## 4. Análisis de Diagramas de Tareas FreeRTOS y Concurrencia

Para integrar este autosintonizador en otro proyecto (como **Striker 2** o **Striker v3**) sin degradar el lazo de control ni corromper las muestras temporales que espera la red LSTM, es fundamental comprender cómo interactúan las tareas en el firmware de referencia y cómo deben acoplarse en el tuyo.

### 4.1. Arquitectura de Tareas en el Firmware de Referencia (`bench_pid_autotuning`)

En el diseño de referencia, las prioridades y afinidades de núcleo siguen una jerarquía estricta donde **la telemetría nunca puede expropiar (*preempt*) al lazo de control ni a la lectura de sensores**:

```text
===================================================================================================
                        DIAGRAMA DE TAREAS Y PRIORIDADES FREERTOS (1 kHz)
===================================================================================================

 [ CORE 1 (o sin anclar): LAZO DE TIEMPO REAL DURO ]
 --------------------------------------------------------------------------------------------------
  Prioridad 9 (Máxima) : vTaskControl (Rueda R, L, B)   ───► Periodo Ts = 1 ms (1 kHz)
                         • Ejecuta trayectoria de excitación (setpoint_m[k])
                         • Calcula ley de control discreta pid_calc_discrete() -> u[k]
                         • Actualiza ciclo útil PWM en variadores ESC (bldc_pwm)
                                  ▲
                                  │ Estado filtrado: ω_m[k] = Δθ / Δt [rad/s]
                                  │
  Prioridad 8 (Alta)   : vTaskEncoder (AS5600 R, L, B)  ───► Muestreo de alta frecuencia
                         • Lee ángulo magnético por ADC / I2C
                         • Estima velocidad angular y filtra ruido de derivación
                                  │
 =================================│================================================================
                                  │ Lectura de (state[3], setpoint[3]) vía hooks
                                  ▼
 [ CORE 0 (PRO_CPU): TELEMETRÍA Y AUTOSINTONIZACIÓN TCP/WiFi ]
 --------------------------------------------------------------------------------------------------
  Prioridad 7 (Media)  : autotuning_task (Stack: 8192 B, anclada a Core 0)
                         • Llama a autotune_run(server_ip, &HOOKS)
                         • Muestrea cada 1 ms con vTaskDelay(pdMS_TO_TICKS(1))
                         • Empaqueta 100 muestras (6400 B) -> telemetry_send_batch() por TCP
                         • Al terminar la trayectoria -> telemetry_request_inference()
                         • Bloquea hasta 30 s en telemetry_receive_pid()
                         • Con el robot detenido -> invoca apply_gains(kp, ki, kd)
                                  │
                                  ▼
  Stack LwIP / WiFi    : Transmisión TCP/IP (Puerto 8888) <───► PC (python_server.py + LSTM)
===================================================================================================
```

### 4.2. Hallazgos Críticos del Análisis de Tareas (Qué debes cuidar en tu proyecto)

1. **Reloj de FreeRTOS obligatorio a 1000 Hz (`CONFIG_FREERTOS_HZ=1000`):**
   * Tanto el lazo de control como `autotune_run()` muestrean cada **1 ms** usando `vTaskDelay(pdMS_TO_TICKS(1))`.
   * Por defecto en ESP-IDF, `CONFIG_FREERTOS_HZ` vale `100` (1 tick = 10 ms), lo que hace que `pdMS_TO_TICKS(1)` evalúe a **`0` ticks**. Si olvidas cambiarlo a `1000` en `sdkconfig`, `vTaskDelay(0)` equivale a un simple `taskYIELD()`, haciendo que la tarea de telemetría corra sin pausa acaparando la CPU y enviando muestras separadas por microsegundos en lugar de 1 ms.
2. **Defecto de concurrencia en `multiple_movements()` del código de referencia:**
   * En `control_main.c` del repositorio original, existen **3 tareas de control** (una por rueda) y las tres invocan la función generadora de trayectoria `multiple_movements()`. Dado que dicha función guarda su contador de tiempo en una variable `static` interna sin distinguir qué rueda la llama, **el reloj de la trayectoria avanza 3 veces por cada milisegundo** (reduciendo la duración real de los tramos a ~1/3 de lo nominal).
   * **Cómo corregirlo en tu proyecto:** Actualiza el generador de trayectoria desde **una única tarea coordinadora de cinemática** a 1 kHz (o calcula el tiempo usando `xTaskGetTickCount()`), la cual publique los 3 *setpoints* $(\omega_R, \omega_L, \omega_B)$ de forma atómica.
3. **Huecos de muestreo durante `telemetry_send_batch()`:**
   * La tarea `autotuning_task` recolecta 100 muestras (100 ms) y luego llama a `telemetry_send_batch()`, que realiza una escritura bloqueante en el socket TCP antes de volver al bucle de 1 ms. Durante los milisegundos que tarda el envío TCP, el robot sigue moviéndose, por lo que entre la muestra 99 de un lote y la muestra 0 del siguiente hay un pequeño salto temporal (~19 de cada 100 ventanas deslizantes de 20 muestras cruzan una frontera de lote). Los modelos incluidos toleran este comportamiento porque fueron entrenados bajo el mismo esquema, pero anclar `autotuning_task` al **Core 0** y el control al **Core 1** garantiza que el bloqueo de red jamás retrase la actualización del PWM.
4. **Sincronización segura al leer `(state, setpoint)` y al escribir `(Kp, Ki, Kd)`:**
   * `read_wheels()` es llamada desde el Core 0 a 1 kHz mientras el Core 1 actualiza `state` y `setpoint`. Asegúrate de copiar los 3 flotantes de forma atómica (con un `portMUX_TYPE` spinlock muy breve o leyendo variables alineadas de 32 bits).
   * `apply_gains()` solo se invoca cuando `trajectory_done()` devuelve `true` (con las ruedas detenidas entre el Run 1 y el Run 2), evitando transitorios bruscos en mitad de la maniobra.

---

## 5. Compatibilidad: Léelo Antes de Escribir Código

Las ganancias que devuelve el modelo LSTM **solo tienen sentido físico si tu lazo de control se parece al lazo con el que se entrenó la red**. Verifica cada fila de esta tabla antes de integrar:

| Aspecto | Referencia (lo que vio el modelo en entrenamiento) | Qué debes verificar en tu robot |
| :--- | :--- | :--- |
| **Plataforma física** | ESP32-S3, 3 ruedas omni a 120°, motores BLDC por ESC (PWM 50 Hz), encoder AS5600 por salida analógica/ADC. | Con otro hardware (otra reducción, inercia o ESC), el modelo puede no transferir directamente ([§9.5](#95-cuándo-los-modelos-incluidos-aplican)). |
| **Muestreo y control** | **1 kHz (`SAMPLE_TIME = 1 ms`)**. La telemetría muestrea cada `vTaskDelay(pdMS_TO_TICKS(1))`, por lo que exige `CONFIG_FREERTOS_HZ=1000`. | Tu lazo de velocidad y tu muestreo de telemetría deben correr exactamente a **1 kHz**. |
| **Variable controlada** | Velocidad angular de cada rueda, $\Delta\text{ángulo}[\text{rad}] / \Delta t[\text{s}]$ filtrada (`encoder_data_t.state`). | Mismas unidades ($\text{rad/s}$). `state` y `setpoint` deben estar en la misma escala y signo. |
| **Orden de ruedas** | **Índice `0` = Derecha (`R`), `1` = Izquierda (`L`), `2` = Trasera (`B`)**. | Respétalo estrictamente tanto al llenar la telemetría como al aplicar las ganancias recibidas. |
| **Estructura del PID** | `PID_CAL_TYPE_DISCRETE` de `pid_ext.c` (ecuación abajo), $\beta = 0.4$, `sample_time = 1`, salida saturada a **$\pm 80$**. | Si tu PID usa otra ecuación o saturación, las ganancias **no son directamente válidas**. |
| **Magnitud de ganancias** | Iniciales de referencia: $K_p \in [2.47, 2.86]$, $K_i \in [0.0019, 0.0033]$, $K_d \in [0.0019, 0.0034]$. | Tus ganancias iniciales antes del Run 1 deben ser de orden de magnitud similar. |
| **Rango de señales** | Con la trayectoria de referencia, $|\text{setpoint}|$ llega a **~17 rad/s** en tramos lineales y **~20 rad/s** en el circular (calculado desde `movements[]` y `cal_lin_to_ang_velocity`). | Usa una trayectoria con amplitud y dinámica similares. **No hay normalización** en el servidor ([§9.2](#92-contrato-de-entrada)). |

### Ecuación del PID Discreto de Referencia (`pid_calc_discrete`)

Con $T_s = \text{sample\_time} = 1$ y filtro derivativo $\beta = \text{beta} = 0.4$:

```text
a0 = 1 + β·Ts          a1 = -(2 + β·Ts)          a2 = 1
b0 = Kp(1 + β·Ts) + Ki·Ts(1 + β·Ts) + Kd·β
b1 = -Kp(2 + β·Ts) + Ki·Ts + 2·Kd·β
b2 = Kp + Kd·β

u[k] = ( b0·e[k] + b1·e[k-1] + b2·e[k-2] - a1·u[k-1] - a2·u[k-2] ) / a0    (saturado a ±80)
```

> **¿Qué hacer si tu PID tiene otra forma?** (por ejemplo, forma paralela estándar, $T_s$ en segundos como `0.001f`, o salida en cuentas PWM de `0..1000`): la solución más directa y segura es **adoptar `pid_ext` (`drivers/pid_lib/`)** para las 3 ruedas de tu robot.

---

## 6. Servidor Python de Inferencia (`python_server.py`)

### 6.1. Instalación de Dependencias

El archivo `requirements.txt` original del repositorio **no incluye `tensorflow` ni `keras`**, y el servidor los necesita siempre al arrancar (incluso si usas el modo clásico `MODE = 1`):

```bash
# En el monorepo, ubica los modelos .h5 junto a python_server.py o ejecuta desde la carpeta que contenga models/
cd software/ground_station/autotuning_servers
python3 -m venv .venv
source .venv/bin/activate          # En Windows PowerShell: .\.venv\Scripts\Activate.ps1
pip install numpy matplotlib pandas tensorflow keras
```

* **Compatibilidad de versiones:** Los archivos `.h5` fueron guardados con **Keras 3.10.0** (backend TensorFlow). Está verificado que cargan y corren sin problemas con **TensorFlow 2.21.0 + Keras 3.15.1 en Python 3.13**. Evita usar Keras 2 o versiones antiguas de TensorFlow.

### 6.2. Ejecución del Servidor

```bash
# IMPORTANTE: El directorio desde donde ejecutas python_server.py debe contener la carpeta models/*.h5
# Si usas el monorepo, asegúrate de copiar o enlazar la carpeta models/ dentro de autotuning_servers/
cd software/ground_station/autotuning_servers
python3 python_server.py
```

Debes ejecutarlo desde la carpeta que contenga `models/<modelo>.h5`, ya que el servidor usa rutas relativas para cargar el modelo y crear las carpetas `autotuning_logs_new/` y `autotuning_plots_new/`.
* **Salida esperada al arrancar:** `Telemetry server initialized...` seguido de `Server listening on port 8888`.

### 6.3. Constantes que Debes Revisar en `python_server.py`

| Constante | Valor por defecto | Qué hace y cómo ajustarla |
| :--- | :--- | :--- |
| **`MODE`** | `1` | `0` = devuelve las ganancias predichas por la red **LSTM**.<br>`1` = devuelve `CLASSICAL_PID_CONSTANTS` (valores fijos). **Cámbialo a `0` para usar la red neuronal.** *(En modo `1` la LSTM igual se carga y ejecuta, pero su salida se descarta).* |
| **`model_name`** | `lstm64tanh_32relu_exp_1401` | Nombre del modelo a cargar desde `models/` (ver comparativa en [§9.1](#91-modelos-incluidos)). |
| **`RUNS_TO_CONVERGE`** | `2` | Número de pasadas antes de enviar `converged = 1` (ver [§2](#2-flujo-de-una-sesión-de-autosintonización)). Mantén `2`. |
| **`PORT`** | `8888` | Puerto TCP de escucha. Debe coincidir con `TELEMETRY_SERVER_PORT` del firmware. |
| **`CLASSICAL_PID_CONSTANTS`** | `[8.38, 6.53, 3.57, 0.05, 0.08, 0.09, 0.008, 0.013, 0.016]` | Orden: `[Kp R,L,B, Ki R,L,B, Kd R,L,B]`. Solo se envían cuando `MODE = 1`. |
| **`pid_constants_before`** | Fijo dentro de `handle_client` | Ganancias iniciales usadas únicamente para nombrar archivos CSV y títulos de gráficas. Si cambias tus ganancias iniciales en el robot, actualízalo aquí. |

### 6.4. Configuración de Red y Firewall
* El servidor escucha en `0.0.0.0:8888`.
* El robot (ESP32-S3) y el PC deben estar en la **misma subred WiFi 2.4 GHz**.
* **Abre el puerto `8888/TCP` en el firewall de Windows/Linux** del PC y asigna una IP estática (o reserva DHCP) al PC.
* La dirección IP se pasa a `telemetry_init(server_ip)` y se copia en un arreglo `char[16]`: máximo **15 caracteres** (`xxx.xxx.xxx.xxx`).

### 6.5. Qué Hace el Servidor por cada Mensaje y Qué Escribe a Disco

| Mensaje TCP Recibido | Acción del Servidor |
| :--- | :--- |
| **`DATA_BATCH` (`0x01`)** | Verifica el CRC32, desempaqueta las muestras de 64 B y las acumula en memoria. No envía respuesta. |
| **`INFERENCE_REQUEST` (`0x06`), fase `"before"`** | Guarda las muestras acumuladas como `"before"` (Run 1), construye ventanas deslizantes de $20 \times 15$, ejecuta la LSTM, promedia las predicciones y responde con `PID_RESPONSE` (`converged = 0`). |
| **`INFERENCE_REQUEST` (`0x06`), fase `"after"`** | Guarda las muestras del Run 2 como `"after"`, escribe los CSV/JSON a disco, genera la gráfica comparativa `.png`, responde con las ganancias finales y `converged = 1`, y reinicia su estado para otra sesión. |

**Archivos generados en disco:**
* `autotuning_logs_new/autotuning_<ts>_before_Kp[..]_Ki[..]_Kd[..].csv` (más `_raw.bin` y `_meta.json`): Telemetría del Run 1 con las ganancias iniciales.
* `autotuning_logs_new/autotuning_<ts>_after_....csv` (más `_raw.bin` y `_meta.json`): Telemetría del Run 2 con las nuevas ganancias.
* `autotuning_plots_new/pid_tuning_comparison_<ts>.png`: Gráfica comparativa antes/después de las 3 ruedas.

> **Advertencia sobre los nombres de archivo en modo RNN (`MODE = 0`):** Los nombres de archivo `*_after_*` y el campo `pid_constants` del JSON muestran las ganancias mal agrupadas (por ejemplo, $K_p$ mezclado con valores de $K_i$ o $K_d$) debido a una doble transposición en el servidor (explicada en [§9.3](#93-contrato-de-salida-y-su-orden)). **Lo que recibe el robot por red sí es 100% correcto.** Confía siempre en el log serial del robot (`Applying PID constants`).

### 6.6. Rendimiento y Límites del Servidor
* **Tiempos de inferencia (medidos en CPU de 2 vCPU):**
  * `lstm64tanh_32relu`: **0.84 s** (5 000 ventanas), **2.0 s** (20 000 ventanas), **4.7 s** (50 000 ventanas).
  * `lstm128tanh_32relu`: **1.3 s** (5 000 ventanas), **2.9 s** (20 000 ventanas), **7.2 s** (50 000 ventanas).
  * El firmware espera hasta **30 segundos** (`telemetry_receive_pid(..., 30000)`), por lo que hay margen amplio.
* **Límites operativos:**
  1. **Un solo cliente a la vez:** Guarda el estado de la sesión en variables globales. Si quieres sintonizar varios robots en paralelo, lanza un proceso por robot en puertos distintos.
  2. **Mínimo 21 muestras por inferencia:** Un *run* de $N$ muestras genera $N - 20$ ventanas. Con $\le 20$ muestras el servidor lanza `UnboundLocalError: cannot access local variable 'batch_outputs'` y cierra la conexión.
  3. **Aplica `abs()` a las salidas:** Si la red predice un valor negativo, el servidor lo vuelve positivo sin aviso. Por eso es obligatorio usar el filtro de seguridad (*envelope*) en el firmware ([§7.7](#77-aplicar-ganancias-con-seguridad-envelope)).
  4. **Reinicio tras aborto:** Si el robot se reinicia a mitad de sesión, **reinicia también `python_server.py`** para limpiar los buffers globales `before`/`after`.

---

## 7. Integración Paso a Paso en tu Firmware

### 7.1. Añadir el Módulo `telemetry` a tu Proyecto ESP-IDF
1. Copia `telemetry.c` y `telemetry.h` (ubicados en `firmware/apps/bench_pid_autotuning/main/`) a tu proyecto o componente compartido.
2. En el `CMakeLists.txt` de tu aplicación (directiva `REQUIRES`), asegúrate de incluir:
   ```cmake
   REQUIRES driver freertos esp_timer nvs_flash esp_adc esp_wifi esp_netif lwip
   ```
3. En tu `sdkconfig` (o mediante `idf.py menuconfig` $\rightarrow$ *Component config* $\rightarrow$ *FreeRTOS* $\rightarrow$ *Kernel*):
   ```ini
   CONFIG_FREERTOS_HZ=1000
   ```

### 7.2. Gestión de WiFi y Dirección IP del Servidor
`telemetry.c` asume que el ESP32-S3 **ya está conectado a la red WiFi en modo estación (STA)**. Si tu firmware de Striker ya inicializa el WiFi, **no copies `wifi_init()` de `firmware_main.c`**. Pasa el SSID, la contraseña y la IP del servidor mediante `Kconfig`/`menuconfig` o NVS (nunca subas credenciales de redes reales quemadas con `#define` a Git).

### 7.3. Contrato de Datos: Estructura `robot_sample_t`
Cada muestra se toma cada **1 ms** y ocupa exactamente **64 bytes** (`little-endian`, `__attribute__((packed))`):

| Campo | Tipo en C | Tamaño | Contenido y Reglas |
| :--- | :--- | :---: | :--- |
| `timestamp_ms` | `uint32_t` | 4 B | Tiempo en ms (`xTaskGetTickCount() * portTICK_PERIOD_MS`). |
| `motor_state[3]` | `float[3]` | 12 B | Velocidad angular medida de cada rueda en orden **`[R, L, B]`** ($\text{rad/s}$). |
| `motor_setpoint[3]` | `float[3]` | 12 B | Referencia (*setpoint*) de velocidad de cada rueda en orden **`[R, L, B]`** ($\text{rad/s}$). |
| `errors[m][0]` | `float` ($\times 3$) | 12 B | Error actual $e_m(k) = \text{setpoint}_m(k) - \text{state}_m(k)$ de la rueda $m$. |
| `errors[m][1]` | `float` ($\times 3$) | 12 B | Error de la muestra anterior $e_m(k-1)$ (hace 1 ms). |
| `errors[m][2]` | `float` ($\times 3$) | 12 B | Error de dos muestras atrás $e_m(k-2)$ (hace 2 ms). |

* **Nota importante sobre `errors[3][3]`:** En memoria, el arreglo bidimensional es `errors[rueda][retardo]`, es decir, primero van $[e_R(k), e_R(k-1), e_R(k-2)]$, luego los 3 de `L` y luego los 3 de `B`. Además, se calcula directamente como `setpoint - state` con los mismos valores enviados en el paquete.

### 7.4. Trayectoria de Excitación
El modelo LSTM infiere las ganancias observando la respuesta transitoria del robot ante cambios de consigna. La trayectoria de excitación de referencia (`vTaskControl` en `control_main.c`) consta de 4 tramos:

| Tramo # | Tipo de Movimiento | Dirección | Velocidad Lineal Chasis | Ángulo | Duración / Geometría |
| :---: | :--- | :--- | :---: | :---: | :--- |
| **1** | `LINEAR` | Adelante | $12\text{ cm/s}$ | $90^\circ$ | $10\text{ s}$ |
| **2** | `LINEAR` | Adelante | $10\text{ cm/s}$ | $0^\circ$ | $5\text{ s}$ |
| **3** | `LINEAR` | Atrás | $20\text{ cm/s}$ | $0^\circ$ | $7\text{ s}$ |
| **4** | `CIRCULAR` | Horario | $5\text{ cm/s}$ | $360^\circ$ | Radio $20\text{ cm}$ |

> **Recomendación:** Usa esta misma secuencia (o una con escalones, inversión de giro y tramo curvo de amplitud comparable: $|\text{setpoint}| \approx 17\text{–}20\text{ rad/s}$). **Realiza siempre las primeras pruebas con el robot sobre un soporte (ruedas en el aire).**

### 7.5. Capa de Pegamento (Código Listo para Copiar en tu Proyecto)

Copia estos dos archivos (`autotune_port.h` y `autotune_port.c`) en tu firmware. Esta capa corrige además el bug de variables locales sin inicializar en $e(k-1)$ y $e(k-2)$ del código de referencia.

#### `autotune_port.h`
```c
#pragma once
#include <stdbool.h>
#include "esp_err.h"

typedef struct {
    /* 1. Lee estado y setpoint de las 3 ruedas, orden [R, L, B], misma unidad (rad/s). */
    void (*read_wheels)(float state[3], float setpoint[3]);
    /* 2. Arranca (o reinicia desde cero) la trayectoria de excitación. */
    void (*trajectory_start)(void);
    /* 3. Devuelve true cuando la trayectoria terminó y el robot ya está detenido. */
    bool (*trajectory_done)(void);
    /* 4. Aplica las nuevas ganancias con TU API de PID (incluyendo validación §7.7). */
    void (*apply_gains)(const float kp[3], const float ki[3], const float kd[3]);
} autotune_hooks_t;

/* Función bloqueante: corre la sesión completa (Run 1 + Inferencia + Run 2).
 * Llamar desde una tarea dedicada en el Core 0 con prioridad menor al control. */
esp_err_t autotune_run(const char *server_ip, const autotune_hooks_t *hooks);
```

#### `autotune_port.c`
```c
#include <string.h>
#include "autotune_port.h"
#include "telemetry.h"

static float s_e1[3], s_e2[3]; /* Historial explícito e(k-1), e(k-2) por rueda */

static void fill_sample(robot_sample_t *s, const float st[3], const float sp[3])
{
    s->timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    for (int m = 0; m < 3; m++) {
        float e = sp[m] - st[m];
        s->motor_state[m]    = st[m];
        s->motor_setpoint[m] = sp[m];
        s->errors[m][0] = e;        /* e(k)   */
        s->errors[m][1] = s_e1[m];  /* e(k-1) */
        s->errors[m][2] = s_e2[m];  /* e(k-2) */
        s_e2[m] = s_e1[m];
        s_e1[m] = e;
    }
}

esp_err_t autotune_run(const char *server_ip, const autotune_hooks_t *h)
{
    pid_response_t resp = {0};
    float st[3], sp[3];
    robot_sample_t s;

    if (telemetry_init(server_ip) != ESP_OK) return ESP_FAIL;
    vTaskDelay(pdMS_TO_TICKS(2000)); /* Esperar estabilidad de red */
    if (telemetry_connect() != ESP_OK) return ESP_FAIL;

    do {
        memset(s_e1, 0, sizeof s_e1);
        memset(s_e2, 0, sizeof s_e2);
        h->trajectory_start();

        while (!h->trajectory_done()) { /* 1) Recolectar ventana de 100 muestras y enviar */
            for (int i = 0; i < TELEMETRY_SAMPLE_WINDOW; i++) {
                h->read_wheels(st, sp);
                fill_sample(&s, st, sp);
                if (telemetry_add_sample(&s) != ESP_OK) break;
                vTaskDelay(pdMS_TO_TICKS(1)); /* Muestreo a 1 kHz */
            }
            if (telemetry_send_batch() != ESP_OK) {
                telemetry_disconnect();
                return ESP_FAIL;
            }
        }

        if (telemetry_request_inference() != ESP_OK ||       /* 2) Pedir inferencia */
            telemetry_receive_pid(&resp, 30000) != ESP_OK) { /* 3) Esperar respuesta (30 s máx) */
            telemetry_disconnect();
            return ESP_FAIL;
        }

        float kp[3], ki[3], kd[3]; /* Copia segura desde estructura packed */
        for (int m = 0; m < 3; m++) {
            kp[m] = resp.kp[m];
            ki[m] = resp.ki[m];
            kd[m] = resp.kd[m];
        }
        h->apply_gains(kp, ki, kd); /* 4) Validar y aplicar nuevas ganancias */
    } while (!resp.converged);

    telemetry_disconnect();
    return ESP_OK;
}
```

#### Ejemplo de conexión en tu aplicación (`main.c`):
```c
static void hk_read(float st[3], float sp[3]) {
    st[0] = right_wheel_rad_s();  sp[0] = right_wheel_setpoint();
    st[1] = left_wheel_rad_s();   sp[1] = left_wheel_setpoint();
    st[2] = back_wheel_rad_s();   sp[2] = back_wheel_setpoint();
}

static const autotune_hooks_t HOOKS = {
    .read_wheels      = hk_read,
    .trajectory_start = my_traj_start,
    .trajectory_done  = my_traj_done,
    .apply_gains      = my_apply_gains,
};

static void autotune_task(void *arg) {
    esp_err_t r = autotune_run("192.168.0.50", &HOOKS); /* IP del PC */
    ESP_LOGI("AT", "autotune_run -> %s", esp_err_to_name(r));
    vTaskDelete(NULL);
}

/* Lanzar bajo demanda (por comando UDP/Web o botón), en Core 0 con prioridad 7 (< control):
   xTaskCreatePinnedToCore(autotune_task, "autotune", 8192, NULL, 7, NULL, 0); */
```

### 7.6. Qué Debe Pasar en tu Lado en Cada Run
1. `trajectory_start()` prepara el robot y reinicia el contador de tiempo de la trayectoria de excitación a cero.
2. `read_wheels()` devuelve los últimos valores de velocidad medida y referencia de las 3 ruedas `[R, L, B]` a 1 kHz.
3. `trajectory_done()` pasa a `true` cuando finalizan los 4 tramos; **en ese instante el robot debe quedar completamente detenido ($\text{setpoint} = 0$)**.
4. `apply_gains()` se ejecuta con el robot detenido antes de iniciar el Run 2.

### 7.7. Aplicar Ganancias con Seguridad (*Envelope* de Validación)
Dado que el servidor no acota los valores predichos por la red neuronal (solo aplica `abs()`), **siempre debes validar que las ganancias recibidas sean finitas y estén dentro de un rango seguro** antes de escribirlas en tus controladores PID:

```c
#include <math.h>
#include <stdbool.h>

/* Envelope de seguridad por rueda [R, L, B] (~0.25x a 4x de las ganancias iniciales de referencia) */
static const float KP_MIN[3] = {0.6f, 0.6f, 0.7f},   KP_MAX[3] = {10.0f, 10.0f, 11.0f};
static const float KI_MIN[3] = {0.0f, 0.0f, 0.0f},   KI_MAX[3] = {0.01f, 0.013f, 0.009f};
static const float KD_MIN[3] = {0.0f, 0.0f, 0.0f},   KD_MAX[3] = {0.008f, 0.009f, 0.014f};

static bool gains_ok(const float kp[3], const float ki[3], const float kd[3])
{
    for (int m = 0; m < 3; m++) {
        if (!isfinite(kp[m]) || !isfinite(ki[m]) || !isfinite(kd[m])) return false;
        if (kp[m] < KP_MIN[m] || kp[m] > KP_MAX[m]) return false;
        if (ki[m] < KI_MIN[m] || ki[m] > KI_MAX[m]) return false;
        if (kd[m] < KD_MIN[m] || kd[m] > KD_MAX[m]) return false;
    }
    return true;
}

void my_apply_gains(const float kp[3], const float ki[3], const float kd[3])
{
    if (!gains_ok(kp, ki, kd)) {
        /* Conserva las ganancias actuales y registra el rechazo en el log */
        ESP_LOGW("AT", "Ganancias fuera del envelope seguro; rechazadas.");
        return;
    }
    /* Aplica en tus 3 PIDs (ej. pid_update_parameters sin borrar el integrador):
       for (int m = 0; m < 3; m++) tu_pid_set_gains(m, kp[m], ki[m], kd[m]); */
}
```

---

## 8. Protocolo de Red TCP (Referencia Binaria)

* **Transporte:** TCP, puerto `8888`, formato `little-endian`, estructuras empaquetadas (`packed`).
* **Checksum:** CRC32 estándar (polinomio `0xEDB88320`, idéntico a `zlib.crc32` en Python).
* **Cabecera común (8 bytes, formato `<BBHI`):**
  * `type` (`uint8_t`), `version` (`uint8_t`, siempre `1`), `length` (`uint16_t`, bytes de payload), `checksum` (`uint32_t`, CRC32 del payload).

| `type` | Nombre del Mensaje | Dirección | Estructura del Payload |
| :---: | :--- | :---: | :--- |
| **`0x01`** | `DATA_BATCH` | Robot $\rightarrow$ Servidor | `sample_count` (`uint16_t`) + `sequence_number` (`uint16_t`) + `sample_count` $\times$ **64 B** (`<I15f`). |
| **`0x02`** | `PID_RESPONSE` | Servidor $\rightarrow$ Robot | **38 B** (`<9fBB`): `kp[3]`, `ki[3]`, `kd[3]` (orden `[R, L, B]`), `iteration` (`uint8_t`), `converged` (`uint8_t`). |
| **`0x06`** | `INFERENCE_REQUEST` | Robot $\rightarrow$ Servidor | Sin payload (`length = 0`, `checksum = 0`). |
| **`0x05`** | `IDENT_DATA` | Robot $\rightarrow$ Servidor Ident. | Usado solo por `motor_ident_server.py` en el puerto `8889`. |
| **`0x03`, `0x04`** | `ACK`, `ERROR` | Servidor $\rightarrow$ Robot | Definidos en cabecera; `data_logger_server.py` envía `ACK`, pero `python_server.py` no los usa. |

---

## 9. Modelos LSTM Incluidos y Contratos de Entrada/Salida

### 9.1. Modelos Incluidos en `models/`
Todos comparten la misma topología secuencial en Keras: **1 capa `LSTM (tanh)` $\rightarrow$ `Dense (ReLU)` $\rightarrow$ `Dense (9, lineal)`**, entrenados con función de pérdida MSE:

| Archivo en `models/` | Unidades LSTM | Capa Dense | Parámetros | Tamaño | Notas de Uso |
| :--- | :---: | :---: | :---: | :---: | :--- |
| `lstm64tanh_32relu_exp_1401.h5` | 64 | 32 | 22 857 | 308 KB | Modelo configurado **por defecto** en `python_server.py` (más rápido). |
| `lstm64tanh_64relu_exp_1401.h5` | 64 | 64 | 25 225 | 336 KB | Variante intermedia de 64 unidades recurrentes. |
| **`lstm128tanh_32relu_exp_1401.h5`** | **128** | **32** | **78 153** | **972 KB** | **Reportado como el de mejor desempeño en hardware real.** |
| `lstm128tanh_64relu_exp_1401.h5` | 128 | 64 | 82 569 | 1.0 MB | Modelo de mayor capacidad. |

### 9.2. Contrato de Entrada de la Red
* **Tensor de entrada:** `(n_ventanas, 20, 15)` en `float32`. La longitud de secuencia temporal es de **20 muestras** ($\approx 20\text{ ms}$ a 1 kHz), generadas con ventana deslizante de paso 1 (`X[i : i+20]`).
* **Orden exacto de las 15 características por cada instante de tiempo:**
  * Índices `0–2`: `state` `[R, L, B]`
  * Índices `3–5`: `setpoint` `[R, L, B]`
  * Índices `6–8`: `e(k)` `[R, L, B]`
  * Índices `9–11`: `e(k-1)` `[R, L, B]`
  * Índices `12–14`: `e(k-2)` `[R, L, B]`
* **Sin normalización:** `python_server.py` alimenta las unidades crudas directamente al modelo y promedia las salidas de todas las ventanas del *run*.

### 9.3. Contrato de Salida y Orden de las 9 Ganancias
El vector de salida `(1, 9)` del modelo está ordenado **por parámetro**:
```text
[Kp_R, Kp_L, Kp_B,  Ki_R, Ki_L, Ki_B,  Kd_R, Kd_L, Kd_B]
```
*(Nota técnica: en `python_server.py`, `predict_pid()` reordena erróneamente el vector asumiendo que viene por rueda, pero luego `create_pid_response()` aplica `.T.flatten()`, deshaciendo la permutación antes de enviarlo por red. Por ello el paquete binario que recibe el ESP32-S3 tiene el orden correcto, aunque los nombres de los logs en el servidor muestren los grupos mezclados).*

---

## 10. Métricas de Desempeño del Servidor

Al finalizar cada *run*, `evaluate_pid_performance()` calcula para cada rueda (con $\text{error} = \text{state} - \text{setpoint}$):
* **Oscilación:** $\text{Var}(\Delta \text{error}) / 10$.
* **Error en estado estable:** $\text{Media}(|\text{error}|)$ en el último $30\%$ de muestras del *run*, dividida por $5$.
* **Score por rueda:** $0.6 \times \text{oscilación} + 0.4 \times \text{estado\_estable}$ *(menor es mejor)*.
* **Score global:** Promedio de las 3 ruedas.

---

## 11. Checklist de Puesta en Marcha (Paso a Paso)

Ejecuta estas verificaciones **en estricto orden** sin saltar al paso siguiente hasta validar el anterior:

- [ ] **Paso 1 — Servidor solo (sin robot):** Pon `MODE = 0` en `python_server.py`, arráncalo y ejecuta `python3 fake_robot.py 5` (código en [Apéndice A](#141-apéndice-a-robot-simulado-fake_robotpy)). Debes recibir dos respuestas `RESPONSE`, la primera con `converged: 0` y la segunda con `converged: 1`.
- [ ] **Paso 2 — Telemetría real sin inferencia:** Corre `data_logger_server.py` (puerto 8888) en lugar de `python_server.py`. Envía únicamente lotes con `telemetry_add_sample` + `telemetry_send_batch` (no uses `autotune_run()` aquí porque el logger no responde a `INFERENCE_REQUEST`). Valida el CSV generado con `python3 check_csv.py <archivo.csv>` ([Apéndice B](#142-apéndice-b-verificador-de-sanidad-de-csv-check_csvpy)): verifica que `dt mediano` sea $\approx 1.0\text{ ms}$ y el historial de errores dé `OK`.
- [ ] **Paso 3 — Unidades y orden de ruedas:** Grafica el CSV con `python3 analyze_telemetry.py <archivo.csv>` y comprueba que `state` siga a `setpoint` en la misma escala y signo para las ruedas `0 (R)`, `1 (L)` y `2 (B)`.
- [ ] **Paso 4 — Recepción y orden de ganancias (Modo Clásico):** Con el robot **en soporte (ruedas en el aire)** y `python_server.py` en `MODE = 1`, corre una sesión completa con `autotune_run()`. El log del robot debe mostrar exactamente `kp = [8.38, 6.53, 3.57]`, `ki = [0.05, 0.08, 0.09]`, `kd = [0.008, 0.013, 0.016]`. Con el *envelope* de [§7.7](#77-aplicar-ganancias-con-seguridad-envelope) activo, verifica que sean rechazadas (confirmando que tu protección funciona).
- [ ] **Paso 5 — Modo RNN en soporte:** Cambia a `MODE = 0` en `python_server.py`, reinicia el servidor y ejecuta la sesión con el robot en soporte. Verifica en el servidor `Sent PID response: iteration=1, converged=False` $\rightarrow$ `Phase changed to 'after'` $\rightarrow$ `Saved comparison plot` $\rightarrow$ `Sent final acknowledgment`.
- [ ] **Paso 6 — Modo RNN en piso:** Con espacio despejado en la cancha y el *envelope* de seguridad de [§7.7](#77-aplicar-ganancias-con-seguridad-envelope) activo, ejecuta la sintonización real en piso.

---

## 12. Problemas Conocidos del Código de Referencia

| Problema en el código original | Efecto | Mitigación recomendada |
| :--- | :--- | :--- |
| **Credenciales WiFi e IP en `#define` dentro de `firmware_main.c`** | Exposición de credenciales en el historial de Git y falta de portabilidad. | Usar `Kconfig` (`idf.py menuconfig`) o partición NVS. |
| **Sin reconexión automática TCP** | Si el servidor se cae o cierra el socket, el firmware devuelve `ESP_FAIL`. | Reintentar invocando nuevamente `autotune_run()` desde tu tarea de aplicación. |
| **Menos de 21 muestras en una inferencia** | Excepción en `python_server.py`, cierre de socket sin respuesta. | Asegurar que la trayectoria de prueba dure bastante más de $20\text{ ms}$. |
| **`abs()` silencioso sobre las ganancias en el servidor** | Una predicción negativa de la red se convierte en positiva sin aviso. | Mantener activo el filtro `gains_ok()` en el firmware ([§7.7](#77-aplicar-ganancias-con-seguridad-envelope)). |
| **Estado global de sesión en `python_server.py`** | Solo soporta 1 robot por proceso; si un run se aborta a la mitad, contamina el siguiente. | Reiniciar `python_server.py` si un experimento se interrumpe antes de `converged = 1`. |
| **Etiquetas mezcladas en nombres `*_after_*` y JSON en modo RNN** | El log del servidor muestra grupos `Kp/Ki/Kd` permutados. | Guiarse por el log serial del ESP32-S3 (`Applying PID constants`). |
| **Contabilidad desfasada si `RUNS_TO_CONVERGE > 2`** | Empareja las ganancias recién predichas con el *score* del *run* anterior. | Mantener `RUNS_TO_CONVERGE = 2`. |
| **Historial de errores sin inicializar en la 1.ª muestra** | `robot_sample_t sample;` local enviaba basura en $e(k-1), e(k-2)$ en $k=0$. | Usar `autotune_port.c` ([§7.5](#75-capa-de-pegamento-código-listo-para-copiar)), que limpia `s_e1` y `s_e2` con `memset`. |
| **`multiple_movements()` llamada desde 3 tareas** | El reloj `static` interno avanza 3 veces por cada ms. | Llamar al generador de trayectoria desde **una sola tarea**. |

---

## 13. Solución de Problemas (Troubleshooting)

| Síntoma observado | Causa probable | Qué hacer |
| :--- | :--- | :--- |
| **Robot:** `Connection failed: errno ...` | Servidor apagado, IP incorrecta, otra subred o firewall bloqueando. | Hacer `ping` al PC, revisar IP pasada a `telemetry_init()` y abrir puerto `8888/TCP`. |
| **Servidor:** `Checksum mismatch` | Estructura distinta de 64 B/muestra o alineación incorrecta. | Verificar `__attribute__((packed))`, orden de campos y `sizeof(robot_sample_t) == 64`. |
| **Robot:** `Failed to receive PID response` | El servidor lanzó excepción (pocas muestras o error de Keras). | Revisar la consola de Python y reiniciar `python_server.py`. |
| **Respuesta fija con `8.38, 6.53, 3.57, ...`** | El servidor sigue en modo clásico (`MODE = 1`). | Cambiar a `MODE = 0` en `python_server.py` y reiniciar el servidor. |
| **Error al cargar `.h5` (`Could not locate function 'mse'`)** | Versión de Keras/TensorFlow incompatible. | Instalar Keras 3.x + TensorFlow 2.21+ en el entorno virtual ([§6.1](#61-instalación-de-dependencias)). |
| **El servidor no arranca (`No such file ... models/...`)** | Se ejecutó `python_server.py` desde otro directorio de trabajo. | Hacer `cd` a la carpeta que contiene `python_server.py` y el subdirectorio `models/`. |
| **Ganancias absurdas o rechazadas por el *envelope*** | Entradas fuera del rango de entrenamiento (unidades, escala u orden de ruedas). | Auditar el CSV con `check_csv.py` ([Apéndice B](#142-apéndice-b-verificador-de-sanidad-de-csv-check_csvpy)) y revisar [§5](#5-compatibilidad-léelo-antes-de-escribir-código). |
| **El robot oscila bruscamente tras aplicar ganancias** | Ecuación de PID distinta a `pid_ext` o ganancias aplicadas en otro orden de ruedas. | Verificar el orden `[R, L, B]` y adoptar `pid_calc_discrete` ([§5](#5-compatibilidad-léelo-antes-de-escribir-código)). |

---

## 14. Apéndices: Herramientas de Validación (`fake_robot.py` y `check_csv.py`)

### 14.1. Apéndice A: Robot Simulado (`fake_robot.py`)
Este script habla el protocolo binario exacto del ESP32-S3 y permite validar de extremo a extremo `python_server.py`, el cálculo de CRC32 y la carga de los modelos `.h5` sin necesidad de encender el robot:

```python
"""Fake ESP32: speaks the exact firmware protocol against python_server.py."""
import socket, struct, zlib, sys, numpy as np

HOST, PORT = "127.0.0.1", 8888
HDR    = struct.Struct("<BBHI")   # type, version, length, crc32 (8 B)
SAMPLE = struct.Struct("<I15f")   # ts + 3 state + 3 setpoint + 9 errors (64 B)
RESP   = struct.Struct("<9fBB")   # kp[3] ki[3] kd[3] iteration converged (38 B)
DATA, INFER, PIDRESP = 0x01, 0x06, 0x02
BATCH  = 100

def run_samples(n_batches, seed):
    """Synthetic 1 kHz run: setpoint steps, first-order wheel response + noise."""
    rng = np.random.default_rng(seed)
    n = n_batches * BATCH
    sp = np.zeros((n, 3))
    sp[n // 4 : n // 2] = [-12, 12, 0]
    sp[n // 2 : 3 * n // 4] = [12, 12, -12]
    st = np.zeros((n, 3))
    for k in range(1, n):
        st[k] = st[k - 1] + 0.02 * (sp[k] - st[k - 1]) + rng.normal(0, 0.05, 3)
    e = sp - st
    out, ts = [], 1000
    for k in range(n):
        ek1 = e[k - 1] if k > 0 else np.zeros(3)
        ek2 = e[k - 2] if k > 1 else np.zeros(3)
        # Firmware layout: errors[motor][k, k-1, k-2]
        errs = np.stack([e[k], ek1, ek2], axis=1).reshape(-1)
        out.append(SAMPLE.pack(ts + k, *st[k], *sp[k], *errs))
    return out

def send_run(sock, samples, seq0=0):
    for b in range(len(samples) // BATCH):
        chunk = samples[b * BATCH : (b + 1) * BATCH]
        payload = struct.pack("<HH", len(chunk), seq0 + b) + b"".join(chunk)
        crc = zlib.crc32(payload) & 0xFFFFFFFF
        sock.sendall(HDR.pack(DATA, 1, len(payload), crc) + payload)

def request_and_read(sock):
    sock.sendall(HDR.pack(INFER, 1, 0, 0))
    h = b""
    while len(h) < HDR.size:
        h += sock.recv(HDR.size - len(h))
    t, v, ln, crc = HDR.unpack(h)
    p = b""
    while len(p) < ln:
        p += sock.recv(ln - len(p))
    assert t == PIDRESP, f"unexpected type {t}"
    assert ln == RESP.size == 38, f"payload {ln}"
    assert (zlib.crc32(p) & 0xFFFFFFFF) == crc, "crc mismatch"
    f = RESP.unpack(p)
    return dict(kp=f[0:3], ki=f[3:6], kd=f[6:9], iteration=f[9], converged=f[10])

if __name__ == "__main__":
    n_batches = int(sys.argv[1]) if len(sys.argv) > 1 else 5
    s = socket.create_connection((HOST, PORT))
    s.settimeout(120)
    print("sample size:", SAMPLE.size, "bytes; header:", HDR.size, "bytes")
    send_run(s, run_samples(n_batches, 1))
    r1 = request_and_read(s)
    print("RESPONSE 1:", r1)
    send_run(s, run_samples(n_batches, 2))
    r2 = request_and_read(s)
    print("RESPONSE 2:", r2)
    s.close()
```

### 14.2. Apéndice B: Verificador de Sanidad de CSV (`check_csv.py`)
Ejecuta `python3 check_csv.py <ruta_al_csv>` sobre cualquier archivo capturado por `data_logger_server.py` o `python_server.py` para verificar que tu firmware esté muestreando a **1 kHz** y que el historial $[e(k), e(k-1), e(k-2)]$ sea consistente:

```python
"""Sanidad de un CSV de telemetría (data_logger_server.py / python_server.py)."""
import sys
import numpy as np
import pandas as pd

df = pd.read_csv(sys.argv[1])
dt = np.diff(df.timestamp_ms.values.astype(float))
print(
    f"muestras={len(df)}  dt mediano={np.median(dt):.1f} ms  "
    f"p95={np.percentile(dt, 95):.1f} ms  max={dt.max():.0f} ms"
)

ok = True
for m in range(3):
    e  = df[f"motor_setpoint_{m}"] - df[f"motor_state_{m}"]
    d0 = np.abs(df[f"error_{m}_k"] - e).max()
    d1 = np.abs(df[f"error_{m}_k1"].values[1:] - df[f"error_{m}_k"].values[:-1]).max()
    d2 = np.abs(df[f"error_{m}_k2"].values[2:] - df[f"error_{m}_k"].values[:-2]).max()
    print(f"rueda {m}: |e(k)-(sp-st)|={d0:.1e}  hist k-1={d1:.1e}  hist k-2={d2:.1e}")
    ok &= max(d0, d1, d2) < 1e-3

st = [f"motor_state_{m}" for m in range(3)]
sp = [f"motor_setpoint_{m}" for m in range(3)]
print(
    "max |state|:", df[st].abs().max().round(2).tolist(),
    " max |setpoint|:", df[sp].abs().max().round(2).tolist()
)
print("OK" if ok else "REVISAR: historial de errores inconsistente")
```