| Supported Targets | ESP32-S3 |
| ----------------- | -------- |

# WHEEL-MOVEMENT (`striker_v1`): FreeRTOS-Based Single-Wheel Control & Calibration System

Esta aplicación implementa el banco de control en tiempo real, calibración de sensores/actuadores y sintonización interactiva de ganancias PID para **una rueda individual** del robot **Striker 1** sobre el SoC **ESP32-S3** utilizando **ESP-IDF y FreeRTOS**.

Como señala la documentación original del proyecto, la aplicación integra los controladores del encoder magnético **AS5600**, el sensor láser de largo alcance **VL53L1X** (ToF LiDAR) y la unidad de medición inercial de 9 ejes **TM151** (IMU), además del módulo de fusión sensorial (`sensor_fusion.c`). No obstante, **en el estado actual del código, únicamente el encoder magnético AS5600 se utiliza de forma efectiva para cerrar el lazo de control de velocidad y estimar la distancia recorrida** (véase el detalle en la [Sección 6](#6-auditoría-de-uso-real-de-sensores-y-módulos-auxiliares-documentación-original-vs-código-real)).

---

## 1. Estructura del Proyecto en el Monorepo Unificado

Tras la centralización de los controladores en `firmware/components/`, el directorio `firmware/apps/striker_v1/wheel-movement/` conserva únicamente los archivos de aplicación y rutinas de prueba/calibración en `main/`:

```text
firmware/apps/striker_v1/wheel-movement/
├── CMakeLists.txt                  # Configuración raíz del proyecto ESP-IDF
├── partitions.csv                  # Tabla de particiones de la memoria Flash
├── sdkconfig                       # Configuración del kernel FreeRTOS, Kconfig del proyecto y periféricos
├── README.md                       # Documentación técnica, auditoría del kernel FreeRTOS y TODOs
└── main/
    ├── CMakeLists.txt              # Registro del ejecutable (compila main.c por defecto) y enlace con components/
    ├── Kconfig.projbuild           # Menú personalizado de configuración (pines GPIO, PWM, WiFi, MAIN_MODE)
    ├── main.c                      # Aplicación multi-modo (MAIN_MODE 0: Control PID, 1: Cal. ESC, 3: Probe AS5600)
    ├── pid_calibration.c           # Aplicación alternativa con arquitectura de eventos UART + Mutex + Task Notify
    └── encoder_test.c              # Aplicación standalone para configurar y probar el sensor AS5600 por I2C/ADC
```

### Selección del Archivo Principal en `main/CMakeLists.txt`
Por defecto, `main/CMakeLists.txt` compila **`"main.c"`** como archivo principal de la aplicación. Los archivos **`pid_calibration.c`** y **`encoder_test.c`** contienen su propia función `app_main()`, por lo que actúan como ejecutables alternativos de laboratorio (para compilar cualquiera de ellos basta con sustituir `"main.c"` por `"pid_calibration.c"` o `"encoder_test.c"` en `SRCS` dentro de `main/CMakeLists.txt`).

### Componentes Centralizados Enlazados desde `firmware/components/`
* **`sensors/as5600/v1_striker1`**: Configuración de registros por I2C (`CONF`, posiciones de inicio/fin `ZPOS`/`MPOS`) y lectura de ángulo en tiempo real por salida analógica (`AS5600_ADC_GetAngle`).
* **`sensors/tm151/v1_striker1`**: Driver de comunicación serial UART (`EasyProtocol`, `EasyQueue`, `EasyRetrieve`) para la IMU TM151.
* **`sensors/vl53l1x/v1_striker1`**: Driver I2C para el sensor láser Time-of-Flight VL53L1X.
* **`actuators/bldc_pwm/v1_striker1`**: Generación de señales PWM/MCPWM a 50 Hz e inversión de sentido de giro para el variador ESC del motor Brushless.
* **`control/pid_lib/v1_striker1`**: Controlador PID discreto (`pid_ext.c`) configurado en modo incremental (`PID_CAL_TYPE_INCREMENTAL`).
* **`control/sensor_fusion/v1_striker1`**: Algoritmos de estimación y filtrado de velocidad/posición (`estimate_velocity_encoder`, `estimate_velocity_imu`, `estimate_velocity_lidar`, `estimate_velocity`).
* **`platform_hal/v1_striker1`**: Capa de abstracción de hardware para ESP32-S3 (ADC Oneshot, I2C Master, UART, MCPWM).

---

## 2. Inicialización del Sistema (`app_main`) y Configuración de Hardware

### 2.1. Configuración de Pines y Periféricos (Kconfig vs. Valores Fijos)
A diferencia de `robot-mov`, el archivo `main.c` de `wheel-movement` lee su configuración desde las macros generadas por **`main/Kconfig.projbuild`**, mientras que `pid_calibration.c` y `encoder_test.c` definen los pines directamente con `#define`:

| Subsistema / Periférico | Macro en `main.c` (`Kconfig`) | Valor en `pid_calibration.c` / `encoder_test.c` | Descripción Técnica |
| :--- | :--- | :--- | :--- |
| **Modo de Ejecución** | `CONFIG_MAIN_MODE` (`MAIN_MODE`) | `0` en `encoder_test.c` | `0`: Control PID + Sensores<br>`1`: Calibración de ESC<br>`3`: Lectura de prueba AS5600 *(en `Kconfig` figura como `2`)* |
| **AS5600 (I2C + ADC)** | `CONFIG_AS5600_I2C_MASTER_SCL_GPIO`<br>`CONFIG_AS5600_I2C_MASTER_SDA_GPIO`<br>`CONFIG_AS5600_OUT_GPIO`<br>`CONFIG_AS5600_I2C_MASTER_NUM` | `SCL = GPIO 5` *(4 en `encoder_test.c`)*<br>`SDA = GPIO 4` *(5 en `encoder_test.c`)*<br>`OUT = GPIO 6` (ADC)<br>`I2C_NUM = 0` | Configura el AS5600 por I2C en modo **Analog Output RR (10%–90%)**, filtro lento `16x` y lee el ángulo por el canal ADC de `GPIO 6`. |
| **VL53L1X (ToF LiDAR)** | `CONFIG_VL53L1X_I2C_PORT`<br>`CONFIG_VL53L1X_SDA_GPIO`<br>`CONFIG_VL53L1X_SCL_GPIO` | `I2C_PORT = 1`<br>`SDA = GPIO 41`<br>`SCL = GPIO 42` | Modo de distancia `Short`, *Timing Budget* de $20\,000\text{ }\mu\text{s}$ y medición continua cada `SAMPLE_TIME`. |
| **TM151 (IMU Serial)** | `CONFIG_TM151_UART_TX`<br>`CONFIG_TM151_UART_RX`<br>`CONFIG_TM151_UART_BAUDRATE` | `TX = GPIO 17`, `RX = GPIO 18`<br>`Baudrate = 115600`<br>`Buffer = 1024 B` | Puerto UART dedicado para recepción de tramas inerciales. |
| **Motor BLDC / ESC** | `CONFIG_PWM_GPIO`<br>`CONFIG_PWM_REV_GPIO`<br>`CONFIG_PWM_FREQ` | `PWM_GPIO = 20`<br>`PWM_REV_GPIO = 21`<br>`50 Hz`, Res: `100000`, Cal: `[35, 120]` | Control de tracción y señal de reversa del variador electrónico ESC. |
| **Controlador PID** | `PID_KP`, `PID_KI`, `PID_KD` | `Kp=1.66, Ki=0.6, Kd=0.05` *(rama HEAD)* o `0.01, 0.01, 0.001` *(main)* | `PID_CAL_TYPE_INCREMENTAL`, salida acotada en $[-70, +70]$ en `main.c` ($[-60, +60]$ en `pid_calibration.c`). |

### 2.2. Secuencia de Inicialización en `app_main()` (`main.c`, `MAIN_MODE == 0`)
1. **Inicialización del LiDAR VL53L1X:** Configura el bus `I2C_PORT 1`, establece modo `Short`, presupuesto de $20\text{ ms}$ y arranca lecturas continuas (`VL53L1X_startContinuous`), esperando $500\text{ ms}$. *(Nota: si el sensor no responde por I2C, `app_main` hace `return` prematuro y detiene todo el arranque).*
2. **Armado del Motor BLDC:** Inicializa el periférico PWM en `PWM_GPIO` y `PWM_REV_GPIO`, habilita la salida con ciclo útil `0%` y espera **$4\text{ segundos}$** (`vTaskDelay(4000 / portTICK_PERIOD_MS)`) para asegurar el armado del ESC.
3. **Configuración Híbrida (I2C + ADC) del Encoder AS5600:**
   * Inicializa el bus I2C `0` y escribe el registro `CONF` (`0x07`): modo normal, sin histéresis, salida analógica ratiométrica 10%–90% (`AS5600_OUTPUT_STAGE_ANALOG_RR`) y filtro lento `16x`.
   * Verifica la escritura leyendo `AS5600_REG_CONF_H` y configura el rango angular completo (`0x0000` a `0x0FFF`).
   * Inicializa el canal ADC (`AS5600_InitADC(&gAs5600)`) y espera $500\text{ ms}$.
4. **Inicialización de la IMU TM151:** Configura el puerto UART a `115600 baud`.
5. **Creación del Bloque PID y Fusión Sensorial:** Instancia el bloque de control incremental (`pid_new_control_block`) y limpia las estructuras de estado con `sf_init(&imu_data, &encoder_data, &lidar_data)`.
6. **Creación de Tareas FreeRTOS:** Crea las tareas sin afinidad de núcleo (`xTaskCreate`): `encoder_task` (Prio 9), `control_task` (Prio 9) y `uart_task` (Prio 10). Las tareas `imu_task` y `lidar_task` están comentadas.

---

## 3. Análisis de la Estructura de Tareas FreeRTOS

Dado que el proyecto cuenta con dos implementaciones principales de aplicación (`main.c` y `pid_calibration.c`), además del test `encoder_test.c`, la siguiente tabla consolida **todas las tareas definidas en el código fuente**:

### 3.1. Tareas en `main.c` (`MAIN_MODE == 0`)
| Nombre de Tarea | Función C | Afinidad de Núcleo | Prioridad | Stack (Palabras / Bytes) | Periodo / Evento Bloqueante | Propósito en el Sistema |
| :--- | :--- | :---: | :---: | :---: | :--- | :--- |
| **`uart_task`** | `vTaskUART` / `uart_task` | `tskNO_AFFINITY` (`xTaskCreate`) | **10** *(Máx)* | `4096` | Bloqueante en `uart_read_bytes(..., 1000 ms)` | Lee comandos ASCII por consola serial (`P`, `X`, `S`, `D`, `I`) y modifica en caliente `pid_param`, `duty` o `goal_distance`. |
| **`encoder_task`** | `vTaskEncoder` | `tskNO_AFFINITY` (`xTaskCreate`) | **9** | `2048` | `2 ms` (`vTaskDelay(SAMPLE_TIME / portTICK_PERIOD_MS)`) | Lee el ángulo analógico (`AS5600_ADC_GetAngle`) y actualiza `encoder_data.velocity` y `encoder_data.distance`. |
| **`control_task`** | `vTaskControl` | `tskNO_AFFINITY` (`xTaskCreate`) | **9** | `4096` | `2 ms` (`vTaskDelay(SAMPLE_TIME / portTICK_PERIOD_MS)`) | Ejecuta `pid_compute()`, actualiza el PWM del motor (`bldc_set_duty`), gestiona pulsos `move_one_time` y detiene el motor al alcanzar `goal_distance`. |
| *`imu_task` (comentada)* | `vTaskIMU` | `tskNO_AFFINITY` | *9* | `2048` | `2 ms` (`vTaskDelay`) | Lee aceleración cruda del TM151 (`SerialPort_DataReceived_RawAcc`) y llama a `estimate_velocity_imu()`. |
| *`lidar_task` (comentada)*| `vTaskLidar` | `tskNO_AFFINITY` | *9* | `2048` | `2 ms` (`vTaskDelay`) | Lee distancia del VL53L1X por I2C y llama a `estimate_velocity_lidar()`. |
| *`tcp_server` (no creada)*| `tcp_server` | *No instanciada en `app_main`* | — | — | Bloqueante en `accept()` y `recv()` | Servidor TCP en puerto `PORT` que recibe comandos `P, X, S, D, I` y activa banderas en la unión `movement_flags`. |

### 3.2. Tareas en `pid_calibration.c` (Arquitectura Orientada a Eventos UART)
| Nombre de Tarea | Función C | Afinidad de Núcleo | Prioridad | Stack | Periodo / Evento Bloqueante | Propósito en el Sistema |
| :--- | :--- | :---: | :---: | :---: | :--- | :--- |
| **`control_task`** | `control_task` | `tskNO_AFFINITY` | **10** | `4096` | `2 ms` (`vTaskDelay(2 / portTICK_PERIOD_MS)`) | Unifica en una sola tarea la lectura del ADC (`AS5600_ADC_GetAngle`), la estimación de velocidad y el cálculo del PID (`pid_compute` + `bldc_set_duty`). |
| **`uart_handler`** | `vTaskUartHandler`| `tskNO_AFFINITY` | **10** | `4096` | Bloqueante en `xQueueReceive(uart_queue, portMAX_DELAY)` | Espera eventos `UART_DATA` generados por la ISR del driver UART0, ensambla caracteres en `uart_buffer` y al recibir `\n` o `\r` despierta al parser con `xTaskNotifyGive()`. |
| **`uart_parser`** | `vTaskUartParser` | `tskNO_AFFINITY` | **10** | `4096` | Bloqueante en `ulTaskNotifyTake(pdTRUE, portMAX_DELAY)` | Parsea tramas JSON `{"default":"kp ki kd setpoint"}`, toma el Mutex `xPidMutex` y actualiza `pid_update_parameters()`. |
| *`uart_task` (comentada)* | `uart_task` | `tskNO_AFFINITY` | *10* | `4096` | `uart_read_bytes(..., 1000 ms)` | Versión antigua por sondeo reemplazada por el par `uart_handler` + `uart_parser`. |

---

## 4. Análisis del Uso de Objetos del Kernel de FreeRTOS y Sincronización

### 4.1. Auditoría Comparativa de Objetos del Kernel (`main.c` vs. `pid_calibration.c`)

El análisis del código revela una diferencia muy instructiva entre los archivos del proyecto:

1. **En `main.c` (Sin objetos de sincronización del kernel):**
   * Incluye `<freertos/queue.h>` y `<driver/gptimer.h>`, pero **no instancia ninguna cola, semáforo ni mutex**.
   * Usa `xTaskCreate()` sin fijar afinidad de núcleo (`tskNO_AFFINITY`), permitiendo que el scheduler SMP migre las tareas dinámicamente entre el `Core 0` y el `Core 1`.
   * Toda la comunicación entre `vTaskEncoder`, `vTaskControl` y `vTaskUART` se realiza mediante **variables globales desprotegidas** (`encoder_data`, `pid_param`, `pid`, `goal_distance`, `reached_goal`, `move_one_time`, `duty`).
2. **En `pid_calibration.c` (Uso rico de primitivas del kernel de FreeRTOS):**
   * **Cola de Eventos de Driver (`QueueHandle_t uart_queue`):** Creada e inyectada directamente en `uart_driver_install(UART_NUM_0, ..., &uart_queue, 0)`. La ISR de recepción UART publica estructuras `uart_event_t` en la cola y despierta a `vTaskUartHandler` sin consumir CPU por *polling*.
   * **Notificación Directa a Tarea (`xTaskNotifyGive` / `ulTaskNotifyTake`):** Utilizada como un semáforo binario ligero y ultrarrápido para sincronizar `vTaskUartHandler` (productor de línea completa) con `vTaskUartParser` (consumidor/intérprete JSON) usando su `TaskHandle_t xHandleParserTask`.
   * **Mutex del Kernel (`SemaphoreHandle_t xPidMutex`):** Creado con `xSemaphoreCreateMutex()`. La tarea `vTaskUartParser` adquiere el mutex (`xSemaphoreTake(xPidMutex, portMAX_DELAY)`) antes de modificar `pid_param` y llamar a `pid_update_parameters(pid, &pid_param)`, liberándolo después con `xSemaphoreGive(xPidMutex)`. *(Sin embargo, como se detalla en la Sección 4.3, `control_task` olvida tomar ese mismo mutex al leer `pid`).*

---

### 4.2. Diagrama de Relación de Tareas y Objetos de FreeRTOS

A continuación se contrastan las dos arquitecturas presentes en el código de `wheel-movement`:

```text
=================================================================================================================
   ARQUITECTURA A: main.c (MAIN_MODE == 0) — Tareas Separadas sin Sincronización (Memoria Global Compartida)
=================================================================================================================

  [ encoder_task ] (Prio 9, Stack 2048, tskNO_AFFINITY)      [ uart_task ] (Prio 10, Stack 4096, tskNO_AFFINITY)
  • Lee AS5600_ADC_GetAngle(&gAs5600)                        • Bloqueada en uart_read_bytes(UART_NUM_0, 1000 ms)
  • Llama estimate_velocity_encoder(&encoder_data)           • Parsea comandos 'P', 'X', 'S', 'D', 'I'
  • [Objeto Kernel: vTaskDelay(2 ms)]                        • Modifica directo pid_param, duty, goal_distance
                   │                                                          │
                   │ Escribe sin Mutex                                        │ Escribe sin Mutex y llama
                   ▼                                                          ▼ pid_update_parameters(pid)
  +===========================================================================================================+
  |         MEMORIA GLOBAL COMPARTIDA (encoder_data, pid, pid_param, goal_distance, move_one_time, duty)      |
  +===========================================================================================================+
                                                       │
                                                       │ Lee cada 2 ms sin Mutex
                                                       ▼
                               [ control_task ] (Prio 9, Stack 4096, tskNO_AFFINITY)
                               • Lee encoder_data.velocity y encoder_data.distance
                               • Ejecuta pid_compute(pid, est_velocity, &output) -> bldc_set_duty(&pwm, output)
                               • Evalúa move_one_time y parada en (est_dist > goal_distance)
                               • [Objeto Kernel: vTaskDelay(2 ms)]

=================================================================================================================
   ARQUITECTURA B: pid_calibration.c — Pipeline de Eventos UART con Queue, Task Notification y Mutex
=================================================================================================================

  [ ISR Hardware UART_NUM_0 ]
              │
              │ Publica evento UART_DATA
              ▼
  ( QueueHandle_t: uart_queue )  [Profundidad: 10 eventos]
              │
              │ xQueueReceive(..., portMAX_DELAY)
              ▼
  [ vTaskUartHandler ] (Prio 10, Stack 4096)
  • Lee bytes de FIFO UART y arma cadena en uart_buffer[]
  • Al detectar '\n' o '\r' dispara notificación:
              │
              │ xTaskNotifyGive(xHandleParserTask)   <── [Buffer compartido: char uart_buffer[256] sin doble buffer]
              ▼
  [ vTaskUartParser ]  (Prio 10, Stack 4096)
  • Se despierta de ulTaskNotifyTake(pdTRUE, portMAX_DELAY)
  • Extrae JSON {"default":"kp ki kd setpoint"}
  • xSemaphoreTake(xPidMutex, portMAX_DELAY)
  • Actualiza pid_param y pid_update_parameters(pid, &pid_param)
  • xSemaphoreGive(xPidMutex)
              │
              ├──► ( SemaphoreHandle_t: xPidMutex ) ◄── ¡NOTA CRÍTICA!: control_task (Prio 10) ejecuta
                                                        pid_compute(pid, ...) cada 2 ms SIN tomar xPidMutex.
```

---

### 4.3. Diagnóstico de Problemas de Concurrencia, Tiempo Real y Conflictos de Código

1. **Conflicto de Merge de Git sin resolver en `main.c` (Error de compilación inmediato):**
   * El archivo `main/main.c` contiene marcadores de conflicto de Git crudos (`<<<<<<< HEAD`, `=======`, `>>>>>>> main`) en tres lugares: líneas 106–119 (definición de `PID_KP`, `PID_KI`, `PID_KD`), líneas 370–462 (entre `uart_task` comentada, `wifi_init_sta` y `vTaskUART`) y líneas 543–571 (bloques `case 'D':` y `case 'I':` duplicados dentro de `tcp_server` intentando usar la variable inexistente `data` en lugar de `rx_buffer`, además de errores tipográficos como `struct main sockaddr_in`, `if (len <= o)` con la letra `o` en vez de `0`, y falta de `;` tras `void get_ip_address(void)`).
2. **Macro `SAMPLE_TIME` redefinida y riesgo de `vTaskDelay(0)`:**
   * En las líneas 66–67 de `main.c` se declara:
     ```c
     #define SAMPLE_TIME CONFIG_SAMPLE_TIME
     #define SAMPLE_TIME 2
     ```
     La segunda línea sobrescribe silenciosamente el valor configurado por el usuario en `menuconfig`. Además, si `CONFIG_FREERTOS_HZ` está en `100` Hz (`portTICK_PERIOD_MS = 10`), `2 / 10` vale **`0` ticks**.
3. **Exclusión mutua incompleta sobre `pid` y `pid_param` (Tanto en `main.c` como en `pid_calibration.c`):**
   * En `pid_calibration.c`, aunque se creó correctamente `xPidMutex` y `vTaskUartParser` lo toma antes de llamar a `pid_update_parameters(pid, &pid_param)`, **`control_task` nunca toma `xPidMutex` antes de llamar a `pid_compute(pid, ...)`**. Un mutex solo protege un recurso compartido si **todos** los hilos (lectores y escritores) lo adquieren.
   * En `main.c` el problema es aún más directo: `vTaskUART` (Prioridad 10) expropia a `vTaskControl` (Prioridad 9) en cualquier instante para sobrescribir `pid_param` y `goal_distance` mientras `vTaskControl` también modifica `pid_param.set_point = 0.0f` al llegar a la meta.
4. **Condición de carrera entre `vTaskUartHandler` y `vTaskUartParser` sobre `uart_buffer` (`pid_calibration.c`):**
   * Ambas tareas tienen **Prioridad 10**. Cuando `vTaskUartHandler` recibe `\r\n` (dos caracteres seguidos de fin de línea), al procesar `\r` pone `uart_buffer_index = 0` y llama a `xTaskNotifyGive()`. Si llegan más bytes inmediatamente o al procesar el `\n` subsiguiente, `vTaskUartHandler` sobrescribe `uart_buffer[0]` mientras `vTaskUartParser` está leyendo `uart_buffer` con `strstr()`.
5. **Orden de inicialización de `xHandleParserTask` (`pid_calibration.c`):**
   * En `pid_calibration.c`, `vTaskUartHandler` se crea *antes* que `vTaskUartParser`, y además `xHandleParserTask` se asigna tanto en el último parámetro de `xTaskCreate(..., &xHandleParserTask)` como dentro de la propia tarea (`xHandleParserTask = xTaskGetCurrentTaskHandle()`). Si llegara una interrupción UART antes de que arranque `vTaskUartParser`, `xTaskNotifyGive(NULL)` fallaría en un `configASSERT`.
6. **Tareas de control y adquisición sin afinidad de núcleo (`xTaskCreate` con `tskNO_AFFINITY`) y con igual prioridad (`9`):**
   * En `main.c`, `vTaskEncoder` y `vTaskControl` se crean sin fijar núcleo y con la misma prioridad `9`. No hay garantía de precedencia (que primero se lea el encoder del ciclo $k$ y luego se ejecute el PID del ciclo $k$), a diferencia de `pid_calibration.c`, donde acertadamente ambas operaciones se ejecutan de forma secuencial dentro de la misma `control_task`.

---

## 5. Módulos de Estimación, Cinemática y Control

### 5.1. Estimación de Velocidad y Posición (`sensor_fusion.c`)
Conservando la arquitectura descrita en la documentación original:
* **`estimate_velocity_encoder(&encoder_data, angle, dt)`**: Calcula la velocidad tangencial de la rueda (en $\text{cm/s}$) a partir del cambio angular medido por el AS5600 y el radio de rueda, aplicando un filtro pasa-bajas y descartando variaciones por debajo del umbral de ruido, además de integrar el desplazamiento en `encoder_data.distance` cuando la bandera `encoder_data.estimate == 1`.
* **`estimate_velocity_imu(&imu_data, acc_x, dt)`**: Integra por regla trapezoidal la aceleración lineal del eje X de la IMU TM151 con suavizado exponencial.
* **`estimate_velocity_lidar(&lidar_data, distance, dt)`**: Estima la velocidad lineal mediante diferencias finitas de la distancia medida por el sensor ToF VL53L1X con filtro de media móvil.
* **`estimate_position(...)` y `estimate_velocity(...)`**: Rutinas diseñadas para promediar la posición entre LiDAR y encoder, y para fusionar las 3 velocidades eligiendo el par con menor discrepancia mutua.

### 5.2. Estrategia de Control PID y Modos de Movimiento (`vTaskControl`)
* **Controlador PID Incremental (`PID_CAL_TYPE_INCREMENTAL`):** Calcula el incremento de esfuerzo de control a partir del error entre `pid_param.set_point` y `est_velocity`, saturando la salida PWM en $[-70, +70]$ (`main.c`) o $[-60, +60]$ (`pid_calibration.c`).
* **Modo Pulso de Ciclo Útil Fijo (`move_one_time` / Comando `'X'`):** Permite aplicar un ciclo útil `duty` en lazo abierto durante exactamente $2000\text{ ms}$ (`temp_ctr < 2000`) y luego detener el motor automáticamente.
* **Modo Control de Distancia por Odometría (Comandos `'D'` e `'I'`):** Activa `encoder_data.estimate = 1`, fija `goal_distance` y mantiene el setpoint de velocidad hasta que `est_dist > goal_distance`, instante en el cual fuerza `pid_param.set_point = 0.0f` y detiene la rueda.

---

## 6. Auditoría de Uso Real de Sensores y Módulos Auxiliares (Documentación Original vs. Código Real)

Tal como advierte honestamente la introducción del `README.md` original (*"Nevertheless, at the time, only the encoder is being used for something useful"*), al contrastar la documentación con el código fuente de `main.c` y `pid_calibration.c` se confirma lo siguiente:

| Sensor / Módulo | Qué se inicializa en `app_main` | Estado Real en el Lazo de Control | Impacto en el Sistema |
| :--- | :--- | :--- | :--- |
| **Encoder Magnético `AS5600`** | Inicializado por I2C + ADC en los 3 archivos (`main.c`, `pid_calibration.c`, `encoder_test.c`). | **100% Activo.** Es el único sensor que alimenta `est_velocity = encoder_data.velocity` y `est_dist = encoder_data.distance`. | El control de velocidad y el corte por distancia dependen exclusivamente de la odometría de la rueda (vulnerable a deslizamiento). |
| **Láser ToF `VL53L1X` (LiDAR)** | En `main.c` se inicializa por I2C al arranque (`VL53L1X_init` y `VL53L1X_startContinuous`). En `pid_calibration.c` ni siquiera se inicializa. | **Inoperante.** La creación de `lidar_task` está comentada en `main.c` y la suma `(distance - lidar_data.start_distance)` en `vTaskControl` también está comentada. | **Impacto crítico:** Aunque no se usa en el lazo, `app_main()` de `main.c` exige que el VL53L1X esté físicamente conectado al bus I2C; si no responde, `VL53L1X_init` falla y **aborta el arranque de toda la aplicación** (`return;`). |
| **IMU 9-DOF `TM151`** | Inicializada por UART en `main.c` (`tm151_init`). Omitida en `pid_calibration.c`. | **Inoperante.** La tarea `imu_task` está comentada en `main.c`. | Consume recursos de UART y memoria RAM (`TM151_BUFFER_SIZE`), pero nunca se lee ningún paquete inercial. |
| **Fusión Sensorial (`sensor_fusion.c`)** | Se llama a `sf_init()`, pero en `vTaskControl` la llamada a `estimate_velocity(imu, lidar, encoder)` está comentada. | **Parcialmente activo:** Solo se usa la función individual `estimate_velocity_encoder()`. La fusión multi-sensorial está desactivada. | Tampoco se aplica el filtro pasa-bajas adicional `beta` en `vTaskControl` (línea comentada). |
| **Servidor WiFi / TCP (`tcp_server`)** | Las funciones `wifi_init_sta()`, `get_ip_address()` y `tcp_server()` están escritas al final de `main.c`. | **Inactivo y con conflicto de merge.** Ninguna de esas 3 funciones es invocada en `app_main()`, y `movement_flags` no es leída por `vTaskControl`. | El control remoto por WiFi quedó a mitad de integración entre dos ramas de Git. |

---

## 7. Lista de Tareas Pendientes (TODOs) con Evaluación de Alternativas (Trade-offs)

A continuación se listan los `TODOs` técnicos para sanear y evolucionar esta aplicación. En los puntos donde existen múltiples caminos arquitectónicos, los estudiantes deben **evaluar las alternativas y justificar cuantitativamente su elección**:

- [ ] **TODO 1: Resolver los Conflictos de Merge en `main.c` y Unificar con `pid_calibration.c`:**
  * **Problema:** `main.c` no compila directamente debido a los marcadores `<<<<<<< HEAD` y errores de sintaxis en `tcp_server`, además de duplicar funcionalidad con `pid_calibration.c` y `encoder_test.c`.
  * **Alternativas a evaluar:**
    * *Alternativa A (Aplicación única gobernada por `Kconfig` / `MAIN_MODE`):* Limpiar `main.c` integrando los modos de calibración (`MAIN_MODE = 0, 1, 2, 3`) mediante compilación condicional `#if` o máquina de estados en tiempo de ejecución, eliminando los archivos `.c` sueltos redundantes.
    * *Alternativa B (Separación de comandos por interfaz abstracta):* Mantener un único motor de control y permitir elegir por `Kconfig` si la interfaz de sintonización entra por **UART (JSON/ASCII)** o por **WiFi (TCP/UDP)**.

- [ ] **TODO 2: Arquitectura de Tareas entre Adquisición de Encoder y Control PID:**
  * **Problema:** En `main.c`, `vTaskEncoder` y `vTaskControl` son dos tareas separadas de igual prioridad (`9`) sin sincronización entre ellas, mientras que en `pid_calibration.c` ambas operaciones están fusionadas en una sola `control_task`.
  * **Alternativas a evaluar:**
    * *Alternativa A (Tarea única fusionada Adquisición + Control, estilo `pid_calibration.c`):* Leer el ADC del AS5600, estimar la velocidad y calcular el PID secuencialmente dentro de un solo bucle periódico. Evaluar la reducción de *jitter*, eliminación de condiciones de carrera entre lectura/control y ahorro de $2048\text{ B}$ de stack.
    * *Alternativa B (Tareas separadas Productor-Consumidor sincronizadas):* Útil si se agregan sensores lentos (I2C/UART). Sincronizar `vTaskEncoder` y `vTaskControl` mediante notificación directa (`xTaskNotify`) o cola de sobreescritura (`xQueueOverwrite`) asignando mayor prioridad al productor o anclándolas a núcleos distintos. Comparar la sobrecarga de cambio de contexto frente a la Alternativa A.

- [ ] **TODO 3: Sincronización Segura al Actualizar Ganancias PID y Consignas en Caliente:**
  * **Problema:** En `main.c` no hay protección al modificar `pid_param`/`pid` desde UART/TCP, y en `pid_calibration.c` el mutex `xPidMutex` solo es tomado por el escritor (`vTaskUartParser`) pero no por el lector (`control_task`).
  * **Alternativas a evaluar:**
    * *Alternativa A (Mutex compartido con timeout cero en el lazo de control):* Hacer que `control_task` tome `xPidMutex` con `xSemaphoreTake(xPidMutex, 0)` (no bloqueante) o proteger únicamente la llamada a `pid_compute()`. Evaluar si el bloqueo del mutex por parte de UART introduce *jitter* en el lazo de $2\text{ ms}$.
    * *Alternativa B (Paso de mensajes por Cola - `QueueHandle_t`):* Hacer que `vTaskUartParser` o `tcp_server` envíen una estructura `pid_cmd_t` por una cola con `xQueueOverwrite()`, y que sea la propia `control_task` (única dueña del bloque `pid`) quien aplique `pid_update_parameters()` al inicio de su ciclo de $2\text{ ms}$ (`xQueueReceive(..., 0)`). Comparar la ausencia total de bloqueos mutuos frente al uso de Mutex.
    * *Alternativa C (Sección crítica SMP / Spinlock `portMUX_TYPE`):* Proteger únicamente la copia de los 4 flotantes (`kp, ki, kd, set_point`) con `taskENTER_CRITICAL(&my_spinlock)` durante unos pocos nanosegundos.

- [ ] **TODO 4: Protección del Buffer de Recepción en el Pipeline UART (`vTaskUartHandler` $\rightarrow$ `vTaskUartParser`):**
  * **Problema:** En `pid_calibration.c`, `vTaskUartHandler` despierta a `vTaskUartParser` con `xTaskNotifyGive()`, pero ambas comparten el mismo arreglo global `char uart_buffer[256]` sin protección contra sobrescritura si llegan nuevos caracteres por el puerto serie.
  * **Alternativas a evaluar:**
    * *Alternativa A (Stream Buffer o Message Buffer de FreeRTOS - `MessageBufferHandle_t`):* Reemplazar `uart_buffer[]` y la notificación por un `xMessageBufferSend()` / `xMessageBufferReceive()`, que copia tramas completas de longitud variable de forma segura entre tareas.
    * *Alternativa B (Doble Buffer / Ping-Pong Buffering con Notificación):* Mantener dos arreglos `uart_buf[2][256]`; mientras `vTaskUartHandler` llena el buffer `activo`, `vTaskUartParser` procesa el buffer `listo` cuyo índice se pasa en el valor de `xTaskNotify()`.
    * *Alternativa C (Fusión de Handler y Parser en una sola tarea de baja prioridad):* Evaluar si realmente se justifican dos tareas separadas de $4096\text{ B}$ de stack cada una ($8\text{ KB}$ en total) para parsear un comando ocasional de consola, o si basta con una sola tarea bloqueada en `uart_queue` con prioridad inferior al control.

- [ ] **TODO 5: Garantizar Periodicidad Exacta del Lazo de Muestreo ($2\text{ ms}$):**
  * **Problema:** El uso de `vTaskDelay(2 / portTICK_PERIOD_MS)` introduce deriva temporal (*time drift*), depende de que `CONFIG_FREERTOS_HZ` sea `1000` y en `main.c` existe una doble definición `#define SAMPLE_TIME`.
  * **Alternativas a evaluar:**
    * *Alternativa A (`vTaskDelayUntil`):* Emplear temporización absoluta del kernel con `CONFIG_FREERTOS_HZ=1000`.
    * *Alternativa B (Timer de Hardware `gptimer` / `esp_timer`):* Despertar a `control_task` cada $2000\text{ }\mu\text{s}$ exactos mediante una interrupción de temporizador y `vTaskNotifyGiveFromISR()`. Medir experimentalmente el *jitter* de ambas opciones con `esp_timer_get_time()`.

- [ ] **TODO 6: Desacoplar la Inicialización de Sensores Opcionales (LiDAR e IMU):**
  * **Problema:** En `main.c`, si el sensor `VL53L1X` no está conectado físicamente, `VL53L1X_init()` aborta `app_main()` impidiendo mover la rueda, a pesar de que el LiDAR ni siquiera se usa en el lazo.
  * **Alternativas a evaluar:**
    * *Alternativa A (Habilitación condicional en `Kconfig.projbuild`):* Agregar opciones booleanas `CONFIG_ENABLE_VL53L1X` y `CONFIG_ENABLE_TM151` en `menuconfig`.
    * *Alternativa B (Degradación elegante en tiempo de ejecución - Graceful Degradation):* Registrar un `ESP_LOGW` si un sensor secundario no responde en el bus I2C/UART y continuar el arranque operando únicamente con los sensores disponibles.

---

## 8. Compilación, Flasheo y Uso (CLI, VS Code GUI y Protocolo de Comunicación)

### 8.1. Compilación desde Línea de Comandos (Windows PowerShell)

> **Importante antes de compilar `main.c`:** Resuelve primero los tres bloques `<<<<<<< HEAD ... >>>>>>> main` en `main/main.c` (o cambia `"main.c"` por `"pid_calibration.c"` en `main/CMakeLists.txt`).

```powershell
# 1. Ubicarse en el directorio de la aplicación
cd firmware\apps\striker_v1\wheel-movement

# 2. Configurar el SoC objetivo (ESP32-S3)
idf.py set-target esp32s3

# 3. (Opcional) Ajustar pines, modo de operación (MAIN_MODE) y frecuencia de FreeRTOS
idf.py menuconfig

# 4. Compilar el firmware
idf.py build

# 5. Flashear y abrir el monitor serial (reemplaza COM5 por tu puerto)
idf.py -p COM5 flash monitor
```

### 8.2. Compilación y Uso desde la Interfaz Gráfica de ESP-IDF en Visual Studio Code (GUI)

1. **Abrir el proyecto de la aplicación:**
   * En VS Code, selecciona **File $\rightarrow$ Open Folder...** y abre `firmware/apps/striker_v1/wheel-movement` (o desde el monorepo presiona `Ctrl + Shift + P` $\rightarrow$ **`ESP-IDF: Select Current ESP-IDF Workspace Folder`** y elige `firmware/apps/striker_v1/wheel-movement`).
2. **Seleccionar el Target (`esp32s3`) y Puerto (`COMX`):**
   * En la barra inferior de VS Code, haz clic en el selector de chip y elige **`esp32s3`**.
   * Haz clic en el ícono de enchufe (**`Select Port to Use`**) y selecciona el puerto `COM` de tu placa.
3. **Configurar Parámetros del Proyecto en el `SDK Configuration Editor`:**
   * Haz clic en el ícono de **engranaje (`SDK Configuration Editor`)** en la barra inferior.
   * Allí encontrarás tanto la configuración del kernel (`configTICK_RATE_HZ = 1000`) como el menú propio de esta aplicación (definido en `main/Kconfig.projbuild`) para elegir `MAIN_MODE`, pines del AS5600, PWM y credenciales WiFi. Haz clic en **Save**.
4. **Compilar, Flashear y Monitorear:**
   * Haz clic en el **cilindro/llave inglesa (`Build Project`)**, luego en el **rayo (`Flash Project`)** y finalmente en la **pantalla (`Monitor Device`)** (o usa el ícono de **llama** para ejecutar los tres pasos en secuencia).

### 8.3. Protocolos de Comandos Seriales para Sintonización y Pruebas en Vivo

Dependiendo del archivo que hayas compilado en `main/CMakeLists.txt`, tienes dos protocolos de comandos a través del monitor serial (`UART_NUM_0` a `115200 baud`):

#### A. Formato ASCII Clásico (en `main.c` y `uart_task` de `pid_calibration.c`)
Envía cualquiera de las siguientes cadenas de texto por la terminal serial:
* **Actualizar ganancias PID y velocidad objetivo (`P`):**
  ```text
  P <Kp> <Ki> <Kd> <SetPoint>
  ```
  *Ejemplo:* `P 1.5 0.1 0.05 10.0` $\rightarrow$ Fija $K_p = 1.5$, $K_i = 0.1$, $K_d = 0.05$ y referencia de velocidad `set_point = 10.0 cm/s`.
* **Cambiar únicamente la velocidad objetivo (`S`):**
  ```text
  S <SetPoint>
  ```
  *Ejemplo:* `S 15.0` $\rightarrow$ Actualiza `set_point = 15.0 cm/s`.
* **Aplicar un pulso de ciclo útil PWM en lazo abierto durante 2 segundos (`X`):**
  ```text
  X <Duty>
  ```
  *Ejemplo:* `X 55` $\rightarrow$ Activa `move_one_time = true` aplicando ciclo útil `55` durante $2000\text{ ms}$ y luego apaga el motor.
* **Mover una distancia objetivo hacia la derecha o izquierda (`D` / `I`):**
  ```text
  D<Distancia_cm>_<Velocidad_cm_s>
  I<Distancia_cm>_<Velocidad_cm_s>
  ```
  *Ejemplo:* `D100_15.0` $\rightarrow$ Mueve la rueda en sentido positivo a $15.0\text{ cm/s}$ hasta acumular $100\text{ cm}$ en el encoder y se detiene automáticamente.

#### B. Formato JSON (en `vTaskUartParser` de `pid_calibration.c`)
Si utilizas las tareas `uart_handler` + `uart_parser` activas por defecto en `pid_calibration.c`, el parser espera una cadena con formato JSON terminada en salto de línea (`\n`):
```json
{"default":"1.66 0.60 0.05 12.0"}
```
*(Donde los 4 valores separados por espacio corresponden a `Kp Ki Kd SetPoint`).*

---

## 9. Variables Críticas de `Kconfig` (`sdkconfig` / `menuconfig`)

Esta aplicación ya cuenta con un archivo **`main/Kconfig.projbuild`** que expone en `menuconfig` las constantes de pines y operación (`CONFIG_MAIN_MODE`, `CONFIG_SAMPLE_TIME`, `CONFIG_AS5600_*`, `CONFIG_PWM_*`, `CONFIG_WIFI_*`). Antes de compilar, asegúrate de verificar en `idf.py menuconfig` (o en el editor gráfico de VS Code) los siguientes parámetros críticos:

| Variable en `sdkconfig` | Ruta en `menuconfig` / `Kconfig.projbuild` | Valor Recomendado | Impacto Crítico en `wheel-movement` |
| :--- | :--- | :---: | :--- |
| **`CONFIG_FREERTOS_HZ`** | *Component config $\rightarrow$ FreeRTOS $\rightarrow$ Kernel $\rightarrow$ configTICK_RATE_HZ* | **`1000`** *(1000 Hz)* | **Crítico:** Tanto `main.c` como `pid_calibration.c` ejecutan `vTaskDelay(2 / portTICK_PERIOD_MS)`. Si `CONFIG_FREERTOS_HZ` está en `100` Hz, `2 / 10` da **`0` ticks**, provocando que `control_task` y `encoder_task` corran sin pausa a máxima velocidad de CPU, corrompiendo el $\Delta t = 2\text{ ms}$ del estimador de velocidad y disparando el *Task Watchdog*. |
| **`CONFIG_MAIN_MODE`** | *Menú del Proyecto (`Kconfig.projbuild`) $\rightarrow$ Main mode* | **`0`** *(Control PID)*<br>`1` *(Cal. ESC)*<br>`3` *(Probe AS5600)* | Selecciona qué bloque `#if MAIN_MODE == ...` se compila en `main.c`. *(Nota: en el comentario de `main.c` dice `2 AS5600 PROBE`, pero la directiva en la línea 630 es `#if MAIN_MODE == 3`; alinéalos en `2`).* |
| **`CONFIG_SAMPLE_TIME`** | *Menú del Proyecto (`Kconfig.projbuild`) $\rightarrow$ Sample time* | **`2`** *(2 ms = 500 Hz)* | Tiempo de muestreo del lazo de velocidad. *(Recuerda eliminar la línea duplicada `#define SAMPLE_TIME 2` en la línea 67 de `main.c` para que respete el valor de `CONFIG_SAMPLE_TIME`).* |
| **`CONFIG_AS5600_MODE`** | *Menú del Proyecto $\rightarrow$ AS5600 Configuration* | **`1`** *(Angle via ADC)* | Define que la lectura continua de ángulo se realice por el conversor analógico-digital (`AS5600_OUT_GPIO`) tras configurar el chip por I2C. |
| **`CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ`** | *Component config $\rightarrow$ ESP System Settings $\rightarrow$ CPU frequency* | **`240`** *(240 MHz)* | Máxima velocidad de reloj del ESP32-S3 para minimizar la latencia de conversión ADC y cómputo en punto flotante. |
| **`CONFIG_COMPILER_OPTIMIZATION_PERF`** | *Compiler options $\rightarrow$ Optimization Level* | **`-O2` (Performance)** | Optimiza el cálculo de filtros exponenciales (`exp`, `fabsf`) y del controlador PID incremental. |
