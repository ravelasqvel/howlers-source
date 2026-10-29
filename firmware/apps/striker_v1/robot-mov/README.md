| Supported Targets | ESP32-S3 |
| ----------------- | -------- |

# ROBOT-MOV (`striker_v1`): FreeRTOS-Based Mobile Robot Control System

Este proyecto implementa el sistema de control de movimiento holonómico en tiempo real para el robot omnidireccional de 3 ruedas **Striker 1** sobre el SoC **ESP32-S3** utilizando **ESP-IDF y FreeRTOS SMP**. Integra la lectura de tres encoders magnéticos (`AS5600` vía ADC), una unidad de medición inercial (`TM151` vía UART), un sensor de distancia láser Time-of-Flight (`VL53L1X` vía I2C), variadores electrónicos para motores Brushless (`bldc_pwm`), controladores PID discretos (`pid_ext`) y un servidor de comandos **WiFi/UDP**.

---

## 1. Estructura del Proyecto (Monorepo Unificado)

Tras la centralización de componentes en `firmware/components/`, la aplicación conserva únicamente su lógica de negocio y configuración de tareas en `main/`, enlazando los controladores compartidos `v1_striker1`:

```text
firmware/apps/striker_v1/robot-mov/
├── CMakeLists.txt                  # Configuración del proyecto ESP-IDF
├── sdkconfig                       # Configuración del kernel FreeRTOS, ADC, UART y WiFi
├── README.md                       # Documentación técnica y análisis de concurrencia
└── main/
    ├── CMakeLists.txt              # Enlace con firmware/components/*/v1_striker1
    ├── include/
    │   ├── control_main.h          # Definición de pines, estructuras y prototipos de tareas
    │   └── mov_calculation.h       # Constantes cinemáticas (R = 3.25 cm, N = 16, DELTA = pi/6)
    ├── main.c                      # Inicialización de periféricos y creación de tareas FreeRTOS
    ├── control_main.c              # Implementación de las 6 funciones de tarea y estructuras globales
    ├── mov_calculation.c           # Cinemática inversa holonómica (lineal, circular y conversión a rueda)
    ├── encoder_calibration.c       # Rutina standalone para calibración/quemado OTP de sensores AS5600
    ├── escs_calibration.c          # Rutina standalone para calibración de límites PWM de los ESCs
    └── test_wheels.c               # Rutina standalone de prueba en lazo abierto de los 3 motores BLDC
```

**Componentes enlazados desde `firmware/components/`:**
* `sensors/as5600/v1_striker1` — Lectura analógica (ADC Oneshot) e I2C de los encoders magnéticos.
* `sensors/tm151/v1_striker1` — Decodificación de tramas seriales (`EasyProtocol` / `EasyQueue`) de la IMU TM151.
* `sensors/vl53l1x/v1_striker1` — Sensor láser ToF VL53L1X.
* `actuators/bldc_pwm/v1_striker1` — Generación PWM de 50 Hz e inversión de giro para los 3 variadores ESC.
* `control/pid_lib/v1_striker1` — Controlador PID discreto (`PID_CAL_TYPE_INCREMENTAL`).
* `control/sensor_fusion/v1_striker1` — Estimación y filtrado de velocidad/posición.
* `comms/wifi_udp/v1_striker1` — Inicialización WiFi Station y utilidades de red.
* `platform_hal/v1_striker1` — Abstracción de periféricos del ESP32-S3 (`ADC`, `UART`, `I2C`, `MCPWM`).

---

## 2. Inicialización del Sistema (`main.c`)

La función `app_main()` inicializa los periféricos antes de arrancar el planificador de tareas:
1. **WiFi Station:** Conecta el ESP32-S3 a la red local (`wifi_init_station()` y `dev_wifi_init()`) y obtiene su dirección IP para escuchar comandos UDP.
2. **Motores BLDC (`pwmR`, `pwmL`, `pwmB`):** Configura los pines PWM y de inversión (`PWM_GPIO_R=20/21`, `PWM_GPIO_L=47/48`, `PWM_GPIO_B=45/0`) a $50\text{ Hz}$ y los deja armados con ciclo útil `0`.
3. **Encoders Magnéticos AS5600 (`gAs5600R`, `gAs5600L`, `gAs5600B`):** Crea una única unidad ADC (`ADC_UNIT_1` mediante `adc_oneshot_unit_handle_t`) y configura 3 canales analógicos en los pines `GPIO 5` (Derecha), `GPIO 6` (Izquierda) y `GPIO 7` (Trasera).
4. **IMU TM151 (`myUART`):** Inicializa el puerto UART en `TX=17`, `RX=18` a `115600 baud`.
5. **Bloques PID (`pidR`, `pidL`, `pidB`):** Instancia los 3 bloques de control en modo incremental (`PID_CAL_TYPE_INCREMENTAL`, salida acotada en $[-70, +70]$).

---

## 3. Análisis de la Estructura de Tareas FreeRTOS

En `main.c` se definen **10 tareas de FreeRTOS** (8 activas en compilación y 2 comentadas para pruebas de IMU y LiDAR), distribuidas explícitamente entre los dos núcleos del **ESP32-S3 (`Core 0` y `Core 1`)** mediante `xTaskCreatePinnedToCore`:

| Nombre de Tarea | Función C | Core | Prioridad | Stack (B) | Periodo / Bloqueo | Rol en el Sistema |
| :--- | :--- | :---: | :---: | :---: | :--- | :--- |
| **`rwh_control_task`** | `vTaskControl` | **Core 1** | **9** *(Máx)* | `4096` | `2 ms` (`SAMPLE_TIME`) | Calcula consigna cinemática, ejecuta PID incremental y actualiza PWM de la rueda **Derecha**. |
| **`lwh_control_task`** | `vTaskControl` | **Core 1** | **9** *(Máx)* | `4096` | `2 ms` (`SAMPLE_TIME`) | Calcula consigna cinemática, ejecuta PID incremental y actualiza PWM de la rueda **Izquierda**. |
| **`bwh_control_task`** | `vTaskControl` | **Core 1** | **9** *(Máx)* | `4096` | `2 ms` (`SAMPLE_TIME`) | Calcula consigna cinemática, ejecuta PID incremental y actualiza PWM de la rueda **Trasera**. |
| **`distance_task`** | `vTaskDistance` | **Core 1** | **8** | `4096` | `10 ms` (`5 * SAMPLE_TIME`) | Temporizador de trayectoria: acumula `time_count` y detiene el robot (`movement = DO_NOT_MOVE`) al alcanzar `goal_time`. |
| **`UDPServer`** | `vTaskUDPServer`| **Core 1** | **8** | `2048` | Bloqueante (`recvfrom`) | Escucha tramas UDP (`"L ..."`, `"C ..."`, `"R ..."`) y actualiza las variables globales de movimiento. |
| **`right_encoder_task`**| `vTaskEncoder` | **Core 0** | **8** | `4096` | `2 ms` (`SAMPLE_TIME`) | Lee el canal ADC de `GPIO 5` (`AS5600_ADC_GetAngle`) y estima `right_encoder_data.velocity`. |
| **`left_encoder_task`** | `vTaskEncoder` | **Core 0** | **8** | `4096` | `2 ms` (`SAMPLE_TIME`) | Lee el canal ADC de `GPIO 6` (`AS5600_ADC_GetAngle`) y estima `left_encoder_data.velocity`. |
| **`back_encoder_task`** | `vTaskEncoder` | **Core 0** | **8** | `4096` | `2 ms` (`SAMPLE_TIME`) | Lee el canal ADC de `GPIO 7` (`AS5600_ADC_GetAngle`) y estima `back_encoder_data.velocity`. |
| *`imu_task` (inactiva)* | `vTaskIMU` | *Core 0* | *8* | `4096` | `2 ms` (`SAMPLE_TIME`) | Lee aceleración triaxial y Yaw desde el buffer UART de la IMU TM151. |
| *`lidar_task` (inactiva)*| `vTaskLidar` | *Sin afinidad*| *9* | `2048` | `2 ms` (`SAMPLE_TIME`) | Lee distancia por I2C del sensor VL53L1X y estima velocidad lineal. |

---

## 4. Análisis del Uso de Objetos del Kernel de FreeRTOS y Sincronización

### 4.1. ¿Qué objetos del kernel utiliza realmente `robot-mov`?

Al inspeccionar `control_main.h`, `main.c` y `control_main.c`, se observa un hallazgo arquitectónico clave:
* Aunque `control_main.h` incluye los encabezados `<freertos/queue.h>`, `<freertos/semphr.h>` y `<driver/gptimer.h>`, **el código de la aplicación no instancia ninguna cola (`QueueHandle_t`), ni semáforo/mutex (`SemaphoreHandle_t`), ni grupo de eventos (`EventGroupHandle_t`)** para comunicar las tareas entre sí.
* Los únicos objetos y servicios del kernel de FreeRTOS que se utilizan activamente son:
  1. **Descriptores de Tarea (`TaskHandle_t`):** Creados con `xTaskCreatePinnedToCore()`, verificados al arranque con `configASSERT(handle)` y consultados dentro de las tareas mediante `xTaskGetCurrentTaskHandle()` y `pcTaskGetName()` para etiquetar mensajes de log. En `vTaskUDPServer` se usa además `vTaskDelete(NULL)` para autodestruir la tarea si falla la apertura del socket.
  2. **Temporización Relativa del Kernel (`vTaskDelay`):** Todas las tareas periódicas (`vTaskEncoder`, `vTaskControl`, `vTaskDistance`, `vTaskIMU`, `vTaskLidar`) regulan su ciclo mediante `vTaskDelay(SAMPLE_TIME / portTICK_PERIOD_MS)`.
  3. **Sockets Bloqueantes de LwIP (Integrados con el Scheduler):** La tarea `vTaskUDPServer` permanece en estado *Blocked* (consumiendo 0% de CPU) dentro de la llamada al sistema `recvfrom(sock, ...)` hasta que el stack WiFi/LwIP despierta la tarea al recibir un datagrama UDP.
  4. **Comunicación Inter-Tareas mediante Memoria Global Compartida Sin Protección (*Unprotected Shared Memory*) y Punteros en `pvParameters`:** Toda la transferencia de datos entre el **Core 0** y el **Core 1** ocurre leyendo y escribiendo directamente estructuras globales (`right_encoder_data`, `left_encoder_data`, `back_encoder_data`) y variables escalares globales (`movement`, `x_vel`, `y_vel`, `goal_time`).

---

### 4.2. Diagrama de Relación de Tareas, Núcleos y Recursos Compartidos

El siguiente diagrama ilustra cómo se relacionan las tareas entre el **Core 0** y el **Core 1**, qué objetos del kernel intervienen en su planificación y dónde se encuentran los puntos de acoplamiento por memoria compartida y periféricos físicos:

```text
=================================================================================================================
                 ARQUITECTURA DE TAREAS Y SINCRONIZACIÓN EN ROBOT-MOV (ESP32-S3 SMP)
=================================================================================================================

      [ CORE 0 (PRO_CPU): ADQUISICIÓN DE SENSORES ]              [ RECURSO HARDWARE COMPARTIDO SIN MUTEX ]
  +---------------------------------------------------+      +-----------------------------------------------+
  | • right_encoder_task (Prio 8, Periodo: 2 ms)      |─────►| Periférico ADC_UNIT_1 (adc_oneshot_handle)    |
  | • left_encoder_task  (Prio 8, Periodo: 2 ms)      |─────►| ¡3 tareas llaman a AS5600_ADC_GetAngle()      |
  | • back_encoder_task  (Prio 8, Periodo: 2 ms)      |─────►|  concurrentemente sobre el mismo handle ADC!  |
  |   [Objeto Kernel: vTaskDelay(2 ms)]               |      +-----------------------------------------------+
  +-------------------------+-------------------------+
                            │
                            │ Escritura directa (sin Mutex / sin Queue)
                            ▼
  +===========================================================================================================+
  |                     MEMORIA GLOBAL COMPARTIDA ENTRE NÚCLEOS (DRAM INTERNA)                                |
  |                                                                                                           |
  |  1. Estado de Ruedas (Pasado por puntero en pvParameters -> control_params_t):                            |
  |     • right_encoder_data.velocity   • left_encoder_data.velocity   • back_encoder_data.velocity           |
  |                                                                                                           |
  |  2. Estado de Comando y Trayectoria (Variables globales directas en control_main.c):                      |
  |     • enum movements_num movement   • float x_vel, y_vel           • float goal_time                      |
  +===========================================================================================================+
            │                                            ▲                                    ▲
            │ Lectura directa cada 2 ms                  │ Escritura al recibir               │ Lectura/Escritura
            │ (sin sección crítica)                      │ paquete UDP                        │ cada 10 ms
            ▼                                            │                                    ▼
  +-----------------------------------------+  +-----------------------------------+  +-----------------------+
  | [ CORE 1 (APP_CPU): LAZO DE CONTROL ]   |  | [ CORE 1: COMUNICACIONES UDP ]    |  | [ CORE 1: SUPERVISOR ]|
  | • rwh_control_task (Prio 9, T = 2 ms)   |  | • UDPServer (Prio 8, Stack 2048B) |  | • distance_task       |
  | • lwh_control_task (Prio 9, T = 2 ms)   |  |   [Objeto: Socket LwIP recvfrom()]|  |   (Prio 8, T = 10 ms) |
  | • bwh_control_task (Prio 9, T = 2 ms)   |  |   Parsea trama "L dir deg vel d": |  |   [vTaskDelay(10 ms)] |
  |   [Objeto Kernel: vTaskDelay(2 ms)]     |  |   1) movement = LINEAR            |  |   Si time_count >=    |
  |   1) Lee encoder_data->velocity         |  |   2) goal_time = dist / vel       |  |      goal_time * 1000:|
  |   2) Lee movement, x_vel, y_vel         |  |   3) linear_movement(&x_vel,      |  |   -> movement =       |
  |   3) Calcula setpoint (cinemática inv.) |  |                      &y_vel)      |  |      DO_NOT_MOVE      |
  |   4) pid_compute() -> bldc_set_duty()   |  +-----------------------------------+  +-----------------------+
  +-----------------------------------------+
```

---

### 4.3. Diagnóstico de Problemas de Concurrencia y Tiempo Real (Guía de Mejora para `Striker v3`)

Este análisis sirve como línea base directa para los estudiantes que desarrollan la nueva versión del firmware:

1. **Condición de carrera en el periférico `ADC_UNIT_1` (`Core 0`):**
   * En `main.c`, las tres estructuras `gAs5600R`, `gAs5600L` y `gAs5600B` reciben exactamente el **mismo `adc_oneshot_unit_handle_t handle`**.
   * Como las tres tareas `right_encoder_task`, `left_encoder_task` y `back_encoder_task` tienen la misma prioridad (`8`) en el `Core 0`, el planificador de FreeRTOS alterna entre ellas por *Time Slicing* (Round-Robin). Si un cambio de contexto ocurre en medio de `adc_oneshot_read()`, dos tareas acceden simultáneamente al hardware del ADC1 sin un `SemaphoreHandle_t` (Mutex), pudiendo corromper la lectura analógica.
   * **Solución:** Unificar la lectura de los 3 encoders en **una sola tarea `vTaskEncoders`** que lea secuencialmente los 3 canales ADC (eliminando cambios de contexto innecesarios y ahorrando $2 \times 4096\text{ B}$ de RAM en stacks), o proteger el `handle` del ADC con un Mutex.
2. **Condición de carrera en las variables globales de trayectoria (`movement`, `x_vel`, `y_vel`, `goal_time`):**
   * Cuando `vTaskUDPServer` recibe un comando `"L Forward ..."`, escribe secuencialmente:
     ```c
     movement = LINEAR;
     goal_time = distance / velocity;
     linear_movement(forward, distance, degrees, &x_vel, &y_vel);
     ```
   * Como las 3 tareas `vTaskControl` corren en el mismo **Core 1** con **mayor prioridad (`9` > `8`)**, en el instante en que vence su `vTaskDelay(2 ms)` expropian a `vTaskUDPServer` a mitad de esas tres líneas. Esto provoca que las ruedas lean `movement == LINEAR` antes de que `x_vel`, `y_vel` o `goal_time` hayan terminado de actualizarse (o que `vTaskDistance` compare `time_count` contra un `goal_time` antiguo y detenga el movimiento inmediatamente).
   * **Solución:** Enviar los comandos desde `vTaskUDPServer` hacia el controlador mediante una **Cola de FreeRTOS (`xQueueSend` / `xQueueOverwrite`)** con una estructura empaquetada `motion_command_t`, de modo que la actualización sea 100% atómica.
3. **Deriva temporal (*Time Drift*) por uso de `vTaskDelay()` en lugar de `vTaskDelayUntil()`:**
   * Tanto `vTaskEncoder` como `vTaskControl` usan `vTaskDelay(SAMPLE_TIME / portTICK_PERIOD_MS)`. Como `vTaskDelay` espera un tiempo relativo *a partir de que termina la iteración actual*, el periodo real de muestreo es $T_{real} = 2\text{ ms} + t_{ejecución} + t_{interferencia}$, pero las ecuaciones de derivada en `estimate_velocity_encoder()` y la integral del PID asumen un $\Delta t = 2.0\text{ ms}$ constante.
   * Además, si `CONFIG_FREERTOS_HZ` está en `100 Hz` (1 tick = 10 ms), la división entera `2 / 10` vale **`0` ticks**. Es indispensable configurar `CONFIG_FREERTOS_HZ=1000` y utilizar **`vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(SAMPLE_TIME))`**.
4. **Bug de variable `static` compartida en `circular_movement()` y `vTaskControl`:**
   * En `mov_calculation.c`, `circular_movement()` declara `static float t = 0;` e incrementa `t += 0.002;`. Si se descomenta dentro de `vTaskControl`, las 3 tareas de rueda (`rwh`, `lwh`, `bwh`) llamarán a `circular_movement()` en cada ciclo, haciendo que `t` avance $3 \times 0.002\text{ s} = 0.006\text{ s}$ cada $2\text{ ms}$ y nunca se reinicie a `0` para una segunda maniobra circular.
   * Igualmente, en `vTaskEncoder` y `vTaskIMU`, los contadores de impresión `static int counter = 0;` son compartidos por todas las instancias que ejecutan la misma función de tarea.

---

## 5. Módulos de Estimación, Cinemática y Control

### 5.1. Cinemática Holonómica (`mov_calculation.c`)
Convierte la velocidad lineal deseada del chasis $(v_x, v_y)$ en la velocidad tangencial/angular de cada una de las 3 ruedas dispuestas a $\delta = 30^\circ$ ($\pi/6\text{ rad}$), con relación de reducción planetaria $N = 16$ y radio de rueda $R = 3.25\text{ cm}$:
* **Rueda Izquierda (`vel_selection = 0`):** $v_L = \frac{N}{5R} (-\sin\delta \cdot v_x - \cos\delta \cdot v_y)$
* **Rueda Trasera (`vel_selection = 1`):** $v_B = \frac{N}{5R} (v_x)$
* **Rueda Derecha (`vel_selection = 2`):** $v_R = \frac{N}{5R} (-\sin\delta \cdot v_x + \cos\delta \cdot v_y)$

### 5.2. Fusión Sensorial (`sensor_fusion.c`)
* **`estimate_velocity_encoder(...)`**: Calcula la velocidad lineal de la rueda a partir del diferencial de ángulo del AS5600 y el radio de rueda, aplicando un filtro pasa-bajas exponencial y umbral de zona muerta contra ruido.
* **`estimate_velocity_imu(...)`**: Integra por regla trapezoidal la aceleración lineal de la IMU TM151 con suavizado exponencial.
* **`estimate_velocity_lidar(...)`**: Deriva la distancia medida por el sensor ToF VL53L1X con filtro de media móvil.
* **`estimate_velocity(...)`**: Compara las tres estimaciones (IMU, encoder y LiDAR), selecciona el par con menor desviación mutua y promedia ambos valores para rechazar deslizamientos o lecturas atípicas.

---

## 6. Compilación, Flasheo y Uso del Servidor UDP

### 6.1. Compilar y Flashear en ESP-IDF (Windows PowerShell)
```powershell
cd firmware\apps\striker_v1\robot-mov
idf.py set-target esp32s3
idf.py build
idf.py -p COM5 flash monitor
```

### 6.2. Formato de Comandos UDP (`vTaskUDPServer`)
La tarea `UDPServer` escucha datagramas de texto ASCII en el puerto configurado (`PORT`) con tres formatos de comando:
* **Movimiento Lineal (`L`):** `L <Forward|Backward> <grados> <velocidad_cm_s> <distancia_cm>`  
  *(Ejemplo: `L Forward 0.0 15.0 100.0` $\rightarrow$ avanza en ángulo $0^\circ$ a $15\text{ cm/s}$ durante $t = 100/15 = 6.67\text{ s}$).*
* **Movimiento Circular (`C`):** `C <direction> <grados> <velocidad_cm_s> <radio_cm>`
* **Rotación sobre su eje (`R`):** `R <direction> <grados> <velocidad_cm_s>`

Puedes enviar estos comandos utilizando la interfaz web y el proxy incluidos en \[suspicious link removed\] (`proxy_udp.py` + `index.html`).
