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
├── README.md                       # Documentación técnica, análisis de concurrencia y TODOs
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
| *`imu_task` (inactiva)* | `vTaskIMU` | *Core 0* | *8* | `4096` | `2 ms` (`SAMPLE_TIME`) | Lee aceleración triaxial y Yaw desde el buffer UART de la IMU TM151 (comentada en `main.c`). |
| *`lidar_task` (inactiva)*| `vTaskLidar` | *Sin afinidad*| *9* | `2048` | `2 ms` (`SAMPLE_TIME`) | Lee distancia por I2C del sensor VL53L1X y estima velocidad lineal (comentada en `main.c`). |

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
   * Como las tres tareas `right_encoder_task`, `left_encoder_task` y `back_encoder_task` tienen la misma prioridad (`8`) en el `Core 0`, el planificador de FreeRTOS alterna entre ellas por *Time Slicing* (Round-Robin). Si un cambio de contexto ocurre en medio de `adc_oneshot_read()`, dos tareas acceden simultáneamente al hardware del ADC1 sin sincronización, pudiendo corromper la lectura analógica.
2. **Condición de carrera en las variables globales de trayectoria (`movement`, `x_vel`, `y_vel`, `goal_time`):**
   * Cuando `vTaskUDPServer` recibe un comando `"L Forward ..."`, escribe secuencialmente:
     ```c
     movement = LINEAR;
     goal_time = distance / velocity;
     linear_movement(forward, distance, degrees, &x_vel, &y_vel);
     ```
   * Como las 3 tareas `vTaskControl` corren en el mismo **Core 1** con **mayor prioridad (`9` > `8`)**, en el instante en que vence su `vTaskDelay(2 ms)` expropian a `vTaskUDPServer` a mitad de esas tres líneas. Esto provoca que las ruedas lean `movement == LINEAR` antes de que `x_vel`, `y_vel` o `goal_time` hayan terminado de actualizarse (o que `vTaskDistance` compare `time_count` contra un `goal_time` antiguo y detenga el movimiento inmediatamente).
3. **Deriva temporal (*Time Drift*) por uso de `vTaskDelay()` en lugar de `vTaskDelayUntil()`:**
   * Tanto `vTaskEncoder` como `vTaskControl` usan `vTaskDelay(SAMPLE_TIME / portTICK_PERIOD_MS)`. Como `vTaskDelay` espera un tiempo relativo *a partir de que termina la iteración actual*, el periodo real de muestreo es $T_{real} = 2\text{ ms} + t_{ejecución} + t_{interferencia}$, pero las ecuaciones de derivada en `estimate_velocity_encoder()` y la integral del PID asumen un $\Delta t = 2.0\text{ ms}$ constante.
   * Además, si `CONFIG_FREERTOS_HZ` está en `100 Hz` (1 tick = 10 ms), la división entera `2 / 10` vale **`0` ticks**. Es indispensable configurar `CONFIG_FREERTOS_HZ=1000` y utilizar mecanismos de temporización absoluta o timers hardware.
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

## 6. Estado Real de la IMU (TM151) en `robot-mov`

Aunque la unidad inercial **TM151** está cableada por UART y se inicializa formalmente en `main.c`, **en la práctica no se utiliza para ninguna función útil en la operación ni en el control del robot**:

1. **La tarea de lectura está comentada en `main.c`:**
   En las líneas 180 a 186 de `main.c`, la creación de `vTaskIMU` (`xTaskCreatePinnedToCore(vTaskIMU, "imu_task", ...)`) está comentada. Por lo tanto, durante el funcionamiento normal, el microcontrolador jamás lee los registros ni procesa los paquetes seriales de la IMU.
2. **La estimación de velocidad está comentada en `control_main.c`:**
   Dentro de `vTaskIMU`, la llamada a `estimate_velocity_imu(...)` se encuentra comentada. Si se llegara a reactivar la tarea, su único efecto sería leer los datos por UART para imprimir la aceleración y el ángulo Yaw en el log serial cada $300\text{ ms}$, sin alimentar ningún lazo del sistema.
3. **El lazo de control PID y la cinemática ignoran por completo la orientación ($\theta$):**
   * En `mov_calculation.c`, la función `cal_lin_to_ang_velocity()` calcula las velocidades asumiendo explícitamente que la velocidad angular del cuerpo es cero (`// wb = 0, so ignore third column`).
   * En `vTaskControl`, la realimentación proviene exclusivamente de los encoders magnéticos (`est_velocity = encoder_data->velocity`). No existe ningún lazo cerrado de orientación (*heading / yaw control*) que use la IMU para corregir desvíos o giros no deseados del robot al desplazarse.
   * Las funciones de `sensor_fusion.c` (`estimate_velocity(...)`), que teóricamente fusionaban IMU, encoder y LiDAR, **no son invocadas por ninguna tarea en ejecución**.

---

## 7. Lista de Tareas Pendientes (TODOs para Striker 1 / Línea Base Striker v3)

Esta sección consolida las oportunidades de mejora y tareas pendientes identificadas en `robot-mov`. Para cada problema se formulan las **alternativas técnicas viables**, de modo que los estudiantes deban **analizar el balance entre sobrecarga de CPU (*overhead*), latencia, consumo de memoria y determinismo temporal** para justificar cuál solución es la más eficiente en su diseño de `Striker v3`:

- [ ] **TODO 1: Habilitar y Cerrar Lazo con la IMU (Heading / Yaw Control):**
  * **Problema:** El robot derrapa o gira involuntariamente durante traslaciones lineales al no tener realimentación de orientación.
  * **Alternativas a evaluar:**
    * *Alternativa A (Control en cascada o paralelo):* Un lazo PID de orientación que calcule $\omega_z$ y lo inyecte en la 3.ª columna de la cinemática inversa. Evaluar si la frecuencia de actualización del TM151 por UART (o BNO055 por I2C) es suficiente para correr al mismo periodo del lazo de rueda ($2\text{ ms}$) o si debe ejecutarse como un lazo externo más lento ($10\text{–}20\text{ ms}$).
    * *Alternativa B (Estimación probabilística de pose):* Integrar el ángulo Yaw y las velocidades en el filtro EKF/UKF (`firmware/components/control/state_estimation/`) y controlar directamente en el espacio de estados $(x, y, \theta)$. Comparar la carga de cómputo matricial frente al beneficio en precisión.

- [ ] **TODO 2: Integrar la Fusión Sensorial en el Lazo de Realimentación:**
  * **Problema:** `sensor_fusion.c` está desconectado y el PID solo usa la velocidad cruda del encoder magnético.
  * **Alternativas a evaluar:**
    * *Alternativa A (Fusión heurística ligera):* Reactivar `estimate_velocity(...)` (par con menor desviación entre IMU, encoder y LiDAR). Evaluar el costo computacional de ejecutarla en cada ciclo de 2 ms frente a su efectividad real ante patinaje de ruedas.
    * *Alternativa B (Filtro complementario o Kalman unidimensional por rueda):* Fusionar aceleración tangencial con derivada del encoder para filtrar ruido de cuantización sin introducir retardo de fase.
    * *Alternativa C (Fusión centralizada en el cuerpo del robot):* Mantener el lazo interno de rueda con el encoder filtrado y delegar la fusión sensorial únicamente a la estimación de odometría global del chasis.

- [ ] **TODO 3: Resolver la Concurrencia y Competencia por el Periférico `ADC_UNIT_1`:**
  * **Problema:** Tres tareas independientes (`Core 0`, Prioridad 8) invocan concurrentemente `AS5600_ADC_GetAngle()` sobre el mismo manejador `adc_oneshot_unit_handle_t` sin sincronización.
  * **Alternativas a evaluar:**
    * *Alternativa A (Unificación en una sola tarea de adquisición):* Reemplazar las 3 tareas por una única tarea `vTaskEncoders` que lea secuencialmente los canales 5, 6 y 7. Evaluar el ahorro en memoria RAM (eliminación de 2 stacks de 4 KB = 8 KB ahorrados) y la eliminación total de cambios de contexto innecesarios.
    * *Alternativa B (Sincronización por Mutex):* Mantener las 3 tareas y proteger las llamadas a `adc_oneshot_read()` con un `SemaphoreHandle_t` tipo Mutex con herencia de prioridad. Comparar la sobrecarga de tiempo de bloqueo (*blocking overhead*) frente a la Alternativa A.
    * *Alternativa C (Lectura por interrupción de Timer Hardware o DMA):* Configurar un temporizador periódico que dispare la conversión ADC en ráfaga y notifique a la tarea de control mediante `xTaskNotifyFromISR()`.

- [ ] **TODO 4: Eliminar las Condiciones de Carrera en la Comunicación de Trayectoria y Comandos:**
  * **Problema:** `vTaskUDPServer` escribe en memoria global compartida (`movement`, `x_vel`, `y_vel`, `goal_time`) mientras `vTaskControl` (mayor prioridad) y `vTaskDistance` leen concurrentemente sin sincronización.
  * **Alternativas a evaluar:**
    * *Alternativa A (Colas de FreeRTOS - `QueueHandle_t`):* Empaquetar el comando en una estructura `motion_cmd_t` y transmitirla con `xQueueOverwrite()` hacia el lazo de control. Evaluar la seguridad de copia por valor y la semántica de paso de mensajes frente al costo en ciclos de CPU.
    * *Alternativa B (Estructura protegida por Mutex o Spinlock):* Mantener una estructura de estado compartida en DRAM y proteger su lectura/escritura mediante un Mutex o una sección crítica ligera de FreeRTOS (`taskENTER_CRITICAL()` / spinlock SMP). Comparar la latencia de acceso frente a las colas.
    * *Alternativa C (Grupo de Eventos + Notificación Directa - `EventGroupHandle_t` / `xTaskNotify`):* Usar bits de eventos para señalizar cambios de modo (*Linear*, *Circular*, *Stop*) y pasar los valores numéricos mediante punteros con doble buffer (*ping-pong buffering*) libre de bloqueos.

- [ ] **TODO 5: Eliminar la Deriva Temporal (*Time Drift*) y Garantizar Muestreo Determinista a 1 kHz:**
  * **Problema:** El uso de `vTaskDelay()` acumula el tiempo de ejecución en cada ciclo, alterando el periodo de integración/derivación, y depende críticamente de `CONFIG_FREERTOS_HZ`.
  * **Alternativas a evaluar:**
    * *Alternativa A (Temporización absoluta con `vTaskDelayUntil`):* Mantener el control en tareas de FreeRTOS y sustituir los retardos relativos por `vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(SAMPLE_TIME))`, asegurando `CONFIG_FREERTOS_HZ=1000`. Evaluar el *jitter* resultante frente a la carga de otras tareas.
    * *Alternativa B (Disparo directo desde Temporizador de Hardware - `gptimer` / `esp_timer`):* Ejecutar el cálculo del PID o la notificación de despertar estrictamente desde la ISR de un temporizador de hardware de alta resolución, desacoplando la base de tiempo de los ticks del kernel. Evaluar las restricciones de código ejecutable dentro de una ISR (`IRAM_ATTR`, funciones no bloqueantes).

- [ ] **TODO 6: Corregir el Estado Compartido y Reentrancia en Funciones Cinemáticas:**
  * **Problema:** Variables locales `static` en `circular_movement()` y contadores de depuración en tareas provocan efectos colaterales cuando múltiples tareas las invocan o cuando se reinicia una trayectoria.
  * **Alternativas a evaluar:**
    * *Alternativa A (Funciones puras y reentrantes):* Eliminar variables `static` internas; pasar el tiempo actual ($t$), tiempo delta ($\Delta t$) o estructuras de contexto como argumentos por referencia desde la tarea que coordina el movimiento.
    * *Alternativa B (Encapsulamiento en clases C++ o estructuras de instancia):* Modelar los generadores de trayectoria como objetos con estado propio (`LinearTrajectory`, `CircularTrajectory`), garantizando que cada instancia maneje su propio reloj y parámetros sin interferir con otras.

- [ ] **TODO 7: Optimización del Dimensionamiento de Memoria de Stacks de FreeRTOS:**
  * **Problema:** Todas las tareas tienen asignados $4096\text{ B}$ de pila por defecto, desperdiciando memoria RAM interna crítica del ESP32-S3.
  * **Alternativas a evaluar:**
    * *Alternativa A (Auditoría con `uxTaskGetStackHighWaterMark`):* Instrumentar el firmware en tiempo de ejecución para registrar el margen mínimo de pila restante en cada tarea bajo la máxima carga de trabajo, y ajustar el stack al valor real medido más un margen de seguridad del $25\text{–}30\%$.
    * *Alternativa B (Uso de Stacks Estáticos - `xTaskCreateStaticPinnedToCore`):* Asignar los buffers de pila en arreglos globales en tiempo de compilación para detectar desbordamientos de memoria en la fase de enlace (`build`) y evitar fragmentación del heap de FreeRTOS.

---

## 8. Compilación, Flasheo y Uso del Servidor UDP

### 8.1. Compilar y Flashear desde Línea de Comandos (Windows PowerShell)
```powershell
cd firmware\apps\striker_v1\robot-mov
idf.py set-target esp32s3
idf.py build
idf.py -p COM5 flash monitor
```

### 8.2. Compilar, Configurar y Flashear desde la Extensión ESP-IDF en Visual Studio Code (GUI)

Si prefieres trabajar directamente desde la interfaz gráfica de **Visual Studio Code** con la extensión oficial **Espressif IDF**:

1. **Abrir el proyecto de la aplicación:**
   * En VS Code, ve a **File $\rightarrow$ Open Folder...** y selecciona directamente la carpeta `firmware/apps/striker_v1/robot-mov` (o si abriste la raíz del monorepo `howlers-source`, presiona `Ctrl + Shift + P`, ejecuta **`ESP-IDF: Select Current ESP-IDF Workspace Folder`** y elige `firmware/apps/striker_v1/robot-mov`).
2. **Seleccionar el SoC Objetivo (`esp32s3`):**
   * En la barra de estado inferior de VS Code (color azul/morado), haz clic sobre el indicador del chip actual (por ejemplo `esp32`) o ejecuta `Ctrl + Shift + P` $\rightarrow$ **`ESP-IDF: Set Espressif Device Target`**.
   * Elige **`esp32s3`** y luego selecciona la placa/interfaz (por ejemplo, **`ESP32-S3 chip (via builtin USB-JTAG)`** o **`ESP-PROG`** según el conector USB que estés usando).
3. **Seleccionar el Puerto Serial (`COMX`):**
   * Conecta el robot por USB y haz clic en el ícono de enchufe (**`ESP-IDF: Select Port to Use`**) en la barra inferior para elegir el puerto correspondiente (por ejemplo, `COM5`).
4. **Abrir el Configurador Gráfico (`SDK Configuration Editor`):**
   * Haz clic en el ícono de **engranaje (`ESP-IDF: SDK Configuration Editor (menuconfig)`)** en la barra inferior para verificar las variables de `Kconfig` detalladas en la **Sección 9** (frecuencia de FreeRTOS a 1000 Hz, credenciales WiFi, etc.) y haz clic en **Save** arriba a la derecha.
5. **Compilar, Flashear y Monitorear con un Clic:**
   * **Compilar (`Build`):** Haz clic en el ícono de **cilindro / llave inglesa (`ESP-IDF: Build your Project`)**.
   * **Flashear (`Flash`):** Haz clic en el ícono de **rayo (`ESP-IDF: Flash your Project`)** (si te pregunta el método por primera vez, selecciona **`UART`**).
   * **Monitor Serial (`Monitor`):** Haz clic en el ícono de **pantalla (`ESP-IDF: Monitor Device`)** para ver los logs de inicialización WiFi (`get_ip_address()`) y las tramas UDP recibidas.
   * *(Atajo todo en uno)*: Haz clic en el ícono de **llama (`ESP-IDF: Build, Flash and Start a Monitor on your Device`)** en la barra inferior para ejecutar los tres pasos seguidos.

### 8.3. Formato de Comandos UDP (`vTaskUDPServer`)
La tarea `UDPServer` escucha datagramas de texto ASCII en el puerto configurado (`PORT`) con tres formatos de comando:
* **Movimiento Lineal (`L`):** `L <Forward|Backward> <grados> <velocidad_cm_s> <distancia_cm>`  
  *(Ejemplo: `L Forward 0.0 15.0 100.0` $\rightarrow$ avanza en ángulo $0^\circ$ a $15\text{ cm/s}$ durante $t = 100/15 = 6.67\text{ s}$).*
* **Movimiento Circular (`C`):** `C <direction> <grados> <velocidad_cm_s> <radio_cm>`
* **Rotación sobre su eje (`R`):** `R <direction> <grados> <velocidad_cm_s>`

Puedes enviar estos comandos utilizando la interfaz web y el proxy incluidos en `software/ground_station/striker1_proxy/` (`proxy_udp.py` + `index.html`).

---

## 9. Variables Críticas de `Kconfig` (`sdkconfig` / `menuconfig`) para `robot-mov`

Antes de compilar y probar `robot-mov` en el robot **Striker 1**, es fundamental revisar y asegurar las siguientes variables de configuración en `idf.py menuconfig` (o en el **SDK Configuration Editor** de VS Code), ya que valores incorrectos por defecto degradan gravemente el lazo de control de $2\text{ ms}$:

| Variable en `sdkconfig` | Ruta en `menuconfig` / `SDK Configuration Editor` | Valor Recomendado | Por qué es crítica en `robot-mov` |
| :--- | :--- | :---: | :--- |
| **`CONFIG_FREERTOS_HZ`** | *Component config $\rightarrow$ FreeRTOS $\rightarrow$ Kernel $\rightarrow$ configTICK_RATE_HZ* | **`1000`** *(1000 Hz)* | **Imprescindible:** `control_main.h` define `SAMPLE_TIME 2` (2 ms) y las tareas llaman a `vTaskDelay(SAMPLE_TIME / portTICK_PERIOD_MS)`. Si se deja el valor por defecto de ESP-IDF (`100` Hz $\rightarrow \text{portTICK\_PERIOD\_MS} = 10$), la división entera `2 / 10` da **`0` ticks**, haciendo que las tareas de control y encoders corran sin pausa (`vTaskDelay(0)`) y saturen la CPU. |
| **`CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ`** | *Component config $\rightarrow$ ESP System Settings $\rightarrow$ CPU frequency* | **`240`** *(240 MHz)* | Garantiza que ambos núcleos Xtensa LX7 operen a su máxima frecuencia de reloj para minimizar el tiempo de cómputo (WCET) de la cinemática trigonométrica (`sinf`, `cosf`), conversión ADC y lazo PID. |
| **`CONFIG_FREERTOS_UNICORE`** | *Component config $\rightarrow$ FreeRTOS $\rightarrow$ Kernel $\rightarrow$ Run FreeRTOS only on first core* | **`n` (Desactivado)** | Debe estar **deshabilitado** (modo **SMP Dual-Core** activo), ya que `main.c` ancla explícitamente las tareas de encoders al **Core 0** (`PRO_CPU`) y las tareas de control PID y servidor UDP al **Core 1** (`APP_CPU`). |
| **`CONFIG_ESP_TASK_WDT_EN`** y **`CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU0/1`** | *Component config $\rightarrow$ ESP System Settings $\rightarrow$ Watch CPU0/CPU1 Idle Task* | **`y`** *(5 s timeout)* | Permite detectar si alguna tarea de prioridad 8 o 9 entra en un lazo infinito sin ceder la CPU (por ejemplo, si `SAMPLE_TIME / portTICK_PERIOD_MS` cae a `0` e impide que corran `IDLE0` o `IDLE1`). |
| **`CONFIG_COMPILER_OPTIMIZATION_PERF`** | *Compiler options $\rightarrow$ Optimization Level* | **`Optimize for performance (-O2)`** | Acelera las rutinas en punto flotante de 32 bits (`float` en FPU) dentro de `mov_calculation.c`, `sensor_fusion.c` y `pid_ext.c`. |
| **`CONFIG_LWIP_SO_RCVBUF`** / **`CONFIG_ESP_WIFI_STATIC_RX_BUFFER_NUM`** | *Component config $\rightarrow$ LWIP / Wi-Fi* | **`16` buffers** | Evita pérdida de datagramas UDP en `vTaskUDPServer` cuando se envían ráfagas de comandos desde el proxy web en `striker1_proxy`. |
| **Parámetros Propios del Proyecto (Candidatos a migrar a `main/Kconfig.projbuild`)** | *Credenciales WiFi (`WIFI_SSID`, `WIFI_PASS`), Puerto UDP (`PORT`) y Tiempo de Muestreo (`SAMPLE_TIME`)* | Según red del laboratorio | Actualmente varios de estos parámetros están definidos como macros `#define` en las cabeceras de `wifi_lib.h` y `control_main.h`. Se recomienda exponerlos en un archivo `main/Kconfig.projbuild` para configurarlos desde `menuconfig` sin modificar el código fuente ni subir contraseñas de WiFi al repositorio. |