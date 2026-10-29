# Guía de Componentes y Drivers Embebidos (`firmware/components`)

Bienvenido al directorio de componentes de hardware y algoritmia embebida del equipo **Howlers FC (Grupo Robosyn — Universidad de Antioquia)** para la liga **RoboCup Small-Size League (SSL)**.

Esta guía está pensada para que cualquier estudiante o nuevo integrante del semillero entienda rápidamente **qué hace cada driver**, **qué versión debe usar según el robot (Striker 1, Striker 2 o Banco de Pruebas)**, **cómo está estructurada la arquitectura de capas en el ESP32-S3** y **cómo agregar o enlazar un nuevo periférico**.

---

## 1. ¿Cómo funciona la arquitectura del firmware?

El código embebido de nuestros robots omnidireccionales de 3 ruedas se divide en **5 capas desacopladas**. Cuando el microcontrolador **ESP32-S3** ejecuta una tarea de FreeRTOS, los datos fluyen a través de los componentes de esta carpeta de la siguiente manera:

```text
+-----------------------------------------------------------------------------------+
|                        APLICACIÓN PRINCIPAL (firmware/apps/)                      |
|         (striker_v1/robot-mov  |  striker_v2/Robot-Control  |  autotuning)        |
+-----------------------------------------------------------------------------------+
          |                                       |                        |
          v                                       v                        v
+------------------------+           +-------------------------+  +-----------------+
|   4. CONTROL & EST.    |           |     5. COMUNICACIÓN     |  |  2. ACTUADORES  |
|  (firmware/components/ |           |  (firmware/components/  |  |  (components/   |
|       control/)        |           |        comms/)          |  |   actuators/)   |
| • pid_lib (Clásico/RNN)|           | • wifi_udp (Telemetría) |  | • bldc_pwm (ESC)|
| • sensor_fusion        |           | • uart_console (Serial) |  | • led_status    |
| • state_estimation     |           +-------------------------+  +-----------------+
|   (EKF, UKF, PF)       |
+------------------------+
          |
          v
+--------------------------------------------------------------+
|                     1. DRIVERS DE SENSORES                   |
|                (firmware/components/sensors/)                |
|  • as5600 (Encoders ruedas)    • bno055 / tm151 (IMUs)       |
|  • vl53l1x / vl6180x (Láser)   • apds9960 (Óptico RGB)       |
+--------------------------------------------------------------+
          |
          v
+--------------------------------------------------------------+
|              3. CAPA DE ABSTRACCIÓN DE HARDWARE              |
|              (firmware/components/platform_hal/)             |
|    Funciones puente I2C, UART, GPIO, ADC y Timers ESP-IDF    |
+--------------------------------------------------------------+
```

---

## 2. Mapa de Navegación Rápida para Estudiantes

Si vas a trabajar en una tarea específica del robot, consulta esta tabla para saber exactamente qué carpeta abrir:

| ¿Qué necesitas hacer? | Carpeta a revisar en `firmware/components/` | Archivos clave |
| :--- | :--- | :--- |
| **Leer la velocidad o ángulo de las 3 ruedas** | `sensors/as5600/` | `as5600_lib.c`, `as5600_defs.h` |
| **Leer la orientación (Yaw / Heading) o aceleración en Striker 2** | `sensors/bno055/` | `bno055.c`, `bno055.h` |
| **Leer la IMU serial del robot Striker 1** | `sensors/tm151/` | `tm151.c`, `tm151.h` |
| **Medir distancia a obstáculos o posesión del balón con láser** | `sensors/vl53l1x/` y `sensors/vl6180x/` | `VL53L1X_api.c`, `vl6180x_api.c` |
| **Mover los motores Brushless (BLDC) o calibrar los variadores ESC** | `actuators/bldc_pwm/` | `bldc_pwm.c`, `bldc_pwm.h` |
| **Cambiar pines I2C, UART o temporizadores del ESP32-S3** | `platform_hal/` | `platform_esp32s3.c`, `platform.h` |
| **Ajustar el lazo PID de velocidad o probar el PID con Red Neuronal** | `control/pid_lib/` | `pid_lib.c`, `autotuning_models/` |
| **Implementar filtros de Kalman (EKF / UKF) o Filtro de Partículas** | `control/state_estimation/` | `EKF/`, `UKF/`, `PF/`, `LinearAlgebra/` |
| **Enviar telemetría por WiFi a la estación base en Python** | `comms/wifi_udp/` | `wifi_lib.c`, `wifi_lib.h` |
| **Crear comandos de prueba por puerto serial USB/UART** | `comms/uart_console/` | `uart_console.c` |

---

## 3. ¿Por qué existen varias versiones de un mismo driver? (`standalone`, `v1_striker1`, `v2_striker2`, `autotuning`)

A lo largo del desarrollo de **Striker 1**, **Striker 2** y las tesis de **Estimación de Estado** y **PID Autotuning**, los drivers base fueron adaptados a las necesidades de cada robot (por ejemplo, cambiando estructuras de configuración de pines, periféricos MCPWM o firmas de inicialización).

Para garantizar que **todos los proyectos compilen sin errores** tras la fusión del repositorio, cada componente conserva subcarpetas identificadas por su plataforma:

* **`standalone/`**: Librería original pura del fabricante o desarrollada de forma independiente (agnóstica a la aplicación). Úsala como base limpia si vas a diseñar un **Striker 3** o un nuevo banco de pruebas desde cero.
* **`v1_striker1/`**: Versión adaptada específicamente a la electrónica y estructura de datos del robot **Striker 1** (`firmware/apps/striker_v1/`).
* **`v2_striker2/`**: Versión adaptada a la arquitectura modular y tareas FreeRTOS del robot **Striker 2** (`firmware/apps/striker_v2/`).
* **`autotuning/` / `autotuning_rnn/`**: Versión empleada en el banco experimental de identificación de motores y control adaptativo con redes neuronales (`firmware/apps/bench_pid_autotuning/`).

---

## 4. Catálogo Detallado de Componentes

### 4.1. Sensores (`firmware/components/sensors/`)

#### `as5600/` — Encoder Magnético Rotativo de 12 bits (AMS AS5600)
* **Función en el robot:** Mide de forma absoluta y sin contacto el ángulo del eje de cada una de las 3 ruedas omnidireccionales mediante el campo magnético de un imán diametral. A partir de la derivada temporal del ángulo se calcula la velocidad angular $\omega_i$ (rad/s o RPM) para el lazo cerrado PID.
* **Protocolo:** I2C (dirección fija `0x36`; requiere buses I2C independientes o multiplexor para las 3 ruedas) + salida analógica/PWM opcional.
* **Variantes:**
  * `standalone/`: Librería HAL completa con soporte para calibración de registros de ángulo y quemado OTP.
  * `v1_striker1/`: Integración directa usada en `robot-mov`, `wheel-movement` y `bench_pid_autotuning`.
  * `v2_striker2/`: Integración optimizada para el lazo de control de `Robot-Control`, `Robot-Navigation` y `Velocity-Control`.

#### `bno055/` — IMU Absoluta de 9 Grados de Libertad (Bosch BNO055)
* **Función en el robot:** Sensor inercial principal de **Striker 2**. Integra acelerómetro triaxial, giroscopio triaxial, magnetómetro y un procesador Cortex-M0 interno que entrega directamente orientación absoluta (ángulos de Euler / Yaw, cuaterniones), velocidad angular y aceleración lineal compensada por gravedad.
* **Protocolo:** I2C / UART.
* **Variantes:**
  * `standalone/`: Driver adaptable completo con soporte para todos los modos de operación (`NDOF`, `IMUPLUS`, `AMG`) y calibración.
  * `v2_striker2/`: Driver integrado en las aplicaciones de Striker 2.

#### `tm151/` — Unidad de Medición Inercial Serial (SYD Dynamics TM151)
* **Función en el robot:** Sensor inercial utilizado exclusivamente en **Striker 1** para obtener orientación y velocidades angulares del chasis.
* **Protocolo:** UART de alta velocidad con decodificación de tramas binarias por paquetes.
* **Variantes:** `standalone/` y `v1_striker1/`.

#### `vl53l1x/` — Sensor Láser Time-of-Flight de Largo Alcance (ST VL53L1X)
* **Función en el robot:** Mide distancia absoluta (hasta 4 metros) mediante el tiempo de vuelo de pulsos láser infrarrojos (940 nm), independientemente del color o reflectancia del objetivo. Se utiliza para detección de obstáculos, localización respecto a las paredes/arcos y detección de robots cercanos.
* **Protocolo:** I2C (dirección por defecto `0x29`, reconfigurable por software mediante pines `XSHUT`).
* **Variantes:** `standalone/`, `v1_striker1/` y `v2_striker2/`.

#### `vl6180x/` — Sensor Láser Time-of-Flight de Corto Alcance (ST VL6180X)
* **Función en el robot:** Medición de precisión milimétrica en corto rango (0 a 10 cm) y sensor de luz ambiental. Ideal para detectar cuándo el balón de golf está en contacto con el rodillo (*dribbler*) y habilitar el disparo del *kicker*.
* **Protocolo:** I2C (`0x29`).
* **Variantes:** `standalone/`.

#### `apds9960/` — Sensor Óptico Digital RGB, Proximidad y Gestos (Broadcom APDS-9960)
* **Función en el robot:** Detección de proximidad por infrarrojos y medición de color RGBC (por ejemplo, para detectar el color naranja característico del balón o marcas en la superficie).
* **Protocolo:** I2C (`0x39`).
* **Variantes:** `standalone/`.

---

### 4.2. Actuadores (`firmware/components/actuators/`)

#### `bldc_pwm/` — Control de Motores Brushless (BLDC) y Variadores ESC
* **Función en el robot:** Genera las señales moduladas por ancho de pulso (PWM / periférico MCPWM del ESP32-S3) para comandar los variadores electrónicos de velocidad (**ESCs**) acoplados a los 3 motores Brushless de tracción. Incluye las rutinas de inicialización segura, secuencias de calibración de rango máximo/mínimo de los ESCs e inversión de sentido de giro.
* **Periférico ESP32-S3:** `MCPWM` / `LEDC` + pines GPIO de dirección.
* **Variantes:**
  * `v1_striker1/`: Implementación para Striker 1 (`escs_calibration.c` y control de tracción).
  * `v2_striker2/`: Implementación modular para los 3 motores de Striker 2.
  * `autotuning/`: Implementación instrumentada para inyectar escalones y trenes de pulsos durante la identificación experimental de motores.

#### `led_status/` — Indicadores Visuales de Estado
* **Función en el robot:** Controla los LEDs de diagnóstico en la tarjeta electrónica de Striker 2 para indicar visualmente el estado del enlace WiFi, fallos de inicialización en el bus I2C o modo de operación activo.
* **Variantes:** `v2_striker2/`.

---

### 4.3. Capa de Abstracción de Plataforma (`firmware/components/platform_hal/`)

Los drivers de sensores como `AS5600`, `BNO055` o `VL53L1X` no llaman directamente a funciones nativas de ESP-IDF (`i2c_master_write_to_device`, `uart_read_bytes`, etc.), sino que invocan funciones genéricas definidas en `platform_hal/`:
* Inicialización de buses I2C Master y puertos UART.
* Lectura y escritura de registros de 8 y 16 bits por I2C.
* Temporización en milisegundos/microsegundos (`vTaskDelay` / `esp_timer_get_time`) y lectura de GPIOs/ADC.

> **Importante para estudiantes:** Si diseñan una nueva PCB y cambian los pines `SDA`/`SCL` del I2C o los pines `TX`/`RX` de la IMU, **solo deben modificar los archivos dentro de `platform_hal/`** (y las cabeceras de configuración de la app), sin tocar el código interno de los sensores.

---

### 4.4. Control, Cinemática y Estimación de Estado (`firmware/components/control/`)

* **`pid_lib/`**:
  * **`v1_striker1/` y `v2_striker2/`**: Controladores PID discretos en C con límite de saturación de salida y anti-windup para regular la velocidad angular de cada rueda ($\omega_1, \omega_2, \omega_3$) y la orientación ($\theta$) del robot. En `v2_striker2` también incluye las transformaciones de cinemática inversa holonómica de 3 ruedas a 120°.
  * **`autotuning_rnn/` y `autotuning_models/`**: Implementación en C del controlador PID adaptativo capaz de ejecutar en tiempo real la inferencia de una **Red Neuronal Recurrente (RNN-LSTM)** entrenada en Python para auto-sintonizar las ganancias $K_p, K_i, K_d$ ante cambios en la dinámica del motor o fricción de la cancha.
* **`sensor_fusion/`**:
  * Algoritmos ligeros para combinar las lecturas de los encoders `AS5600` con el giroscopio/IMU en el lazo local de control.
* **`state_estimation/`**:
  * Librería embebida en C99 puro (proveniente de `mobile-robot-estimation-suite/State_Estimation_C`) que implementa tres estimadores probabilísticos de estado para seguimiento de trayectorias:
    1. **`EKF/`**: Filtro de Kalman Extendido (*Extended Kalman Filter*).
    2. **`UKF/`**: Filtro de Kalman Unscented (*Unscented Kalman Filter*).
    3. **`PF/`**: Filtro de Partículas (*Particle Filter* / Monte Carlo secuencial).
    4. **`LinearAlgebra/`**: Motor propio de operaciones matriciales en C (multiplicación, transposición, descomposición de Cholesky e inversión de matrices) optimizado para ejecutarse en el ESP32-S3 sin librerías externas pesadas.

---

### 4.5. Comunicaciones (`firmware/components/comms/`)

* **`wifi_udp/`**:
  * Gestiona la conexión de la placa ESP32-S3 a la red WiFi del laboratorio/cancha y abre sockets UDP no bloqueantes para recibir consignas de velocidad $(v_x, v_y, \omega)$ desde el servidor en Python y transmitir paquetes de telemetría (velocidades de rueda, PWM, IMU, errores PID) a alta frecuencia.
* **`uart_console/`**:
  * Consola serial interactiva para enviar comandos de prueba, calibrar sensores o cambiar referencias de velocidad desde el monitor serial de ESP-IDF sin necesidad de conexión WiFi.

---

## 5. Guía Paso a Paso: Cómo usar estos componentes en una aplicación

Cuando el script `unify_firmware_drivers.ps1` se ejecuta en el repositorio, todas las aplicaciones dentro de `firmware/apps/` quedan configuradas para compilar directamente contra `firmware/components/` a través de su archivo `main/CMakeLists.txt`.

### 5.1. Estructura en `main/CMakeLists.txt` (Ejemplo para Striker 2)

```cmake
# Ruta relativa desde firmware/apps/striker_v2/<Proyecto>/main hacia firmware/components
set(COMPONENTS_ROOT "../../../../components")

file(GLOB_RECURSE SOURCES
    "*.c"
    "${COMPONENTS_ROOT}/platform_hal/v2_striker2/*.c"
    "${COMPONENTS_ROOT}/sensors/as5600/v2_striker2/*.c"
    "${COMPONENTS_ROOT}/sensors/bno055/v2_striker2/*.c"
    "${COMPONENTS_ROOT}/sensors/vl53l1x/v2_striker2/*.c"
    "${COMPONENTS_ROOT}/actuators/bldc_pwm/v2_striker2/*.c"
    "${COMPONENTS_ROOT}/actuators/led_status/v2_striker2/*.c"
    "${COMPONENTS_ROOT}/control/pid_lib/v2_striker2/*.c"
    "${COMPONENTS_ROOT}/comms/uart_console/v2_striker2/*.c"
)

idf_component_register(
    SRCS ${SOURCES}
    INCLUDE_DIRS
        "."
        "../include"
        "${COMPONENTS_ROOT}/platform_hal/v2_striker2"
        "${COMPONENTS_ROOT}/sensors/as5600/v2_striker2"
        "${COMPONENTS_ROOT}/sensors/bno055/v2_striker2"
        "${COMPONENTS_ROOT}/sensors/vl53l1x/v2_striker2"
        "${COMPONENTS_ROOT}/actuators/bldc_pwm/v2_striker2"
        "${COMPONENTS_ROOT}/actuators/led_status/v2_striker2"
        "${COMPONENTS_ROOT}/control/pid_lib/v2_striker2"
        "${COMPONENTS_ROOT}/comms/uart_console/v2_striker2"
)
```

### 5.2. ¿Cómo agregar un nuevo driver al repositorio? (Buenas prácticas para estudiantes)

Si vas a integrar un nuevo sensor o actuador (por ejemplo, un circuito de disparo para el *kicker* o un sensor de corriente INA219):

1. **Ubícalo en la categoría correcta:**
   * Si mide una variable física $\rightarrow$ `firmware/components/sensors/<nombre_sensor>/`
   * Si ejerce una acción física o potencia $\rightarrow$ `firmware/components/actuators/<nombre_actuador>/`
2. **Separa la lógica del hardware:**
   * No escribas llamadas directas a pines fijos dentro del driver; pasa la configuración mediante un `struct` de inicialización y usa las funciones de `platform_hal/`.
3. **Documenta los encabezados (`.h`):**
   * Incluye comentarios claros indicando las unidades físicas de cada parámetro (por ejemplo, `[rad/s]`, `[mm]`, `[m/s^2]`, `[us]`).
4. **Regístralo en este `README.md`:**
   * Añade una fila en la sección correspondiente indicando el chip, el protocolo de comunicación y en qué robot fue probado.
