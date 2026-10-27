# Howlers Source — Unified Robosyn Monorepo (Universidad de Antioquia)

Repositorio central unificado del equipo **Howlers FC (RoboCup Small-Size League — SSL)** del Grupo de Investigación **Robosyn** de la **Universidad de Antioquia**.

Este *monorepo* consolida en una sola arquitectura el firmware embebido en C para microcontroladores **ESP32-S3 (ESP-IDF + FreeRTOS)**, las librerías de sensores HAL, los algoritmos de estimación probabilística de estado (**EKF, UKF, PF**), la identificación paramétrica de sistemas, el auto-ajuste de controladores **PID mediante redes neuronales recurrentes (RNN-LSTM)** y las estaciones base de telemetría y visión artificial.

---

## Tabla de Contenidos

1. [Origen de los Módulos Integrados](#1-origen-de-los-módulos-integrados)
2. [Estructura del Repositorio](#2-estructura-del-repositorio)
3. [Requisitos Previos](#3-requisitos-previos)
4. [Guía de Compilación y Flasheo de Firmware (Striker 1 y Striker 2)](#4-guía-de-compilación-y-flasheo-de-firmware-striker-1-y-striker-2)
5. [Ejecución de la Suite de Software y Estación Base (Python)](#5-ejecución-de-la-suite-de-software-y-estación-base-python)
6. [Conjuntos de Datos Experimentales (`datasets/`)](#6-conjuntos-de-datos-experimentales-datasets)
7. [Autores y Créditos](#7-autores-y-créditos)

---

## 1. Origen de los Módulos Integrados

Este repositorio unifica el historial completo de commits de cuatro proyectos fundamentales desarrollados para los robots omnidireccionales de 3 ruedas del equipo:

| Proyecto Original | Ubicación en el Monorepo | Descripción |
| :--- | :--- | :--- |
| **[nparra-code/striker-1](https://github.com/nparra-code/striker-1)** | `firmware/apps/striker_v1/`<br>`firmware/components/sensors/tm151/` | Firmware base para el primer prototipo (**Striker 1**), control holonómico de 3 ruedas, calibración de ESCs/encoders y driver IMU **TM151**. |
| **[MaverickST/RoboCup_Striker2](https://github.com/MaverickST/RoboCup_Striker2)** | `firmware/apps/striker_v2/`<br>`firmware/components/sensors/` | Segunda generación (**Striker 2**), lazos de control de velocidad y navegación, integración con IMU **BNO055**, sensores ToF y servidores UDP. |
| **[MaverickST/mobile-robot-estimation-suite](https://github.com/MaverickST/mobile-robot-estimation-suite)** | `software/system_identification/`<br>`software/vision_tracking/`<br>`datasets/state_estimation_exp/` | Suite de identificación dinámica, filtros de estimación de estado (**EKF, UKF, PF**) en C/Python y sistema de referencia (*ground-truth*) por visión artificial. |
| **[nparra-code/pid-autotuning](https://github.com/nparra-code/pid-autotuning)** | `firmware/apps/bench_pid_autotuning/`<br>`software/pid_autotuning/`<br>`datasets/motor_and_pid_logs/` | Sintonización automática de controladores PID mediante **RNN-LSTM** y métodos clásicos, simulador en Jupyter/Python y banco de pruebas en ESP32-S3. |

---

## 2. Estructura del Repositorio

```text
howlers-source/
├── README.md                                  # Documentación general y guía de compilación
├── .gitignore                                 # Reglas de exclusión para ESP-IDF (build/) y Python
│
├── firmware/                                  # Código embebido C/C++ para ESP32-S3 (ESP-IDF)
│   ├── components/                            # Drivers HAL y librerías reutilizables
│   │   └── sensors/
│   │       ├── apds9960/                      # Sensor óptico RGB, luz ambiental y proximidad
│   │       ├── as5600/                        # Encoder magnético rotativo Hall de 12 bits (I2C)
│   │       ├── bno055/                        # IMU absoluta de 9-DOF con fusión por hardware (Striker 2)
│   │       ├── tm151/                         # IMU inercial serie (exclusiva de Striker 1)
│   │       ├── vl53l1x/                       # Sensor láser Time-of-Flight (ToF) de largo alcance
│   │       └── vl6180x/                       # Sensor láser Time-of-Flight (ToF) de corto alcance
│   │
│   └── apps/                                  # Proyectos ejecutables ESP-IDF por plataforma
│       ├── striker_v1/                        # Aplicaciones del Robot Striker 1
│       │   ├── robot-mov/                     # Cinemática holonómica, control principal y calibración
│       │   └── wheel-movement/                # Control individual de motores BLDC y pruebas de rueda
│       │
│       ├── striker_v2/                        # Aplicaciones del Robot Striker 2
│       │   ├── Robot-Control/                 # Control integral del robot y adquisición de telemetría
│       │   ├── Robot-Navigation/              # Navegación autónoma, seguimiento de trayectorias y orientación
│       │   └── Velocity-Control/              # Lazo cerrado de control PID de velocidad angular por rueda
│       │
│       └── bench_pid_autotuning/              # Firmware de identificación de motores y validación PID RNN
│
├── software/                                  # Herramientas de alto nivel, IA, simulación y servidores (Python)
│   ├── ground_station/                        # Estación base unificada (servidores UDP, Web y telemetría)
│   │   ├── striker1_proxy/                    # Proxy UDP e interfaz web de teleoperación para Striker 1
│   │   ├── striker2_server_test/              # Servidor Flask de generación de trayectorias para Striker 2
│   │   ├── striker2_udp_server/               # Servidor UDP ligero de telemetría en tiempo real
│   │   ├── autotuning_servers/                # Data loggers y servidores de identificación de motores
│   │   └── legacy_branches/                   # Variantes preservadas de ramas de desarrollo (kev_dev, ben_dev)
│   │
│   ├── pid_autotuning/                        # Simulación y entrenamiento de PID adaptativo (RNN-LSTM)
│   │   ├── AutotuningPID_RNNTraining.ipynb    # Notebook de entrenamiento de la red neuronal recurrente
│   │   ├── AutotuningPID_Simulator.ipynb      # Simulador comparativo (RNN vs. métodos clásicos)
│   │   ├── python/                            # Módulos de simulación, modelos de motor y Ziegler-Nichols
│   │   ├── rnn/                               # Pesos y arquitecturas de modelos entrenados
│   │   └── results/                           # Gráficas y métricas de desempeño
│   │
│   ├── system_identification/                 # Identificación física y cinemática del robot omnidireccional
│   │   ├── shared_utils/                      # Cinemática, preprocesamiento de señales y carga de datos
│   │   └── src/                               # Algoritmos de estimación paramétrica y validación
│   │
│   └── vision_tracking/                       # Seguimiento por cámara cenital y procesamiento de IMU
│
├── datasets/                                  # Registros experimentales capturados en pruebas reales
│   ├── motor_and_pid_logs/                    # Logs de identificación de motores y pruebas PID (clásico/RNN)
│   └── state_estimation_exp/                  # Experimentos dinámicos (exp1..exp10 e ident_1..ident_10)
│
└── docs/                                      # Documentación técnica, diagramas de arquitectura y referencias
    ├── diagrams/                              # Diagramas de bloques de hardware, control y tareas FreeRTOS
    ├── striker_v1/                            # Documentación específica de Striker 1
    ├── striker_v2/                            # Documentación específica de Striker 2
    └── theses_references/                     # Documentación de las tesis de Estimación y PID Autotuning
```

---

## 3. Requisitos Previos

### Para el Firmware (`firmware/`)
* **Framework:** [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/get-started/index.html) (recomendado **v5.1+** o **v5.3+**).
* **Hardware objetivo:** SoC **ESP32-S3** (Dual-core Xtensa LX7).
* **Entorno en Windows / Linux:**
  * **VS Code + Extensión Oficial ESP-IDF** *(recomendado en Windows)*, o bien
  * **ESP-IDF PowerShell / Command Prompt** en Windows, o terminal Bash en Linux/macOS.

### Para el Software y Estación Base (`software/`)
* **Python:** 3.10 o superior.
* **Gestor de paquetes:** `pip` o `venv` (con librerías `numpy`, `scipy`, `matplotlib`, `flask`, `opencv-python`, `torch`/`tensorflow` según el módulo).

---

## 4. Guía de Compilación y Flasheo de Firmware (Striker 1 y Striker 2)

Cada subproyecto dentro de `firmware/apps/` es una aplicación **ESP-IDF autónoma** que cuenta con su propio `CMakeLists.txt` raíz y enlaza sus controladores locales en `main/CMakeLists.txt`.

### 4.1. Activar el entorno ESP-IDF

Antes de ejecutar cualquier comando `idf.py`, asegúrate de inicializar las variables de entorno de ESP-IDF:

* **En Windows (PowerShell):**
  Abre directamente el acceso directo **"ESP-IDF 5.x PowerShell"** desde el menú Inicio, o ejecuta el script de inicialización de tu instalación:
  ```powershell
  # Ajusta la ruta según tu instalación local de Espressif
  C:\Espressif\frameworks\esp-idf-v5.3\export.ps1
  ```

* **En Linux / macOS (Bash):**
  ```bash
  source $IDF_PATH/export.sh
  ```

---

### 4.2. Compilar y Flashear **Striker 1** (`striker_v1`)

El robot **Striker 1** cuenta con dos aplicaciones dentro de `firmware/apps/striker_v1/`:
* **`robot-mov`**: Firmware principal de movimiento holonómico, lectura de sensores (`AS5600`, `TM151`, `VL53L1X`), calibración de ESCs/encoders y comunicación WiFi/UDP.
* **`wheel-movement`**: Firmware de prueba y control individual de velocidad de ruedas.

#### Pasos en Windows (PowerShell):
```powershell
# 1. Entrar al proyecto principal de Striker 1
cd firmware\apps\striker_v1\robot-mov

# 2. Definir el microcontrolador objetivo (ESP32-S3)
# (Nota: si existe un sdkconfig previo incompatible, ejecuta primero: idf.py fullclean)
idf.py set-target esp32s3

# 3. (Opcional) Configurar credenciales WiFi o parámetros de hardware
idf.py menuconfig

# 4. Compilar el proyecto
idf.py build

# 5. Flashear en la placa y abrir el monitor serial (reemplaza COMX por tu puerto, ej. COM5)
idf.py -p COM5 flash monitor
```

> **Tip:** Para salir del monitor serial de `idf.py` en cualquier momento, presiona **`Ctrl + ]`**.

---

### 4.3. Compilar y Flashear **Striker 2** (`striker_v2`)

El robot **Striker 2** dispone de tres proyectos especializados dentro de `firmware/apps/striker_v2/`:
* **`Robot-Control`**: Aplicación principal de control omnidireccional, lectura de IMU `BNO055` + encoders `AS5600` y envío de telemetría por UDP.
* **`Robot-Navigation`**: Aplicación de navegación autónoma, control de orientación (heading) y seguimiento de trayectorias.
* **`Velocity-Control`**: Lazo cerrado de velocidad por rueda para sintonización y pruebas de respuesta dinámica.

#### Pasos en Windows (PowerShell):
```powershell
# 1. Entrar al proyecto deseado de Striker 2 (por ejemplo, Robot-Control)
cd firmware\apps\striker_v2\Robot-Control
# Alternativas:
# cd firmware\apps\striker_v2\Robot-Navigation
# cd firmware\apps\striker_v2\Velocity-Control

# 2. Definir el microcontrolador objetivo (ESP32-S3)
idf.py set-target esp32s3

# 3. Compilar el firmware
idf.py build

# 4. Flashear y monitorear por puerto serial (reemplaza COMX por tu puerto, ej. COM6)
idf.py -p COM6 flash monitor
```

---

### 4.4. Compilar el Banco de Pruebas de **PID Autotuning** (`bench_pid_autotuning`)

Si deseas cargar el firmware de identificación de motores BLDC y validación del controlador PID adaptativo:

```powershell
cd firmware\apps\bench_pid_autotuning
idf.py set-target esp32s3
idf.py build
idf.py -p COM5 flash monitor
```

---

### 4.5. Compilación desde Visual Studio Code (Alternativa Gráfica)

Si utilizas la extensión oficial **Espressif IDF** en VS Code:
1. Abre la carpeta raíz `howlers-source` en VS Code.
2. Abre la paleta de comandos (`Ctrl + Shift + P`) y selecciona **`ESP-IDF: Select Current ESP-IDF Workspace Folder`** (o abre directamente la subcarpeta de la app, por ejemplo `firmware/apps/striker_v2/Robot-Control`).
3. En la barra inferior de estado de VS Code:
   * Selecciona el puerto **`COM`** correspondiente a tu ESP32-S3.
   * Verifica que el target sea **`esp32s3`**.
   * Haz clic en el ícono de **llave inglesa (Build Project)** para compilar y en el **rayo (Flash Project)** para programar el robot.

---

### 4.6. Cómo vincular nuevos componentes compartidos (`firmware/components`)

Si creas un nuevo proyecto en `firmware/apps/` o deseas migrar progresivamente las carpetas locales de drivers hacia los componentes centralizados en `firmware/components/sensors/`, basta con agregar la directiva `EXTRA_COMPONENT_DIRS` en el archivo `CMakeLists.txt` raíz de la aplicación:

```cmake
cmake_minimum_required(VERSION 3.16)

# Incluir los componentes compartidos del monorepo
set(EXTRA_COMPONENT_DIRS "../../components/sensors")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(striker_app)
```

---

## 5. Ejecución de la Suite de Software y Estación Base (Python)

### Configuración del entorno virtual en Windows (PowerShell)

```powershell
# Crear y activar un entorno virtual en la raíz del repositorio
python -m venv .venv
.\.venv\Scripts\Activate.ps1

# Instalar dependencias base
pip install --upgrade pip
pip install -r firmware\apps\bench_pid_autotuning\requirements.txt
```

### Módulos principales disponibles

* **Estación Base y Servidor de Pruebas para Striker 2 (`software/ground_station/striker2_server_test`):**
  ```powershell
  cd software\ground_station\striker2_server_test
  python app.py
  ```
* **Proxy UDP de Teleoperación para Striker 1 (`software/ground_station/striker1_proxy`):**
  ```powershell
  cd software\ground_station\striker1_proxy
  python proxy_udp.py
  ```
* **Servidores de Captura e Identificación de Motores (`software/ground_station/autotuning_servers`):**
  ```powershell
  cd software\ground_station\autotuning_servers
  python data_logger_server.py
  # O para identificación de constantes de motor:
  python motor_ident_server.py
  ```
* **Identificación Dinámica del Robot (`software/system_identification`):**
  ```powershell
  cd software\system_identification
  python main.py
  ```
* **Simulación y Entrenamiento PID RNN-LSTM (`software/pid_autotuning`):**
  Abre los notebooks `AutotuningPID_RNNTraining.ipynb` y `AutotuningPID_Simulator.ipynb` desde VS Code o Jupyter Lab.

---

## 6. Conjuntos de Datos Experimentales (`datasets/`)

* **`datasets/state_estimation_exp/`**:
  * Incluye los registros experimentales de sensores (`sensors/exp1.txt` a `exp10.txt`), los datos de identificación rescatados de las pruebas en Striker 2 (`striker2_ident1/ident_1.txt` a `ident_10.txt`) y las gráficas/resultados de validación de los filtros **EKF, UKF y PF**.
* **`datasets/motor_and_pid_logs/`**:
  * Contiene las curvas experimentales de respuesta escalón e identificación de motores (`motor_ident_logs/`), así como las pruebas comparativas en hardware real entre sintonización clásica (`00_classical_results/`) y auto-ajuste neuronal (`00_autotuning_results/`).

---

## 7. Autores y Créditos

Desarrollado por integrantes del semillero y grupo de investigación **Robosyn — Howlers FC**, Facultad de Ingeniería, **Universidad de Antioquia (Medellín, Colombia)**:

* **MaverickST** — *Estimación de estado probabilística (EKF/UKF/PF), identificación dinámica y desarrollo de Striker 2.*
* **nparra-code** — *Desarrollo de Striker 1, control de motores BLDC y sintonización automática de PID mediante redes RNN-LSTM.*
* **Equipo Howlers FC (Robosyn - UdeA)** — *Integración de hardware, electrónica de potencia, mecánica y validación en RoboCup SSL.*
