$appPath = "firmware\apps\striker_v1\wheel-movement"
@(Get-Item "$appPath\README.md" -ErrorAction SilentlyContinue; Get-ChildItem -Path "$appPath\main", "$appPath\include", "$appPath\docs" -Recurse -File -ErrorAction SilentlyContinue) | ForEach-Object { "===== ARCHIVO: $($_.FullName.Replace((Get-Location).Path + '\', '')) ====="; if ($_.Extension -match '\.(c|h|cpp|hpp|md|txt|cmake)$' -or $_.Name -eq 'CMakeLists.txt') { Get-Content $_.FullName } else { "[Archivo binario/imagen: $($_.Name)]" } } | Set-Content -Path "codigo_app_para_analizar.txt" -Encoding UTF8


Actúa como un Ingeniero Senior de Sistemas Embebidos en Tiempo Real (ESP-IDF / FreeRTOS SMP sobre ESP32-S3) y Profesor Universitario de Robótica Móvil.

Voy a proporcionarte los archivos de código fuente (`main/`, `include/`, `CMakeLists.txt`, `README.md` original, etc.) de la aplicación `[NOMBRE_DE_LA_APP]` ubicada en `[RUTA_EN_EL_MONOREPO, ej. firmware/apps/striker_v2/Robot-Control]` dentro del repositorio unificado `ravelasqvel/howlers-source`.

Analiza minuciosamente todo el código fuente suministrado junto con el `README.md` original y genera un nuevo archivo `README.md` completo, riguroso y pedagógico (en español) para esta aplicación, siguiendo la directriz de preservación y las 9 secciones obligatorias detalladas a continuación:

### Directriz Crítica sobre el `README.md` Original (Preservación, Contraste y Enriquecimiento)
No descartes ni ignores la información valiosa del `README.md` original. Antes de redactar, cruza su contenido contra el código fuente real y aplica estas tres reglas:
1. **Conservar e Integrar lo Valioso:** Preserva y traduce/integra toda la información útil del `README.md` original que aporte contexto al proyecto: tablas o esquemas de conexionado de hardware (pinouts, buses I2C/UART/PWM), diagramas e imágenes referenciadas en `docs/` o `assets/` (manteniendo o actualizando sus rutas relativas en Markdown), descripciones de alto nivel de funciones, explicaciones de máquinas de estado, parámetros físicos del robot, formatos de paquetes o notas de calibración y autoría.
2. **Actualizar lo que Cambió por la Fusión del Monorepo:** Si el `README.md` original muestra el árbol antiguo con carpetas locales (`drivers/`, `sensors/`, `components/`), actualízalo a la nueva estructura centralizada en `firmware/components/`, pero conserva las descripciones funcionales de los módulos que el autor original redactó.
3. **Contrastar Documentación vs. Implementación Real:** Si el `README.md` original afirma que cierta tarea, sensor o algoritmo hace algo que en el código fuente actual está comentado, incompleto o implementado de forma distinta (o viceversa, si el código tiene tareas/funciones que el `README.md` original omitió), **no borres la intención original**: documenta cuál era el diseño previsto en el `README.md` original y explica con precisión qué está ocurriendo realmente en el código actual (conectándolo con las Secciones 4, 6 y 7).

---

### Estructura Obligatoria de 9 Secciones para el Nuevo `README.md`

### 1. Estructura del Proyecto en el Monorepo Unificado
- Conserva la introducción de propósito general del `README.md` original enriquecida con los detalles técnicos del sistema.
- Muestra el árbol de archivos actual de la aplicación (`CMakeLists.txt`, `sdkconfig`, `main/`, `include/`, `docs/`, etc.), incluyendo las imágenes o diagramas existentes en `docs/` si los hay.
- Lista explícitamente qué componentes centralizados de `firmware/components/` (`sensors/`, `actuators/`, `platform_hal/`, `control/`, `comms/`) enlaza el archivo `main/CMakeLists.txt` y cuál es el rol de cada uno.

### 2. Inicialización del Sistema (`app_main`) y Configuración de Hardware
- Integra cualquier tabla de pines o configuración de hardware del `README.md` original y contrástala con las macros `#define` reales de las cabeceras (`.h`).
- Describe paso a paso la secuencia de inicialización de periféricos en `main.c` (buses I2C, UART, unidades/canales ADC, MCPWM/LEDC para motores BLDC, inicialización de sensores, estructuras PID, WiFi/UDP, timers, etc.) incluyendo los pines GPIO, frecuencias y parámetros configurados en el código.

### 3. Análisis de la Estructura de Tareas FreeRTOS
- Parte de la tabla o listado de tareas del `README.md` original (si existe) y complétala con TODAS las tareas definidas realmente en el código (incluyendo tanto las tareas activas como aquellas que estén comentadas, inactivas o que hayan sido omitidas en la documentación previa).
- La tabla debe especificar para cada tarea:
  1. Nombre de la tarea (string pasado a FreeRTOS).
  2. Función C que la implementa.
  3. Afinidad de núcleo (`Core 0`, `Core 1` o `tskNO_AFFINITY` si usa `xTaskCreate`).
  4. Prioridad asignada.
  5. Tamaño de pila (`Stack` en bytes/palabras).
  6. Periodo de ejecución o evento bloqueante (`vTaskDelay`, `vTaskDelayUntil`, timer, `recvfrom`, etc.).
  7. Propósito detallado en el sistema.

### 4. Análisis del Uso de Objetos del Kernel de FreeRTOS y Sincronización
- **4.1. Auditoría de Objetos del Kernel:** Identifica qué cabeceras de FreeRTOS se incluyen frente a qué objetos del kernel se instancian y usan realmente en el código (`TaskHandle_t`, `QueueHandle_t`, `SemaphoreHandle_t` / Mutexes, `EventGroupHandle_t`, notificaciones directas, `vTaskDelay` vs `vTaskDelayUntil`, timers o sockets bloqueantes), y cómo se transfieren los datos entre tareas (paso de mensajes vs. memoria global compartida / punteros en `pvParameters`).
- **4.2. Diagrama de Relación de Tareas, Núcleos y Recursos Compartidos:** Construye un diagrama de bloques claro en texto/ASCII (y referencia además los diagramas gráficos de `docs/` si el `README.md` original los tenía) que muestre la separación entre `Core 0` y `Core 1`, las tareas con sus prioridades y periodos, los objetos del kernel usados para sincronización/bloqueo, el flujo de datos en memoria compartida y el acceso a periféricos físicos compartidos (I2C, ADC, UART, PWM).
- **4.3. Diagnóstico de Problemas de Concurrencia y Tiempo Real:** Identifica con precisión quirúrgica (citando funciones y variables exactas del código) cualquier condición de carrera (*race condition*) sobre periféricos o variables globales, problemas de expropiación (*preemption*) entre tareas de distinta prioridad, deriva temporal (*time drift*), inversión de prioridades o uso inseguro de variables locales `static` entre múltiples instancias de tareas.

### 5. Módulos de Estimación, Cinemática y Control
- Conserva las explicaciones de funciones y algoritmos que aportaba el `README.md` original (por ejemplo, descripciones de funciones de estimación, filtrado o control) y compleméntalas con las ecuaciones cinemáticas exactas, leyes de control PID (tipo de PID, saturación, constantes) y algoritmos de navegación implementados en los archivos fuente.

### 6. Auditoría de Uso Real de Sensores y Módulos Auxiliares (Documentación Original vs. Código Real)
- Verifica si todos los sensores y módulos descritos en el `README.md` original o inicializados en el código (por ejemplo, IMU `BNO055` / `TM151`, láser `VL53L1X` / `VL6180X`, sensor óptico `APDS9960` o módulos de fusión sensorial) se usan realmente para cerrar el lazo de control, o si alguna tarea/función está comentada, desconectada o limitada solo a imprimir logs por consola. Explica el impacto físico de esto en el movimiento del robot.

### 7. Lista de Tareas Pendientes (TODOs) con Evaluación de Alternativas (Trade-offs)
- Redacta una lista numerada de `TODOs` accionables basados en los problemas encontrados en las secciones 4 y 6 (así como funcionalidades que el `README.md` original planteaba pero quedaron inconclusas en el código).
- **REGLA CRÍTICA PARA LOS TODOs:** Cuando un problema técnico tenga varias soluciones posibles (por ejemplo, sincronización entre tareas, protección de un periférico compartido, periodicidad de muestreo, arquitectura del lazo con la IMU o dimensionamiento de stacks), **NO impongas una única solución cerrada**. Para cada TODO debes:
  1. Enunciar el **Problema** concreto detectado en el código.
  2. Presentar 2 o 3 **Alternativas Técnicas a Evaluar** (*Alternativa A*, *Alternativa B*, *Alternativa C*, ej. Colas vs. Mutex/Spinlock vs. Event Groups + Doble Buffer; tarea única de sensores vs. múltiples tareas con Mutex vs. DMA/Timers).
  3. Indicar qué **compromisos (*trade-offs*)** deben medir y comparar los estudiantes (sobrecarga de CPU, latencia, consumo de SRAM interna, determinismo y *jitter*) para sustentar cuál alternativa es la más eficiente.

### 8. Compilación, Flasheo y Uso (CLI, VS Code GUI y Protocolo de Comunicación)
- **8.1. Compilación desde Línea de Comandos (Windows PowerShell):** Comandos `idf.py set-target esp32s3`, `idf.py build` e `idf.py -p COMX flash monitor`.
- **8.2. Compilación y Uso desde la Interfaz Gráfica de ESP-IDF en Visual Studio Code (GUI):** Instrucciones paso a paso para abrir la carpeta de la aplicación en VS Code, seleccionar el target `esp32s3`, elegir el puerto `COM`, abrir el `SDK Configuration Editor (menuconfig)` y usar los botones de la barra inferior (`Build`, `Flash`, `Monitor` y el botón de llama todo-en-uno).
- **8.3. Protocolo de Comandos / Telemetría y Uso:** Integra cualquier guía de uso, comandos de prueba o formato de paquetes que trajera el `README.md` original, detallando el formato exacto de las tramas UDP/TCP o comandos seriales en el código y con qué servidor de `software/ground_station/` interactúa.

### 9. Variables Críticas de `Kconfig` (`sdkconfig` / `menuconfig`)
- Incluye una tabla con las variables de `sdkconfig` que es indispensable verificar antes de compilar esta aplicación (como `CONFIG_FREERTOS_HZ=1000` y su relación con `SAMPLE_TIME` y `portTICK_PERIOD_MS`, `CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ=240`, `CONFIG_FREERTOS_UNICORE=n`, `CONFIG_ESP_TASK_WDT_EN`, optimización del compilador `-O2`, buffers LwIP/WiFi) y qué constantes `#define` del código (SSID/Password WiFi, IPs, puertos, ganancias o tiempos de muestreo) deberían migrarse a un archivo `main/Kconfig.projbuild`.
