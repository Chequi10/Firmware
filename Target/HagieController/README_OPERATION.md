# Funcionamiento General - Firmware STM32 Hagie

## 1. Objetivo

Este documento describe el funcionamiento general del firmware de la
STM32 utilizado en el sistema Hagie.

La STM32 actúa como controlador de tiempo real entre:

```text
Jetson / Ubuntu
      |
      | USB / UART
      |
      v
    STM32
      |
      +---- Encoders
      |
      +---- IMU CAN / J1939
      |
      +---- Módulos Axiomatic
      |
      +---- Válvulas hidráulicas
```

La Jetson realiza las tareas de alto nivel.

La STM32 se encarga principalmente de:

- comunicación en tiempo real;
- lectura de los 6 encoders;
- control de altura de los 6 cuerpos;
- generación de órdenes hacia las válvulas;
- comunicación CAN;
- recepción de IMU;
- supervisión de fallas;
- watchdogs;
- parada segura del sistema;
- telemetría hacia Ubuntu.


---

# 2. Arquitectura general

El sistema se divide conceptualmente en dos niveles.


## Nivel de alto nivel - Jetson / Ubuntu

Responsabilidades principales:

```text
IA
visión
cámaras
generación de consignas
interfaz de usuario
diagnóstico
supervisión del sistema
```

La Jetson determina principalmente qué altura debe tener cada cuerpo.


## Nivel de tiempo real - STM32

Responsabilidades principales:

```text
lectura encoder
control de altura
manejo de válvulas
CAN
IMU
watchdogs
seguridad
diagnóstico
```

La STM32 no depende de Linux para las acciones críticas de control y
seguridad.


---

# 3. Cantidad de cuerpos

El sistema controla:

```cpp
BODY_COUNT = 6
```

Correspondencia:

```text
body 0 = cuerpo 1
body 1 = cuerpo 2
body 2 = cuerpo 3
body 3 = cuerpo 4
body 4 = cuerpo 5
body 5 = cuerpo 6
```


---

# 4. Encoders

Existen 6 encoders.

Cada uno está asociado a un timer de la STM32 configurado en modo
encoder.

Actualmente:

```text
Encoder 1 -> TIM2
Encoder 2 -> TIM5
Encoder 3 -> TIM3
Encoder 4 -> TIM4
Encoder 5 -> TIM8
Encoder 6 -> TIM1
```

Los timers de 32 bits y 16 bits son manejados por la clase:

```cpp
Encoder
```


## Variables principales

Para cada encoder se almacenan:

```cpp
encoder_position[]
encoder_delta[]
encoder_direction[]
encoder_height_mm[]
```


## Tarea

La lectura se realiza en:

```cpp
Task_encoder()
```

Frecuencia actual:

```text
cada 10 ms
aproximadamente 100 Hz
```


## Secuencia de lectura

```text
timer
  |
  v
Encoder::update()
  |
  +--> posición
  |
  +--> delta
  |
  +--> dirección
  |
  +--> altura en mm
```

La altura se obtiene a partir de la calibración configurada para cada
encoder.


## Calibraciones actuales

Las calibraciones utilizadas durante desarrollo son PROVISORIAS.

No deben considerarse valores físicos definitivos de la Hagie.

Las calibraciones finales deberán realizarse con la máquina real.


---

# 5. Modos de control

Cada cuerpo puede funcionar en uno de dos modos:

```cpp
BODY_CONTROL_MANUAL
BODY_CONTROL_AUTO
```


---

# 6. Modo MANUAL

En MANUAL, la Jetson envía directamente una orden de válvula.

La orden utiliza el comando:

```text
OPCODE 'B'
```

Payload:

```text
[0] = 'B'
[1] = cuerpo
[2] = command MSB
[3] = command LSB
```

El comando es un:

```cpp
int16_t
```

Rango utilizado:

```text
-1000 = bajar al máximo
    0 = detener
+1000 = subir al máximo
```


## Comportamiento

Cuando llega `'B'`:

```text
Jetson
  |
  | B + cuerpo + comando
  v
STM32
  |
  +--> cuerpo pasa a MANUAL
  |
  +--> setBodyValveCommand()
  |
  v
válvula
```


---

# 7. Modo AUTO

En AUTO, la Jetson no controla directamente la válvula.

La Jetson envía una:

```text
altura objetivo
```

mediante:

```text
OPCODE 'D'
```


## Payload

```text
[0] = 'D'
[1] = cuerpo
[2] = altura MSB
[3] = altura LSB
```


## Recepción

Cuando llega una consigna válida:

```cpp
target_height_mm[body] = heightMm;
```

También se almacena:

```cpp
target_last_update_tick[body]
```

para supervisar el timeout individual de consigna.


## Flujo AUTO

```text
Jetson
  |
  | altura objetivo
  v
STM32
  |
  | target_height_mm
  v
Task_height_control
  |
  | compara
  |
  +--> altura objetivo
  |
  +--> altura encoder
  |
  v
error
  |
  v
control proporcional
  |
  v
setBodyValveCommand()
  |
  v
Axiomatic
  |
  v
válvula
```


---

# 8. Control de altura

La tarea:

```cpp
Task_height_control()
```

se ejecuta aproximadamente a:

```text
100 Hz
```

Actualmente utiliza un controlador proporcional simple.


## Parámetros actuales

```cpp
KP = 5.0f
DEADBAND_MM = 10.0f
```

Estos valores son iniciales y deberán ajustarse con la hidráulica real.


## Error

```text
error = altura objetivo - altura actual
```


## Zona muerta

Si:

```text
-10 mm <= error <= +10 mm
```

la válvula queda detenida.


## Control

Si el cuerpo está por debajo de la consigna:

```text
command > 0
```

Si está por encima:

```text
command < 0
```

La salida se limita a:

```text
-1000 ... +1000
```


---

# 9. Condiciones para funcionar en AUTO

Actualmente el control automático se ejecuta solamente cuando:

```text
cuerpo en AUTO
+
comunicación Jetson válida
+
no existe falla crítica que impida movimiento
```

La seguridad definitiva también se aplica nuevamente dentro de:

```cpp
setBodyValveCommand()
```

Esto crea una segunda barrera independiente del controlador AUTO.


---

# 10. Manejo central de válvulas

Toda orden de movimiento pasa por:

```cpp
setBodyValveCommand()
```

Esta función es una parte crítica de la seguridad del sistema.


## Funciones principales

La función:

1. verifica que el cuerpo sea válido;
2. verifica fallas que bloquean movimiento;
3. limita el comando;
4. determina las salidas físicas;
5. apaga primero ambas direcciones;
6. activa solamente una dirección;
7. guarda el comando para diagnóstico.


---

# 11. Dos salidas por cuerpo

Cada cuerpo utiliza dos salidas:

```text
SUBIR
BAJAR
```

Por lo tanto:

```text
6 cuerpos x 2 salidas = 12 salidas
```


## Distribución

```text
Cuerpo 1 -> salidas 0 y 1
Cuerpo 2 -> salidas 2 y 3
Cuerpo 3 -> salidas 4 y 5
Cuerpo 4 -> salidas 6 y 7
Cuerpo 5 -> salidas 8 y 9
Cuerpo 6 -> salidas 10 y 11
```


---

# 12. Seguridad subir / bajar

Antes de activar una dirección se desactivan ambas:

```text
UP = 0
DOWN = 0
```

Después se activa solamente la salida correspondiente.

Esto evita ordenar simultáneamente subir y bajar.


---

# 13. Módulos Axiomatic

Actualmente están previstos:

```cpp
VALVE_MODULE_COUNT = 3
```

Módulos:

```text
Axiomatic 1
Axiomatic 2
Axiomatic 3
```

Direcciones provisionales:

```text
0x21
0x22
0x23
```


## Cantidad de salidas

Cada módulo posee:

```text
4 salidas
```

Por lo tanto:

```text
3 módulos x 4 salidas = 12 salidas
```

exactamente las necesarias para los 6 cuerpos.


---

# 14. Comunicación CAN con Axiomatic

La comunicación utiliza:

```text
CAN1
```

La tarea principal de transmisión es:

```cpp
Task_can1_axiomatic_tx()
```


## Frecuencia

Actualmente:

```text
cada 20 ms
aproximadamente 50 Hz
```


## Flujo

```text
valve_command[]
      |
      v
ValveController
      |
      v
CAN1
      |
      v
Axiomatic
      |
      v
válvulas
```


## Importante

Parte del formato CAN/J1939 utilizado para las válvulas todavía es
provisorio.

Los PGN, identificadores y parámetros definitivos deberán configurarse
cuando se disponga de la configuración final de los módulos Axiomatic.


---

# 15. Recepción CAN

La interrupción CAN recibe las tramas y las coloca en:

```cpp
AxiomaticRxQueueHandle
```

La tarea:

```cpp
Task_can1_axiomatic_rx()
```

procesa posteriormente esas tramas fuera de la interrupción.

Esto evita realizar procesamiento pesado dentro de la ISR.


---

# 16. IMU

La IMU prevista es la Axiomatic:

```text
AX060900
```

La comunicación se realiza mediante:

```text
CAN / J1939
```

La clase encargada es:

```cpp
ImuCan
```


## Estado almacenado

```cpp
ImuState_t
```

contiene:

```text
roll
pitch
gravity

gyro roll
gyro pitch
gyro yaw

accel X
accel Y
accel Z

valid
timestamp
```


---

# 17. Watchdog de IMU

La tarea:

```cpp
Task_imu_watchdog()
```

supervisa que la IMU continúe actualizándose.

Timeout actual:

```text
500 ms
```

Si una IMU previamente válida deja de actualizarse:

```text
imu_state.valid = false
```

y se activa:

```cpp
SYSTEM_FAULT_IMU_TIMEOUT
```


---

# 18. Comunicación Jetson - STM32

La comunicación con Ubuntu / Jetson utiliza:

```text
USART3
115200 baud
8N1
```

La recepción y transmisión utilizan DMA.


---

# 19. Recepción UART con DMA

La recepción utiliza:

```cpp
HAL_UARTEx_ReceiveToIdle_DMA()
```

con un buffer:

```text
256 bytes
```

La interrupción de recepción no procesa directamente el protocolo.

Los bytes recibidos se colocan en:

```cpp
JetsonRxQueueHandle
```


## Flujo RX

```text
Jetson
  |
  v
USART3
  |
  v
DMA
  |
  v
HAL_UARTEx_RxEventCallback()
  |
  v
JetsonRxQueue
  |
  v
Task_jetson_serial_rx()
  |
  v
packet_decoder
  |
  v
interface::handle_packet()
```


---

# 20. Transmisión UART con DMA

Los paquetes que deben enviarse hacia Jetson se colocan en:

```cpp
JetsonTxQueueHandle
```

La tarea:

```cpp
Task_jetson_serial_tx()
```

los transmite mediante:

```cpp
HAL_UART_Transmit_DMA()
```


## Flujo TX

```text
tarea productora
      |
      v
packet_encoder
      |
      v
JetsonTxQueue
      |
      v
Task_jetson_serial_tx
      |
      v
DMA
      |
      v
USART3
      |
      v
Jetson
```


---

# 21. Ventaja de las colas

Las tareas que generan telemetría no necesitan esperar a que UART
termine físicamente de transmitir.

Esto desacopla:

```text
producción de datos
```

de:

```text
transmisión serie
```

y evita bloqueos innecesarios.


---

# 22. Protocolo Jetson -> STM32

Actualmente existen los siguientes comandos principales:


## 'A' - Heartbeat

```text
Jetson -> STM32
```

Indica actividad de comunicación.


## 'B' - Comando directo de válvula

```text
MANUAL
```


## 'C' - Detener todas las válvulas

Coloca los cuerpos en MANUAL y envía:

```text
command = 0
```


## 'D' - Altura objetivo

```text
AUTO
```

Cada cuerpo recibe una altura objetivo independiente.


## 'J' - Reconocer NO_MOVEMENT

Permite reconocer / borrar:

```cpp
BODY_FAULT_NO_MOVEMENT
```

de un cuerpo.


---

# 23. Protocolo STM32 -> Jetson

Actualmente la STM32 transmite principalmente:


## 'E' - Estado de encoders

Contiene las alturas actuales de los 6 cuerpos.


## 'F' - Estado de válvulas

Contiene los comandos actuales de los 6 cuerpos.


## 'G' - Diagnóstico

Contiene:

```text
errores
contadores
system_faults
body_faults[0..5]
```


## 'H' - Estado general STM32

Contiene información como:

```text
uptime
estado comunicación Jetson
cantidad de encoders
cantidad de cuerpos
cantidad de módulos Axiomatic
```


## 'I' - Estado IMU

Contiene:

```text
valid
ángulos
velocidades angulares
aceleraciones
```


---

# 24. Telemetría

La tarea:

```cpp
Task_jetson_telemetry_tx()
```

envía periódicamente:

```text
E - Encoders
F - Válvulas
G - Diagnóstico
H - Estado STM32
I - IMU
```

Frecuencia actual:

```text
100 ms
aproximadamente 10 Hz
```


---

# 25. Watchdog general de Jetson

La STM32 mantiene:

```cpp
jetson_last_valid_packet_tick
```

y:

```cpp
jetson_connection_ok
```

Timeout actual:

```text
500 ms
```


## Funcionamiento

Si la STM32 deja de recibir paquetes válidos de Jetson durante el
timeout:

```text
Jetson communication timeout
```

se ejecuta una parada global.


## Acción

```text
detener cuerpo 1
detener cuerpo 2
detener cuerpo 3
detener cuerpo 4
detener cuerpo 5
detener cuerpo 6
```

Luego:

```cpp
jetson_connection_ok = false;
```

y se activa:

```cpp
SYSTEM_FAULT_JETSON_TIMEOUT
```


---

# 26. Diferencia entre watchdog global y TARGET_TIMEOUT

Son dos protecciones diferentes.


## JETSON_TIMEOUT

Significa:

```text
STM32 dejó de recibir comunicación válida de Jetson
```

Es una falla global.


## TARGET_TIMEOUT

Significa:

```text
un cuerpo determinado dejó de recibir consignas AUTO
```

Puede ocurrir aunque la comunicación general Jetson-STM32 continúe
funcionando correctamente.

Es una falla individual.


---

# 27. Supervisión NO_MOVEMENT

La tarea:

```cpp
Task_body_fault_monitor()
```

también supervisa movimiento físico.

Si existe una orden significativa de movimiento pero la altura del
encoder no cambia suficientemente durante el tiempo establecido:

```cpp
BODY_FAULT_NO_MOVEMENT
```

se detiene solamente ese cuerpo.


---

# 28. TARGET_TIMEOUT

La misma tarea supervisa:

```cpp
target_last_update_tick[body]
```

para cada uno de los seis cuerpos.

Si un cuerpo está en AUTO y pasa demasiado tiempo sin una nueva
consigna:

```cpp
BODY_FAULT_TARGET_TIMEOUT
```

se detiene solamente ese cuerpo y pasa a MANUAL.


---

# 29. FreeRTOS

El firmware utiliza FreeRTOS para separar las distintas funciones de
tiempo real.


## Tareas principales

Actualmente se utilizan, entre otras:

```text
Task_jetson_serial_rx
Task_jetson_serial_tx
Task_jetson_telemetry_tx

Task_encoder

Task_height_control

Task_body_fault_monitor

Task_can1_axiomatic_tx
Task_can1_axiomatic_rx

Task_imu_watchdog
```


---

# 30. Task_jetson_serial_rx

Responsabilidad:

```text
consumir bytes recibidos desde Jetson
```

La tarea espera en:

```cpp
JetsonRxQueueHandle
```

y alimenta el decodificador de protocolo.


---

# 31. Task_jetson_serial_tx

Responsabilidad:

```text
transmitir paquetes completos hacia Jetson
```

Utiliza:

```text
DMA
task notifications
timeout de transmisión
```


---

# 32. Task_jetson_telemetry_tx

Responsabilidad:

```text
generar telemetría periódica
```

Actualmente transmite:

```text
E
F
G
H
I
```


---

# 33. Task_encoder

Responsabilidad:

```text
leer los seis encoders
actualizar posición
actualizar delta
actualizar dirección
calcular altura
```


---

# 34. Task_height_control

Responsabilidad:

```text
control automático de altura
```

Solo trabaja con cuerpos en:

```text
AUTO
```


---

# 35. Task_body_fault_monitor

Responsabilidad actual:

```text
NO_MOVEMENT
TARGET_TIMEOUT
```

Se ejecuta aproximadamente a:

```text
50 Hz
```


---

# 36. Task_can1_axiomatic_tx

Responsabilidad:

```text
enviar comandos hacia los módulos Axiomatic
```

También contiene actualmente la supervisión del watchdog general de
Jetson.


---

# 37. Task_can1_axiomatic_rx

Responsabilidad:

```text
procesar las tramas recibidas por CAN1
```

Actualmente entrega las tramas extendidas al manejador de IMU.


---

# 38. Task_imu_watchdog

Responsabilidad:

```text
supervisar vigencia de datos de IMU
```


---

# 39. Prioridades y diseño

Las tareas críticas de comunicación y control poseen prioridades
superiores a las tareas exclusivamente informativas.

La arquitectura busca evitar:

```text
esperas bloqueantes largas
procesamiento pesado en interrupciones
dependencia directa entre productores y consumidores
```


---

# 40. Colas FreeRTOS

Actualmente existen colas como:

```cpp
JetsonRxQueueHandle
JetsonTxQueueHandle
AxiomaticRxQueueHandle
```

Estas desacoplan:

```text
interrupciones
tareas de procesamiento
tareas de transmisión
```


---

# 41. Interrupciones

Las interrupciones realizan solamente trabajo mínimo.


## CAN RX

```text
leer trama
poner trama en cola
salir
```


## UART RX

```text
identificar bytes nuevos del DMA
poner bytes en cola
salir
```


## UART TX complete

```text
notificar tarea TX
salir
```

El procesamiento principal se realiza posteriormente en tareas
FreeRTOS.


---

# 42. Diagnóstico de UART

Existen contadores para registrar condiciones como:

```text
uart_error_count
tx_queue_dropped
tx_dma_errors
rx_queue_dropped
```

Estas variables ayudan a detectar:

```text
errores físicos UART
colas saturadas
errores DMA
problemas de rendimiento
```


---

# 43. Diagnóstico CAN

Si no puede enviarse una trama CAN correctamente se activa:

```cpp
SYSTEM_FAULT_CAN
```

También existe:

```cpp
axiomatic_rx_dropped
```

para contar tramas descartadas si la cola CAN RX se llena.


---

# 44. LEDs de diagnóstico

Actualmente algunos LEDs de la STM32 se utilizan como indicadores de
actividad de tareas y comunicación.

Ejemplos:

```text
LED encoder
LED CAN
LED actividad comunicación
```

Su función principal durante desarrollo es facilitar la detección
visual de tareas bloqueadas o ausencia de actividad.


---

# 45. Flujo completo de una consigna AUTO

Ejemplo para cuerpo 3:

```text
1. IA / Jetson calcula altura deseada
               |
               v
2. Ubuntu envía OPCODE 'D'
               |
               v
3. STM32 valida cuerpo y altura
               |
               v
4. guarda target_height_mm[2]
               |
               v
5. actualiza target_last_update_tick[2]
               |
               v
6. cuerpo pasa a AUTO
               |
               v
7. Task_height_control compara:
       target vs encoder
               |
               v
8. calcula command
               |
               v
9. setBodyValveCommand()
               |
               v
10. ValveController
               |
               v
11. CAN
               |
               v
12. Axiomatic
               |
               v
13. válvula hidráulica
               |
               v
14. cuerpo se mueve
               |
               v
15. encoder mide movimiento
               |
               +------------------+
               |                  |
               v                  |
16. STM32 vuelve a controlar -----+
```


---

# 46. Flujo completo de seguridad

Durante el funcionamiento AUTO pueden ocurrir diferentes situaciones.


## Caso normal

```text
consigna válida
+
encoder responde
+
Jetson comunica
=
control normal
```


## Jetson desaparece

```text
JETSON_TIMEOUT
->
detener todos los cuerpos
```


## Un cuerpo deja de recibir consignas

```text
TARGET_TIMEOUT
->
detener solo ese cuerpo
```


## Se manda mover pero no se desplaza

```text
NO_MOVEMENT
->
detener solo ese cuerpo
```


---

# 47. Estado seguro

El estado seguro básico para un cuerpo es:

```text
command = 0
```

Esto implica:

```text
salida subir = 0
salida bajar = 0
```

Las funciones de seguridad deben poder ordenar este estado incluso si
existen fallas activas.


---

# 48. Arranque

Al iniciar, las salidas deben comenzar desactivadas.

Los cuerpos inicialmente se encuentran en:

```text
MANUAL
```

y sin orden de movimiento.


---

# 49. Datos actualmente provisorios

Los siguientes elementos todavía deberán ajustarse con hardware real:

```text
calibraciones de encoders
recorridos físicos
KP
deadband
límites mínimos
límites máximos
PGN J1939 definitivos
direcciones/configuración final Axiomatic
escalas finales de actuadores
timeouts finales
```


---

# 50. Elementos ya probados lógicamente

Durante el desarrollo sin hardware completo se han realizado pruebas
lógicas de:

```text
comunicación Jetson-STM32

CRC/protocolo

heartbeat

watchdog Jetson

DMA RX

DMA TX

telemetría

diagnóstico

NO_MOVEMENT

reconocimiento de NO_MOVEMENT

TARGET_TIMEOUT individual

recuperación automática de TARGET_TIMEOUT
```


---

# 51. Elementos pendientes de prueba con hardware

Cuando se disponga de la máquina real será necesario comprobar:

```text
sentido real de cada encoder
cuentas por milímetro
recorrido completo
sentido subir/bajar
corriente y respuesta de válvulas
configuración Axiomatic
PGN J1939
latencias CAN
respuesta hidráulica
ajuste KP
deadband
NO_MOVEMENT real
límites mecánicos
diagnóstico de válvulas
diagnóstico eléctrico de encoder
IMU real
```


---

# 52. Filosofía general del firmware

El firmware sigue estas reglas:

1. La Jetson decide; la STM32 ejecuta y protege.
2. Las funciones críticas permanecen en la STM32.
3. Las interrupciones realizan el mínimo trabajo posible.
4. Las tareas se comunican mediante colas y estados compartidos.
5. Una falla individual debe detener solamente el cuerpo afectado.
6. Una pérdida global de comunicación debe llevar el sistema a estado seguro.
7. `command = 0` siempre debe estar permitido.
8. Ningún cuerpo debe moverse si una falla crítica lo bloquea.
9. Los límites físicos definitivos no se inventan: se calibran en la máquina.
10. La telemetría debe permitir conocer desde Ubuntu el estado real de la STM32.


---

# 53. Documentación relacionada

Consultar también:

```text
README_FAULTS.md
```

para la descripción detallada del sistema de fallas, bits, acciones de
seguridad y condiciones de recuperación.
