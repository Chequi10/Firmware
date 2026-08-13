# Sistema de Fallas - STM32 Hagie

## 1. Objetivo

Este documento describe el sistema de detección, almacenamiento,
transmisión y recuperación de fallas implementado en la STM32 del
sistema de control Hagie.

El sistema diferencia dos tipos principales de fallas:

1. Fallas individuales por cuerpo.
2. Fallas globales del sistema.

El objetivo principal es que una falla individual afecte solamente
al cuerpo correspondiente siempre que sea posible, mientras que una
falla global crítica puede provocar la detención de todos los cuerpos.


---

# 2. Fallas individuales por cuerpo

Cada uno de los 6 cuerpos posee su propio registro de fallas:

```cpp
body_faults[0]   // cuerpo 1
body_faults[1]   // cuerpo 2
body_faults[2]   // cuerpo 3
body_faults[3]   // cuerpo 4
body_faults[4]   // cuerpo 5
body_faults[5]   // cuerpo 6
```

Cada bit del registro representa una falla diferente.

Por esta razón, un mismo cuerpo puede tener más de una falla
simultáneamente.


## Tabla de fallas por cuerpo

| Bit | Hex | Falla | Estado |
|---:|---:|---|---|
| 0 | 0x01 | BODY_FAULT_ENCODER_TIMEOUT | Pendiente |
| 1 | 0x02 | BODY_FAULT_ENCODER_RANGE | Pendiente |
| 2 | 0x04 | BODY_FAULT_NO_MOVEMENT | IMPLEMENTADA Y PROBADA |
| 3 | 0x08 | BODY_FAULT_MIN_LIMIT | Pendiente |
| 4 | 0x10 | BODY_FAULT_MAX_LIMIT | Pendiente |
| 5 | 0x20 | BODY_FAULT_TARGET_TIMEOUT | IMPLEMENTADA Y PROBADA |
| 6 | 0x40 | BODY_FAULT_VALVE_ERROR | Pendiente |


---

# 3. BODY_FAULT_NO_MOVEMENT

Código:

```cpp
BODY_FAULT_NO_MOVEMENT = (1UL << 2)
```

Valor hexadecimal:

```text
0x04
```

Estado:

```text
IMPLEMENTADA Y PROBADA
```


## Objetivo

Detectar cuando se está ordenando movimiento a un cuerpo pero el
encoder no registra un desplazamiento suficiente.


## Funcionamiento

La supervisión se realiza individualmente para cada cuerpo.

Solamente se inicia la vigilancia cuando la orden de válvula supera
el umbral mínimo definido:

```cpp
MOVE_COMMAND_THRESHOLD = 100
```

Por lo tanto, si:

```text
command = 0
```

no se espera movimiento y no se genera esta falla.


## Movimiento mínimo

Actualmente se utiliza:

```cpp
MIN_BODY_MOVEMENT_MM = 2.0f
```

El cuerpo debe desplazarse al menos 2 mm dentro de la ventana de
supervisión.


## Timeout

Actualmente:

```cpp
NO_MOVEMENT_TIMEOUT = 1000 ms
```

Si existe una orden significativa de movimiento y durante ese tiempo
el encoder no registra al menos el movimiento mínimo requerido, se
activa:

```cpp
BODY_FAULT_NO_MOVEMENT
```


## Secuencia

```text
Orden de movimiento
        |
        v
command >= 100 o command <= -100
        |
        v
Comienza vigilancia
        |
        v
¿Encoder se movió >= 2 mm?
        |
     SI | NO
        |  |
        |  v
        | esperar hasta 1000 ms
        |  |
        |  v
        | NO_MOVEMENT = 0x04
        |  |
        |  v
        | detener cuerpo
        v
reiniciar ventana
```


## Acción de seguridad

Cuando se detecta la falla:

```cpp
body_faults[body] |= BODY_FAULT_NO_MOVEMENT;
```

se ordena:

```cpp
setBodyValveCommand(body, 0);
```

Por lo tanto se detiene solamente el cuerpo afectado.


## Bloqueo posterior

Mientras `BODY_FAULT_NO_MOVEMENT` permanezca activa, la función
central de control de válvulas no permite nuevas órdenes de movimiento
para ese cuerpo.

Una orden distinta de cero es convertida en cero.

La orden:

```text
command = 0
```

siempre permanece permitida por seguridad.


## Recuperación

Esta falla NO se borra automáticamente.

Ubuntu/Jetson debe enviar el comando de reconocimiento correspondiente.

Actualmente se utiliza:

```text
OPCODE 'J'
```

Payload:

```text
[0] = 'J'
[1] = cuerpo 0..5
```

La STM32 borra solamente:

```cpp
BODY_FAULT_NO_MOVEMENT
```

del cuerpo indicado.

Después del reconocimiento, el cuerpo queda:

```text
MANUAL
DETENIDO
```

No vuelve automáticamente a moverse.


---

# 4. BODY_FAULT_TARGET_TIMEOUT

Código:

```cpp
BODY_FAULT_TARGET_TIMEOUT = (1UL << 5)
```

Valor hexadecimal:

```text
0x20
```

Estado:

```text
IMPLEMENTADA Y PROBADA
```


## Objetivo

Detectar la pérdida de actualización de la altura objetivo de un
cuerpo que se encuentra funcionando en modo automático.

Esta falla es independiente para cada uno de los 6 cuerpos.


## Importante

`TARGET_TIMEOUT` NO significa que se perdió completamente la
comunicación entre Jetson y STM32.

La comunicación general puede continuar funcionando y el heartbeat
puede seguir llegando correctamente.

La falla significa específicamente:

```text
Un cuerpo está en AUTO
+
dejó de recibir nuevas consignas de altura 'D'
```


## Registro de última consigna

Cada cuerpo mantiene su propio instante de última actualización:

```cpp
target_last_update_tick[0]
target_last_update_tick[1]
target_last_update_tick[2]
target_last_update_tick[3]
target_last_update_tick[4]
target_last_update_tick[5]
```

Cada vez que llega una consigna válida `'D'` se actualiza el timestamp
correspondiente a ese cuerpo.


## Timeout

Actualmente:

```cpp
TARGET_TIMEOUT = 1000 ms
```

Si un cuerpo está en:

```cpp
BODY_CONTROL_AUTO
```

y transcurre más de ese tiempo sin recibir una nueva consigna válida,
se activa:

```cpp
BODY_FAULT_TARGET_TIMEOUT
```


## Acción de seguridad

Cuando ocurre el timeout:

```text
TARGET_TIMEOUT = 0x20
```

la STM32:

1. Marca la falla solamente en ese cuerpo.
2. Cambia ese cuerpo a modo MANUAL.
3. Ordena command = 0.
4. Detiene la vigilancia NO_MOVEMENT de ese cuerpo en ese ciclo.

Los demás cuerpos continúan funcionando normalmente.


## Ejemplo

Si solamente el cuerpo 1 pierde sus consignas:

```text
body_faults =
[
    0x20,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00
]
```

Esto NO genera por sí mismo una falla global.


## Recuperación automática

A diferencia de `NO_MOVEMENT`, `TARGET_TIMEOUT` se recupera
automáticamente.

Cuando vuelve a llegar una consigna `'D'` válida para ese cuerpo:

```cpp
body_faults[body] &=
    ~BODY_FAULT_TARGET_TIMEOUT;
```

y el cuerpo vuelve a:

```cpp
BODY_CONTROL_AUTO
```


## Secuencia probada

```text
Llegan consignas D
        |
        v
AUTO
        |
        v
body_fault = 0x00
        |
        v
dejan de llegar D a ese cuerpo
        |
        v
1000 ms
        |
        v
TARGET_TIMEOUT = 0x20
        |
        v
cuerpo detenido + MANUAL
        |
        v
vuelve a llegar D válida
        |
        v
TARGET_TIMEOUT se borra
        |
        v
body_fault = 0x00
        |
        v
AUTO
```

Esta secuencia fue comprobada mediante una prueba lógica desde Ubuntu.


---

# 5. Diferencia entre NO_MOVEMENT y TARGET_TIMEOUT

Estas dos fallas protegen situaciones diferentes.


## NO_MOVEMENT

```text
Jetson manda consigna
        |
        v
STM32 ordena mover válvula
        |
        v
cuerpo debería moverse
        |
        X
encoder no registra movimiento suficiente
        |
        v
NO_MOVEMENT = 0x04
```

El problema está relacionado con la falta de movimiento físico
detectado.


## TARGET_TIMEOUT

```text
Jetson
  |
  | D
  | D
  | D
  X deja de mandar D para ese cuerpo
  |
  v
STM32 detecta falta de actualización
  |
  v
TARGET_TIMEOUT = 0x20
```

El problema está relacionado con la pérdida de la consigna automática
de ese cuerpo.


---

# 6. Fallas globales

Las fallas globales se almacenan en:

```cpp
system_faults
```

Definiciones actuales:

| Bit | Hex | Falla |
|---:|---:|---|
| 0 | 0x01 | SYSTEM_FAULT_JETSON_TIMEOUT |
| 1 | 0x02 | SYSTEM_FAULT_IMU_TIMEOUT |
| 2 | 0x04 | SYSTEM_FAULT_UART_RX |
| 3 | 0x08 | SYSTEM_FAULT_UART_TX |
| 4 | 0x10 | SYSTEM_FAULT_CAN |


---

# 7. SYSTEM_FAULT_JETSON_TIMEOUT

Esta falla supervisa la comunicación general entre Jetson y STM32.

Es diferente de `BODY_FAULT_TARGET_TIMEOUT`.

Si se pierde completamente la comunicación válida con Jetson durante
el tiempo configurado, se activa:

```cpp
SYSTEM_FAULT_JETSON_TIMEOUT
```

Valor:

```text
0x01
```

Esta es una falla GLOBAL.


## Acción de seguridad

Ante la pérdida de comunicación con Jetson se detienen los cuerpos
por seguridad.

Esto protege el sistema frente a:

- desconexión USB,
- bloqueo del programa Ubuntu,
- pérdida completa de paquetes,
- caída de la Jetson,
- problemas graves de comunicación.


---

# 8. SYSTEM_FAULT_IMU_TIMEOUT

Código:

```cpp
SYSTEM_FAULT_IMU_TIMEOUT
```

Valor:

```text
0x02
```

Supervisa la recepción de información válida de la IMU.

La falla pertenece al registro global:

```cpp
system_faults
```


---

# 9. Fallas UART

Existen dos bits independientes:

```cpp
SYSTEM_FAULT_UART_RX = 0x04
SYSTEM_FAULT_UART_TX = 0x08
```

Permiten informar errores asociados a la comunicación serie entre
STM32 y Jetson.


---

# 10. SYSTEM_FAULT_CAN

Código:

```cpp
SYSTEM_FAULT_CAN
```

Valor:

```text
0x10
```

Representa una condición de falla global relacionada con la
comunicación CAN utilizada por la STM32.


---

# 11. Fallas definidas pero todavía no implementadas

Las siguientes fallas ya poseen un bit reservado en el protocolo,
pero todavía NO deben interpretarse como funciones completamente
implementadas.


## BODY_FAULT_ENCODER_TIMEOUT

```text
0x01
```

Estado:

```text
PENDIENTE
```

Su objetivo futuro será detectar pérdida o ausencia anormal de
actividad del encoder.

No debe implementarse simplemente como:

```text
"no llegaron pulsos durante X tiempo"
```

porque un encoder incremental puede permanecer sin pulsos
perfectamente cuando el cuerpo está detenido.

Actualmente parte de esta protección funcional está cubierta por
`NO_MOVEMENT`: si se ordena mover un cuerpo y no se detecta
desplazamiento suficiente, el sistema detiene ese cuerpo.


---

## BODY_FAULT_ENCODER_RANGE

```text
0x02
```

Estado:

```text
PENDIENTE
```

Detectará una lectura del encoder fuera de un rango físicamente
posible.

Debe implementarse cuando se disponga de la calibración real de los
6 cuerpos.

Los valores actuales utilizados durante desarrollo son provisorios
y NO deben utilizarse como límites físicos definitivos.


---

## BODY_FAULT_MIN_LIMIT

```text
0x08
```

Estado:

```text
PENDIENTE
```

Se utilizará para proteger el límite físico inferior de cada cuerpo.

Debe configurarse utilizando las dimensiones y calibraciones reales
de la máquina.


---

## BODY_FAULT_MAX_LIMIT

```text
0x10
```

Estado:

```text
PENDIENTE
```

Se utilizará para proteger el límite físico superior de cada cuerpo.

Debe configurarse utilizando las dimensiones y calibraciones reales
de la máquina.


---

## BODY_FAULT_VALVE_ERROR

```text
0x40
```

Estado:

```text
PENDIENTE
```

Se utilizará para representar fallas relacionadas con las válvulas
o con el diagnóstico recibido desde los controladores Axiomatic.

Su implementación definitiva dependerá de la información de
diagnóstico disponible desde los módulos reales.


---

# 12. Protección central de movimiento

La función:

```cpp
setBodyValveCommand()
```

constituye una barrera central de seguridad.

Actualmente bloquea movimiento si el cuerpo posee alguna de estas
fallas:

```cpp
BODY_FAULT_NO_MOVEMENT
BODY_FAULT_TARGET_TIMEOUT
```

Conceptualmente:

```cpp
blockingFaults =
    BODY_FAULT_NO_MOVEMENT |
    BODY_FAULT_TARGET_TIMEOUT;
```

Si alguna está activa y se intenta enviar:

```text
command != 0
```

la orden se convierte en:

```text
command = 0
```

Esto evita que otra parte del programa pueda volver a activar
accidentalmente una válvula de un cuerpo bloqueado.


---

# 13. Diagnóstico hacia Jetson

La STM32 transmite periódicamente el estado de fallas hacia
Ubuntu/Jetson mediante el paquete de diagnóstico.

Se envían:

```text
system_faults
body_faults[0]
body_faults[1]
body_faults[2]
body_faults[3]
body_faults[4]
body_faults[5]
```

Esto permite que Ubuntu conozca tanto las fallas globales como las
fallas individuales de cada cuerpo.


## Ejemplo

```text
system_faults = 0x00

body_faults =
[
    0x04,
    0x00,
    0x20,
    0x00,
    0x00,
    0x00
]
```

Significa:

```text
Sistema global:
sin fallas

Cuerpo 1:
NO_MOVEMENT

Cuerpo 2:
sin fallas

Cuerpo 3:
TARGET_TIMEOUT

Cuerpos 4, 5 y 6:
sin fallas
```

Como las fallas utilizan bits, un cuerpo también puede presentar
varias simultáneamente.


---

# 14. Filosofía de seguridad

El sistema sigue las siguientes reglas generales:

### 1. Una falla individual debe afectar solamente al cuerpo involucrado

Siempre que la condición lo permita, los otros cuerpos deben continuar
funcionando.


### 2. Una falla global puede detener todo el sistema

Una pérdida completa de comunicación con Jetson es un ejemplo de una
condición que requiere una acción global.


### 3. Detener siempre debe estar permitido

Aunque exista una falla crítica:

```text
command = 0
```

debe poder ejecutarse.


### 4. Una falla no debe producir movimiento

Las rutinas de manejo de fallas solamente pueden mantener o llevar
el actuador a una condición segura.


### 5. Las fallas se mantienen separadas por bits

Esto permite que varias fallas puedan coexistir y ser diagnosticadas
independientemente.


### 6. Los límites físicos no deben inventarse

`ENCODER_RANGE`, `MIN_LIMIT` y `MAX_LIMIT` se implementarán cuando
se conozcan y calibren los recorridos físicos reales de la Hagie.


---

# 15. Estado actual

## Implementado y probado

```text
BODY_FAULT_NO_MOVEMENT
BODY_FAULT_TARGET_TIMEOUT
```

## Implementado en el sistema global

```text
SYSTEM_FAULT_JETSON_TIMEOUT
SYSTEM_FAULT_IMU_TIMEOUT
SYSTEM_FAULT_UART_RX
SYSTEM_FAULT_UART_TX
SYSTEM_FAULT_CAN
```

## Pendiente de implementación definitiva

```text
BODY_FAULT_ENCODER_TIMEOUT
BODY_FAULT_ENCODER_RANGE
BODY_FAULT_MIN_LIMIT
BODY_FAULT_MAX_LIMIT
BODY_FAULT_VALVE_ERROR
```


---

# 16. Próximos pasos

Cuando se disponga del hardware y calibraciones reales:

1. Determinar recorrido físico real de cada cuerpo.
2. Configurar límites mínimos y máximos individuales.
3. Implementar `ENCODER_RANGE`.
4. Implementar `MIN_LIMIT`.
5. Implementar `MAX_LIMIT`.
6. Definir diagnóstico de pérdida/falla eléctrica de encoder.
7. Implementar `ENCODER_TIMEOUT`.
8. Analizar diagnóstico real de los módulos Axiomatic.
9. Implementar `VALVE_ERROR`.
10. Realizar pruebas de falla con hardware real.
