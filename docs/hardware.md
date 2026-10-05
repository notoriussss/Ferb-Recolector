# Hardware y conexiones

Los valores marcados **medir** salen de fotos o de hojas de datos y hay que confirmarlos con el robot en la mano.

## Recolector

| Pieza | Modelo | Notas |
|---|---|---|
| Chasis | LewanSoul / Hiwonder mecanum | 186 × 161 × 91 mm, ruedas de 65 mm |
| Motores | 4× TT 1:120, 3–6 V | sin encoders |
| Computador | Raspberry Pi 5, 8 GB | necesita 5 V / 5 A |
| Driver | HAT Cokoino 4WD (2× DRV8833) | **máximo 10,8 V → batería 2S**; vendido para Pi 4, confirmar pines en la Pi 5 |
| Regulador Pi | XL4015 a 5,1 V | aparte del HAT |
| Batería | 2S Li-ion 18650 (7,4 V) | con fusible e interruptor general |
| Pala | impresa en 3D (ver plan, sección 6) | un cubo a la vez |
| Servo de la pala | SG90 | 0° abajo, ≈40° arriba (**medir**) |
| Cámara | módulo de cámara Pi o webcam USB | 640 × 480 |
| Sensores de línea | 4× TCRT5000 | uno por esquina |
| Ultrasonido | HC-SR04 | frente; el eco a 5 V necesita divisor a 3,3 V |
| Presencia en la pala | FC-51 (IR) | 1 = hay cubo |
| Color en la pala | TCS34725 (I2C) | confirma el color del cubo |
| IMU (opcional) | MPU6050 (I2C) | giros aproximados |

## Pines (por llenar)

Rellenar con lo que se mida en el HAT y copiar los mismos números en `src/mobile_base/config/recolector.yaml`.

| Señal | GPIO (BCM) | Estado |
|---|---|---|
| Motor FL IN1 / IN2 | ? / ? | medir |
| Motor FR IN1 / IN2 | ? / ? | medir |
| Motor RL IN1 / IN2 | ? / ? | medir |
| Motor RR IN1 / IN2 | ? / ? | medir |
| Servo pala | ? | medir |
| TCRT5000 FL / FR / RL / RR | ? / ? / ? / ? | medir |
| HC-SR04 TRIG / ECHO | ? / ? | medir |
| FC-51 | ? | medir |
| I2C SDA / SCL | 2 / 3 | fijo en la Pi |

## Pista

| Medida | Valor | Estado |
|---|---|---|
| Tablero | ≈ 1,93 × 1,22 m (de la foto) | medir |
| Ancho de la cinta | ≈ 19 mm | medir |
| Cuadrado de entrega | ≈ 24 cm de lado | medir |
| Cubos | 2,5 cm, ≈ 9,4 g | medir |
| Colores | rojo, naranja, amarillo, verde, azul, morado/vinotinto | confirmar vinotinto |
