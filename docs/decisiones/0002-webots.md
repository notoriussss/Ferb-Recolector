# 0002. Webots para simular

**Estado:** aceptada

## Contexto
Hay que probar la lógica sin gastar batería ni arriesgar los robots. Gazebo quedó descartado para el curso. Se necesita simular ruedas mecanum, cámara y sensores de línea.

## Decisión
Webots R2025a. Ya existe un mundo fiel a la pista (`simulation/webots/`) con el PROTO `RecolectorUCAB`: mecanum con fricción a 45°, sin encoders, pala con servo, cámara, 4 sensores de línea, ultrasonido, IR, sensor de color e IMU. El siguiente paso es `webots_ros2` para que los mismos nodos ROS muevan el robot simulado.

## Consecuencias
- En Windows se corre Webots nativo y ROS 2 en WSL2.
- La fricción simulada no es igual a la real; los giros se calibran en el robot.
