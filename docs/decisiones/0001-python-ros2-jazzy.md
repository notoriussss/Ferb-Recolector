# 0001. Python 3.12 + ROS 2 Jazzy en Ubuntu 24.04

**Estado:** aceptada

## Contexto
La pizarra de clase fija la pila Pi 5 → HAT → SO → ROS 2 → aplicación. Somos 4 personas con niveles distintos y hay poco tiempo. El robot hace control a 20–50 Hz, visión a 15 Hz y nada de cálculo pesado.

## Decisión
- Ubuntu 24.04 LTS, porque es la versión que soporta oficialmente ROS 2 Jazzy y corre en la Pi 5.
- ROS 2 Jazzy (LTS hasta 2029).
- Todo el código en Python 3.12 con `rclpy`. Mensajes propios en un paquete `ament_cmake` (es la única forma de generar interfaces).
- OpenCV para visión y `lgpio` para GPIO.

## Consecuencias
- Todo el equipo puede leer y revisar cualquier paquete.
- Si algún nodo resulta lento en la Pi (por ejemplo la visión), se puede reescribir solo ese nodo en C++ sin tocar el resto, porque los tópicos no cambian.
- RPi.GPIO no sirve en la Pi 5; hay que usar `lgpio` o `gpiozero`.
