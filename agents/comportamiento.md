# Guía: comportamiento

**Dueño:** rol 3 · **Archivos:** `src/mobile_base/mobile_base/{behavior_fsm,obstacle_guard}.py`, `src/recolector_interfaces/`, `docs/arquitectura.md`, `docs/interfaz-externa.md`

## Lo que hay que saber

- La propuesta de nodos, tópicos y estados está en `docs/arquitectura.md`. Es un punto de partida, no una obligación.
- La velocidad pasa siempre por `obstacle_guard` antes del motor.
- Idea de niveles del ultrasonido (del plan): >30 cm normal, 15–30 cm mitad de velocidad, <15 cm parado.
- El cuadrado de entrega es negro y está pegado a la línea: hay que distinguir "llegué al cuadrado" de "me voy a salir".
- La pala lleva un cubo a la vez.
- Los otros robots son de otros equipos; lo que se acuerde con ellos va en `docs/interfaz-externa.md`.
- Conviene escribir la máquina de estados como lógica pura sin `rclpy`, para probarla con `pytest`.
