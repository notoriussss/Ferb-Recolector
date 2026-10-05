# Lista para revisar un PR

1. ¿Hace lo que dice la descripción?
2. **Seguridad:** ¿algo publica en `cmd_vel` saltándose `obstacle_guard`? ¿Sigue frenando sin comandos? ¿Se subió una velocidad sin medir?
3. **Tópicos:** si cambió un tópico o mensaje, ¿se actualizó `docs/arquitectura.md`?
4. **Parámetros:** ¿hay números mágicos que deberían ir al YAML, con unidad en el nombre?
5. **Pruebas:** ¿la lógica nueva tiene prueba? ¿Pasan `colcon build` y `colcon test`?
6. **Laptop:** ¿el nodo corre sin el hardware real?
7. **Estilo:** código en inglés, comentarios en español, `ruff` limpio, commits `area: qué`.
8. **Archivos de más:** nada de `build/`, `install/`, `log/`, fotos pesadas ni credenciales.
