# Cómo contribuir

## Flujo

1. Toma o crea un issue con la plantilla "Tarea".
2. Crea una rama desde `main`: `feature/<area>-<tema>`, `fix/<area>-<tema>` o `docs/<tema>`.
3. Commits pequeños con el formato `area: qué`, en español:
   - `mobile_base: frenar si no llega cmd_vel`
   - `vision: rango HSV del morado`

   Áreas: `interfaces`, `mobile_base`, `vision`, `bringup`, `simulation`, `tools`, `scripts`, `docs`, `ci`.
4. Antes del PR: `ruff check src tools`, `colcon build` y `colcon test`.
5. Abre el PR contra `main` con la plantilla llena y pide revisión al dueño del área (`docs/equipo.md`).
6. Se fusiona con squash cuando la CI está verde y lo aprobó alguien que no es el autor.

## Reglas

- `main` protegida.
- Un PR, un cambio. Si pasa de ~400 líneas, pártelo.
- Si cambias un tópico o mensaje, actualiza `docs/arquitectura.md` en el mismo PR.
- Parámetros nuevos en YAML, con unidad en el nombre.
- Código en inglés; comentarios, logs y documentación en español.
- Si usaste un asistente de IA, revisa tú el código: el autor del PR responde por él.
