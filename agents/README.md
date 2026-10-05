# agents/

Guías cortas para que un asistente de IA (o alguien nuevo) entienda un área sin leer todo el repo. `AGENTS.md` en la raíz tiene las reglas generales.

| Archivo | Área | Dueño |
|---|---|---|
| `base-movil.md` | motores, HAT, línea, ultrasonido, pala | rol 1 |
| `vision.md` | cámara, HSV, sensor de color | rol 2 |
| `comportamiento.md` | máquina de estados, seguridad, mensajes | rol 3 |
| `simulacion-integracion.md` | Webots, launch files, CI | rol 4 |
| `revisor.md` | lista para revisar un PR | todos |

Uso:
- **Claude Code** lee `CLAUDE.md` → `AGENTS.md` solo. Para una tarea: "lee `agents/vision.md` y ayúdame con …".
- **Copilot, Cursor o Codex:** abre la guía en el editor o pégala al inicio del chat.
- **Revisar un PR:** "revisa este diff con `agents/revisor.md`".

Cada dueño mantiene su guía: lo que se aprenda a la mala (un pin, un umbral, una falla) va aquí.
