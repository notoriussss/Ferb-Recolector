# 0003. Un repositorio solo para nuestro recolector

**Estado:** aceptada (05/10/2026)

## Contexto
En la pista hay varios robots, pero cada uno es un proyecto distinto de otro equipo. Nuestro equipo de 4 personas construye un solo robot recolector.

## Decisión
Este repo contiene solo nuestro robot. No hay código de los otros robots ni de coordinación de una flota. Lo que haya que acordar con los otros equipos se anota en `docs/interfaz-externa.md`.

## Consecuencias
- El repo es más chico y cada persona sabe qué le toca.
- Si se acuerda comunicación por ROS con los otros robots, los mensajes comunes se agregan como dependencia acordada, sin copiar su código.
