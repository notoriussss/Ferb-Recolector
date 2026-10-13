# Un cubo visto por la cámara: color, ángulo y distancia aproximada.
# El ángulo y la distancia se calculan sobre el piso, suponiendo cámara a altura fija
# e inclinada hacia abajo (ver vision/color_detection.py).

string color          # nombre del color: rojo, naranja, amarillo, verde, azul, morado
float32 bearing_rad   # ángulo respecto al frente de la cámara; positivo = a la izquierda
float32 distance_m    # distancia sobre el piso desde la cámara hasta el borde de abajo del cubo
int32 u               # columna del centro del cubo en la imagen (px)
int32 v               # fila del borde de abajo del cubo en la imagen (px)
int32 width_px        # ancho de la caja en la imagen (px)
int32 height_px       # alto de la caja en la imagen (px)
