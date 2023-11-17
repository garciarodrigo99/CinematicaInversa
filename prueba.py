import math

# Puntos en el plano
x, y = -1, -1

# Calcular ángulo utilizando math.atan2
angulo = math.atan2(y, x)

# Convertir el ángulo a grados
angulo_grados = math.degrees(angulo)

# Imprimir resultados
print(f"Ángulo para ({x}, {y}): {angulo} radianes")
print(f"Ángulo en grados: {angulo_grados} grados")
