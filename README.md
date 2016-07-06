# Range_Based_Loc
El programa utiliza un robot U que estima la posición de 3 robots vecinos W ultilizando las medidas de los sensores de distancia 
y la odometría.

Estos datos se comparten desde el robot u hacia los robots w y viceversa en cada iteración (round).

Finalmente se calcula la posición y angulo relativo de los robots W con respecto al robot U.

El programa inicia estableciendo posiciones iniciales para todos los robots y comandos de movimiento. También 
se pueden incluir varianzas en el movimiento y en las medidas de la distancia (se encuentran en 0)

- El comando movement_command, contiene la acción de control del robot U y movement_command2 la de los robots W.

En cada iteración (100 iteraciones) 
- Se ejecuta el comando de moviminento para U y para todos W.
- Se actualiza la distancia obtenida de los sensores get_Distance.
- Se obtiene la odometría cada cierto numero de iteraciones (count_round = 10). 10 datos de odometria.
- Para cada robot W se estima la posición y orientación teniendo en cuenta
  - a = odometria de robot U entre round j y k.
  - b = odometria de robot W(i) entre round j y k.
  - c = distancia de U a W(i) en instante j.
  - d = distancia de U a W(i) en instante k.
  
  - Se realiza una optimización para encontrar dos incognitas: Posicion relativa y orientacion de cada robot W en instante k
  referenciados en el sistema de coordenadas del robot U en el instante k.

  
