Implementar abiertos como una cola con prioridad que ordene los nodos según prioridad,
también lo podemos implementar como una cola simple.

La información de cómo de cerca está una casilla de un obstáculo está dentro del costmap.

Evidentemente tenemos un explosión de estados, entonces la primera versión del A\* va a tardar
bastante en encontrar un camino, entonces aquí entra la segunda parte de la entrega, el
emplear una de las técnicas de [la web sobre le A* para juegos](http://theory.stanford.edu/~amitp/GameProgramming/) para que podemos y así
vaya más rápido. Estamos hablando de algo parecido a la **poda** alfa-beta pero no tiene que ser esa.

Podemos tomar por ejemplo la celda visible, mi *vecino*, que esté más alejada ya que de nuestra pos inicial hasta esta casilla será espacio libre. Habrá que tener cuidado con el padre de cada casilla, puesto que ahora no será una casilla colindante a ella sino la pos. actual del robot.

Dice que se va a usar el amcl un módulo de localización que tiene una utilidad a la que le puedes pasar directamente la posición inicial en la que se encuentra el robot sin tener que hacer el pose y tal con el ratón (supongo que es para Rviz). Pero cuando se le ponen las coordendas hay que tener en cuenta que el mapa de Rviz está girado 270º con respecto al mapa. En otras palabras las coordenas de Rviz son:
la coordenada que haya en Stage compuesto con una **rotación de 270º**. Por tanto cuando la pongamos en el amcl tenemos que hacer esta transformación para pasarle las coordendas (cambiar la x por la y y cambiar el signo de la y o algo así).

*Tarea extra* encontrar como hacer con ROS esto automáticamente.

Hay otro paquete de localización que es fake_localization que es el usado para el local si no encontramos la forma de hacer esto de forma automática de hacer la rotación entonces no podemos usar el fake_localization ya que no le podemos pasar directamente la posición del robot.

Ideas para tener en cuenta obstáculos:
1. Tener un parámetro alfa que multiplicará a un término que le sumaremos a la distancia euclídea. Este sumando se calculará como la mínima distancia a un obstáculo.
2. No lo sabemos seguro pero si en las casillas del costamp hay distintos grados de valor indicando la cercanía de obstáculos quizás se pueda usar directamente esto para mejorar la *h* con el objetivo de encontrar el camino más corto y más seguro.

Otras notas:
1. Configurar INFLATED_OBSTACLE sobre todo si las casillas son muy finas para no chocarnos.
