Implementar abiertos como una cola con prioridad que ordene los nodos según prioridad,
también lo podemos implementar como una cola simple.

La información de cómo de cerca está una casilla de un obstáculo está dentro del costmap.

Evidentemente tenemos un explosión de estados, entonces la primera versión del A\* va a tardar
bastante en encontrar un camino, entonces aquí entra la segunda parte de la entrega, el
emplear una de las técnicas de [la web sobre le A* para juegos](http://theory.stanford.edu/~amitp/GameProgramming/) para que podemos y así
vaya más rápido. Estamos hablando de algo parecido a la **poda** alfa-beta pero no tiene que ser esa.

Podemos tomar por ejemplo la celda visible, mi *vecino*, que esté más alejada ya que de nuestra pos inicial hasta esta casilla será espacio libre. Habrá que tener cuidado con el padre de cada casilla, puesto que ahora no será una casilla colindante a ella sino la pos. actual del robot.
