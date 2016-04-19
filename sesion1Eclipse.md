Ejemplo 5:

Vamos a ver cómo hacer un programa de manera rápida para que me calcule la combinación
optima para esto. Si lo quisiéramos hacer en C++ tardaríamos mucho en hacerlo,
en cambio con el paradigma que vamosa usar vamos a tarda mucho menos.

En primer lugar nos tenemos que ver cómo podemos modelar un PSC. La codificación
de un problema de satisfacción de restricciones tiene tres elementos:

### VARIABLES
T1 T2 T3 T4 cada una de las variables lo que representa es un tren, que será asignado a una locomotora.


### DOMINIOS

Cada tren puede ser asignado a cualquiera de las locomotoras disponibles, entonces tenemos que:

Dom(Ti) = {1,2,3} con i = 1,2,3,4

Una vez establecemos los dominios estamos estableciendo también la estructura interna de los nodos, los nodos siempre van a tener la misma estructura que es la de una asignación.

Luego nos preguntamos si un determinado estado, si una asignación concreta, es solución o no de nuestro problema.

### RESTRICCIONES

Veamos ahora cómo expresamos las restricciones, por escrito nos podemos permitir cierta libertad en el lenguaje matemático que empleamos para implementar estas restricciones. Luego ya veremos cómo se traduce esto al lenguaje de programación concreto.

<center>
|    | 8 | 9 | 10 | 11 | 12 | 13 | 14 | 15 |
|----|---|---|----|----|----|----|----|----|
| T1 | X | X |    |    |    |    |    |    |
| T2 |   | X | X  | X  | X  | X  |    |    |
| T3 |   |   |    |    | X  | X  | X  |    |
| T4 |   |   |    | X  | X  | X  | X  | X  |
</center>
lo que estamos haciendo es un diagrama de Gantt para usar los horarios de los trenes para poder establecer las restricciones.

Como no se pueden solapar los horarios puesto que una locomotora sólo puede tirar de un objeto a la vez, entonces:

* T1 != T2
* T2 != T3
* T2 != T4
* T3 != T4

No podemos poner en el dominio de cada tren sólo las locomotoras que pueden con él porque ya estaríamos resolviendo en parte el problema, por esto la siguiente restricción, la tenemos que codificar aquí. L3 no tiene potencia para arrastrar a T3:

* T3 != 3   

L2 y L3 no tienen potencia para arrastrar a T4:

* T4 != 3   
* T4 != 2   

**Nota:** *no hemos tenido en cuenta la restricción de que la locomotora necesita una hora para estar disponible para llevar otro tren, pero en nuestro caso con las restricciones que ya hemos impuesto esto no nos afecta.*

Luego ya le pondremos una instrucción en nuestro lenguaje de programación para que comience el proceso de búsqueda.

## Proceso de búsqueda

Lo que se usa es un procedimiento de búsqueda con backtracking, partiendo de un nodo vacío y luego ir progresivamente rellenando las asignaciones.
No hacemos que los hijos de un nodo sean las posibles asignaciones, sino que lo que hacemos que los hijos serán posibles asignaciones a una variable, de modo que el proceso de búsqueda sea una construcción incremental de una asignación.

<center>

![Alt text](http://g.gravizo.com/g?
  digraph G {
    T1 -> T2 [label = "!="];
    T2 -> T3 [label = "!="];
    T2 -> T4 [label = "!="];
    T3 -> T4 [label = "!="];
    T3 -> T3 [label = "!= 3"];
    T4 -> T4 [label = "!= 3"];
    T4 -> T4 [label = "!= 2"];
  }
)

</center>

Entonces como podemos ver en este grafo el la más restrictiva es T4 puesto que es la que participa en un mayor número de restricciones, con lo cual por lógica esta es a la primera a la que le damos valor.

## Cuestiones sobre Prolog

* En prolog las variables han de comenzar siempre por mayúscula.
* X = 1+2 lo que hace es crear un vínculo con +(1,2), una variable en prolog no es lo mismo que es programación, es como las incógnitas de álgebra. Prolog lo que hace es interpretar +(1,2) como un predicado. Si lo que queremos es víncular una variable con el resultado de una expresión lo que usamos es el is, **X is 1+2**.
* Las variables en Prolog no hay que definirlas, con poner un nombre que empiece por una letra mayúscula ya estoy usando una variable.
* **fail** lo que hace es indicarle a Prolog lo que hace es decirle a Prolog que queremos que haga un backtraking, entonces en el ejemplo en el que tenemos el predicadao *selectValor(X)* es como si este predicado tuviese 4 hijos, los 4 valores que le podemos asignar a la X, entonces cada vez que haga un backtraking lo que hace es volver arriba y bajar a **otro** hijo. Siempre que pongamos un fail hemos de darle una *alternativa de escape* porque en otro caso lo que hará es siempre fallar.
* Cuando cambiamos la definición de un predicado es bueno hacer File > clear toplevel module
