(define (problem ej5)

  (:domain GAME)

  (:objects
    zapatilla1 oscar1 oscar2 oscar3 bikini1 manzana1 manzana2 manzana3 manzana4 - objeto
    Mononoke - personaje
    Gauss - personaje
    Layton - personaje
    Escarlata - personaje
    Costigan - personaje
    Z1 Z2 Z3 Z4 Z5 Z6 Z7 Z8 Z9 Z10 Z11 Z12 Z13 Z14 Z15 Z16 Z17 Z18 Z19 Z20 Z21 Z22 Z23 Z24 Z25 - zona
  )

  (:INIT
    (= (energia) 100)
    (= (puntos) 0)
    (= (capacidad) 5)

    (arena Z1) (arena Z5) (arena Z6) (arena Z8) (arena Z10) (arena Z17) (arena Z18) (arena Z19) (arena Z24)
    (piedra Z2) (piedra Z4) (piedra Z12) (piedra Z13) (piedra Z23)
    (agua Z3) (agua Z7) (agua Z16)
    (bosque Z11) (bosque Z20) (bosque Z21) (bosque Z22) (bosque Z25)
    (precipicio Z9) (precipicio Z14) (precipicio Z15)

    (princesa Mononoke) (principe Gauss) (profesor Layton) (bruja Escarlata) (leo Costigan)

   	(ady Z1 Z2) (ady Z1 Z3) (ady Z1 Z4) (ady Z2 Z5) (ady Z2 Z6) (ady Z2 Z7)
    (ady Z3 Z8) (ady Z3 Z9) (ady Z3 Z10) (ady Z4 Z11) (ady Z4 Z12) (ady Z4 Z13)
    (ady Z14 Z5) (ady Z15 Z6) (ady Z16 Z7) (ady Z17 Z8) (ady Z18 Z9) (ady Z19 Z10)
    (ady Z20 Z11) (ady Z21 Z12) (ady Z22 Z13) (ady Z14 Z23) (ady Z15 Z23) (ady Z16 Z23)
    (ady Z17 Z24) (ady Z18 Z24) (ady Z19 Z24) (ady Z20 Z25) (ady Z21 Z25) (ady Z22 Z25)

    (en R Z1)

    (en zapatilla1 Z2) (en oscar1 Z21) (en oscar2 Z22) (en oscar3 Z20) (en bikini1 Z11)
    (en manzana1 Z2) (en manzana2 Z4) (en manzana3 Z8) (en manzana4 Z16)

    (zapatilla zapatilla1) (bikini bikini1)
    (oscar oscar1) (oscar oscar2) (oscar oscar3)
    (manzana manzana1) (manzana manzana2) (manzana manzana3) (manzana manzana4)

    (en Mononoke Z2)
    (en Gauss Z4)
    (en Layton Z8)
    (en Escarlata Z16)
    (en Costigan Z25)

    (ManoVacia)
  )

  (:metric minimize (puntos))

  (:goal
    (exists (?o1 ?o2 ?o3 ?o4 ?o5 - objeto)
      (and (tiene Mononoke ?o1) (tiene Costigan ?o2) (tiene Gauss ?o3) (tiene Layton ?o4) (tiene Escarlata ?o5))
    )
  )


)
