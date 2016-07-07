(define (problem ej1)

  (:domain GAME)

  (:objects
    O1 O2 O3 O4 O5 - objeto
    Mononoke - personaje
    Gauss - personaje
    Layton - personaje
    Escarlata - personaje
    Costigan - personaje
    Z1 Z2 Z3 Z4 Z5 Z6 Z7 Z8 Z9 Z10 Z11 Z12 Z13 Z14 Z15 Z16 Z17 Z18 Z19 Z20 Z21 Z22 Z23 Z24 Z25 - zona
  )

  (:INIT
    (= (energia) 16)

   	(ady Z1 Z2)
    (ady Z1 Z3)
    (ady Z1 Z4)
    (ady Z2 Z5)
    (ady Z2 Z6)
    (ady Z2 Z7)
    (ady Z3 Z8)
    (ady Z3 Z9)
    (ady Z3 Z10)
    (ady Z4 Z11)
    (ady Z4 Z12)
    (ady Z4 Z13)
    (ady Z14 Z5)
    (ady Z15 Z6)
    (ady Z16 Z7)
    (ady Z17 Z8)
    (ady Z18 Z9)
    (ady Z19 Z10)
    (ady Z20 Z11)
    (ady Z21 Z12)
    (ady Z22 Z13)
    (ady Z14 Z23)
    (ady Z15 Z23)
    (ady Z16 Z23)
    (ady Z17 Z24)
    (ady Z18 Z24)
    (ady Z19 Z24)
    (ady Z20 Z25)
    (ady Z21 Z25)
    (ady Z22 Z25)

    (en R Z1)

    (en O1 Z2)
    (en O2 Z3)
    (en O3 Z5)
    (en O4 Z7)
    (en O5 Z11)

    (en Mononoke Z2)
    (en Gauss Z4)
    (en Layton Z8)
    (en Escarlata Z16)
    (en Costigan Z25)

    (MV)
  )

  (:goal
    (exists (?o1 ?o2 ?o3 ?o4 ?o5 - objeto)
      (and (tiene Mononoke ?o1) (tiene Costigan ?o2) (tiene Gauss ?o3) (tiene Layton ?o4) (tiene Escarlata ?o5))
    )
  )
)
