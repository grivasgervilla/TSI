;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; ejercicio 3
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(define (domain GAME)
  (:requirements :strips :typing :adl)
  (:types zona
          ;bosque agua precipicio arena piedra - zona
          elemento
          personaje robot objeto - elemento
			    objeto
  )
  (:constants R - elemento)

  (:functions
    (energia) - number
    (puntos) - number
  )

  (:predicates
    (en ?e - elemento ?z - zona)
    (ady ?z1 - zona ?z2 - zona)
    (ManoVacia)
    (MochilaVacia)
    (cogido ?o - objeto)
    (almacenado ?o - objeto)
    (tiene ?p - personaje ?o - objeto)

    (oscar ?o - objeto)
    (manzana ?o - objeto)
    (rosa ?o - objeto)
    (algoritmo ?o - objeto)
    (oro ?o - objeto)
    (zapatilla ?o - objeto)
    (bikini ?o - objeto)

    (bosque ?z - zona)
    (agua ?z - zona)
    (precipicio ?z - zona)
    (arena ?z - zona)
    (piedra ?z - zona)

    (leo ?p - personaje)
    (profesor ?p - personaje)
    (principe ?p - personaje)
    (princesa ?p - personaje)
    (bruja ?p - personaje)
  )

  (:action coger
	     :parameters (?o - objeto ?z - zona)
	     :precondition (and (en ?o ?z) (en R ?z) (ManoVacia))
	     :effect
	     (and
         (not (en ?o ?z))
		     (not (ManoVacia))
         (cogido ?o)
		   )
  )

  (:action almacenar
	     :parameters (?o - objeto ?z - zona)
	     :precondition (and (en ?o ?z) (en R ?z) (MochilaVacia))
	     :effect
	     (and
         (not (en ?o ?z))
		     (not (MochilaVacia))
         (almacenado ?o)
		   )
  )

  (:action soltar
	     :parameters (?o - objeto ?z - zona)
	     :precondition (and (cogido ?o) (en R ?z))
	     :effect
	     (and
         (not (cogido ?o))
  		   (ManoVacia)
  		   (en ?o ?z)
       )
  )

  (:action descargar
	     :parameters (?o - objeto ?z - zona)
	     :precondition (and (almacenado ?o) (en R ?z))
	     :effect
	     (and
         (not (almacenado ?o))
  		   (MochilaVacia)
  		   (en ?o ?z)
       )
  )

  (:action ir
 	     :parameters (?z1 - zona ?z2 - zona)
 	     :precondition (and (> (energia) 0)
                          (en R ?z1)
                          (or (ady ?z1 ?z2) (ady ?z2 ?z1))
                          (or
                            (and (not (bosque ?z2)) (not (agua ?z2)) )
                            (and (bosque ?z2) (exists (?o - objeto) (and (zapatilla ?o) (or (cogido ?o) (almacenado ?o))) ) )
                            (and (agua ?z2) (exists (?o - objeto) (and (bikini ?o) (or (cogido ?o) (almacenado ?o)))) )
                          )
                          (not (precipicio ?z2))
                       )
 	     :effect
 	     (and
          (when (piedra ?z1) (decrease (energia) 2))
          (when (not (piedra ?z1)) (decrease (energia) 1))
          (not (en R ?z1))
   		    (en R ?z2)
      )
   )

  (:action dar
      :parameters (?p - personaje ?o - objeto ?z - zona)
      :precondition (and (en ?p ?z) (en R ?z) (cogido ?o))
      :effect
      (and
        (when (and (leo ?p) (oscar ?o)) (increase (puntos) 10))
        (when (and (leo ?p) (rosa ?o)) (increase (puntos) 1))
        (when (and (leo ?p) (manzana ?o)) (increase (puntos) 10))
        (when (and (leo ?p) (algoritmo ?o)) (increase (puntos) 4))
        (when (and (leo ?p) (oro ?o)) (increase (puntos) 5))

        (when (and (princesa ?p) (oscar ?o)) (increase (puntos) 10))
        (when (and (princesa ?p) (rosa ?o)) (increase (puntos) 10))
        (when (and (princesa ?p) (manzana ?o)) (increase (puntos) 10))
        (when (and (princesa ?p) (algoritmo ?o)) (increase (puntos) 3))
        (when (and (princesa ?p) (oro ?o)) (increase (puntos) 4))

        (when (and (bruja ?p) (oscar ?o)) (increase (puntos) 10))
        (when (and (bruja ?p) (rosa ?o)) (increase (puntos) 5))
        (when (and (bruja ?p) (manzana ?o)) (increase (puntos) 10))
        (when (and (bruja ?p) (algoritmo ?o)) (increase (puntos) 1))
        (when (and (bruja ?p) (oro ?o)) (increase (puntos) 3))

        (when (and (profesor ?p) (oscar ?o)) (increase (puntos) 10))
        (when (and (profesor ?p) (rosa ?o)) (increase (puntos) 4))
        (when (and (profesor ?p) (manzana ?o)) (increase (puntos) 10))
        (when (and (profesor ?p) (algoritmo ?o)) (increase (puntos) 10))
        (when (and (profesor ?p) (oro ?o)) (increase (puntos) 1))

        (when (and (principe ?p) (oscar ?o)) (increase (puntos) 10))
        (when (and (principe ?p) (rosa ?o)) (increase (puntos) 3))
        (when (and (principe ?p) (manzana ?o)) (increase (puntos) 10))
        (when (and (principe ?p) (algoritmo ?o)) (increase (puntos) 5))
        (when (and (principe ?p) (oro ?o)) (increase (puntos) 10))

        (tiene ?p ?o)
        (not (cogido ?o))
        (ManoVacia)
      )
  )
)
