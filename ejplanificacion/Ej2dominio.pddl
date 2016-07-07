;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; ejercicio 2
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(define (domain GAME)
  (:requirements :strips :typing :adl)
  (:types zona
          elemento
          personaje robot objeto - elemento
			    oscar manzana rosa algoritmo oro - objeto
  )
  (:constants R - elemento)

  (:functions
    (energia) - number
  )

  (:predicates
    (en ?e - elemento ?z - zona)
    (ady ?z1 - zona ?z2 - zona)
    (MV)
    (cogido ?o - objeto)
    (tiene ?p - personaje ?o - objeto)
  )

  (:action coger
	     :parameters (?o - objeto ?z - zona)
	     :precondition (and (en ?o ?z) (en R ?z) (MV))
	     :effect
	     (and
         (not (en ?o ?z))
		     (not (MV))
         (cogido ?o)
		   )
  )

  (:action dejar
	     :parameters (?o - objeto ?z - zona)
	     :precondition (and (cogido ?o) (en R ?z))
	     :effect
	     (and
         (not (cogido ?o))
  		   (MV)
  		   (en ?o ?z)
       )
  )

 (:action ir
	     :parameters (?z1 - zona ?z2 - zona)
	     :precondition (and (> (energia) 0) (en R ?z1) (or (ady ?z1 ?z2) (ady ?z2 ?z1)))
	     :effect
	     (and
         (decrease (energia) 1)
         (not (en R ?z1))
  		   (en R ?z2)
       )
  )

  (:action dar
      :parameters (?p - personaje ?o - objeto ?z - zona)
      :precondition (and (en ?p ?z) (en R ?z) (cogido ?o))
      :effect
      (and
        (tiene ?p ?o)
        (not (cogido ?o))
        (MV)
      )
  )
)
