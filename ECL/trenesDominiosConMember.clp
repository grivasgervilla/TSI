encontrarAsignacion(T1, T2, T3, T4):-
  %empieza lo que llamaríamos el proceso de búsqueda
  member(T1, [1,2,3]),
  member(T2, [1,2,3]),
  member(T3, [1,2,3]),
  member(T4, [1,2,3]),
  %acaba el proceso de búsqueda

  %empieza el proceso de comprobación de consistencia
  T1=\=T2, T2=\=T3,T2=\=T4,T3=\=T4,
  T3=\=3,
  T4=\=3, T4=\=2.
  %acaba el proceso de comprobación de consistencia
