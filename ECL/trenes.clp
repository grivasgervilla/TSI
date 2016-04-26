%Dominio de T1
valorT1(1).
valorT1(2).
valorT1(3).
%Dominio de T2
valorT2(1).
valorT2(2).
valorT2(3).
%Dominio de T3
valorT3(1).
valorT3(2).
valorT3(3).
%Dominio de T4
valorT4(1).
valorT4(2).
valorT4(3).

encontrarAsignacion(T1, T2, T3, T4):-
  valorT1(T1),
  valorT2(T2),
  valorT3(T3),
  valorT4(T4),
  T1=\=T2, T2=\=T3,T2=\=T4,T3=\=T4,
  T3=\=3,
  T4=\=3, T4=\=2.
