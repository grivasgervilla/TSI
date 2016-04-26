% La base de conocimiento
dist(hamburg, bremen, 80).
dist(hamburg, berlin, 230).
dist(bremen, dortmund, 200).
dist(hannover, nuernberg, 380).
dist(dortmund, koeln, 80).
dist(kassel, frankfurt, 180).
dist(nuernberg, muenchen, 160).
dist(hamburg, hannover, 110).
dist(bremen, hannover, 100).
dist(hannover, kassel, 140).
dist(dortmund, kassel, 130).
dist(kassel, wuerzburg, 180).
dist(frankfurt, wuerzburg, 110).

%Ver si hay ruta directa entre dos ciudades.
dire(X,Y,Km) :- dist(Y,X,Km); dist(X,Y,Km).

%Predicado cierto si hay ruta entre From y To
route(From, To):-writeln(From),dire(From, To, Dist).
route(From, To):-writeln(From),dire(From,X,Dist),route(X,To).

%Predicado cierto si hay una ruta de From a To sin repetir ciudades
route(From, To, Route):-writeln(From),dire(From, To, Dist).
route(From,To, Route):-writeln(From), dire(From,X,Dist),not(member(X, Route)),append([X], Route, Route1),route(X,To,Route1).

%Calculo de caminos con coste en Km
route(From, To, Route, [], Km):-dire(From,To,Km).
route(From, To, Route, CiudadesNecesarias, Km):- 
	dire(From,X,Dist),
	not(member(X, Route)),
	append([X], Route, Route1),
	(member(X,CiudadesNecesarias)
		-> delete(X,CiudadesNecesarias,CiudadesNecesarias2)
		; CiudadesNecesarias2 = CiudadesNecesarias
	),
	route(X,To,Route1,CiudadesNecesarias2,Dist2), 
	Km is Dist+Dist2.

%Funcion que calcula el minimo de dos valores
minimo(X,Y,Z):-
    (X =< Y
     -> Z = X
     ;  Z = Y  
    ).

listMin([L|Ls], Min) :-
       listMin(Ls, L, Min).

listMin([], Min, Min).

listMin([L|Ls], Min0, Min) :-
    minimo(L, Min0, Min1),
    listMin(Ls, Min1, Min).

shortestPath(From,To,Route,CiudadesNecesarias,Kmminimum):-
	findall(Km,route(From,To,Route,CiudadesNecesarias,Km),Kms),
    	listMin(Kms, Kmminimum).



