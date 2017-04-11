#= -------------------------------------------------------------------------------------------------
Fonction translate3D!(P,T)
Exécute la translation d'une matrice de point dans l'espace 3D
P =: Matrice de points [x,3]
T =: Vecteur de translation [1,3]
--------------------------------------------------------------------------------------------------=#
function translate3D(P,T)
  n = size(P,1)
  P = P - ones(n)*T'
end
