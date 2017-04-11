#= -------------------------------------------------------------------------------------------------
Fonction unitvector(P1, P2)
Calcule un vecteur unitaire avec comme entrée deux points
P1 =: Point 1 définissant la droite
P2 =: Point 2 définissant la droite
v =: Vecteur unitaire
--------------------------------------------------------------------------------------------------=#
function unitvector(P1, P2)
  v = (P2-P1)./norm(P2-P1)
  return v
end
