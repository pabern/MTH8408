#= -------------------------------------------------------------------------------------------------
Fonction findclosepoint(P1, P2, P3)
Calcule le point sur une droite infinie étant le plus près d'un point donné de l'espace.
P1 =: Point 1 définissant la droite
P2 =: Point 2 définissant la droite
P3 =: Point donné de l'espace
P4 =: Point recherché 
--------------------------------------------------------------------------------------------------=#
function findclosepoint(P1, P2, P3)
  v = unitvector(P1, P2)
  P4 = P2 + (sum((P3-P2).*v))*v
  return P4
end
