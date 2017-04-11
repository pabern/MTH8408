#= -------------------------------------------------------------------------------------------------
Fonction cart2spher(P)
Transformation d'un point cartésien en un point sphérique
P =: Matrice de points [x,3]
T =: Vecteur de translation [1,3]
--------------------------------------------------------------------------------------------------=#
function cart2spher(P)
  x = P[1]
  y = P[2]
  z = P[3]
  r = sqrt((x^2) + (y^2) + (z^2))
  if x == 0
    theta = 0
  else
    theta = atan(y/x)
  end
  if r == 0
    phi = 0
  else
    phi = acos(z/r)
  end
  return [r theta phi]
end
