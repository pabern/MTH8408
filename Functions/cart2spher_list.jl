#= -------------------------------------------------------------------------------------------------
Fonction cart2spher_list(P)
Transformation d'une liste de points cartésiens en points sphériques
P =: Matrice de points [x,3]
T =: Vecteur de translation [1,3]
--------------------------------------------------------------------------------------------------=#
function cart2spher_list(P)
  n = length(P[:,1])
  x = P[:,1]
  y = P[:,2]
  z = P[:,3]
  r = sqrt(diag(P*P'))

  theta = atan(y./x)

  phi = acos(z./r)
  
  M = [r theta phi]
  return M
end
