#= -------------------------------------------------------------------------------------------------
Fonction F(x)
Calcule la fonction objectif à optimiser.
x =: [12,1]
F =: Fonction objectif
--------------------------------------------------------------------------------------------------=#
function F(x)
  wheelRate = wheelrate(x)
  fonctionObjectif = norm(wheelRate - (0.2 * Z' + 28))^2
  return fonctionObjectif
end
