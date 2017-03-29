workspace()
using PyPlot
include("./MyModule.jl")
using MyModule
using ForwardDiff



# Les points d'entrée du datum de suspension
caFront       = [2260 220 290] #1 - Immobile
caRear        = [1890 250 290] #2 - Immobile
wheelCarrier  = [2110 520 320] #3 - Immobile
push          = [2110 245 600] #4 - Variable
chassis       = [2110 200 535] #5 - Variable
rockerAxis    = [2115 200 535] #6 - Variable
shockA        = [2110 200 595] #7 - Variable
shockB        = [2110  30 575] #8 - Variable

# 4,5,7 devraient être dans le même plan, potentiellement. Et 6 devrait être normal à ce plan. Bon, ce n'est pas nécessaire, mais ça facilite la fabrication.

travel = 51                    #9 - Immobile
springRate = 61.3              #10 - Immobile

# Nombre de points de la discrétisation
n = 100
# Variables de départs, x_0
x = zeros(15)
x[1:3] = push
x[4:6] = chassis
x[7:9] = rockerAxis
x[10:12] = shockA
x[13:15] = shockB
# Paramètres du modèle, Q
Q = zeros(11)
Q[1:3] = caFront
Q[4:6] = caRear
Q[7:9] = wheelCarrier
Q[10] = travel
Q[11] = springRate

(pointWC,pointList) = transGeo(x,Q,n)
(wheelRate,check) = fwheelRate(pointWC,pointList,Q[11])
# Fonction objectif F
if check == false
  fonctionObjectif = F(x,Q,n)
  else
    error("Le point de départ n'est pas valide !")
end

cfg = ForwardDiff.JacobianConfig(x)
Jf = zeros(n-1,15)
ForwardDiff.jacobian!(Jf, F, x, cfg)

grad = Jf'*F(x)
gradNorm0 = norm(grad)
gradNorm = gradNorm0
fk = 0.5 * norm(F(x))# Valeur de la fonction en x_0
f0 = fk

k = 0
a = 10e-4
while k < 200 && gradNorm > 1.0e-6 * gradNorm0  # stopping conditions
  d = -grad
  slope = dot(grad,d) # Valeur de la descente
  t = 1
  check = true
  while check == true # On valide si la fonction est réalisable
    x2 = x + (t*d)
    (z,wheelRate,check) = WZ(x2)
    if check == true
      t /= 1.5
      continue
    end
    if 0.5 * norm(F(x2)) > fk + (a*t*slope)
      t /= 1.5
      check = true
      continue
    end
  end # End While check == true

  x += (t*d)
  cfg = ForwardDiff.JacobianConfig(x)
  Jf = zeros(n-1,26)
  ForwardDiff.jacobian!(Jf, F, x, cfg)

  grad = Jf'*F(x)
  gradNorm = norm(grad)
  fk = 0.5 * norm(F(x))
  k += 1 # Next iteration
  @printf "%2d " k
  @printf "%9.2e " fk
  @printf "%7.1e " gradNorm
  @printf "%7.1e\n" t

end # end while
