workspace()
using PyPlot
include("./MyModule.jl")
using MyModule
include("./f.jl")
using ForwardDiff
# X0 = x
n = 200
caFront       = [2260 220 290] #1
caRear        = [1890 250 290] #2
wheelCarrier  = [2110 520 320] #3
push          = [2110 245 600] #4
chassis       = [2110 200 535] #5
rockerAxis    = [2115 200 535] #6
shockA        = [2110 200 595] #7
shockB        = [2110 30  575]  #8

travel = 51                    #9
springRate = 61.3              #10

x = zeros(26)
x[1:3] = caFront
x[4:6] = caRear
x[7:9] = wheelCarrier
x[10:12] = push
x[13:15] = chassis
x[16:18] = rockerAxis
x[19:21] = shockA
x[22:24] = shockB
x[25] = travel
x[26] = springRate
#=
pointList = [caFront;     # 1
            caRear;       # 2
            wheelCarrier; # 4
            push;         # 5
            chassis;      # 6
            rockerAxis;   # 7
            shockA;       # 8
            shockB]       # 9

plot_pointList(pointList)
=#

(z,wheelRate,check) = WZ(x)

if check == true
  error("Le point de départ n'est pas valide !")
end

#plot(z,wheelRate)
#gui()

cfg = ForwardDiff.JacobianConfig(x)
Jf = zeros(n-1,26)
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
