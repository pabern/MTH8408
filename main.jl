workspace()
#using Plots
include("./MyModule.jl")
using MyModule
include("./f.jl")
using ForwardDiff
# X0 = x
n = 200
caFront       = [2260 220 290] #1
caRear        = [1890 250 290] #2
wheelCarrier  = [2110 520 320] #3
push          = [2210 245 590] #4
chassis       = [2210 200 535] #5
rockerAxis    = [2215 200 535] #6
shockA        = [2110 200 595] #7
shockB        = [2210 30  575]  #8

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

# (z,wheelRate) = WZ(x)

#plot(z,wheelRate)
#gui()

cfg = ForwardDiff.JacobianConfig(x)
Jf = zeros(n-1,26)
ForwardDiff.jacobian!(Jf, F, x, cfg)

∇c = Jf'*F(x)

k = 0
a = 0.5
while k < 10 # stopping conditions

  d = -∇c
  t = 1
  x2 = x = x + (t*d)
  while F(x2) > F(x)+ (a*t*∇c'*d)
    t = t/2
    x2 = x = x + (t*d)
  end #end Armijo while

  x = x + (t*d)
  cfg = ForwardDiff.JacobianConfig(x)
  Jf = zeros(n-1,26)
  ForwardDiff.jacobian!(Jf, F, x, cfg)

  ∇c = Jf'*F(x)

  k += 1 # Nest iteration

end # end while
