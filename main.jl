workspace()
include("./MyModule.jl")
using MyModule
include("./f.jl")

n = 200
caFront       = [2260 220 290] #1
caRear        = [1890 250 290] #2
wheelCarrier  = [2110 520 320] #3
push          = [2210 245 590] #4
chassis       = [2210 200 535] #5
rockerAxis    = [2215 200 535] #6
shockA        = [2110 200 595] #7
shockB        = [2210 30 575]  #8

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

wheelRate = f(x)
#=
out = zeros(26,n-1)
cfg = ForwardDiff.JacobianConfig(x)
ForwardDiff.jacobian!(out, f, x, cfg)



using Plots
(z, wheelRate) = f(x)
plot(z,wheelRate)
plot!(z,0.2*z+28)
gui()


using ReverseDiff: GradientTape, GradientConfig, gradient, gradient!, compile_gradient


inputs = x
results = similar(x)
cfg = GradientConfig(inputs)
gradient!(results, f, inputs, cfg)

=#

using ForwardDiff

out = zeros(n-1,26)
cfg = ForwardDiff.JacobianConfig(x)
ForwardDiff.jacobian!(out, f, x, cfg) # Erreur ici
