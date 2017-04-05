include("./ini.jl")
#using PyPlot

caFront       = [2260 220 290] #1 - Immobile
caRear        = [1890 250 290] #2 - Immobile
wheelCarrier  = [2110 520 320] #3 - Immobile
push          = [2110 245 600] #4 - Variable
chassis       = [2110 200 535] #5 - Variable
rockerAxis    = [2115 200 535] #6 - Variable
shockA        = [2110 200 595] #7 - Variable
shockB        = [2110  30 575] #8 - Variable

travel = 51                    #9 - Immobile
springRate = 61.3              #10 - Immobile


# Nombre de points de la discrétisation
global n = 100
# Variables de départs, x_0
x0 = zeros(15)
x0[1:3] = push
x0[4:6] = chassis
x0[7:9] = rockerAxis
x0[10:12] = shockA
x0[13:15] = shockB
# Paramètres du modèle, Q
global Q = zeros(11)
Q[1:3] = caFront
Q[4:6] = caRear
Q[7:9] = wheelCarrier
Q[10] = travel
Q[11] = springRate
Z = -Q[10] *ones(1,n-1) + (2*Q[10])/(n-2) * (0:(n-2))'
F(x0)

using NLPModels

# Constraint (rockerAxis - chassis) orthogonal to the rocker plane
# constraints = x-> [(x[4:6]-x[7:9])'*(x[4:6]-x[1:3]);(x[4:6]-x[7:9])'*(x[4:6]-x[10:12])]
r = 50 # Eloignement maximal du point initial
nlp = ADNLPModel(x->F(x), x0, lvar=x0-r, uvar=x0+r)

using Ipopt
# Setting Ipopt Solver with time_limit
model = NLPtoMPB(nlp, IpoptSolver( limited_memory_update_type="bfgs",max_cpu_time=40.0))
# Solving problem :
MathProgBase.optimize!(model)
MathProgBase.status(model)
# Taking solution :
xfinal = MathProgBase.getsolution(model)

# Comparing with the initial solution :
F(xfinal)
wr0 = wheelrate(x0)
plot(Z',wr0)
plot(Z',0.2*Z'+28)
wrfinal = wheelrate(xfinal)
plot(Z',wrfinal)
