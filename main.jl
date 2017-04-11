#= -----------------------------------------------------------------------------
MTH8408 - Project d'optimisation
Optimisation du wheel rate d'une formule SAE
Auteurs : Patrick Bernier, Hugo Chareyre
------------------------------------------------------------------------------=#
include("./ini.jl")
#using PyPlot
using NLPModels
using Ipopt

# Récupération des points de suspension
caFront       = [2260 220 290] #1 - Immobile
caRear        = [1890 250 290] #2 - Immobile
wheelCarrier  = [2110 520 320] #3 - Immobile
push          = [2110 245 600] #4 - Variable
chassis       = [2110 200 535] #5 - Variable
shockA        = [2110 200 595] #6 - Variable
shockB        = [2110  30 575] #7 - Variable

# Récupération des paramètres de conception
travel = 51                    #8 - Immobile
springRate = 61.3              #9 - Immobile


# Nombre de points de la discrétisation
global n = 100
# Variables de départs, x0
x0 = zeros(12)
x0[1:3] = push
x0[4:6] = chassis
x0[7:9] = shockA
x0[10:12] = shockB
# Paramètres du modèle, Q
global Q = zeros(11)
Q[1:3] = caFront
Q[4:6] = caRear
Q[7:9] = wheelCarrier
Q[10] = travel
Q[11] = springRate
# Débattement en Z incrémental selon travel, le déplacement maximal en Z
global Z = -Q[10] *ones(1,n-1) + (2*Q[10])/(n-2) * (0:(n-2))'

# ------------------- Début de la section d'optimisation -----------------------

r = 50 # Eloignement maximal du point initial
nlp = ADNLPModel(x->F(x), x0, lvar=x0-r, uvar=x0+r)


# Initialisation de Ipopt avec approximation L-BFGS et limite de temps et d'itérations
model = NLPtoMPB(nlp, IpoptSolver(hessian_approximation = "limited-memory", limited_memory_update_type="bfgs", max_cpu_time=200.0,max_iter=15))

# Résolution
MathProgBase.optimize!(model)
MathProgBase.status(model)
# Récupération de la solution
xfinal = MathProgBase.getsolution(model)

#=
# Comparing with the initial solution :
F(xfinal)
wr0 = wheelrate(x0)
plot(Z',wr0)
plot(Z',0.2*Z'+28)
wrfinal = wheelrate(xfinal)
plot(Z',wrfinal)
=#
