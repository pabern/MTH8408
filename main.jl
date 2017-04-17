#= -----------------------------------------------------------------------------
MTH8408 - Project d'optimisation
Optimisation du wheel rate d'une formule SAE
Auteurs : Patrick Bernier, Hugo Chareyre
------------------------------------------------------------------------------=#
include("./ini.jl")
#using PyPlot
using NLPModels
using Ipopt
using ExcelReaders

# Il faut que le fichier soit à la même place que main.jl
cd(dirname(Base.source_path()))
# On lit ce qui est nécessaire
data = readxl("Input.xlsx", "Sheet1!A1:D11")

caFront       = [data[2,2] data[2,3] data[2,4]] #1 - Immobile
caRear        = [data[3,2] data[3,3] data[3,4]] #2 - Immobile
wheelCarrier  = [data[4,2] data[4,3] data[4,4]] #3 - Immobile
push          = [data[5,2] data[5,3] data[5,4]] #4 - Variable
chassis       = [data[6,2] data[6,3] data[6,4]] #5 - Variable
shockA        = [data[7,2] data[7,3] data[7,4]] #6 - Variable
shockB        = [data[8,2] data[8,3] data[8,4]] #7 - Variable

# Récupération des paramètres de conception
travel = data[10,2]                   #8 - Immobile
springRate = data[11,2]              #9 - Immobile


# Nombre de points de la discrétisation
global n = 25
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
h(x0)
r = 20 # Eloignement maximal du point initial
nlp = ADNLPModel(x->F(x), x0, lvar=x0-r, uvar=x0+r)
nlp = ADNLPModel(x->F(x), x0, lvar=x0-r, uvar=x0+r,
                  c=x->h(x), lcon=zeros(n))

# Initialisation de Ipopt avec approximation L-BFGS et limite de temps et d'itérations
model = NLPtoMPB(nlp, IpoptSolver(hessian_approximation = "limited-memory",
                                  limited_memory_update_type="bfgs",
                                     max_cpu_time=500.0,max_iter=500))

# Résolution
MathProgBase.optimize!(model)
MathProgBase.status(model)
# Récupération de la solution
xfinal = MathProgBase.getsolution(model)


# Comparing with the initial solution :
F(xfinal)
wr0 = wheelrate(x0)
plot(Z',wr0)
plot(Z',0.2*Z'+28,"g-",linewidth=3)
wrfinal = wheelrate(xfinal)
plot(Z',wrfinal,"r-",linewidth=3)

# ------------------- Analyse Stochastique -----------------------
srand(1234)
K = 1000
σ = 1
WRp = zeros(K,n-1)
for i=1:K
  ϵ = σ * randn(12)
  xp = xfinal + ϵ
  WRp[i,:] = wheelrate(xp)'
  plot(Z',WRp[i,:],"b-", alpha=0.1)
end
legend(["\$0.2 Z+28\$","\$W(x)\$","\$W(x+ϵ^k)\$"],loc=0)
title("Comportement idéal, solution et solutions avec perturbations")

WRp_sorted = zeros(K,n-1)
for i=1:(n-1)
  WRp_sorted[:,i] = sort(WRp[:,i])
end


α = 0.05
WRsup = WRp_sorted[trunc(Int,(1-α/2)*K),:]
WRinf = WRp_sorted[trunc(Int,(α/2)*K),:]

plot(Z' ,0.2*Z'+28,"g-",linewidth=3)
plot(Z',wrfinal,"r-",linewidth=3)
plot(Z',WRsup,"b-")
plot(Z',WRinf,"b-")
legend(["\$0.2 Z+28\$","\$W(x)\$","\$W_{sup}(1-α)\$","\$W_{inf}(1-α)\$"],loc=0)
title("Intervalle de confiance empirique de niveau \$1-α=0.95\$")
