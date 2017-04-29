#= -----------------------------------------------------------------------------
MTH8408 - Project d'optimisation
Optimisation du wheel rate d'une formule SAE
Auteurs : Patrick Bernier, Hugo Chareyre
------------------------------------------------------------------------------=#
using PyPlot
include("./ini.jl")
using NLPModels
using KNITRO
using ExcelReaders

print("...Loading Data\n")
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
print("...Optimization module initialization\n")
# Options KNITRO:
# https://www.artelys.com/tools/knitro_doc/3_referenceManual/userOptions.html
# Différentes options pour l'approximations des hessiens :
HessianApprox = KTR_HESSOPT_EXACT
# KTR_HESSOPT_EXACT : Hessien exact
# KTR_HESSOPT_BFGS : BFGS complet
# KTR_HESSOPT_SR1  : SR1 (Symetric Rank 1)
# KTR_HESSOPT_LBFGS
# La résolution par L-BFGS s'avère être plus efficace en temp de calcul.
# La résolution par hessien exact est plus longue, certainement à cause de
# l'évaluation du hessien par ForwardDiff.

r = 30 # Eloignement maximal du point initial
lc = zeros(n+1)
lc[(n+1):end] = 177^2
uc = ones(n+1)
uc[1:n]=KTR_INFBOUND
uc[(n+1)] = 267^2
L(x0)
r = [5.0, 30, 30, 5, 30, 30, 5, 30, 30, 5, 30, 30]
nlp = ADNLPModel(F, x0, lvar = x0-r, uvar= x0+r, c=x->append!(h(x),L1(x)), lcon=lc,ucon=uc)
#nlp = ADNLPModel(F, x0, lvar = x0-r, uvar= x0+r, c=x->h(x), lcon=zeros(n))


solver = KnitroSolver(KTR_PARAM_HONORBNDS=KTR_HONORBNDS_ALWAYS,
                      KTR_PARAM_BAR_FEASIBLE=KTR_BAR_FEASIBLE_STAY,
                      KTR_PARAM_BAR_FEASMODETOL=0.0,
                      KTR_PARAM_HESSOPT= HessianApprox,
                      KTR_PARAM_MAXTIMECPU=300.0,
                      KTR_PARAM_FTOL=1e-2)

model = NLPtoMPB(nlp, solver)

# Résolution
print("...Begining of optimization\n")
MathProgBase.optimize!(model)
MathProgBase.status(model)
# Récupération de la solution
xfinal = MathProgBase.getsolution(model)

# Comparaison avec la solution initial et idéale
plot_solution(xfinal,x0, 0.05,1,20)

# ------------------- Optimisation Stochastique -----------------------
print("...Begining of robust optimization\n")
# Ici le modèle comprend dans sa valeur objectif une part de solutions
# perturbées pour que la solution soit plus robuste
srand(1234)
global K = 15
σ = 1
global epsilon = σ*randn(K,12)
global η = 1/K
function Fstoch(x)
  fonctionObjectif = 0
  for i=1:K
    xp = x + epsilon[i,:]
    fonctionObjectif += η* norm(wheelrate(xp)- (0.2 * Z' + 28))^2
  end
  wheelRate = wheelrate(x)
  fonctionObjectif += norm(wheelRate - (0.2 * Z' + 28))^2
  return fonctionObjectif
end

Fstoch(x0)
# On réoptimise ici à partir de la solution obtenue précédemment
r=10
nlpStoch = ADNLPModel(Fstoch, xfinal, lvar = xfinal-r, uvar= xfinal+r, c=x->h(x), lcon=zeros(n))
solver = KnitroSolver(KTR_PARAM_HONORBNDS=KTR_HONORBNDS_ALWAYS,
                      KTR_PARAM_BAR_FEASIBLE=KTR_BAR_FEASIBLE_STAY,
                      KTR_PARAM_BAR_FEASMODETOL=0.0,
                      KTR_PARAM_HESSOPT= HessianApprox,
                      KTR_PARAM_MAXTIMECPU=300.0,
                      KTR_PARAM_FTOL=1e-8)
modelStoch = NLPtoMPB(nlpStoch, solver)
# Résolution
MathProgBase.optimize!(modelStoch)
MathProgBase.status(modelStoch)
# Récupération de la solution
xfinalStoch = MathProgBase.getsolution(modelStoch)
# Comparaison avec la solution initial et idéale
plot_solution(xfinalStoch,x0, 0.05,1,2000,["xinitial.png","xfinalRobuste.png",
              "3dSolutionRobuste.png"])

# Affichage de la solution
@printf "_______Solution_______"
@printf "caFront \t %d  \t %d \t %d \n" Q[1] Q[2] Q[3]
@printf "caRear \t\t %d  \t %d \t %d \n" Q[4] Q[5] Q[6]
@printf "wheelCarrier \t %d  \t %d \t %d \n" Q[7] Q[8] Q[9]
@printf "push \t\t %d  \t %d \t %d \n" xfinalStoch[1] xfinalStoch[2] xfinalStoch[3]
@printf "chassis \t %d  \t %d \t %d \n" xfinalStoch[4] xfinalStoch[5] xfinalStoch[6]
@printf "shockA \t\t %d  \t %d \t %d \n" xfinalStoch[7] xfinalStoch[7] xfinalStoch[7]
@printf "shockB \t\t %d  \t %d \t %d \n\n" xfinalStoch[10] xfinalStoch[11] xfinalStoch[12]
@printf "travel \t\t %d \n" Q[10]
@printf "springRate \t %d \n\n" Q[11]
