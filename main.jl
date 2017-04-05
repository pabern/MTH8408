include("./ini.jl")


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

F(x)

using NLPModels

# Constraint (rockerAxis - chassis) orthogonal to the rocker plane
constraints = x-> [(x[4:6]-x[7:9])'*(x[4:6]-x[1:3]);(x[4:6]-x[7:9])'*(x[4:6]-x[10:12])]
r = 1 # Eloignement maximal du point initial
nlp = ADNLPModel(x->F(x), x0, lvar=x0-r, uvar=x0+r,
                 c=constraints, lcon=[0.0;0.0], ucon=[0.0;0.0])



using Ipopt
model = NLPtoMPB(nlp, IpoptSolver( limited_memory_update_type="bfgs"))
MathProgBase.optimize!(model)
