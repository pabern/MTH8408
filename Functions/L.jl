function L(x)
  # Calcul la longueur entre les deux Shock (A et B) pour tous les points de la
  # discrétisation
  # Rappel que x sont variables et Q et n sont des paramètres
  caFront       = Q[1:3]' #1
  caRear        = Q[4:6]' #2
  wheelCarrier  = Q[7:9]' #3
  push          = x[1:3]' #4
  chassis       = x[4:6]' #5
  shockA        = x[7:9]' #6
  shockB        = x[10:12]'#7
  travel = Q[10]                 #8
  springRate = Q[11]             #9

  # Identification du point de rotation de wheelCarrier autour de caAxis
  WCRot = findclosepoint(caFront, caRear, wheelCarrier)

  # Céation d'une liste de points à manipuler
  pointList = [caFront;     # 1
              caRear;       # 2
              WCRot;        # 3
              wheelCarrier; # 4
              push;         # 5
              chassis;      # 6
              find_rockerAxis(chassis,push,shockA); # 7
              shockA;       # 8
              shockB]       # 9

  # Translation de l'origine (0 0 0) au point chassis
  pointList = translate3D(pointList,pointList[6,:])

  # Rotation autour de l'axe des Z pour amener éliminer un degré de liberté
  t = cart2spher(pointList[7,:])
  angleRot = (π/2) - t[2]
  pointList = rotate3D(pointList,[0 0 0], [0 0 1], angleRot)

  # Rotation autour de l'axe des Y afin qu'il ne reste qu'un degré de liberté, soit la rotation en X
  t = cart2spher(pointList[7,:])
  angleRot = (π/2) - t[3]
  pointList = rotate3D(pointList,[0 0 0], [0 1 0], angleRot)

  # Calcul des points du mouvement incémental de wheelCarrier dans l'espace
  pointListWC = translate3D(pointList,pointList[3,:])

  rotationTravel = atan(travel/(norm(WCRot-wheelCarrier)))
  rotationTravel = -rotationTravel *ones(1,n) + (2*rotationTravel)/n * (0:(n-1))'

  pointWCtemp = rotate3D_tlist(pointListWC[4:4,:], pointListWC[1:1,:], pointListWC[2:2,:], rotationTravel')
  pointWC = translate3D(pointWCtemp, -pointList[3:3,:]')
  #display(pointWC)
  # Séparation des points de travail
  caFront       = pointList[1:1,:]
  caRear        = pointList[2:2,:]
  WCRot         = pointList[3:3,:]
  wheelCarrier  = pointList[4:4,:]
  push          = pointList[5:5,:]
  chassis       = pointList[6:6,:]
  rockerAxis    = pointList[7:7,:]
  shockA        = pointList[8:8,:]
  shockB        = pointList[9:9,:]

  # Transfert des points de wheelCarrier en sphérique pour le calcul selon Hartenbeg
  sphericWC = cart2spher_list(pointWC)

  rWC = sphericWC[:,1]
  θWC = sphericWC[:,2]
  ϕWC = sphericWC[:,3]

  # Longueur des 4 membrures de l'équation
  a = [norm(WCRot-wheelCarrier) norm(push-wheelCarrier) norm(push-chassis) norm(chassis-WCRot)]

  # Résolution de l'équation à 4 membrures selon une modification de la méthode algébrique d'Hartenberg
  A = sin(θWC).*sin(ϕWC)
  B = cos(ϕWC)
  C = (rWC.^2 + a[3]^2 - a[2]^2)./(2*rWC*a[3])
  delta = A.^2 + B.^2 - C.^2

  ϕPush = 2*atan((A-sqrt(delta))./(B+C))

  # Calcul de la position du rocker suite à la rotation
  t = cart2spher(push)
  ϕIni = t[3]
  pushIni = rotate3D(push,chassis,[0 1 0],ϕIni)
  shockAIni = rotate3D(shockA,chassis,[0 1 0],ϕIni)

  shockA = rotate3D_tlist(shockAIni,chassis,[0 -1 0],ϕPush)

  # Calcul de la longueur du shock
  L = ((shockA[:,1] - shockB[1]).^2 + (shockA[:,2] - shockB[2]).^2 + (shockA[:,3] - shockB[3]).^2)
  return L
end
