function wheelrate(x)
  # Rappel que x sont variables et Q et n sont des paramètres
  caFront       = Q[1:3]' #1
  caRear        = Q[4:6]' #2
  wheelCarrier  = Q[7:9]' #3
  push          = x[1:3]' #4
  chassis       = x[4:6]' #5
  rockerAxis    = x[7:9]' #6
  shockA        = x[10:12]' #7
  shockB        = x[13:15]'  #8
  travel = Q[10]                 #9
  springRate = Q[11]             #10

  # Identify point of rotation of wheelCarrier around caAxis
  WCRot = findclosepoint(caFront, caRear, wheelCarrier)

  # Create the list to manipulate
  pointList = [caFront;     # 1
  caRear;       # 2
  WCRot;        # 3
  wheelCarrier; # 4
  push;         # 5
  chassis;      # 6
  rockerAxis;   # 7
  shockA;       # 8
  shockB]       # 9
  pointList[7,:] = find_rockerAxis(chassis,push,shockA)
  #  plot_pointList(pointList)

  # Translation of Origin (0 0 0) to chassis
  pointList = translate3d(pointList,pointList[6,:])
  #  plot_pointList(pointList)

  # Z axis rotation to eliminate θ in spherical coordinate
  t = cart2spher(pointList[7,:])
  angleRot = (π/2) - t[2]
  pointList = rotate3d(pointList,[0 0 0], [0 0 1], angleRot)
  #  plot_pointList(pointList)

  # Y axis rotation to only leave ϕ as a variable in spherical coordinate
  t = cart2spher(pointList[7,:])
  angleRot = (π/2) - t[3]
  pointList = rotate3d(pointList,[0 0 0], [0 1 0], angleRot)
  #  plot_pointList(pointList)

  # Calculate the points of the wheel carrier after its rotation according to the desired travel
  # Translation to the origin in order for the rotation to function correctly
  pointListWC = translate3d(pointList,pointList[3,:])

  rotationTravel = atan(travel/(norm(WCRot-wheelCarrier)))
  rotationTravel = -rotationTravel *ones(1,n) + (2*rotationTravel)/n * (0:(n-1))'

  pointWCtemp = rotate3d_tlist(pointListWC[4:4,:], pointListWC[1:1,:], pointListWC[2:2,:], rotationTravel')
  pointWC = translate3d(pointWCtemp, -pointList[3:3,:]')

  # Point re-establishment
  caFront       = pointList[1:1,:]
  caRear        = pointList[2:2,:]
  WCRot         = pointList[3:3,:]
  wheelCarrier  = pointList[4:4,:]
  push          = pointList[5:5,:]
  chassis       = pointList[6:6,:]
  rockerAxis    = pointList[7:7,:]
  shockA        = pointList[8:8,:]
  shockB        = pointList[9:9,:]

  springRate = Q[11]

  # Sherical coordinates of wheel carrier positions
  sphericWC = cart2spher_list(pointWC)

  rWC = sphericWC[:,1]
  θWC = sphericWC[:,2]
  ϕWC = sphericWC[:,3]
  # Four bar linkage length of members
  a = [norm(WCRot-wheelCarrier) norm(push-wheelCarrier) norm(push-chassis) norm(chassis-WCRot)]

  # Four bar linkage equation
  A = sin(θWC).*sin(ϕWC)
  B = cos(ϕWC)
  C = (rWC.^2 + a[3]^2 - a[2]^2)./(2*rWC*a[3])
  delta = A.^2 + B.^2 - C.^2

  ϕPush = 2*atan((A-sqrt(delta))./(B+C))

  t = cart2spher(push)
  ϕIni = t[3]

  # Rocker position after rotation
  pushIni = rotate3d(push,chassis,[0 1 0],ϕIni)
  shockAIni = rotate3d(shockA,chassis,[0 1 0],ϕIni)

  shockA = rotate3d_tlist(shockAIni,chassis,[0 -1 0],ϕPush)

  # Lenght of shock
  L = sqrt((shockA[:,1] - shockB[1]).^2 + (shockA[:,2] - shockB[2]).^2 + (shockA[:,3] - shockB[3]).^2)

  # Wheel travel
  w1 = pointWC[1:end-1,3]
  w2 = pointWC[2:end,3]
  wheelTravel = abs(w2-w1)

  # Spring travel
  v1 = L[1:end-1]
  v2 = L[2:end]
  springTravel = abs(v2-v1)

  motionRatio = wheelTravel./springTravel
  wheelRate = springRate./(motionRatio.^2)
  return wheelRate
end
