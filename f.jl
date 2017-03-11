function f1(x,n)
  caFront       = x[1:3]' #1
  caRear        = x[4:6]' #2
  wheelCarrier  = x[7:9]' #3
  push          = x[10:12]' #4
  chassis       = x[13:15]' #5
  rockerAxis    = x[16:18]' #6
  shockA        = x[19:21]' #7
  shockB        = x[22:24]'  #8
  travel = x[25]                 #9
  springRate = x[26]             #10

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

  # Translation of Origin (0 0 0) to chassis
  pointList = translate3d(pointList,pointList[6,:])

  # Z axis rotation to eliminate θ in spherical coordinate
  t = cart2spher(pointList[7,:])
  angleRot = (π/2) - t[2]
  pointList = rotate3d(pointList,[0 0 0], [0 0 1], angleRot)
  # Y axis rotation to only leave ϕ as a variable in spherical coordinate
  t = cart2spher(pointList[7,:])
  angleRot = (π/2) - t[3]
  pointList = rotate3d(pointList,[0 0 0], [0 1 0], angleRot)

  # Calculate the points of the wheel carrier after its rotation according to the desired travel
  # Translation to the origin in order for the rotation to function correctly
  pointListWC = translate3d(pointList,pointList[3,:])

  rotationTravel = atan(travel/(norm(WCRot-wheelCarrier)))
  #rotationTravel = linspace(-rotationTravel,rotationTravel,200)
  rotationTravel = -rotationTravel *ones(1,n) + (2*rotationTravel)/n * (0:(n-1))'

  pointWC = zeros(n,3)
  for i = 1:1:length(rotationTravel)
    pointWC[i,:] = rotate3d(pointListWC[4:4,:], pointListWC[1:1,:], pointListWC[2:2,:], rotationTravel[i])
    pointWC[i,:] = translate3d(pointWC[i:i,:], -pointList[3:3,:]')
  end


  return (pointWC, pointList)
end

function f2(pointWC, pointList)
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
  # Sherical coordinates of wheel carrier positions
  rWC = zeros(200,1)
  θWC = zeros(200,1)
  ϕWC = zeros(200,1)
  for i = 1:1:size(pointWC,1)
    t = cart2spher(pointWC[i:i,:])
    rWC[i] = t[1]
    θWC[i] = t[2]
    ϕWC[i] = t[3]
  end

  # Four bar linkage length of members
  a = zeros(1,4)
  a[1] = norm(WCRot-wheelCarrier)
  a[2] = norm(push-wheelCarrier)
  a[3] = norm(push-chassis)
  a[4] = norm(chassis-WCRot)

  # Four bar linkage equation
  A = cos(θWC).*sin(ϕWC)
  B = cos(ϕWC)
  C = (rWC./(2*a[3])) + (((a[3].^2)-(a[2].^2))./(2*rWC.*a[3]))
  delta = A.^2 + B.^2 - C.^2

  ϕPush = 2*atan((A-sqrt(delta))./(B+C))

  t = cart2spher(push)
  ϕIni = t[3]

  # Rocker position after rotation
  pushIni = rotate3d(push,chassis,[0 1 0],ϕIni)
  shockAIni = rotate3d(shockA,chassis,[0 1 0],ϕIni)

  shockA = zeros(200,3)
  for i = 1:1:size(ϕPush,1)
      shockA[i,:] = rotate3d(shockAIni,chassis,[0 -1 0],ϕPush[i])
  end

  # Lenght of shock
  L = zeros(n,1)
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

  # Wheel rate plot
  x = w1-wheelCarrier[3]
  # plot(x,wheelRate)
  return (x, wheelRate)
end

function f(x)
  n = 200
  (pointWC, pointList) = f1(x,n)
  (z, wheelRate) = f2(pointWC, pointList)
  return (wheelRate)
end
