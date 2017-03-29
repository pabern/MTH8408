module MyModule
export findclosepoint
export unitvector
export translate3d
export cart2spher
export rotate3d
export rotate3d_tlist
export cart2spher_list
export plot_pointList
export transGeo
export fwheelRate
export F

function unitvector(P1, P2)
  v = (P2-P1)./norm(P2-P1)
  return v
end

function findclosepoint(P1, P2, P3)
  v = unitvector(P1, P2)
  P4 = P2 + (sum((P3-P2).*v))*v
  return P4
end

function translate3d(P,T)
  # P : Liste de n points : size(P) = (n,3)
  n = size(P,1)
  point = P - ones(n)*T'
  return point
end

function cart2spher(P)

  x = P[1]
  y = P[2]
  z = P[3]

  r = sqrt((x^2) + (y^2) + (z^2))

  if x == 0
    theta = 0
  else
    theta = atan(y/x)
  end

  if r == 0
    phi = 0
  else
    phi = acos(z/r)
  end

  M = [r theta phi]
  return M
end


function cart2spher_list(P)
  n = length(P[:,1])
  x = P[:,1]
  y = P[:,2]
  z = P[:,3]
  r = sqrt(diag(P*P'))
  # si x proche de zéros, on le décale un peu pour éviter une erreur
  # a tester sans cette ligne :
  #xprime = x + 1e-5*ones(n,1)
  #theta = atan(y./xprime)
  theta = atan(y./x)
  # De même ici, à tester.
  #rprime = r + 1e-5*ones(n,1)
  #phi = acos(z./rprime)
  phi = acos(z./r)
  M = [r theta phi]
  return M
end


function rotate3d(P, Q1, Q2, t)
  u = unitvector(Q1, Q2)
  c = cos(t)
  s = sin(t)
  c1 = 1-c

  ux = u[1]
  uy = u[2]
  uz = u[3]
  ux2 = ux^2
  uy2 = uy^2
  uz2 = uz^2
  uxy = ux*uy
  uxz = ux*uz
  uyz = uy*uz

  R = [c+(ux2*c1) (uxy*c1)-(uz*s) (uxz*c1)+(uy*s);
  (uxy*c1)+(uz*s) c+(uy2*c1) (uyz*c1)-(ux*s);
  (uxz*c1)-(uy*s) (uyz*c1)+(ux*s) c+(uz2*c1)]

  point = P*R
  return point
end

function rotate3d_tlist(P, Q1, Q2, t)
  n = length(t)
  u = unitvector(Q1, Q2)
  c = cos(t)'
  s = sin(t)'
  c1 = (1-c)

  ux = u[1]
  uy = u[2]
  uz = u[3]
  ux2 = ux^2
  uy2 = uy^2
  uz2 = uz^2
  uxy = ux*uy
  uxz = ux*uz
  uyz = uy*uz
  reduceMat = vcat(vcat(eye(n),eye(n)),eye(n))
  R = [c+(ux2*c1) (uxy*c1)-(uz*s) (uxz*c1)+(uy*s);
  (uxy*c1)+(uz*s) c+(uy2*c1) (uyz*c1)-(ux*s);
  (uxz*c1)-(uy*s) (uyz*c1)+(ux*s) c+(uz2*c1)]
  k = reshape(diag(reduceMat*((ones(n)*P)*R)),(n,3))
  return k
end

function plot_pointList(pointList)
  n = size(pointList,1)
  xx = reshape(pointList[:,[1]],n)
  yy = reshape(pointList[:,[2]],n)
  zz = reshape(pointList[:,[3]],n)
  figure()
  plot3D(xx,yy,zz)
  scatter3D(xx,yy,zz)
  xlabel("X")
  ylabel("Y")
  zlabel("Z")
end

function transGeo(x,Q,n)
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

  return (pointWC, pointList)
end

function fwheelRate(pointWC, pointList, springRate)
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

  if any(d->d < 0 ,delta)
    check = true
    return (0,check)
  else
    check = false
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
    return (wheelRate,check)
  end
end

function F(x,Q,n)
  (pointWC, pointList) = transGeo(x,Q,n)
  (wheelRate,check) = fwheelRate(pointWC, pointList, Q[11])
  Z = -Q[10] *ones(1,n-1) + (2*Q[10])/(n-2) * (0:(n-2))'
  fonctionObjectif = wheelRate - (0.2 * Z' + 28)
  return fonctionObjectif
end

end # End MyModule
