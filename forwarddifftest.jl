#= workspace()
using ForwardDiff

x = [4,5,6,6,8,9]
function ftest(x)
z = 3*x.^2
return z
end


out = zeros(size(x,1),size(x,1))
cfg = ForwardDiff.JacobianConfig(x)
ForwardDiff.jacobian!(out, ftest, x, cfg)
=#

workspace()
using ForwardDiff

caFront       = [2260 220 290] #1
caRear        = [1890 250 290] #2
wheelCarrier  = [2110 520 320] #3
push          = [2210 245 590] #4
chassis       = [2210 200 535] #5
rockerAxis    = [2215 200 535] #6
shockA        = [2110 200 595] #7
shockB        = [2210 30  575]  #8

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

function f(x) # pointList = f(x)

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

  v = (caRear-caFront)./norm(caRear-caFront)
  WCRot = caRear + (sum((wheelCarrier-caRear).*v))*v

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
  n = size(pointList,1)
  pointList = pointList - ones(n)*pointList[6,:]'


  x1 = pointList[7,1]
  y1 = pointList[7,2]

  if x1 == 0
    theta = 0
  else
    theta = atan(y1/x1)
  end

  angleRot = (π/2) - theta
  u = ([0 0 1]-[0 0 0])./norm([0 0 1]-[0 0 0])
  c = cos(angleRot)
  s = sin(angleRot)
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

  pointList = pointList*R

  x1 = pointList[7,1]
  y1 = pointList[7,2]
  z1 = pointList[7,2]

  r = sqrt((x1^2) + (y1^2) + (z1^2))

  if r == 0
    phi = 0
  else
    phi = acos(z1/r)
  end

  angleRot = (π/2) - phi
  u = ([0 1 0]-[0 0 0])./norm([0 1 0]-[0 0 0])
  c = cos(angleRot)
  s = sin(angleRot)
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

  pointList = pointList*R
  # Tout va bien jusqu'ici !

  n = size(pointList,1)
  pointListWC = pointList - ones(n)*pointList[3,:]'

  rotationTravel = atan(travel/(norm(WCRot-wheelCarrier)))
  rotationTravel = -rotationTravel *ones(1,3) + (2*rotationTravel)/3 * (0:(3-1))'

  pointWC = zeros(3,3)
    angleRot = rotationTravel[1]
    u = (pointListWC[2:2,:]-pointListWC[1:1,:])./norm(pointListWC[2:2,:]-pointListWC[1:1,:])
    c = cos(angleRot)
    s = sin(angleRot)
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

    pointWC[1,:] = pointListWC[4:4,:]*R

    pointWC[1,:] = pointWC[1,:] - -pointList[3,:]

    angleRot = rotationTravel[2]
    u = (pointListWC[2:2,:]-pointListWC[1:1,:])./norm(pointListWC[2:2,:]-pointListWC[1:1,:])
    c = cos(angleRot)
    s = sin(angleRot)
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

    pointWC[2,:] = pointListWC[4:4,:]*R

    pointWC[2,:] = pointWC[2,:] - -pointList[3,:]

    angleRot = rotationTravel[3]
    u = (pointListWC[2:2,:]-pointListWC[1:1,:])./norm(pointListWC[2:2,:]-pointListWC[1:1,:])
    c = cos(angleRot)
    s = sin(angleRot)
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

    pointWC[3,:] = pointListWC[4:4,:]*R

    pointWC[3,:] = pointWC[3,:] - -pointList[3,:]

  return (pointWC, pointList)

end


out = zeros(size(x,1)+1+200,size(x,1))
cfg = ForwardDiff.JacobianConfig(x)
ForwardDiff.jacobian!(out, f, x, cfg)
