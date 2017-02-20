module transformation
export f1

function f1(caFront, caRear, WCRot, wheelCarrier, push, chassis, rockerAxis, shockA, shockB,travel)
  
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
rotationTravel = linspace(-rotationTravel,rotationTravel,200)

pointWC = zeros(200,3)
for i = 1:1:length(rotationTravel)
  pointWC[i,:] = rotate3d(pointListWC[4:4,:], pointListWC[1:1,:], pointListWC[2:2,:], rotationTravel[i])
  pointWC[i,:] = translate3d(pointWC[i:i,:], -pointList[3:3,:])
end
  
  P = [pointList;
        pointWC]
  
  return P
  
end # End of function

end # End of module


