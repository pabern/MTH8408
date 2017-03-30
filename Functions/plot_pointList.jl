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
