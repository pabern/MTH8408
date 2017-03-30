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
