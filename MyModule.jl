module MyModule
export findclosepoint
export unitvector
export translate3d
export cart2spher
export rotate3d

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
  T = repmat(T,size(P,1))
  point = P-T
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

function rotate3d(P, Q1, Q2, t)
  u = (Q2 - Q1)/norm(Q2 - Q1)

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
  for i = 1:1:length(point)
    if abs(point[i]) < 1e-10
      point[i] = 0
    end
  end
  return point
end

end # End MyModule
