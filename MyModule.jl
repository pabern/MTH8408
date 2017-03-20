module MyModule
export findclosepoint
export unitvector
export translate3d
export cart2spher
export rotate3d
export rotate3d_tlist
export cart2spher_list

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
  # P : Liste de n point : size(P) = (n,3)
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


end # End MyModule
