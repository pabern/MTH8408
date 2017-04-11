#= -------------------------------------------------------------------------------------------------
Fonction rotate3D(P, Q1, Q2, t)
Rotation d'un point autour d'un axe quelconque dans l'espace 3D
P =: Point sur lequel la rotation est faite
Q1 =: Point 1 de l'axe quelconque u
Q2 =: Point 2 de l'axe quelconque u
t =: Angle de rotation
--------------------------------------------------------------------------------------------------=#
function rotate3D(P, Q1, Q2, t)
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

  # Matrice de rotation en 3D
  R = [c+(ux2*c1) (uxy*c1)-(uz*s) (uxz*c1)+(uy*s);
  (uxy*c1)+(uz*s) c+(uy2*c1) (uyz*c1)-(ux*s);
  (uxz*c1)-(uy*s) (uyz*c1)+(ux*s) c+(uz2*c1)]

  point = P*R
  return point
end
