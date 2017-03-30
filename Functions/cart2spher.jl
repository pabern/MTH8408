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
  return [r theta phi]
end
