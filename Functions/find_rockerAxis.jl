function find_rockerAxis(chassis,push,shockA)
  # Renvoie un vecteur normé orthogonal au plan formé par les vecteurs
  # (push-chassis) et (shockA-chassis)
  a = shockA[3]-chassis[3]
  b = push[3] - chassis[3]
  c = shockA[2] - chassis[2]
  d = push[2] - chassis[2]
  A = [a -b ; -c d] / (d*a-c*b)
  r = [1;A*[-push[1]+chassis[1];-shockA[1]+chassis[1]]]
  r = chassis + r'/norm(r)
  # check:
  # (r-chassis)*(push-chassis)'==0
  # (r-chassis)*(shockA-chassis)'==0
  return r
end
