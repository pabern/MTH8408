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
