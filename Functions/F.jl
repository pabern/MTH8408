function F(x)
  (wheelRate,check) = wheelrate(x)
  Z = -Q[10] *ones(1,n-1) + (2*Q[10])/(n-2) * (0:(n-2))'
  fonctionObjectif = wheelRate - (0.2 * Z' + 28)
  return fonctionObjectif
end
