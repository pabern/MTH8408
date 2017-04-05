function F(x)
  wheelRate = wheelrate(x)
  Z = -Q[10] *ones(1,n-1) + (2*Q[10])/(n-2) * (0:(n-2))'
  fonctionObjectif = norm(wheelRate - (0.2 * Z' + 28))^2
  return fonctionObjectif
end
