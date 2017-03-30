function translate3d(P,T)
  n = size(P,1)
  point = P - ones(n)*T'
  return point
end
