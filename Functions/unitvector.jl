function unitvector(P1, P2)
  v = (P2-P1)./norm(P2-P1)
  return v
end
