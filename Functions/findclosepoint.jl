function findclosepoint(P1, P2, P3)
  v = unitvector(P1, P2)
  P4 = P2 + (sum((P3-P2).*v))*v
  return P4
end
