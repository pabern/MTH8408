function grad_F(x)
  fObj = F(x)
  cfg = ForwardDiff.JacobianConfig(x)
  m = size(x,1)
  Jf = zeros(n-1,m)
  ForwardDiff.jacobian!(Jf, F, x, cfg)
  return Jf'*fObj
end
