workspace()
using ForwardDiff

x = [4,5,6,6,8,9]
function ftest(x)
  z = 3*x.^2
  return z
end


out = zeros(size(x,1),size(x,1))
cfg = ForwardDiff.JacobianConfig(x)
ForwardDiff.jacobian!(out, ftest, x, cfg)
