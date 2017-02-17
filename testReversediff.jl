using ReverseDiff: GradientTape, GradientConfig, gradient, gradient!, compile_gradient

f(a, b) = a.^2+2*b
a = [5]
b = [6]
inputs = (a, b)
results = (similar(a), similar(b))
cfg = GradientConfig(inputs)
gradient!(results, f, inputs, cfg)
