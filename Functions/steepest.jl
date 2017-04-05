function steepest(f, g, x0,er = 1.0e-6,ea = 0, maxit = 20):
    xk = x0
    n = size(x0)
    fk = f(xk)
    gk = g(xk)
    gNorm = norm(gk)
    gNorm0 = gNorm
    k = 0
    print("it:$(k)  obj:$(fk)  ||∇||:$(gNorm)")
    while gNorm > (ea + er * gNorm0) and k < maxit:
        dk = -gk
        t = armijo(xk, dk, f, g)
        xk += t * dk
        fk = f(xk)
        gk = g(xk)
        gNorm = norm(gk)
        k += 1
        print("it:$(k)  obj:$(fk)  ||∇||:$(gNorm) t:$(t)")
    return xk
end
