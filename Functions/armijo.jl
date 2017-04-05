function armijo(xk, dk, f, g):
    fk = f(xk)
    gk = g(xk)
    slope = dot(gk, dk)  # Doit être < 0
    t = 1.0
    while f(xk + t * dk) > fk + 1.0e-4 * t * slope:
        t /= 1.5
    return t
end
