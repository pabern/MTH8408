function plot_solution(xfinal,x0, α=0.05, σ=1, K=500,fnames=["xinitial.png",
                      "xfinal.png","3dsolution.png"])
  #=
  xfinal : solution finale à tracer
  x0 : solution initiale
  alpha : niveau de risque pour l'intervalle de confiance empirique
  sigma : variance de fabrication
  K : Nombre de trajectoires pour l'intervalle de confiance
  fnames : Array : chemines des fichiers pour la sauvegarde des figues
  =#
  wr0 = wheelrate(x0)
  wrfinal = wheelrate(xfinal)
  WRp = zeros(K,n-1)
  for i=1:K
    ϵ = σ * randn(12)
    xp = xfinal + ϵ
    WRp[i,:] = wheelrate(xp)'
  end
  WRp_sorted = zeros(K,n-1)
  for i=1:(n-1)
    WRp_sorted[:,i] = sort(WRp[:,i])
  end
  WRsup = WRp_sorted[trunc(Int,(1-α/2)*K)+1,:]
  WRinf = WRp_sorted[trunc(Int,(α/2)*K)+1,:]

  # plots :
  fig1 = figure()
  plot(Z',0.2*Z'+28)
  plot(Z',wr0)
  legend(["\$0.2 Z+28\$","\$W(x_0)\$"],loc=0)
  title("Solution initiale et idéale")
  xlabel("Débattement de la roue (mm)")
  ylabel("Wheel Rate (N/mm)")
  savefig(fnames[1])
  fig2 = figure()
  plot(Z',0.2*Z'+28,"r-")
  plot(Z',wrfinal,"g-")
  plot(Z',WRsup,"b-")
  plot(Z',WRinf,"b-")
  legend(["\$0.2 Z+28\$","\$W(x^*)\$","\$IC(x^*;1-α)\$"],loc=0)
  title("Intervalle de confiance empirique de niveau \$1-α=0.95\$")
  xlabel("Débattement de la roue (mm)")
  ylabel("Wheel Rate (N/mm)")
  savefig(fnames[2])

  fig3 = figure()
  scatter3D(Q[1:3:7],Q[2:3:8],Q[3:3:9])
  plot3D([Q[1],Q[7],Q[4]],[Q[2],Q[8],Q[5]],[Q[3],Q[9],Q[6]],c="b")
  scatter3D(x0[1:3:10],x0[2:3:11],x0[3:3:12],c="r")
  scatter3D(xfinal[1:3:10],xfinal[2:3:11],xfinal[3:3:12],c="g")
  plot3D(xfinal[1:3:10],xfinal[2:3:11],xfinal[3:3:12],c="g")
  plot3D([xfinal[1],xfinal[7]],[xfinal[2],xfinal[8]],[xfinal[3],xfinal[9]],c="g")
  plot3D([xfinal[1],Q[7]],[xfinal[2],Q[8]],[xfinal[3],Q[9]],c="g")
  xlabel("X")
  ylabel("Y")
  zlabel("Z")
  savefig(fnames[3])
end
