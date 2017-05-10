# Wheel Rate Optimization for FSAE

Prerequesites on machine :
1. All the current repository in a folder.
2. Last release of julia. Ideally Atom with Juno.
3. Unix machine - Windows will bug with Ipopt.
Prerequesites in Julia : 
1. PyPlot : Pkg.add("PyPlot"). Run ENV["PYTHON"] = ""; Pkg.build("PyPlot") if ERROR.
2. NLPModels : Pkg.add("NLPModels") & Pkg.add("ForwardDiff")
3. Ipopt : Pkg.add("Ipopt")
4. ExcelReaders : Pkg.add("ExcelReaders"). Run ENV["PYTHON"] = ""; Pkg.build("PyCall") if ERROR.

Both PyPlot and ExcelReaders might take many tries to work. Same for the ADNLPModel line in code. Once it works, there won't be problems for other tries.

Instructions :
1. Change initial points in Excel.
2. Run in atom or through Julia console.

Image files won't appear on screen. They will be located in the same folder as the main.jl.
