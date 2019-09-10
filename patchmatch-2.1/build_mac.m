% Works after doing mex -setup and selecting gcc, with XCode 3.2.6 (gcc 4.2.1)

mex CXXOPTIMFLAGS='-O6 -w -s -ffast-math -fomit-frame-pointer -fstrength-reduce -fopenmp -msse2 -funroll-loops -fPIC' CXXFLAGS='${CXXFLAGS} -DNDEBUG -DUNIX_MODE -DMEXMODE -fopenmp' LDFLAGS='${LDFLAGS} -fopenmp -undefined dynamic_lookup -bundle' mexutil.cpp nn.cpp nnmex.cpp patch.cpp vecnn.cpp simnn.cpp allegro_emu.cpp knn.cpp -cxx -output nnmex
mex CXXOPTIMFLAGS='-O6 -w -s -ffast-math -fomit-frame-pointer -fstrength-reduce -fopenmp -msse2 -funroll-loops -fPIC' CXXFLAGS='${CXXFLAGS} -DNDEBUG -DUNIX_MODE -DMEXMODE -fopenmp' LDFLAGS='${LDFLAGS} -fopenmp -undefined dynamic_lookup -bundle' mexutil.cpp nn.cpp votemex.cpp patch.cpp vecnn.cpp simnn.cpp allegro_emu.cpp knn.cpp -cxx -output votemex
