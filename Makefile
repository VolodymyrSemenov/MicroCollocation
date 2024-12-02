# CHANGEME: Here is the name of all object files corresponding to the source code
OBJS = optimization.o main.o hessian.o jacobian.o

# C++ Compiler options
CXXFLAGS = -O03 -DNDEBUG -std=c++17

COMPILER = g++


prefix=/usr/local
exec_prefix=${prefix}

# Include directories
INCL = `PKG_CONFIG_PATH=/util/Ipopt/lib/pkgconfig: pkg-config --cflags ipopt`
#INCL = -I${prefix}/include/coin-or -I/usr/local/include/coin-or/hsl   -DIPOPTLIB_BUILD $(ADDINCFLAGS)

# Linker flags
export LD_LIBRARY_PATH=/util/Ipopt/lib
LIBS = `PKG_CONFIG_PATH=/util/Ipopt/lib/pkgconfig: pkg-config --libs ipopt -rpath=/util/Ipopt/lib`
#LIBS = -L${exec_prefix}/lib -lipopt -L/usr/local/lib -lcoinhsl  -framework Accelerate  -ldl


all: collocation
	./collocation

collocation: formulas.hpp $(OBJS) 
	$(COMPILER) $(CXXFLAGS) -o $@ $(OBJS) $(LIBS)

formulas.hpp: Mathematica/h0_1.txt Mathematica/generate_header.py 
	python3 Mathematica/generate_header.py 

Mathematica/h0_1.txt: Mathematica/create_constraints.wls Mathematica/create_cost_function.wls Mathematica/create_other_constraints.py
	wolframscript -script Mathematica/create_constraints.wls
	wolframscript -script Mathematica/create_cost_function.wls
	python3 Mathematica/create_other_constraints.py

main.o: main.cpp optimization.hpp 
	$(COMPILER) $(CXXFLAGS) $(INCL) -c -o $@ $<

# Default rules
.SUFFIXES: .cpp .o

.cpp.o: $< globals.hpp formulas.hpp
	$(COMPILER) $(CXXFLAGS) $(INCL) -c -o $@ $<
	
clean:
	rm -rf collocation $(OBJS) ipopt.out

