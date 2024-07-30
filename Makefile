# CHANGEME: Here is the name of all object files corresponding to the source code
OBJS = optimization.o main.o jacobian.o hessian.o 

# C++ Compiler options
CXXFLAGS = -O03 -DNDEBUG -std=c++17

COMPILER = clang++

prefix=/usr/local
exec_prefix=${prefix}

# Include directories
INCL = `PKG_CONFIG_PATH=/usr/local/lib/pkgconfig: pkg-config --cflags ipopt`
#INCL = -I${prefix}/include/coin-or -I/usr/local/include/coin-or/hsl   -DIPOPTLIB_BUILD $(ADDINCFLAGS)

# Linker flags
LIBS = `PKG_CONFIG_PATH=/usr/local/lib/pkgconfig: pkg-config --libs ipopt`
#LIBS = -L${exec_prefix}/lib -lipopt -L/usr/local/lib -lcoinhsl  -framework Accelerate  -ldl


all: collocation
	./collocation

collocation: formulas.hpp $(OBJS) 
	$(COMPILER) $(CXXFLAGS) -o $@ $(OBJS) $(LIBS)

formulas.hpp: Mathematica/generate_header.py 
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

