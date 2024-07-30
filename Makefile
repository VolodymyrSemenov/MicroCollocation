# CHANGEME: Here is the name of all object files corresponding to the source code
OBJS = hessian.o jacobian.o optimization.o main.o 

# C++ Compiler options
CXXFLAGS = -O03 -DNDEBUG -std=c++17

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
	g++ $(CXXFLAGS) -o $@ $(OBJS) $(LIBS)

formulas.hpp: Mathematica/h0_1.txt Mathematica/generate_header.py 
	python3 Mathematica/generate_header.py 

Mathematica/h0_1.txt: Mathematica/create_constraints.wls Mathematica/create_cost_function.wls Mathematica/create_other_constraints.py
	wolframscript -script Mathematica/create_constraints.wls
	wolframscript -script Mathematica/create_cost_function.wls
	python3 Mathematica/create_other_constraints.py

main.o: main.cpp optimization.hpp 
	g++ $(CXXFLAGS) $(INCL) -c -o $@ $<

# Default rules
.SUFFIXES: .cpp .o

.cpp.o: $< globals.hpp formulas.hpp
	g++ $(CXXFLAGS) $(INCL) -c -o $@ $<
	
clean:
	rm -rf collocation $(OBJS) ipopt.out formulas.hpp
	rm -f Mathematica/*.txt

