# CHANGEME: Here is the name of all object files corresponding to the source
#           code that you wrote in order to define the problem statement
OBJS = main.o 

##########################################################################
#  Usually, you don't have to change anything below.  Note that if you   #
#  change certain compiler options, you might have to recompile Ipopt.   #
##########################################################################

# C++ Compiler options
CXXFLAGS = -O03 -DNDEBUG 


prefix=/usr/local
exec_prefix=${prefix}

# Include directories
INCL = `PKG_CONFIG_PATH=/usr/local/lib/pkgconfig: pkg-config --cflags ipopt`
#INCL = -I${prefix}/include/coin-or -I/usr/local/include/coin-or/hsl   -DIPOPTLIB_BUILD $(ADDINCFLAGS)

# Linker flags
LIBS = `PKG_CONFIG_PATH=/usr/local/lib/pkgconfig: pkg-config --libs ipopt`
#LIBS = -L${exec_prefix}/lib -lipopt -L/usr/local/lib -lcoinhsl  -framework Accelerate  -ldl


# VLAD Created Rules
all: formulas.hpp collocation
	./collocation

formulas.hpp: equations
	python3 Mathematica/generate_header.py 

equations:
	wolframscript -script Mathematica/create_constraints.wls
	wolframscript -script Mathematica/create_cost_function.wls
	python3 Mathematica/create_other_constraints.py

clean:
	rm -rf collocation $(OBJS) ipopt.out formulas.hpp
	rm -f Mathematica/*.txt

# Default rules
.SUFFIXES: .cpp .o

collocation: $(OBJS)
	g++ $(CXXFLAGS) -o $@ $(OBJS) $(ADDLIBS) $(LIBS)

.cpp.o: 
	g++ $(CXXFLAGS) $(INCL) -c -o $@ $<
