
# ALG=KREYVIUM
ALG=TRIVIUM

# -D USE_FIRST_CRITERION

MACRO=-D ALG_$(ALG) -D WORKER_NUM=4 -D GUROBI_NUM=32
CXX=g++
CXXFLAGS=-O3 -std=c++14 -fopenmp -pthread $(MACRO)
INC=-I $(GUROBI_HOME)/include/
LIB=-L $(GUROBI_HOME)/lib -lgurobi_c++ -lgurobi91 -lm


all : search_$(ALG).run retrieve_$(ALG).run 

search_$(ALG).run : src/search.cpp
	$(CXX) $(CXXFLAGS) -m64 -g -o search_$(ALG).run src/search.cpp src/cube.hpp $(INC) $(LIB)

retrieve_$(ALG).run : src/retrieve.cpp
	$(CXX) $(CXXFLAGS) -m64 -g -o retrieve_$(ALG).run src/retrieve.cpp src/cube.hpp $(INC) $(LIB)


.PHONY: clean
clean : 
	rm *_$(ALG).run