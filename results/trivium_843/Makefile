TARGET=trivium_verify

CC=g++
OPT= -std=c++11 -O3 -m64
INC=-I/${GUROBI_HOME}/include/
LIB=-L/${GUROBI_HOME}/lib -lgurobi_c++ -lgurobi91 -lm

SRC=$(TARGET).cpp

all: $(TARGET)
$(TARGET): $(SRC)
	$(CC) $(OPT) -o $(TARGET)  $(SRC)  $(INC) $(LIB) 

clean:
	rm -f $(TARGET) 


