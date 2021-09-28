CPP = g++

CCFLAGS = -Wall -O3 -g -std=c++11
INCLUDES = ./include/
LFLAGS = 
LIBS = -lm
BIN = bin
SRC = src
SRCS = $(wildcard $(SRC)/*.cpp)
OBJS:= $(patsubst ${SRC}%.cpp,${BIN}%.o,$(SRCS))


# define the executable file 
MAIN = hanan_router

.PHONY: depend clean

$(MAIN): $(OBJS) 
	$(CPP) $(CCFLAGS) -I$(INCLUDES) -o $(MAIN) $(OBJS) $(LFLAGS) $(LIBS)

$(BIN)/%.o: $(SRC)/%.cpp 
	mkdir -p $(BIN)
	$(CPP) $(CCFLAGS) -I$(INCLUDES) -c $<  -o $@

clean:
	rm -f $(MAIN) $(BIN)/*.o

