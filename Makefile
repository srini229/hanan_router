CPP = g++
#CCFLAGS = -Wall -g -std=c++11 -D_GLIBCXX_PARALLEL -march=native -funroll-loops -fopenmp
CCFLAGS = -Wall -Ofast -g -std=c++11 -D_GLIBCXX_PARALLEL -march=native -funroll-loops -fopenmp
INCLUDES = ./include
LFLAGS = 
LIBS = -lm
BIN = bin
SRC = src
SRCS = $(wildcard $(SRC)/*.cpp)
OBJS:= $(patsubst ${SRC}%.cpp,${BIN}%.o,$(SRCS))
DEPS:= $(OBJS:.o=.d)
DEPFLAGS = -MMD -MP

MAIN = hanan_router

.PHONY: depend clean

$(MAIN): $(OBJS) 
	$(CPP) $(CCFLAGS) -I$(INCLUDES) -o $(MAIN) $(OBJS) $(LFLAGS) $(LIBS)

$(BIN)/%.o: $(SRC)/%.cpp 
	@mkdir -p $(BIN)
	$(CPP) $(CCFLAGS) -I$(INCLUDES) $(DEPFLAGS) -c $< -o $@

clean:
	rm -rf $(MAIN) $(BIN)/*.o

-include $(DEPS)

