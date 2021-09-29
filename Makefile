CPP = g++
CCFLAGS = -Wall -O3 -g -std=c++11
INCLUDES = ./include/
LFLAGS = 
LIBS = -lm
BIN = bin
SRC = src
SRCS = $(wildcard $(SRC)/*.cpp)
OBJS:= $(patsubst ${SRC}%.cpp,${BIN}%.o,$(SRCS))
DEPS := $(OBJS:.o=.d)
DEPDIR := $(BIN)/.deps
DEPFLAGS = -MT $@ -MMD 

# define the executable file 
MAIN = hanan_router

.PHONY: depend clean

$(MAIN): $(OBJS) 
	$(CPP) $(CCFLAGS) -I$(INCLUDES) -o $(MAIN) $(OBJS) $(LFLAGS) $(LIBS)

-include $(DEPS)

$(BIN)/%.o: $(SRC)/%.cpp 
	@mkdir -p $(BIN)
	$(CPP) $(CCFLAGS) $(DEPFLAGS) -I$(INCLUDES) -c $<  -o $@

clean:
	rm -rf $(MAIN) $(BIN)/*.o $(DEPDIR)

