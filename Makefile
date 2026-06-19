CPP = g++
GCC_TARGET := $(shell gcc -dumpmachine)
MARCH = 
ifeq (,$(findstring darwin,$(GCC_TARGET)))
    MARCH += -march=native -fopenmp
endif
#CCFLAGS = -Wall -g -std=c++14 -D_GLIBCXX_PARALLEL -march=native -funroll-loops -fopenmp
CCFLAGS = -Wall -g -std=c++14 -funroll-loops $(MARCH)
# Kissat SAT solver (header + static lib). Override KISSAT_DIR if it lives elsewhere.
KISSAT_DIR ?= /Users/ramprasath/scratch/kissat
KISSAT_INC = $(KISSAT_DIR)/src
KISSAT_LIB = $(KISSAT_DIR)/build/libkissat.a
INCLUDES = ./include
LFLAGS =
DEBUG = 0
LIBS = -lm $(KISSAT_LIB)
# Optional CP-SAT (Google OR-Tools) alternative router: build with CPSAT=1.
# Override ORTOOLS_DIR if it lives elsewhere.
ORTOOLS_DIR ?= /opt/ortools
ifeq ($(CPSAT),1)
  CPSAT_STD  = -std=c++17
  CPSAT_DEF  = -DUSE_ORTOOLS -DOR_PROTO_DLL=
  CPSAT_INC  = -I$(ORTOOLS_DIR)/include
  ORTOOLS_LIBS = -L$(ORTOOLS_DIR)/lib -lortools $(wildcard $(ORTOOLS_DIR)/lib/libabsl_*.dylib) -Wl,-rpath,$(ORTOOLS_DIR)/lib
else
  CPSAT_STD  = -std=c++14
  CPSAT_DEF  =
  CPSAT_INC  =
  ORTOOLS_LIBS =
endif
LIBS += $(ORTOOLS_LIBS)
BIN = bin
SRC = src
SRCS := $(wildcard $(SRC)/*.cpp)
OBJS := $(patsubst ${SRC}%.cpp,${BIN}%.o,$(SRCS))
DEPS := $(OBJS:.o=.d)
DEPFLAGS = -MMD -MP

MAIN = hanan_router

OPTFLAGS = -O3 -ffast-math
#OPTFLAGS =
ifeq ($(DEBUG), 1)
OPTFLAGS =
endif

# coverage build : clang source-based instrumentation, separate objects, no -O3
# so the coverage mapping stays line-accurate
COVMAIN = $(MAIN)_cov
COVBIN = $(BIN)/cov
COVDIR = coverage
COVOBJS := $(patsubst $(SRC)%.cpp,$(COVBIN)%.o,$(SRCS))
COVDEPS := $(COVOBJS:.o=.d)
COVFLAGS = -fprofile-instr-generate -fcoverage-mapping
HDRS := $(wildcard $(SRC)/*.h)
ifeq (,$(findstring darwin,$(GCC_TARGET)))
    LLVM_PROFDATA = llvm-profdata
    LLVM_COV = llvm-cov
else
    LLVM_PROFDATA = xcrun llvm-profdata
    LLVM_COV = xcrun llvm-cov
endif

.PHONY: depend clean test coverage cpsat

$(MAIN): $(OBJS) 
	$(CPP) $(CCFLAGS) $(OPTFLAGS) -I$(INCLUDES) -o $(MAIN) $(OBJS) $(LFLAGS) $(LIBS)

$(BIN)/%.o: $(SRC)/%.cpp
	@mkdir -p $(BIN)
	$(CPP) $(CCFLAGS) $(OPTFLAGS) -I$(INCLUDES) -I$(KISSAT_INC) -DDEBUG=$(DEBUG) $(DEPFLAGS) -c $< -o $@

# CpRoute pulls in OR-Tools (C++17) when CPSAT=1; otherwise it is a small stub.
$(BIN)/CpRoute.o: $(SRC)/CpRoute.cpp
	@mkdir -p $(BIN)
	$(CPP) -Wall -g $(CPSAT_STD) -funroll-loops $(MARCH) $(OPTFLAGS) $(CPSAT_DEF) -I$(INCLUDES) -I$(KISSAT_INC) $(CPSAT_INC) -DDEBUG=$(DEBUG) $(DEPFLAGS) -c $< -o $@

test: $(MAIN)
	cd test && ./run_smoke.sh ../$(MAIN)

# CP-SAT alternative router validation (needs a CPSAT=1 build).
cpsat: $(MAIN)
	cd test && ./run_cpsat.sh ../$(MAIN)

$(COVMAIN): $(COVOBJS)
	$(CPP) $(CCFLAGS) $(COVFLAGS) -I$(INCLUDES) -o $(COVMAIN) $(COVOBJS) $(LFLAGS) $(LIBS)

$(COVBIN)/%.o: $(SRC)/%.cpp
	@mkdir -p $(COVBIN)
	$(CPP) $(CCFLAGS) $(COVFLAGS) -I$(INCLUDES) -I$(KISSAT_INC) -DDEBUG=$(DEBUG) $(DEPFLAGS) -c $< -o $@

coverage: $(COVMAIN)
	rm -rf $(COVDIR)
	mkdir -p $(COVDIR)/profraw
	cd test && LLVM_PROFILE_FILE=$(CURDIR)/$(COVDIR)/profraw/smoke-%p.profraw ./run_smoke.sh ../$(COVMAIN)
	$(LLVM_PROFDATA) merge -sparse $(COVDIR)/profraw/*.profraw -o $(COVDIR)/smoke.profdata
	$(LLVM_COV) report ./$(COVMAIN) -instr-profile=$(COVDIR)/smoke.profdata $(SRCS) $(HDRS) | tee $(COVDIR)/summary.txt
	$(LLVM_COV) report ./$(COVMAIN) -instr-profile=$(COVDIR)/smoke.profdata -show-functions $(SRCS) $(HDRS) > $(COVDIR)/functions.txt
	$(LLVM_COV) show ./$(COVMAIN) -instr-profile=$(COVDIR)/smoke.profdata -format=html -output-dir=$(COVDIR)/html $(SRCS) $(HDRS)
	@echo "per-function report : $(COVDIR)/functions.txt ; html : $(COVDIR)/html/index.html"

clean:
	rm -rf $(MAIN) $(BIN)/*.o $(BIN)/*.d $(COVMAIN) $(COVBIN) $(COVDIR)

-include $(DEPS)
-include $(COVDEPS)

