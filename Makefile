CPP = g++
GCC_TARGET := $(shell gcc -dumpmachine)
MARCH = 
ifeq (,$(findstring darwin,$(GCC_TARGET)))
    MARCH += -march=native -fopenmp
endif
#CCFLAGS = -Wall -g -std=c++14 -D_GLIBCXX_PARALLEL -march=native -funroll-loops -fopenmp
CCFLAGS = -Wall -g -std=c++14 -funroll-loops -pthread $(MARCH)
INCLUDES = ./include
INCDIRS = -I$(INCLUDES) -I$(SRC)/flute
LFLAGS = 
DEBUG = 0
LIBS = -lm
BIN = bin
SRC = src
SRCS := $(wildcard $(SRC)/*.cpp) $(wildcard $(SRC)/flute/*.cpp)
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

.PHONY: depend clean test coverage

$(MAIN): $(OBJS) 
	$(CPP) $(CCFLAGS) $(OPTFLAGS) $(INCDIRS) -o $(MAIN) $(OBJS) $(LFLAGS) $(LIBS)

$(BIN)/%.o: $(SRC)/%.cpp 
	@mkdir -p $(dir $@)
	$(CPP) $(CCFLAGS) $(OPTFLAGS) $(INCDIRS) -DDEBUG=$(DEBUG) $(DEPFLAGS) -c $< -o $@

test: $(MAIN)
	cd test && ./run_smoke.sh ../$(MAIN)

$(COVMAIN): $(COVOBJS)
	$(CPP) $(CCFLAGS) $(COVFLAGS) $(INCDIRS) -o $(COVMAIN) $(COVOBJS) $(LFLAGS) $(LIBS)

$(COVBIN)/%.o: $(SRC)/%.cpp
	@mkdir -p $(dir $@)
	$(CPP) $(CCFLAGS) $(COVFLAGS) $(INCDIRS) -DDEBUG=$(DEBUG) $(DEPFLAGS) -c $< -o $@

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

