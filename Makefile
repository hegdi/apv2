PROJECT_ROOT = $(dir $(abspath $(lastword $(MAKEFILE_LIST))))

OBJS = apv2.o sais.o
CFLAGS = -Wall -Wextra
LDLIBS =

ifeq ($(BUILD_MODE),debug)
	CFLAGS += -Og -g3
else ifeq ($(BUILD_MODE),run)
	CFLAGS += -Ofast
else ifeq ($(BUILD_MODE),linuxtools)
	CFLAGS += -g -pg -fprofile-arcs -ftest-coverage
	LDFLAGS += -pg -fprofile-arcs -ftest-coverage
	EXTRA_CLEAN += apv2.gcda apv2.gcno $(PROJECT_ROOT)gmon.out
	EXTRA_CMDS = rm -rf apv2.gcda
else
    $(error Build mode $(BUILD_MODE) not supported by this Makefile)
endif

all:	apv2

apv2:	$(OBJS)
	$(CXX) $(LDFLAGS) -o $@ $^ $(LDLIBS)
	$(EXTRA_CMDS)

%.o:	$(PROJECT_ROOT)%.cpp
	$(CXX) -c $(CFLAGS) $(CXXFLAGS) $(CPPFLAGS) -o $@ $<

%.o:	$(PROJECT_ROOT)%.c
	$(CC) -c $(CFLAGS) $(CPPFLAGS) -o $@ $<

clean:
	rm -fr apv2 $(OBJS) $(EXTRA_CLEAN)
