# force some options
DEBUG = yes

-include ../Makefile.include
CFLAGS += -I./ -I../ -Icu/
LDFLAGS += -L./ -Lcu/ -lcu -lrt -lm -L../ -lccd

CHECK_REG=cu/check-regressions
CHECK_TS ?=

OBJS = common.o support.o vec3.o polytope.o boxbox.o spheresphere.o \
       cylcyl.o boxcyl.o mpr_boxbox.o mpr_cylcyl.o mpr_boxcyl.o
BENCH_OBJS = bench-boxbox.o


all: test bench bench2

test: cu $(OBJS) main.c
	$(CC) $(CFLAGS) -o $@ main.c $(OBJS) $(LDFLAGS)

bench: cu bench.c support.o
	$(CC) $(CFLAGS) -o $@ bench.c support.o $(LDFLAGS)
bench2: cu bench2.c support.o
	$(CC) $(CFLAGS) -o $@ bench2.c support.o $(LDFLAGS)

%.o: %.c %.h
	$(CC) $(CFLAGS) -c -o $@ $<
%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

check: all
	@echo ""
	@echo "----------------------------------------";
	./test $(CHECK_TS)
	@echo "----------------------------------------";
	@echo "Checking regressions:";
	$(PYTHON) $(CHECK_REG) regressions
	@echo ""

check-valgrind: all
	valgrind -q --leak-check=full --show-reachable=yes --trace-children=yes \
             --error-limit=no \
             ./test $(CHECK_TS)

check-valgrind-gen-suppressions: all
	valgrind -q --leak-check=full --show-reachable=yes --trace-children=yes \
             --gen-suppressions=all --log-file=out --error-limit=no \
             ./test $(CHECK_TS)

cu:
	$(MAKE) ENABLE_TIMER=yes -C cu/

clean:
	rm -f *.o
	rm -f objs/*.o
	rm -f test bench bench2
	rm -f tmp.*
	rm -f regressions/tmp.*

.PHONY: all clean check check-valgrind cu
	
