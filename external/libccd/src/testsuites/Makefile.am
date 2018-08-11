SUBDIRS = cu

AM_CPPFLAGS = -I $(srcdir)/.. -I $(builddir)/.. -I $(srcdir)/cu

LDADD = $(builddir)/cu/libcu.la $(builddir)/../libccd.la


check_PROGRAMS = test bench bench2

test_SOURCES = main.c \
		common.c common.h \
        support.c support.h \
		vec3.c vec3.h \
		polytope.c polytope.h \
		boxbox.c boxbox.h \
		spheresphere.c spheresphere.h \
		cylcyl.c cylcyl.h \
		boxcyl.c boxcyl.h \
		mpr_boxbox.c mpr_boxbox.h \
		mpr_cylcyl.c mpr_cylcyl.h \
		mpr_boxcyl.c mpr_boxcyl.h

bench_SOURCES = bench.c \
        support.c support.h

bench2_SOURCES = bench2.c \
        support.c support.h

