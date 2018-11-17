# all-local:
# 	cd zlib && ./configure && make
# 	cd libiberty && ./configure && make
# 	cd bfd && ./configure --target=sparc && make
# 
# clean-local:
# 	cd zlib && make clean
# 	cd libiberty && make clean
# 	cd bfd && make clean

SUBDIRS = opcodes

bin_PROGRAMS = sis
sis_SOURCES = \
			  help.c \
			  float.c \
			  grlib.c \
			  func.c \
			  exec.c \
			  erc32.c \
			  leon2.c \
			  leon3.c \
			  sis.c

AM_CFLAGS = -g -O0
sis_CPPFLAGS = -I$(top_srcdir)/include -I$(top_srcdir)/libiberty -I$(top_srcdir)/bfd
sis_LDFLAGS = -lreadline
sis_LDADD = $(top_srcdir)/zlib/libz.a $(top_srcdir)/libiberty/libiberty.a $(top_srcdir)/bfd/libbfd.a $(top_srcdir)/opcodes/libopcodes.a
