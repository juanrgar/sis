all-local:
	cd zlib && ./configure && make
	cd libiberty && ./configure && make
	cd bfd && ./configure --target=sparc && make

clean-local:
	cd zlib && make clean
	cd libiberty && make clean
	cd bfd && make clean

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

sis_CPPFLAGS = -I$(top_srcdir)/include
sis_LDFLAGS = -lreadline
