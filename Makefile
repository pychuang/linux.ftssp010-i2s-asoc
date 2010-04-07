EXTRA_CFLAGS += -Isound/soc
obj-m	:= a369evb.o
obj-m	+= a369-pcm.o
obj-m	+= ftssp010-i2s.o

