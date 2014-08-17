HFS= /opt/hfs13.0.498

DSONAME = sop_pathdeform.so
SOURCES = sop_pathdeform.cpp
ICONS = icon/SOP_path_deform.svg
INSTDIR = $(HOME)/houdini13.0

OPTIMIZER = -O2
#OPTIMIZER = -g

include $(HFS)/toolkit/makefiles/Makefile.gnu

all:	install clean
install:	default	icons clean
	@mv $(DSONAME) $(INSTDIR)/dso
clean:
	@rm *.o
