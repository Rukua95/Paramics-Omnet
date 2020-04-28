H = @

INCLUDES = -I../../include \
           -I../../include/user \


include ../../make/$(HOSTTYPE)/plugin.mak

TARGET = base
OBJLIST = plugin.o 

all: $(TARGET).so

$(TARGET).so: $(OBJLIST) 
	$H echo creating shared object $(TARGET).so 
	$H cd $(HOSTTYPE); ld $(LIBOPTS) $(TARGET).so $(OBJLIST); cd ..

$(OBJLIST):
	$H echo "compiling $*.c ..."
	$H $(COMPILE) -o $(HOSTTYPE)/$*.o $*.c

clean:
	$H rm -f $(HOSTTYPE)/*.o $(HOSTTYPE)/*.so $(HOSTTYPE)/so_locations
