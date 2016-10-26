pkfltmodule.so:	pack_float.o pkfltWrapper.o
				gcc -shared pack_float.o pkfltWrapper.o -o pkfltmodule.so
				cp pkfltmodule.so ../../scripts

pack_float.o:	pack_float.c
				gcc -fpic -o pack_float.o -c pack_float.c

pkfltWrapper.o:	pkfltWrapper.c
				gcc -fpic -I /usr/include/python2.7 -o pkfltWrapper.o -c pkfltWrapper.c

clean:
				rm *.o *.so ../../scripts/pkfltmodule.so

