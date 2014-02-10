
CC=clang
CXX=clang++

all:
	$(CC) -Wall -ggdb -O0 -o tsip *.c

clean:
	rm *.o
	rm tsip

