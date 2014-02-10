
CC=clang
CXX=clang++

all:
	$(CC) -Wall -o tsip *.c

clean:
	rm *.o
	rm tsip

