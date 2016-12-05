CFlags=-c -Wall -O3  -std=gnu++11 -Iinclude -Izmdp/include
LDFlags= -Lzmdp/lib/linux3 -lzmdpPomdpCore -lzmdpPomdpParser -lzmdpPomdpBounds -lzmdpCommon -lboost_thread -lboost_system
CC=g++
RM=rm

all: tag.o tiger.o zmdpCommonTime.o tag tiger

tag: tag.o
	@mkdir -p bin
	$(CC) obj/$< -o bin/$@ $(LDFlags)

tiger: tiger.o
	@mkdir -p bin
	$(CC) obj/$< -o bin/$@ $(LDFlags)

tag.o: src/tag.cpp
	@mkdir -p obj
	$(CC) $(CFlags) $< -o obj/$@

tiger.o: src/tiger.cpp
	@mkdir -p obj
	$(CC) $(CFlags) $< -o obj/$@

zmdpCommonTime.o: zmdp/src/common/zmdpCommonTime.cc
	@mkdir -p obj
	$(CC) $(CFlags) $< -o obj/$@

clean:
	$(RM) obj/tiger.o obj/tag.o obj/zmdpCommonTime.o
	$(RM) bin/tag bin/tiger

install:
	@mkdir -p /usr/local/include/lightpomcp
	@cp include/* /usr/local/include/lightpomcp



