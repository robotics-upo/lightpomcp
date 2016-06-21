#!/usr/bin/env bash

git submodule init
git submodule update
make -C zmdp/src
make
sudo make install
