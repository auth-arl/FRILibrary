#!/usr/bin/env bash

cd Linux
mkdir -p x64/debug/bin
mkdir -p x64/release/bin
mkdir -p x64/debug/lib
mkdir -p x64/release/lib
mkdir -p x64/debug/obj
mkdir -p x64/release/obj
mkdir -p x86/debug/bin
mkdir -p x86/release/bin
mkdir -p x86/debug/lib
mkdir -p x86/release/lib
mkdir -p x86/debug/obj
mkdir -p x86/release/obj
make clean all
cd ..
