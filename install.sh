#!/bin/bash

if [ -z "$SOFF_PREFIX" ]; then
  echo "Define the SOFF_PREFIX environment variable before running $0."
  exit 1
fi

if [ ! -d "$SOFF_PREFIX" ]; then
  echo "SOFF_PREFIX=$SOFF_PREFIX is not a valid directory.";
  exit 1
fi

set -e

echo "[SOFF] Compiling clang..."
mkdir -p compiler-build
cd compiler-build
if [ ! -f "Makefile" ]; then
  ../compiler/configure
fi
make ENABLE_OPTIMIZED=1 -j
cd ..
echo "[SOFF] Copying files to $SOFF_PREFIX..."
mkdir -p "$SOFF_PREFIX/bin"
cp compiler-build/Release+Asserts/bin/clang "$SOFF_PREFIX/bin/soff-compiler"
cp bin/* "$SOFF_PREFIX/bin/"
mkdir -p "$SOFF_PREFIX/etc"
cp etc/* "$SOFF_PREFIX/etc/"
mkdir -p "$SOFF_PREFIX/include/soff"
cp include/* "$SOFF_PREFIX/include/soff/"

echo "[SOFF] soff-compiler installation complete."
echo "[SOFF] Please manually copy one of the target files to $SOFF_PREFIX/etc/target.conf.";
