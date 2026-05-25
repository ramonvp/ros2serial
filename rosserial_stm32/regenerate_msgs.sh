#!/bin/bash
BUILD_DIR=$(rospack find rosserial_stm32)/build
rm -rf $BUILD_DIR
mkdir -p $BUILD_DIR/Inc 
$(rospack find rosserial_stm32)/src/rosserial_stm32/make_libraries.py $BUILD_DIR > /dev/null

echo "Your files are ready here: $BUILD_DIR/Inc"

