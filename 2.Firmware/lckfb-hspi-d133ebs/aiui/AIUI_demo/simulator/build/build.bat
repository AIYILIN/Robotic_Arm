set PATH=D:\Program Files (x86)\AiUIBuilder\tool\cmake\bin;D:\Program Files (x86)\AiUIBuilder\tool\mingw\bin;%PATH%
cmake -DCMAKE_C_COMPILER=gcc -DCMAKE_MAKE_PROGRAM=mingw32-make -G "MinGW Makefiles" ../
cmake --build . --target all -j 16
