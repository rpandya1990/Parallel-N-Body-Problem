# n-body simulation


## To build and run it!
The Makefile is setup to build from a linux box (mine is Arch Linux)
 using `clang`, but you can easily change that line (`clang++`) for
 `g++`, and it will compile just as well.  Also note that you must have
library `freeglut` installed in your system.  You can look at the
Makefile to see where `freeglut` can be installed.

``` shell
    cd opengl-demo-nbody
    make
    ./n-body-sim    # runs the demo (will start in fullscreen, `alt+F4 to exit`)
```

