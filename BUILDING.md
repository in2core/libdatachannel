# libdatachannel - Building instructions

## Clone repository and submodules

```bash
$ git clone https://github.com/paullouisageneau/libdatachannel.git
$ cd libdatachannel
$ git submodule update --init --recursive
```

## Build with CMake

The CMake library targets `libdatachannel` and `libdatachannel-static` respectively correspond to the shared and static libraries. The default target will build tests and examples. The option `USE_GNUTLS` allows to switch between OpenSSL (default) and GnuTLS, and the option `USE_NICE` allows to switch between libjuice as submodule (default) and libnice.

If you only need Data Channels, the option `NO_MEDIA` allows to make the library lighter by removing media support. Similarly, `NO_WEBSOCKET` removes WebSocket support.

### POSIX-compliant operating systems (including Linux and Apple macOS)

```bash
$ cmake -B build -DUSE_GNUTLS=1 -DUSE_NICE=0
$ cd build
$ make -j2
```

### Apple macOS with XCode project

```bash
$ cmake -B "$BUILD_DIR" -DUSE_GNUTLS=0 -DUSE_NICE=0 -G Xcode
```

Xcode project is generated in *build/* directory.

#### Solving **Could NOT find OpenSSL** error

You need to add OpenSSL root directory if your build fails with the following message:

```
Could NOT find OpenSSL, try to set the path to OpenSSL root folder in the
system variable OPENSSL_ROOT_DIR (missing: OPENSSL_CRYPTO_LIBRARY
OPENSSL_INCLUDE_DIR)
```

for example:
```bash
$ cmake -B build -DUSE_GNUTLS=0 -DUSE_NICE=0 -G Xcode -DOPENSSL_ROOT_DIR=/usr/local/Cellar/openssl\@1.1/1.1.1h/
```

### Microsoft Windows with MinGW cross-compilation
```bash
$ cmake -B build -DCMAKE_TOOLCHAIN_FILE=/usr/share/mingw/toolchain-x86_64-w64-mingw32.cmake # replace with your toolchain file
$ cd build
$ make -j2
```

### Microsoft Windows with Microsoft Visual C++
```bash
$ cmake -B build -G "NMake Makefiles"
$ cd build
$ nmake
```

## Build directly with Make (Linux only)

The option `USE_GNUTLS` allows to switch between OpenSSL (default) and GnuTLS, and the option `USE_NICE` allows to switch between libjuice as submodule (default) and libnice.

If you only need Data Channels, the option `NO_MEDIA` removes media support. Similarly, `NO_WEBSOCKET` removes WebSocket support.

```bash
$ make USE_GNUTLS=1 USE_NICE=0
```

