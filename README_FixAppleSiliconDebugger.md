# [Fix Debugger Hang on Apple Silicon](https://github.com/platformio/platform-atmelavr/issues/292#issuecomment-1296811066)

```shell
brew tap osx-cross/avr
brew install avr-gcc # currently installs 9.4.0, takes >10 minutes
brew install avr-gdb # eh, but avr-gdb is not included. This seems to do the trick
cd ~/.platformio/packages/toolchain-atmelavr/bin/
mv avr-gdb avr-gdb.orig
ln -s `which avr-gdb` avr-gdb
```

- Shell output after installing avr-gdb
```shell
gdb requires special privileges to access Mach ports.
You will need to codesign the binary. For instructions, see:

  https://sourceware.org/gdb/wiki/BuildingOnDarwin

On 10.12 (Sierra) or later with SIP, you need to run this:

  echo "set startup-with-shell off" >> ~/.gdbinit
  ```