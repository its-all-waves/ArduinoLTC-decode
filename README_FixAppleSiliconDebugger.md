# Fix Debugger Hang on Apple Silicon
https://github.com/platformio/platform-atmelavr/issues/292#issuecomment-1296811066


```shell
brew tap osx-cross/avr
brew install avr-gcc # currently installs 9.4.0, takes >10 minutes
brew install avr-gdb # eh, but avr-gdb is not included. This seems to do the trick
cd ~/.platformio/packages/toolchain-atmelavr/bin/
mv avr-gdb avr-gdb.orig
ln -s `which avr-gdb` avr-gdb
```


- Shell output after installing avr-gdb below. **Follow the instructions! Codesigning instructions below, as well!**


```shell
gdb requires special privileges to access Mach ports.
You will need to codesign the binary. For instructions, see:

  https://sourceware.org/gdb/wiki/BuildingOnDarwin

On 10.12 (Sierra) or later with SIP, you need to run this:

  echo "set startup-with-shell off" >> ~/.gdbinit
  ```


  # Codesigning on Macos to Allow Gdb to Control Other Processes
  https://www.tweaking4all.com/forum/postid/4685/

2. Install GDB

Note: if you have an old version of GDB; consider renaming or deleting it.
(in my case with: sudo rm /usr/local/bin/gdb)
```shell
brew install gdb
```
you can confirm it with being available with:
gdb --version

3. Sign GDB

[at the end of this post: how to create your self-signed certificate if needed](#Optional-Generate-a-certificate-for-signing-GDB)

- create a plist file (eg. gdb-entitlement.xml) with this content:
```xml
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN"
"http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
    <key>com.apple.security.cs.allow-jit</key>
    <true/>
    <key>com.apple.security.cs.allow-unsigned-executable-memory</key>
    <true/>
    <key>com.apple.security.cs.allow-dyld-environment-variables</key>
    <true/>
    <key>com.apple.security.cs.disable-library-validation</key>
    <true/>
    <key>com.apple.security.cs.disable-executable-page-protection</key>
    <true/>
    <key>com.apple.security.cs.debugger</key>
    <true/>
    <key>com.apple.security.get-task-allow</key>
    <true/>
</dict>
</plist>
```
- determine where GDB is located with the "which" statement:
```shell
$ which gdb   
/usr/local/bin/gdb
 (the path on your system may be different)
```
 - sign the GDB binary (in the directory where you just created the gdb-entitlements.xml file):
```shell
codesign --entitlements gdb-entitlements.xml -fs "S4F99MAJG8" /usr/local/bin/gdb
Note:
```

- Replace "S4F99MAJG8" with your certificate or gbd-cert if you self-signed a cert.
- Make sure "user/local/bin/gdb" matches the path of your GDB (see the "which" command)

You can verify if signing went OK with:

```shell 
codesign --verify --verbose /usr/local/bin/gdb
``` 

 

Note:

- Do not forget to change your Lazarus settings
- You will probably need to do a Clean Build for your project before GDB works properly with Lazarus.

 


## Optional: Generate a certificate for signing GDB

I copied these steps from the Internet for your convenience.
As an app developer I do have my own Apple certificate and have no need for a self-signed CERT.

- Launch Keychain: Applications > Utilities > Keychain Access.
- In Keychains list on the left, right-click on the System item and select Unlock Keychain "System".
- From the toolbar: Keychain Access > Certificate Assistant > Create a Certificate.
- Choose a name (e.g. gdb-cert <- remember this name!).
- Set Identity Type to "Self Signed Root".
- Set Certificate Type to "Code Signing".
- Check the "Let me override defaults" checkbox.

At this point, you can go on with the installation process until you get the "Specify a Location For The Certificate" dialogue box. Here you need to set Keychain to "System". Finally, you can click on the "Create" button.

After these steps, you can see the new certificate under System keychains.
From the contextual menu of the newly created certificate (right-click on it) select the Get info option.
In the dialogue box, expand the Trust item and set Code signing to Always Trust.
Then, from the Keychains list on the left, right-click on the System item and select Lock Keychain "System".
You may have to reboot your system.
________________________________________________________________________________

  # avr-stub Debugger Docs
  https://docs.platformio.org/en/latest/plus/debug-tools/avr-stub.html#configuration

________________________________________________________________________________

