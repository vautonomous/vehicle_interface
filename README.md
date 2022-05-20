# isuzu_interface

This package is developed to provide communication between Autoware.Universe and vehicle.

## usb-naming

usb-naming script is used to assign a name to communication port. After go to the directory run the command:

```$ sudo ./usb-naming.sh```

Now, check the name of the LLC port name by using:

`$ ls /dev/tty*`

You should see `/dev/ttyLLC`.
