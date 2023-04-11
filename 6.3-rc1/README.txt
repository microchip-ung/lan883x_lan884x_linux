========================================================================================================== 
Linux Device drivers for Microchip LAN7431 Ethernet Controller and LAN8841 PHY
Version: 1.0
========================================================================================================== 
 
Contents: 
 
1. Platforms and OS versions supported 
2. Device support 
3. Driver structure and file description 
4. Building and installing the driver 
5. Release history 
6. Known issues 


1. Platforms and kernel versions supported 
------------------------------------------ 
 
    - x86/x64 PC 
    - kernel 6.3-rc3
 
2. Device support 
----------------- 
 
This release supports: 
 -EVB_LAN7431 rev.2 and EVB_LAN8841 rev.1

  EVB LAN7431 rev. 2 (newer than B0)
    jumpers j5, j11, middle of j16.

  EVB LAN8841 rev.1
    Set j1, j2, j4 jumpers to EDS setting.


3. Driver structure and file description 
---------------------------------------- 
    This driver package will create the two modules (drivers): 
 
    lan743x.ko:       Device driver for LAN7431
    micrel.ko:        Device driver for LAN8841 
 
    Source files: 
        lan743x_main.h      -   lan743x hardware specific header file 
        lan743x_main.c      -   lan743x hardware specific source file 
        micrel_phy.h        -   micrel phy header file including lan8841 phy. 
        micrel.c            -   micrel phy source file including lan8841 phy. 
 
4. Building and installing the drivers 
------------------------------------- 
    The following instructions work fine for a PC build environment, embedded 
    platforms may need slight build modifications, consult your platform documentation. 
    a. Obtain the kernel source tree for the platform in use and build it. 
	Reboot to the kernel you built.
    b. Extract the custom driver source code from the zip file.
	Copy and overwrite linux-6.3-rc1/drivers/net/ethernet/microchip/lan743x_main.h
	Copy and overwrite linux-6.3-rc1/drivers/net/ethernet/microchip/lan743x_main.c
	Copy and overwrite linux-6.3-rc1/include/linux/micrel_phy.h
	Copy and overwrite linux-6.3-rc1/drivers/net/phy/micrel.c
        Delete object files if that exists.
          rm drivers/net/ethernet/microchip/lan743x_main.o
          rm drivers/net/phy/micrel.o
    c. Compile drivers at the root of your linux-6.3-rc1 source tree.
	sudo make drivers/net/ethernet/microchip/lan743x.ko
	sudo make drivers/net/phy/micrel.ko
	You should get no errors.
    d. Unload the existing drivers.
	sudo rmmod lan743x
	sudo rmmod micrel
	Note that micrel module may not be loaded at default.
    e. Load the new drivers in order.
	sudo insmod drivers/net/phy/micrel.ko
	sudo insmod drivers/net/ethernet/microchip/lan743x.ko
	Note this loading is not "install" drivers on the kernel and rebooting
	the PC will load the original in-kernel drivers.
	To see the driver kernel log, open a separate terminal window then run 
	dmesg -w
	you can check if the drivers are loaded correctly.
 
5.  Release history 
-------------------

    v1.0 (internal release) 4/11/2023
 
6. Known Issues 
---------------------------- 
    none.


