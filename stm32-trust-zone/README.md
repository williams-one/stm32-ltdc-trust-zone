## Templates_TrustZoneEnabled Example Description

This project provides a reference template based on the STM32Cube HAL API that can be used
to build any firmware application when TrustZone security is activated **(Option bit TZEN=1)**.

This project is composed of two sub-projects:

 - One for the secure application part (Project_s)
 - One for the non-secure application part (Project_ns).

Please remember that on system with security enabled, the system always boots in secure and
the secure application is responsible for launching the non-secure application.

This project mainly shows how to switch from secure application to non-secure application
thanks to the system isolation performed to split the internal Flash and internal SRAM memories
into two halves:

 - The first half for the secure application and
 - The second half for the non-secure application.

User Option Bytes configuration:

Please note the internal Flash is fully secure by default in TZEN=1 and User Option Bytes
SECWM1_PSTRT/SECWM1_PEND and SECWM2_PSTRT/SECWM2_PEND should be set according to the application
configuration. Here the proper User Option Bytes setup in line with the project linker/scatter
file is as follows:

     - TZEN=1
     - SECWM1_PSTRT=0x0   SECWM1_PEND=0xFF  meaning all pages of Bank1 set as secure
     - SECWM2_PSTRT=0xFF  SECWM2_PEND=0x0   meaning no page of Bank2 set as secure, hence Bank2 non-secure

Any attempt by the non-secure application to access unauthorized code, memory or
peripheral generates a fault as demonstrated in non secure application by commenting some
code instructions in Secure/main.c (LED I/O release).

This project is targeted to run on STM32U5G9xx device on STM32U5G9J-DK2 boards from STMicroelectronics.

The reference template project configures the maximum system clock frequency at 160Mhz in non-secure
application.

#### <b>Notes</b>

 1. The following sequence is needed to disable TrustZone:

      - **Boot from user Flash memory**:
         a.	Make sure that secure and non-secure applications are well loaded and executed (jump done on non-secure application).
         b.	If not yet done, set RDP to level 1 through STM32CubeProgrammer. Then only Hotplug connection is possible during non-secure application execution.
         c.	Use a power supply different from ST-LINK in order to be able to connect to the target.
         d.	Uncheck the TZEN box and set RDP to level 0 (option byte value 0xAA), then click on Apply.

     - **Boot from RSS**:
         a.	Make sure to apply a high level on BOOT0 pin (make sure that nSWBOOT0 Option Byte is checked).
         b.	If not yet done, set RDP to level 1 through STM32CubeProgrammer. Then only Hotplug connection is possible during non-secure application execution.
         c.	Use a power supply different from ST-LINK in order to be able to connect to the target.
         d.	Uncheck the TZEN box and set RDP to level 0 (option byte value 0xAA), then click on Apply.

	Please refer to AN5347 for more details.
