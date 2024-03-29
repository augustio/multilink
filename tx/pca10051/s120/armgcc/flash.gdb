target remote localhost: 2331

# NVMC erase enable
set {int} 0x4001e504 = 0x02

# NVMC erase all non-volatile user memory
set {int} 0x4001e50c = 0x01

# NVMC erase all UICRs
set {int} 0x4001e514 = 0x01

# NVMC enable write
set {int} 0x4001e504 = 0x01

# Flash the softdevice
load  ../../../../components/softdevice/s120/hex/s120_softdevice.hex

# Flash the software
load _build/nrf51422_xxac_s120.hex
