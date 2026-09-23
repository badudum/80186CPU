# Written by tools/img2hex.py from big16.img.
# gen_bios.py and gen_disk.py import this so the BIOS's
# INT 13h describes the same disk the image was built for.
SPT = 63
HEADS = 16
SECTORS = 32768
