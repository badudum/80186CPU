# Written by tools/img2hex.py from disk01.img.
# gen_bios.py and gen_disk.py import this so the BIOS's
# INT 13h describes the same disk the image was built for.
SPT = 18
HEADS = 2
SECTORS = 2880
