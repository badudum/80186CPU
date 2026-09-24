#!/usr/bin/env python3
"""Add or replace one file in a FAT12 image, in place.

    python3 tools/addfile.py disk.img BENCH.BAS local/BENCH.BAS

Written because rebuilding a 16 MB image through mkdisk.py to change one
small file means re-laying every cluster, and because editing the image by
hand is how a directory entry ends up disagreeing with its FAT chain.
Replacing an existing file frees its old chain first, so repeated runs do not
leak clusters -- which they silently did the first time this was done inline.
"""
import struct
import sys

SECTOR = 512


def main(path, name, src):
    img = bytearray(open(path, "rb").read())
    res = struct.unpack_from("<H", img, 14)[0]
    nfat = img[16]
    rootn = struct.unpack_from("<H", img, 17)[0]
    spf = struct.unpack_from("<H", img, 22)[0]
    spc = img[13]
    root = res + nfat * spf
    data = root + (rootn * 32 + SECTOR - 1) // SECTOR
    fat = res * SECTOR
    total = struct.unpack_from("<H", img, 19)[0] or \
        struct.unpack_from("<I", img, 32)[0]
    ccount = (total - data) // spc

    def get(c):
        o = (c * 3) // 2
        v = img[fat + o] | (img[fat + o + 1] << 8)
        return (v >> 4) if (c & 1) else (v & 0xFFF)

    def put(c, val):
        o = (c * 3) // 2
        v = img[fat + o] | (img[fat + o + 1] << 8)
        v = ((val << 4) | (v & 0x000F)) if (c & 1) else \
            ((v & 0xF000) | (val & 0xFFF))
        for f in range(nfat):                      # every copy, not just the first
            b = (res + f * spf) * SECTOR + o
            img[b] = v & 0xFF
            img[b + 1] = (v >> 8) & 0xFF

    ent = name.split(".")[0].ljust(8)[:8] + \
        (name.split(".")[1] if "." in name else "").ljust(3)[:3]
    ent = ent.upper().encode()

    slot = None
    for i in range(rootn):
        e = root * SECTOR + i * 32
        if img[e:e + 11] == ent:                   # replacing: free the old chain
            c = struct.unpack_from("<H", img, e + 26)[0]
            while 2 <= c < 0xFF0:
                nxt = get(c)
                put(c, 0)
                c = nxt
            slot = e
            break
        if slot is None and img[e] in (0x00, 0xE5):
            slot = e
    if slot is None:
        raise SystemExit("root directory full")

    body = open(src, "rb").read()
    free = [c for c in range(2, ccount + 2) if get(c) == 0]
    need = max(1, (len(body) + spc * SECTOR - 1) // (spc * SECTOR))
    if len(free) < need:
        raise SystemExit("not enough free clusters")
    use = free[:need]
    for i, c in enumerate(use):
        off = (data + (c - 2) * spc) * SECTOR
        img[off:off + spc * SECTOR] = b"\x00" * (spc * SECTOR)
        chunk = body[i * spc * SECTOR:(i + 1) * spc * SECTOR]
        img[off:off + len(chunk)] = chunk
        put(c, 0xFFF if i == need - 1 else use[i + 1])

    img[slot:slot + 11] = ent
    img[slot + 11] = 0x20
    img[slot + 12:slot + 26] = b"\x00" * 14
    struct.pack_into("<H", img, slot + 26, use[0])
    struct.pack_into("<I", img, slot + 28, len(body))
    open(path, "wb").write(img)
    print("%s: %s, %d bytes in %d cluster(s)" % (path, name, len(body), need))


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2], sys.argv[3])
