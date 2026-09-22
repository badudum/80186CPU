#!/usr/bin/env python3
"""
Tests for the FAT12 builder.

The reader below is written from the on-disk format, not from the builder: it
takes nothing but the image bytes, reads the BPB to find everything else, and
walks the FAT the way the boot sector does. That independence is the point --
a test that asked the builder where it put things would agree with itself
whatever it had written.

The 12-bit packing gets specific attention because it is where FAT12 is easy to
get wrong: entries alternate between the low and high nibble of a shared byte,
so an implementation can be right for even clusters and wrong for odd ones and
still pass any test that only ever reads a short file.

Run:  python3 tools/test_fat12.py
"""
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from fat12 import Fat12, Fat12Error, SECTOR

fails = 0
count = 0


def chk(desc, got, want):
    global fails, count
    count += 1
    if got != want:
        print("FAIL %-52s got %r want %r" % (desc, got, want))
        fails += 1


def chk_true(desc, cond):
    chk(desc, bool(cond), True)


def expect_error(desc, fn):
    global fails, count
    count += 1
    try:
        fn()
    except Fat12Error:
        return
    print("FAIL %s did not raise" % desc)
    fails += 1


# ---------------------------------------------------------------------------
# An independent FAT12 reader: image bytes in, files out.
# ---------------------------------------------------------------------------
class Reader:
    def __init__(self, img):
        self.img = img
        b = img

        def u16(o):
            return b[o] | (b[o + 1] << 8)

        if b[510] != 0x55 or b[511] != 0xAA:
            raise ValueError("no boot signature")

        self.bytes_per_sector = u16(11)
        self.sectors_per_cluster = b[13]
        self.reserved = u16(14)
        self.num_fats = b[16]
        self.root_entries = u16(17)
        self.total_sectors = u16(19)
        self.media = b[21]
        self.sectors_per_fat = u16(22)
        self.spt = u16(24)
        self.heads = u16(26)

        self.root_start = self.reserved + self.num_fats * self.sectors_per_fat
        self.root_sectors = (self.root_entries * 32) // self.bytes_per_sector
        self.data_start = self.root_start + self.root_sectors

        fo = self.reserved * self.bytes_per_sector
        self.fat = b[fo:fo + self.sectors_per_fat * self.bytes_per_sector]

    def fat_entry(self, n):
        """Unpack the 12-bit entry for cluster n."""
        off = (n * 3) // 2
        if n % 2 == 0:
            return self.fat[off] | ((self.fat[off + 1] & 0x0F) << 8)
        return (self.fat[off] >> 4) | (self.fat[off + 1] << 4)

    def entries(self):
        out = []
        base = self.root_start * self.bytes_per_sector
        for i in range(self.root_entries):
            e = self.img[base + i * 32:base + i * 32 + 32]
            if e[0] in (0x00, 0xE5):
                continue
            name = e[0:8].decode("ascii").rstrip()
            ext = e[8:11].decode("ascii").rstrip()
            out.append({
                "name": name + ("." + ext if ext else ""),
                "attrs": e[11],
                "cluster": e[26] | (e[27] << 8),
                "size": int.from_bytes(e[28:32], "little"),
            })
        return out

    def read(self, name):
        for e in self.entries():
            if e["name"] == name.upper():
                data = b""
                c = e["cluster"]
                guard = 0
                while 2 <= c < 0xFF0:
                    lba = self.data_start + (c - 2) * self.sectors_per_cluster
                    off = lba * self.bytes_per_sector
                    data += self.img[off:off + self.bytes_per_sector
                                     * self.sectors_per_cluster]
                    c = self.fat_entry(c)
                    guard += 1
                    if guard > self.total_sectors:
                        raise ValueError("cluster chain does not terminate")
                return data[:e["size"]]
        raise KeyError(name)


# ---------------------------------------------------------------------------
# Structure
# ---------------------------------------------------------------------------
fs = Fat12(total_sectors=256, sectors_per_track=16, heads=4)
fs.set_boot_code(b"\x90" * 64)
img = fs.image()

chk("image is the declared size", len(img), 256 * SECTOR)

r = Reader(img)
chk("bytes per sector", r.bytes_per_sector, 512)
chk("sectors per cluster", r.sectors_per_cluster, 1)
chk("reserved sectors", r.reserved, 1)
chk("number of FATs", r.num_fats, 2)
chk("total sectors", r.total_sectors, 256)
chk("media descriptor", r.media, 0xF8)
chk("sectors per track survives", r.spt, 16)
chk("heads survives", r.heads, 4)
chk("fs type string", img[54:62], b"FAT12   ")
chk("extended boot signature", img[38], 0x29)

# The jump has to land exactly on the boot code, or the BPB executes.
chk("jump over the BPB", img[0:3], bytes([0xEB, 0x3C, 0x90]))
chk("boot code sits at 3Eh", img[0x3E], 0x90)

chk("FAT[0] carries the media byte", r.fat_entry(0), 0xF00 | 0xF8)
chk("FAT[1] is the end marker", r.fat_entry(1), 0xFFF)

# Both FAT copies must be identical, or a tool that reads the second one sees
# a different filesystem.
f0 = r.reserved * SECTOR
f1 = (r.reserved + r.sectors_per_fat) * SECTOR
n = r.sectors_per_fat * SECTOR
chk("the two FAT copies match", img[f0:f0 + n], img[f1:f1 + n])

# ---------------------------------------------------------------------------
# Files round-trip
# ---------------------------------------------------------------------------
fs = Fat12(total_sectors=256, sectors_per_track=16, heads=4)
fs.set_boot_code(b"\xEB\xFE")

short = b"hello from a FAT12 volume"
# Long enough to need several clusters, so the chain is actually followed, and
# with a distinct byte per sector so a mis-ordered chain is visible.
long = b"".join(bytes([0x41 + i]) * SECTOR for i in range(5))
exact = b"X" * SECTOR                       # exactly one cluster, no slack

fs.add_file("SHORT.TXT", short)
fs.add_file("KERNEL.BIN", long)
fs.add_file("EXACT.DAT", exact)
img = fs.image()

r = Reader(img)
names = sorted(e["name"] for e in r.entries())
chk("directory lists every file", names, ["EXACT.DAT", "KERNEL.BIN", "SHORT.TXT"])

chk("short file round-trips", r.read("SHORT.TXT"), short)
chk("multi-cluster file round-trips", r.read("KERNEL.BIN"), long)
chk("exactly-one-cluster file round-trips", r.read("EXACT.DAT"), exact)

for e in r.entries():
    if e["name"] == "KERNEL.BIN":
        chk("file size recorded", e["size"], len(long))
        chk_true("start cluster is a data cluster", e["cluster"] >= 2)

# ---------------------------------------------------------------------------
# 12-bit packing, on both alignments
# ---------------------------------------------------------------------------
# Walk KERNEL.BIN's chain by hand and confirm it is contiguous and terminated.
start = [e for e in r.entries() if e["name"] == "KERNEL.BIN"][0]["cluster"]
chain = []
c = start
while 2 <= c < 0xFF0:
    chain.append(c)
    c = r.fat_entry(c)
chk("chain has one cluster per sector of the file", len(chain), 5)
chk("chain is contiguous", chain, list(range(start, start + 5)))
chk("chain terminates with EOF", c, 0xFFF)
chk_true("chain covers both even and odd clusters",
         any(x % 2 == 0 for x in chain) and any(x % 2 == 1 for x in chain))

# Directly: write a known pattern into adjacent entries and read it back. An
# implementation that shares the middle byte wrongly corrupts its neighbour.
fs2 = Fat12(total_sectors=256, sectors_per_track=16, heads=4)
fs2.set_boot_code(b"\xEB\xFE")
for i in range(2, 12):
    fs2.fat[i] = 0xABC if i % 2 == 0 else 0x123
img2 = fs2.image()
r2 = Reader(img2)
ok = all(r2.fat_entry(i) == (0xABC if i % 2 == 0 else 0x123) for i in range(2, 12))
chk_true("adjacent 12-bit entries do not corrupt each other", ok)

# ---------------------------------------------------------------------------
# Refusals
# ---------------------------------------------------------------------------
expect_error("a name that does not fit 8.3",
             lambda: Fat12(256, 16, 4).add_file("FAR.TOO.LONG.NAME", b"x"))
expect_error("boot code larger than the sector",
             lambda: Fat12(256, 16, 4).set_boot_code(b"\x90" * 500))
expect_error("a volume too small to format", lambda: Fat12(4, 16, 4))
expect_error("a file with nowhere to go",
             lambda: Fat12(32, 16, 4).add_file("BIG.BIN", b"x" * 200000))


def duplicate():
    f = Fat12(256, 16, 4)
    f.add_file("A.TXT", b"1")
    f.add_file("a.txt", b"2")


expect_error("the same name twice", duplicate)

print()
print("=" * 46)
print(" fat12: %d checks, %d failed" % (count, fails))
print("=" * 46)
sys.exit(1 if fails else 0)
