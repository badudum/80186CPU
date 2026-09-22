#!/usr/bin/env python3
"""
Build a FAT12 volume image.

This exists so the block device holds a real filesystem rather than a blob the
boot sector knows the layout of by magic. The point is that the on-disk
structures are the standard ones: a BIOS parameter block the boot sector reads
at runtime, a FAT with 12-bit packed entries, a fixed-size root directory of
32-byte entries, and a data area addressed in clusters. Nothing here is a
private convention, so the boot code is a real FAT12 loader and the same image
would be readable by anything else that speaks FAT12.

GEOMETRY MUST MATCH THE DEVICE. The BPB records sectors-per-track and heads,
and the boot sector uses them to turn a logical sector number into the CHS
triple INT 13h wants. If those disagree with what the BIOS's INT 13h actually
does, every read lands on the wrong sector -- so the caller passes the real
geometry in and nothing is guessed.

ONE SECTOR PER CLUSTER. Deliberate: it makes "cluster N" and "logical sector
data_start + N - 2" the same arithmetic, which keeps the boot sector small.
The format allows more and this builder refuses anything else rather than
emitting an image the boot code cannot walk.
"""

SECTOR = 512
DIR_ENTRY = 32

ATTR_READ_ONLY = 0x01
ATTR_HIDDEN = 0x02
ATTR_SYSTEM = 0x04
ATTR_VOLUME_ID = 0x08
ATTR_ARCHIVE = 0x20


class Fat12Error(Exception):
    pass


def _name_to_83(name):
    """'KERNEL.BIN' -> the 11 raw bytes a directory entry stores."""
    if "." in name:
        base, ext = name.split(".", 1)
    else:
        base, ext = name, ""
    base = base.upper()
    ext = ext.upper()
    if len(base) > 8 or len(ext) > 3:
        raise Fat12Error("%r does not fit 8.3" % name)
    return (base.ljust(8) + ext.ljust(3)).encode("ascii")


class Fat12:
    def __init__(self, total_sectors, sectors_per_track, heads,
                 root_entries=112, media=0xF8, label="FPGA80186",
                 oem="FPGA8018", serial=0x80186FA7):
        if total_sectors < 16:
            raise Fat12Error("volume too small to hold a filesystem")
        if root_entries % (SECTOR // DIR_ENTRY):
            raise Fat12Error("root_entries must be a whole number of sectors")

        self.total_sectors = total_sectors
        self.spt = sectors_per_track
        self.heads = heads
        self.root_entries = root_entries
        self.media = media
        self.label = label
        self.oem = oem
        self.serial = serial

        self.reserved = 1
        self.num_fats = 2
        self.sectors_per_cluster = 1

        self.root_sectors = (root_entries * DIR_ENTRY) // SECTOR

        # Sizing the FAT is mildly circular: the number of clusters depends on
        # how many sectors the FATs take, which depends on the cluster count.
        # Solve it by trying sizes until one is big enough for the clusters it
        # leaves behind.
        self.sectors_per_fat = 1
        while True:
            data_sectors = (total_sectors - self.reserved
                            - self.num_fats * self.sectors_per_fat
                            - self.root_sectors)
            if data_sectors <= 0:
                raise Fat12Error("volume too small once the FAT is accounted for")
            clusters = data_sectors // self.sectors_per_cluster
            # Entries 0 and 1 are reserved, so the table holds clusters + 2.
            need_bytes = ((clusters + 2) * 3 + 1) // 2
            need = (need_bytes + SECTOR - 1) // SECTOR
            if need <= self.sectors_per_fat:
                break
            self.sectors_per_fat = need

        self.data_start = (self.reserved + self.num_fats * self.sectors_per_fat
                           + self.root_sectors)
        self.cluster_count = ((total_sectors - self.data_start)
                              // self.sectors_per_cluster)
        if self.cluster_count >= 4085:
            raise Fat12Error("too many clusters for FAT12 (%d)" % self.cluster_count)

        # FAT entries 0 and 1 carry the media descriptor and an end marker.
        self.fat = [0] * (self.cluster_count + 2)
        self.fat[0] = 0xF00 | self.media
        self.fat[1] = 0xFFF

        self.dirents = []
        self.clusters = {}          # cluster number -> 512 bytes
        self.next_free = 2
        self.boot_code = b""

    # ---- construction ----
    def set_boot_code(self, code):
        """Bytes that will sit at offset 3Eh of the boot sector."""
        if len(code) > SECTOR - 0x3E - 2:
            raise Fat12Error("boot code is %d bytes, %d available"
                             % (len(code), SECTOR - 0x3E - 2))
        self.boot_code = code

    def add_file(self, name, data, attrs=ATTR_ARCHIVE):
        raw = _name_to_83(name)
        if any(d[:11] == raw for d in self.dirents):
            raise Fat12Error("%s already exists" % name)
        if len(self.dirents) >= self.root_entries:
            raise Fat12Error("root directory is full")

        need = max(1, (len(data) + SECTOR - 1) // SECTOR)
        if self.next_free + need > self.cluster_count + 2:
            raise Fat12Error("no room for %s (%d sectors)" % (name, need))

        first = self.next_free
        for i in range(need):
            c = first + i
            chunk = data[i * SECTOR:(i + 1) * SECTOR]
            self.clusters[c] = chunk.ljust(SECTOR, b"\0")
            # Chain to the next cluster, or mark the end of the file.
            self.fat[c] = (c + 1) if i + 1 < need else 0xFFF
        self.next_free += need

        e = bytearray(DIR_ENTRY)
        e[0:11] = raw
        e[11] = attrs
        e[22:24] = (0x6000).to_bytes(2, "little")     # time, arbitrary
        e[24:26] = (0x5A21).to_bytes(2, "little")     # date, arbitrary
        e[26:28] = first.to_bytes(2, "little")
        e[28:32] = len(data).to_bytes(4, "little")
        self.dirents.append(bytes(e))
        return first

    # ---- emission ----
    def _boot_sector(self):
        b = bytearray(SECTOR)
        # A short jump over the BPB to the code at 3Eh, then the NOP that
        # every FAT boot sector carries in the third byte.
        b[0:3] = bytes([0xEB, 0x3C, 0x90])
        b[3:11] = self.oem.encode("ascii").ljust(8)[:8]
        b[11:13] = SECTOR.to_bytes(2, "little")
        b[13] = self.sectors_per_cluster
        b[14:16] = self.reserved.to_bytes(2, "little")
        b[16] = self.num_fats
        b[17:19] = self.root_entries.to_bytes(2, "little")
        b[19:21] = self.total_sectors.to_bytes(2, "little")
        b[21] = self.media
        b[22:24] = self.sectors_per_fat.to_bytes(2, "little")
        b[24:26] = self.spt.to_bytes(2, "little")
        b[26:28] = self.heads.to_bytes(2, "little")
        b[28:32] = (0).to_bytes(4, "little")          # hidden sectors
        b[32:36] = (0).to_bytes(4, "little")          # total sectors, 32-bit
        b[36] = 0x00                                  # drive number
        b[37] = 0x00
        b[38] = 0x29                                  # extended boot signature
        b[39:43] = self.serial.to_bytes(4, "little")
        b[43:54] = self.label.encode("ascii").ljust(11)[:11]
        b[54:62] = b"FAT12   "
        b[0x3E:0x3E + len(self.boot_code)] = self.boot_code
        b[510] = 0x55
        b[511] = 0xAA
        return bytes(b)

    def _fat_bytes(self):
        """Pack the 12-bit entries, two into every three bytes."""
        out = bytearray(self.sectors_per_fat * SECTOR)
        for i in range(0, len(self.fat), 2):
            lo = self.fat[i] & 0xFFF
            hi = self.fat[i + 1] & 0xFFF if i + 1 < len(self.fat) else 0
            off = (i * 3) // 2
            out[off] = lo & 0xFF
            out[off + 1] = ((lo >> 8) & 0x0F) | ((hi & 0x0F) << 4)
            out[off + 2] = (hi >> 4) & 0xFF
        return bytes(out)

    def image(self):
        img = bytearray(self.total_sectors * SECTOR)

        img[0:SECTOR] = self._boot_sector()

        fat = self._fat_bytes()
        for n in range(self.num_fats):
            start = (self.reserved + n * self.sectors_per_fat) * SECTOR
            img[start:start + len(fat)] = fat

        root_start = (self.reserved + self.num_fats * self.sectors_per_fat) * SECTOR
        for i, e in enumerate(self.dirents):
            off = root_start + i * DIR_ENTRY
            img[off:off + DIR_ENTRY] = e

        for c, data in self.clusters.items():
            lba = self.data_start + (c - 2) * self.sectors_per_cluster
            img[lba * SECTOR:(lba + 1) * SECTOR] = data

        return bytes(img)

    def describe(self):
        return (
            "FAT12: %d sectors, %d reserved, %d FATs x %d sectors, "
            "%d root entries (%d sectors), data from sector %d, %d clusters"
            % (self.total_sectors, self.reserved, self.num_fats,
               self.sectors_per_fat, self.root_entries, self.root_sectors,
               self.data_start, self.cluster_count))
