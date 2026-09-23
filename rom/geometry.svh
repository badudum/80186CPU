// Written by tools/gen_disk.py -- do not edit.
// The disk geometry the BIOS's INT 13h and the boot sector's
// BPB both use. Testbenches include this so there is one
// source of truth rather than a constant per file.
localparam int GEO_SPT     = 63;
localparam int GEO_HEADS   = 16;
localparam int GEO_SECTORS = 256;
// What INT 13h AH=08 reports: the geometry of the disk the
// BIOS was built for, which may be larger than this image.
localparam int GEO_BIOS_CYLS = 32;
