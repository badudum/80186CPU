# ---------------------------------------------------------------------------
# Push a disk image into the board's SDRAM over the USB-Blaster.
#
#   quartus_stp -t tools/jtag_load.tcl ms-dos/disk01.img
#   quartus_stp -t tools/jtag_load.tcl ms-dos/disk01.img 0x100000
#
# The FPGA must already be programmed:
#   quartus_pgm -m jtag -o "p;output_files/FPGA80186.sof@2"
#
# and that build must have DISK_IN_SDRAM set, or the device reads from the
# on-chip ROM and whatever is written here is ignored.
#
# WHAT IT DOES. Talks to the virtual JTAG node in modules/jtag_loader.sv:
#   IR 1  set the write pointer          IR 3  hold/release the CPU
#   IR 2  stream 16-bit words            IR 4  read status back
#
# The CPU is held in reset for the whole load, so it cannot execute out of
# memory being rewritten underneath it, and released at the end -- which also
# restarts the machine, so it boots from the image just written.
#
# CHUNKING. Words are streamed thousands at a time in one DR scan. A scan per
# word would be correct and unusably slow: 1.44 MB is 737,280 words, and at
# even a millisecond of host overhead each that is hours. One scan per chunk
# turns it into seconds.
#
# FRAMING. Each scan is prefixed with sixteen zero bits and a SYNC word before
# the data. The loader has no way to know where the host's bits begin -- the
# SLD hub and every other SLD node in the chain shift junk in ahead of them --
# so it hunts for SYNC and frames words from there. Skip the prefix and every
# word of the image lands rotated. See modules/jtag_loader.sv.
#
# VERIFICATION IS NOT OPTIONAL. JTAG cannot be stalled mid-scan, so if TCK
# outruns the SDRAM writer a word is dropped. The loader latches that as an
# overflow bit and counts what it actually wrote; this script reads both back
# and fails loudly rather than leaving a silently corrupt disk image, which
# would show up later as unexplained filesystem damage.
# ---------------------------------------------------------------------------

set IR_ADDR   1
set IR_DATA   2
set IR_CTRL   3
set IR_STATUS 4

# Words per DR scan. Larger is faster but builds a longer hex string in Tcl.
set CHUNK_WORDS 2048

# Must match SYNC in modules/jtag_loader.sv.
set SYNC "B2C1"

proc usage {} {
    puts "usage: quartus_stp -t tools/jtag_load.tcl <image> \[base_address]"
    puts "       base address defaults to 0x100000, where storage.sv expects"
    puts "       sector 0 to live."
    exit 1
}

if {[llength $quartus(args)] < 1} { usage }
set imgfile [lindex $quartus(args) 0]
set base 0x100000
if {[llength $quartus(args)] > 1} { set base [lindex $quartus(args) 1] }

if {![file exists $imgfile]} {
    puts "error: $imgfile does not exist"
    exit 1
}

# ---- find the cable and device ----
set cables [get_hardware_names]
if {[llength $cables] == 0} {
    puts "error: no programming cable found. Is the USB-Blaster plugged in?"
    exit 1
}
set cable [lindex $cables 0]
puts "cable: $cable"

set devices [get_device_names -hardware_name $cable]
set device ""
foreach d $devices {
    # Skip the HPS/SoC ARM node; the FPGA is the one with a virtual JTAG node.
    if {[string match "*5CSE*" $d] || [string match "*EP*" $d]} { set device $d }
}
if {$device eq ""} { set device [lindex $devices end] }
puts "device: $device"

# ---- read the image ----
set f [open $imgfile rb]
fconfigure $f -translation binary
set data [read $f]
close $f

set nbytes [string length $data]
if {$nbytes % 2} { append data "\x00"; incr nbytes }
set nwords [expr {$nbytes / 2}]
puts [format "image : %s, %d bytes, %d words, %d sectors" \
      $imgfile $nbytes $nwords [expr {($nbytes + 511) / 512}]]
puts [format "base  : 0x%06X" $base]

# ---- helpers ----
# device_virtual_dr_shift takes the value as a hex string, least significant
# bit first in the shifted order.
proc shift_ir {dev ir} {
    device_virtual_ir_shift -instance_index 0 -ir_value $ir -no_captured_ir_value
}

proc shift_dr {dev len value} {
    return [device_virtual_dr_shift -instance_index 0 -length $len \
            -dr_value $value -value_in_hex]
}

# ---- do it ----
open_device -hardware_name $cable -device_name $device

set t0 [clock milliseconds]

device_lock -timeout 10000

# Hold the CPU while memory is rewritten underneath it.
shift_ir $device $IR_CTRL
shift_dr $device 8 "01"

# Set the write pointer. This also zeroes the word counter and clears any
# overflow left from a previous attempt.
shift_ir $device $IR_ADDR
shift_dr $device 24 [format "%06X" $base]

# Stream the words.
shift_ir $device $IR_DATA
set sent 0
while {$sent < $nwords} {
    set n [expr {min($CHUNK_WORDS, $nwords - $sent)}]

    # Build the chunk least-significant word first, which is the order the
    # loader shifts them in.
    set hex ""
    for {set i [expr {$n - 1}]} {$i >= 0} {incr i -1} {
        set idx [expr {($sent + $i) * 2}]
        binary scan [string range $data $idx [expr {$idx + 1}]] cucu lo hi
        append hex [format "%02X%02X" $hi $lo]
    }

    # The prefix goes at the LEAST significant end, because that is what is
    # shifted out first: sixteen zeros to flush the chain's lead-in, then SYNC.
    append hex $SYNC "0000"

    shift_dr $device [expr {($n + 2) * 16}] $hex
    incr sent $n

    if {$sent % 65536 == 0 || $sent == $nwords} {
        puts [format "  %d / %d words (%.0f%%)" $sent $nwords \
              [expr {100.0 * $sent / $nwords}]]
    }
}

# ---- verify ----
shift_ir $device $IR_STATUS
set st [shift_dr $device 32 "00000000"]
set stv [expr {"0x$st"}]
set written [expr {$stv & 0xFFFFFF}]
set overflow [expr {($stv >> 31) & 1}]

# ---- read the image back ----
# The counters above only say that the right NUMBER of words was accepted.
# They said exactly that while every word in memory was rotated by seven bits,
# because the framing was off -- so the count is not evidence that the image is
# right. This reads real words back out and compares them, which is.
proc peek_data {} {
    device_virtual_ir_shift -instance_index 0 -ir_value 6 -no_captured_ir_value
    set v [device_virtual_dr_shift -instance_index 0 -length 32 \
           -dr_value "00000000" -value_in_hex]
    set n [expr {"0x$v"}]
    return [list [expr {($n >> 16) & 0xFF}] [expr {$n & 0xFFFF}]]
}
proc peek_next {} {
    lassign [peek_data] seq0 d
    device_virtual_ir_shift -instance_index 0 -ir_value 5 -no_captured_ir_value
    device_virtual_dr_shift -instance_index 0 -length 32 -dr_value "00000000" \
        -value_in_hex
    for {set t 0} {$t < 50} {incr t} {
        lassign [peek_data] seq d
        if {$seq != $seq0} { return $d }
    }
    return -1
}

set bad 0
set spots {0 1 2 3 255 256 1000 100000 368639}
foreach w $spots {
    if {$w >= $nwords} { continue }
    shift_ir $device $IR_ADDR
    shift_dr $device 24 [format "%06X" [expr {$base + $w * 2}]]
    set got [peek_next]
    set idx [expr {$w * 2}]
    binary scan [string range $data $idx [expr {$idx + 1}]] cucu lo hi
    set exp [expr {($hi << 8) | $lo}]
    if {$got != $exp} {
        puts [format "  MISMATCH at word %d (0x%06X): read %04X, expected %04X" \
              $w [expr {$base + $w * 2}] $got $exp]
        incr bad
    }
}

# Setting the address for the spot checks reset the word counter, so report
# what the load itself measured, not what is in the counter now.

# Release the CPU, which restarts the machine on the image just written.
shift_ir $device $IR_CTRL
shift_dr $device 8 "00"

device_unlock
close_device

set secs [expr {([clock milliseconds] - $t0) / 1000.0}]
puts [format "wrote %d words in %.1f s" $written $secs]

if {$bad} {
    puts "ERROR: $bad of [llength $spots] spot checks read back wrong."
    puts "       The image in memory does not match the file; do not trust it."
    exit 1
}
puts "readback: [llength $spots] spot checks match"

if {$overflow} {
    puts "ERROR: the loader reported an OVERFLOW."
    puts "       TCK is outrunning the SDRAM writer and words were dropped."
    puts "       Lower the cable's clock and retry:"
    puts "         jtagconfig --setparam \"$cable\" JtagClock 6M"
    exit 1
}
if {$written != $nwords} {
    puts "ERROR: the loader wrote $written words, expected $nwords."
    puts "       The image in memory is incomplete; do not trust it."
    exit 1
}

puts "OK: image loaded and verified, CPU released."
