# ---------------------------------------------------------------------------
# Read the CPU profile ring out of the board.
#
#   quartus_stp -t tools/jtag_prof.tcl
#
# Sixteen {CS, IP} samples taken in HARDWARE, one every 2^20 clocks -- about
# two thirds of a second of real execution. The CPU is not disturbed.
#
# WHY IN HARDWARE. Sampling in the BIOS's timer interrupt is the obvious way
# and it does not work once a guest operating system is running: MS-DOS hooks
# INT 08h and chains to the previous handler, so the return address the BIOS
# sees is MS-DOS's own chaining call every single time. The profile comes back
# perfectly consistent and describes nothing but the profiler. This reads the
# CPU's CS:IP straight off the register file instead.
#
# Sampling is frozen while the ring is read so the sixteen samples belong to
# one window rather than being overwritten mid-scan.
# ---------------------------------------------------------------------------
set IR_CTRL 3
set IR_PROF 7

set cable [lindex [get_hardware_names] 0]
set device ""
foreach d [get_device_names -hardware_name $cable] {
    if {[string match "*5CSE*" $d]} { set device $d }
}
if {$device eq ""} { puts "error: no FPGA found"; exit 1 }

open_device -hardware_name $cable -device_name $device
device_lock -timeout 10000

# CTRL bit 1 freezes sampling; bit 0 (CPU hold) stays clear.
device_virtual_ir_shift -instance_index 0 -ir_value $IR_CTRL -no_captured_ir_value
device_virtual_dr_shift -instance_index 0 -length 8 -dr_value "02" -value_in_hex

device_virtual_ir_shift -instance_index 0 -ir_value $IR_PROF -no_captured_ir_value
set samples {}
for {set i 0} {$i < 16} {incr i} {
    set v [device_virtual_dr_shift -instance_index 0 -length 32 \
           -dr_value "00000000" -value_in_hex]
    lappend samples $v
}

device_virtual_ir_shift -instance_index 0 -ir_value $IR_CTRL -no_captured_ir_value
device_virtual_dr_shift -instance_index 0 -length 8 -dr_value "00" -value_in_hex

device_unlock
close_device

puts "CPU profile, 16 samples over ~0.65 s:"
array set seen {}
foreach v $samples {
    set n [expr {"0x$v"}]
    set cs [format "%04X" [expr {($n >> 16) & 0xFFFF}]]
    set ip [format "%04X" [expr {$n & 0xFFFF}]]
    puts "  $cs:$ip"
    incr seen($cs)
}
puts ""
puts "by segment:"
foreach {k n} [array get seen] { puts [format "  %s  %d" $k $n] }
