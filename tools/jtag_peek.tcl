# ---------------------------------------------------------------------------
# Read words back out of SDRAM over JTAG.
#
#   quartus_stp -t tools/jtag_peek.tcl 0x100000 16
#
# Uses the loader's PEEK instructions; the CPU is left running and untouched,
# so this is safe to use on a machine that is mid-boot.
#
# WHY IT POLLS. PEEK asks the system side for a word and the answer arrives
# some system clocks later, across a clock crossing from TCK. Assuming it has
# arrived by the next scan is a race -- and one that was observed going both
# ways in a single debugging session, so readback was sometimes one word stale
# and sometimes not. That is worse than no readback at all, because it silently
# misattributes every value to the wrong address. So each result carries a
# sequence number, and this waits for it to change.
#
#   IR 5 PEEK   request the word at the pointer, advance the pointer, and
#               return the PREVIOUS {seq, data}
#   IR 6 PEEKD  return {seq, data} with no side effect -- safe to poll
# ---------------------------------------------------------------------------
set IR_ADDR  1
set IR_PEEK  5
set IR_PEEKD 6

if {[llength $quartus(args)] < 1} {
    puts "usage: quartus_stp -t tools/jtag_peek.tcl <address> \[count]"
    exit 1
}
set base [expr {[lindex $quartus(args) 0]}]
set count 8
if {[llength $quartus(args)] > 1} { set count [lindex $quartus(args) 1] }

set cable [lindex [get_hardware_names] 0]
set device ""
foreach d [get_device_names -hardware_name $cable] {
    if {[string match "*5CSE*" $d]} { set device $d }
}
if {$device eq ""} { puts "error: no FPGA found"; exit 1 }

# Returns {seq data}, both as integers, without disturbing the pointer.
proc peek_data {} {
    device_virtual_ir_shift -instance_index 0 -ir_value 6 -no_captured_ir_value
    set v [device_virtual_dr_shift -instance_index 0 -length 32 \
           -dr_value "00000000" -value_in_hex]
    set n [expr {"0x$v"}]
    return [list [expr {($n >> 16) & 0xFF}] [expr {$n & 0xFFFF}]]
}

# Reads the word at the pointer and advances it.
proc peek_next {} {
    lassign [peek_data] seq0 d
    device_virtual_ir_shift -instance_index 0 -ir_value 5 -no_captured_ir_value
    device_virtual_dr_shift -instance_index 0 -length 32 -dr_value "00000000" \
        -value_in_hex
    for {set t 0} {$t < 50} {incr t} {
        lassign [peek_data] seq d
        if {$seq != $seq0} { return $d }
    }
    error "PEEK never completed -- the system side is not answering"
}

proc peek_set_addr {a} {
    device_virtual_ir_shift -instance_index 0 -ir_value 1 -no_captured_ir_value
    device_virtual_dr_shift -instance_index 0 -length 24 \
        -dr_value [format "%06X" $a] -value_in_hex
}

if {[info exists quartus(args)] && [llength $quartus(args)] > 0} {
    open_device -hardware_name $cable -device_name $device
    device_lock -timeout 10000

    peek_set_addr $base
    set out ""
    for {set i 0} {$i < $count} {incr i} {
        append out [format "%04X " [peek_next]]
    }

    device_unlock
    close_device
    puts [format "PEEK %06X: %s" $base $out]
}
