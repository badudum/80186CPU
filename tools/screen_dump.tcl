# ---------------------------------------------------------------------------
# Read the board's text buffer back over JTAG and save it.
#
#   quartus_stp -t tools/screen_dump.tcl
#   python3 tools/screen_dump.py
#
# The build must have ISMCE set, which makes the two VRAM byte banks (VRML for
# characters, VRMH for attributes) visible to the In-System Memory Content
# Editor. That is what lets the screen be inspected with no monitor attached --
# the only way to see what the machine actually printed when debugging it from
# somewhere else.
#
# This reads only; it never writes the buffer.
# ---------------------------------------------------------------------------

set outdir "/tmp/fpga80186_screen"
file mkdir $outdir

set cables [get_hardware_names]
if {[llength $cables] == 0} {
    puts "error: no programming cable found."
    exit 1
}
set cable [lindex $cables 0]

set device ""
foreach d [get_device_names -hardware_name $cable] {
    if {[string match "*5CSE*" $d] || [string match "*EP*" $d]} { set device $d }
}
if {$device eq ""} { puts "error: no FPGA found in the chain."; exit 1 }

set insts [get_editable_mem_instances -hardware_name $cable -device_name $device]

# {index depth width mode type name} -- the name is last.
proc index_of {insts want} {
    foreach inst $insts {
        if {[string equal -nocase [lindex $inst 5] $want]} { return [lindex $inst 0] }
    }
    return -1
}

set lo [index_of $insts VRML]
set hi [index_of $insts VRMH]

if {$lo < 0 || $hi < 0} {
    puts "error: VRML/VRMH not found among the editable memories."
    puts "       The programmed build needs ISMCE set. Found:"
    foreach inst $insts { puts "         [lindex $inst 5]" }
    exit 1
}

begin_memory_edit -hardware_name $cable -device_name $device
save_content_from_memory_to_file -instance_index $lo \
    -mem_file_path $outdir/vram_lo.mif -mem_file_type mif
save_content_from_memory_to_file -instance_index $hi \
    -mem_file_path $outdir/vram_hi.mif -mem_file_type mif
end_memory_edit

puts "saved $outdir/vram_lo.mif and $outdir/vram_hi.mif"
puts "now run: python3 tools/screen_dump.py"
