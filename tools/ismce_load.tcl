# ---------------------------------------------------------------------------
# Replace an on-chip memory's contents in a running board, over JTAG.
#
#   quartus_stp -t tools/ismce_load.tcl                 # list what is editable
#   quartus_stp -t tools/ismce_load.tcl BIOL rom/bios.lo.mif
#   quartus_stp -t tools/ismce_load.tcl BIOH rom/bios.hi.mif
#   quartus_stp -t tools/ismce_load.tcl FONT rom/font.mif
#
# The FPGA must already be programmed with a build made with ISMCE set:
#   quartus_pgm -m jtag -o "p;output_files/FPGA80186.sof@2"
#
# WHY THIS IS USEFUL. Changing BIOS code otherwise means a full synthesis, fit
# and timing run -- fifteen minutes to test a one-line edit. This rewrites the
# ROM in place in seconds:
#
#   python3 tools/gen_bios.py rom/
#   python3 tools/hex2mif.py rom/bios.lo.hex rom/bios.lo.mif 8
#   python3 tools/hex2mif.py rom/bios.hi.hex rom/bios.hi.mif 8
#   quartus_stp -t tools/ismce_load.tcl BIOL rom/bios.lo.mif
#   quartus_stp -t tools/ismce_load.tcl BIOH rom/bios.hi.mif
#   ... then press KEY0 to reset the board and run the new BIOS.
#
# WHAT IT CANNOT DO. This reaches ON-CHIP memory only. The disk image lives in
# SDRAM when DISK_IN_SDRAM is set, precisely because it is too large for block
# RAM -- use tools/jtag_load.tcl for that.
#
# The board is NOT reset automatically. Rewriting a ROM under a running CPU
# leaves it executing whatever it had already fetched, so press the reset
# button afterwards.
# ---------------------------------------------------------------------------

proc find_device {} {
    set cables [get_hardware_names]
    if {[llength $cables] == 0} {
        puts "error: no programming cable found. Is the USB-Blaster plugged in?"
        exit 1
    }
    set cable [lindex $cables 0]

    set devices [get_device_names -hardware_name $cable]
    set device ""
    foreach d $devices {
        if {[string match "*5CSE*" $d] || [string match "*EP*" $d]} { set device $d }
    }
    if {$device eq ""} { set device [lindex $devices end] }
    return [list $cable $device]
}

lassign [find_device] cable device
puts "cable : $cable"
puts "device: $device"

set insts [get_editable_mem_instances -hardware_name $cable -device_name $device]

if {[llength $insts] == 0} {
    puts ""
    puts "No editable memories found."
    puts "The programmed build must have been made with ISMCE set:"
    puts "  set_parameter -name ISMCE 1      (in FPGA80186.qsf)"
    exit 1
}

# ---- no arguments: just report what is there ----
if {[llength $quartus(args)] < 2} {
    puts ""
    puts "Editable memories:"
    puts "  name     index  depth  width  mode  type"
    foreach inst $insts {
        # The list is {index depth width mode type name} -- the instance name
        # is LAST, which is worth stating because getting it wrong silently
        # prints the type instead and nothing matches by name.
        puts [format "  %-8s %-6s %-6s %-6s %-5s %s" \
              [lindex $inst 5] [lindex $inst 0] [lindex $inst 1] \
              [lindex $inst 2] [lindex $inst 3] [lindex $inst 4]]
    }
    puts ""
    puts "usage: quartus_stp -t tools/ismce_load.tcl <NAME> <file.mif>"
    exit 0
}

set want [lindex $quartus(args) 0]
set miffile [lindex $quartus(args) 1]

if {![file exists $miffile]} {
    puts "error: $miffile does not exist"
    exit 1
}

# ---- find the instance by name ----
set index -1
foreach inst $insts {
    if {[string equal -nocase [lindex $inst 5] $want]} {
        set index [lindex $inst 0]
    }
}
if {$index < 0} {
    puts "error: no editable memory called '$want'."
    puts "       Run with no arguments to list what is available."
    exit 1
}

puts "loading $miffile into '$want' (instance $index)"

begin_memory_edit -hardware_name $cable -device_name $device
if {[catch {
    update_content_to_memory_from_file \
        -instance_index $index -mem_file_path $miffile -mem_file_type mif
} err]} {
    end_memory_edit
    puts "error: $err"
    puts "       A width or depth mismatch between the .mif and the memory is"
    puts "       the usual cause; check the DEPTH and WIDTH lines in the file."
    exit 1
}
end_memory_edit

puts "OK: '$want' updated. Press KEY0 to reset the board and run it."
