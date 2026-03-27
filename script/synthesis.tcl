# ============================================================
# TPU_FP32_PIPELINE synthesis.tcl
# ============================================================

set TOP video_sram_nb_attn_m5_top_fp32

define_design_lib WORK -path ./work

# ------------------------------------------------------------
# Search path
# ------------------------------------------------------------
set_app_var search_path [list \
  . \
  ./src \
  ./src/AMOLED \
  ./src/AMOLED/attn_core \
  ./src/AMOLED/feature_sram \
  ./src/AMOLED/rgb2y_luma \
  ./src/AMOLED/tile_feature_extracter \
  ./src/AMOLED/tile_neighborhood_fetch \
  ./src/AMOLED/video_in_timing_if \
  ./include \
]

# ------------------------------------------------------------
# Read RTL files
# Exclude EPU
# ------------------------------------------------------------
set rtl_files [split [exec sh -c {find ./src -type f \( -name "*.sv" -o -name "*.v" \) -not -path "./src/EPU/*" | sort}] "\n"]

analyze -format sverilog $rtl_files
elaborate $TOP
current_design $TOP
link
uniquify
set_fix_multiple_port_nets -all -buffer_constants [get_designs *]

# ------------------------------------------------------------
# HDL / compile options
# ------------------------------------------------------------
set hdlin_infer_mux true
set hdlin_infer_dff true
set hdlin_ff_always_sync_set_reset true
set hdlin_ff_always_async_set_reset true

set_host_options -max_cores 2

# ------------------------------------------------------------
# Basic constraints
# Don't use old DC.sdc from another project
# ------------------------------------------------------------
create_clock -name clk -period 100 [get_ports clk]
set_input_delay  0 -clock clk [remove_from_collection [all_inputs] [get_ports clk]]
set_output_delay 0 -clock clk [all_outputs]

# ------------------------------------------------------------
# Compile
# ------------------------------------------------------------
compile
optimize_registers
remove_unconnected_ports -blast_buses [get_cells * -hier]

# ------------------------------------------------------------
# Naming
# ------------------------------------------------------------
set bus_inference_style {%s[%d]}
set bus_naming_style {%s[%d]}
set hdlout_internal_busses true

change_names -hierarchy -rule verilog

define_name_rules name_rule -allowed "A-Z a-z 0-9 _" -max_length 255 -type cell
define_name_rules name_rule -allowed "A-Z a-z 0-9 _[]" -max_length 255 -type net
define_name_rules name_rule -case_insensitive
change_names -hierarchy -rules name_rule
set_app_var search_path [concat $search_path [list \
  /export/Software/Synopsys/syn/U-2022.12/libraries/syn
]]

set_app_var synthetic_library [list dw_foundation.sldb]

# éè£¡ç stdcell.db è¦ææä½ å¯¦éç¨ç .db
set_app_var target_library [list your_stdcell.db]

set_app_var link_library [list "*" your_stdcell.db dw_foundation.sldb]

puts "search_path        = $search_path"
puts "synthetic_library  = $synthetic_library"
puts "target_library     = $target_library"
puts "link_library       = $link_library"
# ------------------------------------------------------------
# Outputs / reports
# ------------------------------------------------------------
file mkdir ./syn

write_file -format verilog -hierarchy -output ./syn/chip_syn.v
write_sdf ./syn/chip_syn.sdf
write_sdc ./syn/chip_syn.sdc

report_timing > ./syn/timing.log
report_area   > ./syn/area.log
report_power  > ./syn/power.log

report_timing -path full -delay max -nworst 1 -max_paths 1 -significant_digits 2 -sort_by group > ./syn/timing_max.rpt.txt
report_timing -path full -delay min -nworst 1 -max_paths 1 -significant_digits 2 -sort_by group > ./syn/timing_min.rpt.txt

exit
