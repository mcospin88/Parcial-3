## ============================================================
## Basys3.xdc - pinout para top.sv
## ============================================================

## -------------------- Clock 100 MHz -------------------------
set_property PACKAGE_PIN W5 [get_ports {clk}]
set_property IOSTANDARD LVCMOS33 [get_ports {clk}]
create_clock -period 10.000 -name sysclk [get_ports {clk}]

## -------------------- Botones -------------------------------
set_property PACKAGE_PIN U18 [get_ports {btnC}]  ;# BTNC
set_property PACKAGE_PIN T18 [get_ports {btnU}]  ;# BTNU
set_property PACKAGE_PIN W19 [get_ports {btnL}]  ;# BTNL
set_property PACKAGE_PIN T17 [get_ports {btnR}]  ;# BTNR
set_property PACKAGE_PIN U17 [get_ports {btnD}]  ;# BTND
set_property IOSTANDARD LVCMOS33 [get_ports {btnC btnU btnL btnR btnD}]

## -------------------- Switches ------------------------------
set_property PACKAGE_PIN V17 [get_ports {sw[0]}]
set_property PACKAGE_PIN V16 [get_ports {sw[1]}]
set_property PACKAGE_PIN W16 [get_ports {sw[2]}]
set_property PACKAGE_PIN W17 [get_ports {sw[3]}]
set_property PACKAGE_PIN W15 [get_ports {sw[4]}]
set_property PACKAGE_PIN V15 [get_ports {sw[5]}]
set_property PACKAGE_PIN W14 [get_ports {sw[6]}]
set_property PACKAGE_PIN W13 [get_ports {sw[7]}]
set_property PACKAGE_PIN V2  [get_ports {sw[8]}]
set_property PACKAGE_PIN T3  [get_ports {sw[9]}]
set_property PACKAGE_PIN T2  [get_ports {sw[10]}]
set_property PACKAGE_PIN R3  [get_ports {sw[11]}]
set_property PACKAGE_PIN W2  [get_ports {sw[12]}]
set_property PACKAGE_PIN U1  [get_ports {sw[13]}]
set_property PACKAGE_PIN T1  [get_ports {sw[14]}]
set_property PACKAGE_PIN R2  [get_ports {sw[15]}]
set_property IOSTANDARD LVCMOS33 [get_ports {sw[*]}]

## -------------------- LEDs ---------------------------------
set_property PACKAGE_PIN U16 [get_ports {led[0]}]
set_property PACKAGE_PIN E19 [get_ports {led[1]}]
set_property PACKAGE_PIN U19 [get_ports {led[2]}]
set_property PACKAGE_PIN V19 [get_ports {led[3]}]
set_property PACKAGE_PIN W18 [get_ports {led[4]}]
set_property PACKAGE_PIN U15 [get_ports {led[5]}]
set_property PACKAGE_PIN U14 [get_ports {led[6]}]
set_property PACKAGE_PIN V14 [get_ports {led[7]}]
set_property PACKAGE_PIN V13 [get_ports {led[8]}]
set_property PACKAGE_PIN V3  [get_ports {led[9]}]
set_property PACKAGE_PIN W3  [get_ports {led[10]}]
set_property PACKAGE_PIN U3  [get_ports {led[11]}]
set_property PACKAGE_PIN P3  [get_ports {led[12]}]
set_property PACKAGE_PIN N3  [get_ports {led[13]}]
set_property PACKAGE_PIN P1  [get_ports {led[14]}]
set_property PACKAGE_PIN L1  [get_ports {led[15]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[*]}]

## -------------------- 7 segmentos ---------------------------
set_property PACKAGE_PIN W7 [get_ports {seg[0]}]  ;# CA
set_property PACKAGE_PIN W6 [get_ports {seg[1]}]  ;# CB
set_property PACKAGE_PIN U8 [get_ports {seg[2]}]  ;# CC
set_property PACKAGE_PIN V8 [get_ports {seg[3]}]  ;# CD
set_property PACKAGE_PIN U5 [get_ports {seg[4]}]  ;# CE
set_property PACKAGE_PIN V5 [get_ports {seg[5]}]  ;# CF
set_property PACKAGE_PIN U7 [get_ports {seg[6]}]  ;# CG
set_property PACKAGE_PIN V7 [get_ports {seg[7]}]  ;# DP
set_property IOSTANDARD LVCMOS33 [get_ports {seg[*]}]

## Ánodos (activos-bajo)
set_property PACKAGE_PIN U2 [get_ports {an[0]}]
set_property PACKAGE_PIN U4 [get_ports {an[1]}]
set_property PACKAGE_PIN V4 [get_ports {an[2]}]
set_property PACKAGE_PIN W4 [get_ports {an[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {an[*]}]

## -------------------- FIX de BUFG auto-insertado -----------
## Vivado crea un BUFG llamado 'clk_IBUF_BUFG_inst'. Lo fijamos a un sitio
## válido del mismo "half" que el pin W5 para evitar Place 30-574.
## Si X0Y2 no funciona en tu parte, prueba X0Y3 o X0Y4.
set_property LOC BUFGCTRL_X0Y2 [get_cells {clk_IBUF_BUFG_inst}]
