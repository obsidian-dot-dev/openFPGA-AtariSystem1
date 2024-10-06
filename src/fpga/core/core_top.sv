//
//
// User core top-level
//
// Instantiated by the real top-level: apf_top
//

`default_nettype none

module core_top (

//
// physical connections
//

///////////////////////////////////////////////////
// clock inputs 74.25mhz. not phase aligned, so treat these domains as asynchronous

input   wire            clk_74a, // mainclk1
input   wire            clk_74b, // mainclk1 

///////////////////////////////////////////////////
// cartridge interface
// switches between 3.3v and 5v mechanically
// output enable for multibit translators controlled by pic32

// GBA AD[15:8]
inout   wire    [7:0]   cart_tran_bank2,
output  wire            cart_tran_bank2_dir,

// GBA AD[7:0]
inout   wire    [7:0]   cart_tran_bank3,
output  wire            cart_tran_bank3_dir,

// GBA A[23:16]
inout   wire    [7:0]   cart_tran_bank1,
output  wire            cart_tran_bank1_dir,

// GBA [7] PHI#
// GBA [6] WR#
// GBA [5] RD#
// GBA [4] CS1#/CS#
//     [3:0] unwired
inout   wire    [7:4]   cart_tran_bank0,
output  wire            cart_tran_bank0_dir,

// GBA CS2#/RES#
inout   wire            cart_tran_pin30,
output  wire            cart_tran_pin30_dir,
// when GBC cart is inserted, this signal when low or weak will pull GBC /RES low with a special circuit
// the goal is that when unconfigured, the FPGA weak pullups won't interfere.
// thus, if GBC cart is inserted, FPGA must drive this high in order to let the level translators
// and general IO drive this pin.
output  wire            cart_pin30_pwroff_reset,

// GBA IRQ/DRQ
inout   wire            cart_tran_pin31,
output  wire            cart_tran_pin31_dir,

// infrared
input   wire            port_ir_rx,
output  wire            port_ir_tx,
output  wire            port_ir_rx_disable, 

// GBA link port
inout   wire            port_tran_si,
output  wire            port_tran_si_dir,
inout   wire            port_tran_so,
output  wire            port_tran_so_dir,
inout   wire            port_tran_sck,
output  wire            port_tran_sck_dir,
inout   wire            port_tran_sd,
output  wire            port_tran_sd_dir,
 
///////////////////////////////////////////////////
// cellular psram 0 and 1, two chips (64mbit x2 dual die per chip)

output  wire    [21:16] cram0_a,
inout   wire    [15:0]  cram0_dq,
input   wire            cram0_wait,
output  wire            cram0_clk,
output  wire            cram0_adv_n,
output  wire            cram0_cre,
output  wire            cram0_ce0_n,
output  wire            cram0_ce1_n,
output  wire            cram0_oe_n,
output  wire            cram0_we_n,
output  wire            cram0_ub_n,
output  wire            cram0_lb_n,

output  wire    [21:16] cram1_a,
inout   wire    [15:0]  cram1_dq,
input   wire            cram1_wait,
output  wire            cram1_clk,
output  wire            cram1_adv_n,
output  wire            cram1_cre,
output  wire            cram1_ce0_n,
output  wire            cram1_ce1_n,
output  wire            cram1_oe_n,
output  wire            cram1_we_n,
output  wire            cram1_ub_n,
output  wire            cram1_lb_n,

///////////////////////////////////////////////////
// sdram, 512mbit 16bit

output  wire    [12:0]  dram_a,
output  wire    [1:0]   dram_ba,
inout   wire    [15:0]  dram_dq,
output  wire    [1:0]   dram_dqm,
output  wire            dram_clk,
output  wire            dram_cke,
output  wire            dram_ras_n,
output  wire            dram_cas_n,
output  wire            dram_we_n,

///////////////////////////////////////////////////
// sram, 1mbit 16bit

output  wire    [16:0]  sram_a,
inout   wire    [15:0]  sram_dq,
output  wire            sram_oe_n,
output  wire            sram_we_n,
output  wire            sram_ub_n,
output  wire            sram_lb_n,

///////////////////////////////////////////////////
// vblank driven by dock for sync in a certain mode

input   wire            vblank,

///////////////////////////////////////////////////
// i/o to 6515D breakout usb uart

output  wire            dbg_tx,
input   wire            dbg_rx,

///////////////////////////////////////////////////
// i/o pads near jtag connector user can solder to

output  wire            user1,
input   wire            user2,

///////////////////////////////////////////////////
// RFU internal i2c bus 

inout   wire            aux_sda,
output  wire            aux_scl,

///////////////////////////////////////////////////
// RFU, do not use
output  wire            vpll_feed,


//
// logical connections
//

///////////////////////////////////////////////////
// video, audio output to scaler
output  wire    [23:0]  video_rgb,
output  wire            video_rgb_clock,
output  wire            video_rgb_clock_90,
output  wire            video_de,
output  wire            video_skip,
output  wire            video_vs,
output  wire            video_hs,
    
output  wire            audio_mclk,
input   wire            audio_adc,
output  wire            audio_dac,
output  wire            audio_lrck,

///////////////////////////////////////////////////
// bridge bus connection
// synchronous to clk_74a
output  wire            bridge_endian_little,
input   wire    [31:0]  bridge_addr,
input   wire            bridge_rd,
output  reg     [31:0]  bridge_rd_data,
input   wire            bridge_wr,
input   wire    [31:0]  bridge_wr_data,

///////////////////////////////////////////////////
// controller data
// 
// key bitmap:
//   [0]    dpad_up
//   [1]    dpad_down
//   [2]    dpad_left
//   [3]    dpad_right
//   [4]    face_a
//   [5]    face_b
//   [6]    face_x
//   [7]    face_y
//   [8]    trig_l1
//   [9]    trig_r1
//   [10]   trig_l2
//   [11]   trig_r2
//   [12]   trig_l3
//   [13]   trig_r3
//   [14]   face_select
//   [15]   face_start
// joy values - unsigned
//   [ 7: 0] lstick_x
//   [15: 8] lstick_y
//   [23:16] rstick_x
//   [31:24] rstick_y
// trigger values - unsigned
//   [ 7: 0] ltrig
//   [15: 8] rtrig
//
input   wire    [15:0]  cont1_key,
input   wire    [15:0]  cont2_key,
input   wire    [15:0]  cont3_key,
input   wire    [15:0]  cont4_key,
input   wire    [31:0]  cont1_joy,
input   wire    [31:0]  cont2_joy,
input   wire    [31:0]  cont3_joy,
input   wire    [31:0]  cont4_joy,
input   wire    [15:0]  cont1_trig,
input   wire    [15:0]  cont2_trig,
input   wire    [15:0]  cont3_trig,
input   wire    [15:0]  cont4_trig
    
);

// not using the IR port, so turn off both the LED, and
// disable the receive circuit to save power
assign port_ir_tx = 0;
assign port_ir_rx_disable = 1;

// bridge endianness
assign bridge_endian_little = 0;

// cart is unused, so set all level translators accordingly
// directions are 0:IN, 1:OUT
assign cart_tran_bank3 = 8'hzz;
assign cart_tran_bank3_dir = 1'b0;
assign cart_tran_bank2 = 8'hzz;
assign cart_tran_bank2_dir = 1'b0;
assign cart_tran_bank1 = 8'hzz;
assign cart_tran_bank1_dir = 1'b0;
assign cart_tran_bank0 = 4'hf;
assign cart_tran_bank0_dir = 1'b1;
assign cart_tran_pin30 = 1'b0;      // reset or cs2, we let the hw control it by itself
assign cart_tran_pin30_dir = 1'bz;
assign cart_pin30_pwroff_reset = 1'b0;  // hardware can control this
assign cart_tran_pin31 = 1'bz;      // input
assign cart_tran_pin31_dir = 1'b0;  // input

// link port is input only
assign port_tran_so = 1'bz;
assign port_tran_so_dir = 1'b0;     // SO is output only
assign port_tran_si = 1'bz;
assign port_tran_si_dir = 1'b0;     // SI is input only
assign port_tran_sck = 1'bz;
assign port_tran_sck_dir = 1'b0;    // clock direction can change
assign port_tran_sd = 1'bz;
assign port_tran_sd_dir = 1'b0;     // SD is input and not used

// tie off the rest of the pins we are not using
assign cram0_a = 'h0;
assign cram0_dq = {16{1'bZ}};
assign cram0_clk = 0;
assign cram0_adv_n = 1;
assign cram0_cre = 0;
assign cram0_ce0_n = 1;
assign cram0_ce1_n = 1;
assign cram0_oe_n = 1;
assign cram0_we_n = 1;
assign cram0_ub_n = 1;
assign cram0_lb_n = 1;

assign cram1_a = 'h0;
assign cram1_dq = {16{1'bZ}};
assign cram1_clk = 0;
assign cram1_adv_n = 1;
assign cram1_cre = 0;
assign cram1_ce0_n = 1;
assign cram1_ce1_n = 1;
assign cram1_oe_n = 1;
assign cram1_we_n = 1;
assign cram1_ub_n = 1;
assign cram1_lb_n = 1;

//assign dram_a = 'h0;
//assign dram_ba = 'h0;
//assign dram_dq = {16{1'bZ}};
//assign dram_dqm = 'h0;
//assign dram_clk = 'h0;
//assign dram_cke = 'h0;
//assign dram_ras_n = 'h1;
//assign dram_cas_n = 'h1;
//assign dram_we_n = 'h1;

//assign sram_a = 'h0;
//assign sram_dq = {16{1'bZ}};
//assign sram_oe_n  = 1;
//assign sram_we_n  = 1;
//assign sram_ub_n  = 1;
//assign sram_lb_n  = 1;

assign dbg_tx = 1'bZ;
assign user1 = 1'bZ;
assign aux_scl = 1'bZ;
assign vpll_feed = 1'bZ;

// for bridge write data, we just broadcast it to all bus devices
// for bridge read data, we have to mux it
// add your own devices here
always @(*) begin
    casex(bridge_addr)
    default: begin
        bridge_rd_data <= 0;
    end
    32'h10xxxxxx: begin
        // example
        // bridge_rd_data <= example_device_data;
        bridge_rd_data <= 0;
    end
    32'hF8xxxxxx: begin
        bridge_rd_data <= cmd_bridge_rd_data;
    end
    endcase
end


//
// host/target command handler
//
    wire            reset_n;                // driven by host commands, can be used as core-wide reset
    wire    [31:0]  cmd_bridge_rd_data;
    
// bridge host commands
// synchronous to clk_74a
    wire            status_boot_done = pll_core_locked & pll_sdram_locked; 
    wire            status_setup_done = pll_core_locked & pll_sdram_locked; // rising edge triggers a target command
    wire            status_running = reset_n; // we are running as soon as reset_n goes high

    wire            dataslot_requestread;
    wire    [15:0]  dataslot_requestread_id;
    wire            dataslot_requestread_ack = 1;
    wire            dataslot_requestread_ok = 1;

    wire            dataslot_requestwrite;
    wire    [15:0]  dataslot_requestwrite_id;
    wire            dataslot_requestwrite_ack = 1;
    wire            dataslot_requestwrite_ok = 1;

    wire            dataslot_allcomplete;

    wire            savestate_supported;
    wire    [31:0]  savestate_addr;
    wire    [31:0]  savestate_size;
    wire    [31:0]  savestate_maxloadsize;

    wire            savestate_start;
    wire            savestate_start_ack;
    wire            savestate_start_busy;
    wire            savestate_start_ok;
    wire            savestate_start_err;

    wire            savestate_load;
    wire            savestate_load_ack;
    wire            savestate_load_busy;
    wire            savestate_load_ok;
    wire            savestate_load_err;
    
    wire            osnotify_inmenu;

// bridge target commands
// synchronous to clk_74a


// bridge data slot access

    wire    [9:0]   datatable_addr;
    wire            datatable_wren;
    wire    [31:0]  datatable_data;
    wire    [31:0]  datatable_q;

core_bridge_cmd icb (

    .clk                ( clk_74a ),
    .reset_n            ( reset_n ),

    .bridge_endian_little   ( bridge_endian_little ),
    .bridge_addr            ( bridge_addr ),
    .bridge_rd              ( bridge_rd ),
    .bridge_rd_data         ( cmd_bridge_rd_data ),
    .bridge_wr              ( bridge_wr ),
    .bridge_wr_data         ( bridge_wr_data ),
    
    .status_boot_done       ( status_boot_done ),
    .status_setup_done      ( status_setup_done ),
    .status_running         ( status_running ),

    .dataslot_requestread       ( dataslot_requestread ),
    .dataslot_requestread_id    ( dataslot_requestread_id ),
    .dataslot_requestread_ack   ( dataslot_requestread_ack ),
    .dataslot_requestread_ok    ( dataslot_requestread_ok ),

    .dataslot_requestwrite      ( dataslot_requestwrite ),
    .dataslot_requestwrite_id   ( dataslot_requestwrite_id ),
    .dataslot_requestwrite_ack  ( dataslot_requestwrite_ack ),
    .dataslot_requestwrite_ok   ( dataslot_requestwrite_ok ),

    .dataslot_allcomplete   ( dataslot_allcomplete ),

    .savestate_supported    ( savestate_supported ),
    .savestate_addr         ( savestate_addr ),
    .savestate_size         ( savestate_size ),
    .savestate_maxloadsize  ( savestate_maxloadsize ),

    .savestate_start        ( savestate_start ),
    .savestate_start_ack    ( savestate_start_ack ),
    .savestate_start_busy   ( savestate_start_busy ),
    .savestate_start_ok     ( savestate_start_ok ),
    .savestate_start_err    ( savestate_start_err ),

    .savestate_load         ( savestate_load ),
    .savestate_load_ack     ( savestate_load_ack ),
    .savestate_load_busy    ( savestate_load_busy ),
    .savestate_load_ok      ( savestate_load_ok ),
    .savestate_load_err     ( savestate_load_err ),

    .osnotify_inmenu        ( osnotify_inmenu ),
    
    .datatable_addr         ( datatable_addr ),
    .datatable_wren         ( datatable_wren ),
    .datatable_data         ( datatable_data ),
    .datatable_q            ( datatable_q ),

);

// Game type defines
parameter [2:0] GAME_ID_MARBLE_MADNESS 	 = 0;
parameter [2:0] GAME_ID_TEMPLE_OF_DOOM 	 = 1;
parameter [2:0] GAME_ID_PETER_PACK_RAT	 = 2;
parameter [2:0] GAME_ID_ROAD_BLASTERS    = 3;
parameter [2:0] GAME_ID_ROAD_RUNNER      = 4;

reg [2:0] game_id_u;
reg [2:0] game_id;
reg [1:0] analog_speed = 2'b10;
reg [1:0] analog_speed_1;
reg [1:0] analog_speed_2;
reg service_mode;

synch_3 #(
    .WIDTH(3)
) game_id_s (
    game_id_u,
    game_id,
    clk_sys
);

always @(posedge clk_74a) begin
  if(bridge_wr) begin
    casex(bridge_addr)
       32'h80000000: begin 
			game_id_u	<= bridge_wr_data[2:0];  
	   end
	   32'h90000000: begin 
			analog_speed <= bridge_wr_data[1:0];
		end
	   32'hA0000000: begin 
			service_mode 	<= bridge_wr_data[0];
	   end
      32'hB0000000: begin
         do_reset <= ~do_reset;
      end
    endcase
  end
end

///////////////////////////////////////////////
// System
///////////////////////////////////////////////

wire osnotify_inmenu_s;

synch_3 OSD_S (osnotify_inmenu, osnotify_inmenu_s, clk_sys);

///////////////////////////////////////////////
// ROM
///////////////////////////////////////////////

reg         ioctl_download = 0;
reg         ioctl_download_u = 0;
wire        ioctl_wr;
wire [24:0] ioctl_addr;
wire  [7:0] ioctl_dout;

always @(posedge clk_74a) begin
    if (dataslot_requestwrite)     ioctl_download_u <= 1;
    else if (dataslot_allcomplete) ioctl_download_u <= 0;
end


synch_3 ioctl_dl_sync (
    ioctl_download_u,
    ioctl_download,
    clk_sys
);

data_loader #(
    .ADDRESS_MASK_UPPER_4(0),
    .ADDRESS_SIZE(25),
	 .WRITE_MEM_CLOCK_DELAY(13),
	 .WRITE_MEM_EN_CYCLE_LENGTH(1)
) rom_loader (
    .clk_74a(clk_74a),
    .clk_memory(clk_sys),

    .bridge_wr(bridge_wr),
    .bridge_endian_little(bridge_endian_little),
    .bridge_addr(bridge_addr),
    .bridge_wr_data(bridge_wr_data),

    .write_en(ioctl_wr),
    .write_addr(ioctl_addr),
    .write_data(ioctl_dout)
);

// Synchronize the loading of the sram on each WORD

reg do_reset;
reg last_do_reset;
reg manual_reset = 1'b0;
reg [24:0] reset_count = 25'b0;

always @(posedge clk_14) begin
	last_do_reset <= do_reset;
	if (~manual_reset && (do_reset != last_do_reset)) begin
		reset_count <= 25'd1;
		manual_reset <= 1'b1;
	end
	else begin
		if (reset_count == 25'b0001111111111111111111111) begin
			reset_count <= 25'b0;
			manual_reset <= 1'b0;
		end
		else begin
			reset_count <= reset_count + 25'd1;
		end
	end
end


wire [24:0] sdram_addr;
reg  [15:0] sdram_data = 16'b0;
reg         sdram_we = 1'b0;
wire 	      sdram_ready;
wire			sdram_available;


// Synchronize the loading of the sdram on each WORD
always @(posedge clk_sys)
begin
	sdram_we <= 1'b0;
	if (ioctl_download && v_wr && ioctl_addr[0]) begin		
		sdram_data <= {acc_bytes[7:0],ioctl_dout};
		sdram_we <= 1'b1;
	end
end


reg clk14_last;
reg sd_do_read;
reg last_read_acked = 1'b1;

reg [5:0] recover_cycles;

always @(posedge clk_sys)
begin
	sd_do_read <= 1'b0;
	if (~ioctl_download && reset_n) begin
		if (clk14_last == 1'b0 && clk_14 == 1'b1) begin
			if (last_read_acked == 1'b1 || recover_cycles == 6'b111111) begin
				sd_do_read <= 1'b1;
				last_read_acked <= 1'b0;		
				recover_cycles <= 6'b000000;
			end		
			else begin
			  // HACK: If we missed 64-clocks, then try to recover... 
			  recover_cycles <= recover_cycles + 1;
			end
		end
			
		if (sdram_ready || sdram_available) begin
			last_read_acked <= 1'b1;
		end
	end
		
	clk14_last <= clk_14;
end

wire [31:0] sd_read_data_raw;
sdram #(
	.CLOCK_SPEED_MHZ(93.06817),
	.BURST_LENGTH(2),
	.WRITE_BURST(0),
	.CAS_LATENCY(3)	
) sdram
(
	.clk(clk_93),
	.reset(~pll_core_locked),
	// .init_complete(),
	.p0_addr(sdram_addr),
	.p0_data(sdram_data),
	.p0_byte_en(2'b11),
	.p0_q(sd_read_data_raw),
	.p0_wr_req(sdram_we),
	.p0_rd_req(sd_do_read),
	.p0_available(sdram_available),
	.p0_ready(sdram_ready),
	
	.SDRAM_DQ(dram_dq),
	.SDRAM_A(dram_a),
	.SDRAM_DQM(dram_dqm),
	.SDRAM_BA(dram_ba),
	// .SDRAM_nCS()
	.SDRAM_nWE(dram_we_n),
	.SDRAM_nRAS(dram_ras_n),
	.SDRAM_nCAS(dram_cas_n),
	.SDRAM_CKE(dram_cke),
	.SDRAM_CLK(dram_clk)
);


rom_storage rom_storage (
  .wr_en(sram_wr_en_latched), 	  
  
  .addr(sram_addr), 		 
  .din(sram_write_data), 		 
  .dout(sram_read_data), 		 

  // -- sram bus parameters
  .sram_a(sram_a), 	 
  .sram_dq(sram_dq), 
  .sram_oe_n(sram_oe_n), 
  .sram_we_n(sram_we_n), 
  .sram_ub_n(sram_ub_n), 
  .sram_lb_n(sram_lb_n), 
);


reg  [55:0] acc_bytes = 0;
wire [18:0] slv_VADDR;
reg [31:0] slv_VDATA;
// the order in which the files are listed in the .mra file determines the order in which they appear here on the HPS bus
// some files are interleaved as DWORD, some are interleaved as WORD and some are not interleaved and appear as BYTE
// acc_bytes collects previous bytes so that when a WORD or DWORD is complete it is written to the RAM as appropriate
always @(posedge clk_sys)
	if (ioctl_wr && ioctl_download )
		acc_bytes<={acc_bytes[47:0],ioctl_dout}; // accumulate previous bytes


// We write DWORDs and read DWORDs
reg [63:0] slv_VDATA_s;
// Synchronize address (in) and read-data (out)
always @(posedge clk_sys)
begin
	slv_VDATA_s <= {sd_read_data_raw[15:0], sd_read_data_raw[31:16], vrom_byte5, vrom_byte6, 16'hffff};
end

assign sdram_addr = ioctl_download?{1'b0,ioctl_addr[24:1]}:{4'd0, slv_VADDR, 2'd00};

wire sl_wr_SROM0, sl_wr_SROM1, sl_wr_SROM2;
wire sl_wr_ROM0, sl_wr_ROM7;
wire sl_wr_2B, sl_wr_5A , sl_wr_7A, sl_wr_SLAP, sl_MA18n, sl_wr_ep1, sl_wr_ep2;

wire [15:1] slv_MADEC;
wire [15:0] slv_MDATA;
wire [15:0] slv_ROM0, slv_ROM1, slv_ROM2, slv_ROM5, slv_ROM6, slv_ROM7, slv_SLAP;
wire [13:0] slv_SBA;
wire [13:0] slv_PA2B;
wire [ 8:0] slv_PADDR;
wire [ 7:0] slv_SROM0, slv_SROM1, slv_SROM2, slv_eprom_din, slv_eprom_dout;
wire [ 7:0] slv_SDATA, slv_PD4A, slv_PD7A, slv_PD2B;
wire [ 4:0] slv_ROMn;
wire [ 2:0] slv_SROMn;

// Defines which 64K ROM bank to write to SRAM
wire [1:0] slv_write_bank;
assign slv_write_bank =
	(ioctl_addr[24:16] == 9'h41) ? 2'b00 :
	(ioctl_addr[24:16] == 9'h42) ? 2'b01 :
	(ioctl_addr[24:16] == 9'h45) ? 2'b10 :
	(ioctl_addr[24:16] == 9'h46) ? 2'b11 :
	2'b00;

wire [1:0] slv_read_bank;
assign slv_read_bank = 
	(~slv_ROMn[1] &  sl_MA18n) ? 2'b00:
	(~slv_ROMn[2] &  sl_MA18n) ? 2'b01:
	(~slv_ROMn[1] & ~sl_MA18n) ? 2'b10:
	(~slv_ROMn[2] & ~sl_MA18n) ? 2'b11:
	2'b00;
	
// Defines when to enable writing to SRAM (i.e. We have data for ROM 1,2,5, or 6)
wire sram_wr_en;
assign sram_wr_en = 
	(ioctl_addr[24:16] == 9'h41) |
	(ioctl_addr[24:16] == 9'h42) |
	(ioctl_addr[24:16] == 9'h45) |
	(ioctl_addr[24:16] == 9'h46);
	
// SRAM access address
// - When writing, this is the write-bank in upper 2 bits, followed by WORD address of the ROM.
// - When reading, this is the *read* bank in upper 2 bits (muxed from bus signals), followed by WORD address for the ROM.	
wire [16:0] sram_addr;
wire [16:0] sram_read_addr;
wire [15:0] sram_read_data;

reg  [16:0] sram_write_addr;
reg  [15:0] sram_write_data;
reg  		sram_wr_en_latched;

assign sram_addr	   = ioctl_download ? sram_write_addr : sram_read_addr;
assign sram_read_addr  = {slv_read_bank,  slv_MADEC[15:1]};

always @(posedge clk_sys)
begin
    if (ioctl_wr && ioctl_addr[0])
    begin
	   sram_wr_en_latched <= sram_wr_en;
		sram_write_addr <= {slv_write_bank, ioctl_addr[15:1]};
		sram_write_data <= {acc_bytes[7:0], ioctl_dout};
    end  
end


// 0x000000 - 0x3fffff [24:22]=3'b0,  8 banks [21:19] x 64KB [18:3] x 8 planes [2:0] = 0x400000 bytes or 0x80000 DWORDS
wire v_wr;
assign v_wr = (ioctl_wr && ioctl_addr[24:22]==3'h0); // 0x80000 x8
// 0x400000 - 0x407fff
assign sl_wr_ROM0    = (ioctl_wr && ioctl_addr[24:15]==10'h80   && ioctl_addr[0]==1'b1) ? 1'b1 : 1'b0; // 0x4000 x2
// 0x410000 - 0x41ffff
// assign sl_wr_ROM1    = (ioctl_wr && ioctl_addr[24:16]== 9'h41   && ioctl_addr[0]==1'b1) ? 1'b1 : 1'b0; // 0x8000 x2
// 0x420000 - 0x42ffff
// assign sl_wr_ROM2    = (ioctl_wr && ioctl_addr[24:16]== 9'h42   && ioctl_addr[0]==1'b1) ? 1'b1 : 1'b0; // 0x8000 x2
// 0x430000 - 0x437fff - remapped to ROM7
//assign sl_wr_ROM3  = (ioctl_wr && ioctl_download && ioctl_addr[24:15]==10'h86   && ioctl_addr[0]==1'b1) ? 1'b1 : 1'b0; // 0x4000 x2
// 0x450000 - 0x45ffff
// assign sl_wr_ROM5    = (ioctl_wr && ioctl_addr[24:16]== 9'h45   && ioctl_addr[0]==1'b1) ? 1'b1 : 1'b0; // 0x8000 x2
// 0x460000 - 0x46ffff
// assign sl_wr_ROM6    = (ioctl_wr && ioctl_addr[24:16]== 9'h46   && ioctl_addr[0]==1'b1) ? 1'b1 : 1'b0; // 0x8000 x2
// 0x470000 - 0x47ffff
assign sl_wr_ROM7    = (ioctl_wr && ioctl_addr[24:16]== 9'h47   && ioctl_addr[0]==1'b1) ? 1'b1 : 1'b0; // 0x8000 x2
// 0x480000 - 0x487fff
assign sl_wr_SLAP    = (ioctl_wr && ioctl_addr[24:15]==10'h90   && ioctl_addr[0]==1'b1) ? 1'b1 : 1'b0; // 0x4000 x2
// 0x488000 - 0x48bfff
assign sl_wr_SROM0   = (ioctl_wr && ioctl_addr[24:14]==11'h122                        ) ? 1'b1 : 1'b0; // 0x4000 x1
// 0x48C000 - 0x48ffff
assign sl_wr_SROM1   = (ioctl_wr && ioctl_addr[24:14]==11'h123                        ) ? 1'b1 : 1'b0; // 0x4000 x1
// 0x490000 - 0x493fff
assign sl_wr_SROM2   = (ioctl_wr && ioctl_addr[24:14]==11'h124                        ) ? 1'b1 : 1'b0; // 0x4000 x1
// 0x494000 - 0x497fff
assign sl_wr_2B      = (ioctl_wr && ioctl_addr[24:14]==11'h125                        ) ? 1'b1 : 1'b0; // 0x4000 x1
// 0x498000 - 0x4981ff
assign sl_wr_5A      = (ioctl_wr && ioctl_addr[24: 9]==16'h24C0                       ) ? 1'b1 : 1'b0; // 0x200 x1
// 0x498200 - 0x4983ff
assign sl_wr_7A      = (ioctl_wr && ioctl_addr[24: 9]==16'h24C1                       ) ? 1'b1 : 1'b0; // 0x200 x1
// 0x498400 - 0x4985ff
assign sl_wr_ep1     = (ioctl_wr && ioctl_addr[24: 9]==16'h24C2                       ) ? 1'b1 : 1'b0; // 0x200 x1
// 0x498600

// Note: to save some memory we alias ROM3 with ROM7.
// Tweak .mra file accordingly and upload ROM3 into ROM7 slot.
// ROM3 is only used by Indy and nothing else while ROM7 region is unused by Indy.
assign slv_MDATA =
	(~slv_ROMn[0] &  sl_MA18n) ? slv_ROM0:
	(~slv_ROMn[1] &  sl_MA18n) ? sram_read_data:
	(~slv_ROMn[2] &  sl_MA18n) ? sram_read_data:	
	(~slv_ROMn[3] &  sl_MA18n) ? slv_ROM7:

	(~slv_ROMn[1] & ~sl_MA18n) ? sram_read_data:
	(~slv_ROMn[2] & ~sl_MA18n) ? sram_read_data:
	(~slv_ROMn[3] & ~sl_MA18n) ? slv_ROM7:
	(~slv_ROMn[4]            ) ? slv_SLAP:
	16'h0;

assign slv_SDATA =
	(~slv_SROMn[0])?slv_SROM0:
	(~slv_SROMn[1])?slv_SROM1:
	(~slv_SROMn[2])?slv_SROM2:
	8'h0;

	
// Additional BRAMs allocated to store "non-FF" video ROM data that
// would was previously loaded into the high-DWORD of SDRAM. 
// Only Marble Madness and RoadBlasters make use of the high-DWORD,
// for a grand total of 64KB. Use BRAM to avoid having to go to 
// Quad-word bursts on SDRAM, which is marginal on some pockets.

reg [14:0] vrom1_wr_addr;
wire [14:0] vrom1_rd_addr;	
	
reg [7:0] vrom1_wr_data;
wire [7:0] vrom1_rd_data;

reg vrom1_wr_en;

dpram #(15, 8) vrom1
( 
  .clock_a(clk_sys),   .enable_a(),   .wren_a(vrom1_wr_en),   .address_a(vrom1_wr_addr),   .data_a(vrom1_wr_data),   .q_a(),
  .clock_b(clk_sys),   .enable_b(),   .wren_b(),   			  .address_b(vrom1_rd_addr),   .data_b(),   			 .q_b(vrom1_rd_data)
);

reg [14:0] vrom2_wr_addr;
wire [14:0] vrom2_rd_addr;	
	
reg [7:0] vrom2_wr_data;
wire [7:0] vrom2_rd_data;

reg vrom2_wr_en;

dpram #(15, 8) vrom2
( 
  .clock_a(clk_sys),   .enable_a(),   .wren_a(vrom2_wr_en),   .address_a(vrom2_wr_addr),   .data_a(vrom2_wr_data),   .q_a(),
  .clock_b(clk_sys),   .enable_b(),   .wren_b(), 			  .address_b(vrom2_rd_addr),   .data_b(), 				 .q_b(vrom2_rd_data)
);	

// Logic for loading VROM data into extra BRAMs at boot, for games
// that use it.
always @(posedge clk_sys) 
begin
	if (game_id == GAME_ID_MARBLE_MADNESS) begin
		// Treat as one contiguous 64K x 8 region.
		if (ioctl_wr && ioctl_addr[24:18] == 7'b0000010) begin
			vrom1_wr_en <= 1'b1;
			if (ioctl_addr[2:0] == 3'b111) begin
				vrom1_wr_addr <= ioctl_addr[17:3];
				vrom1_wr_data <= acc_bytes[23:16];
			end
		end
		else begin				
			vrom1_wr_en <= 1'b0;
		end
		
		if (ioctl_wr && ioctl_addr[24:18] == 7'b0000011) begin
			vrom2_wr_en <= 1'b1;
			if (ioctl_addr[2:0] == 3'b111) begin
				vrom2_wr_addr <= ioctl_addr[17:3];
				vrom2_wr_data <= acc_bytes[23:16];
			end
		end
		else begin 
			vrom2_wr_en <= 1'b0;
		end
	end
	else if (game_id == GAME_ID_ROAD_BLASTERS) begin
		// Treat as a 32K x 16 region.
		if (ioctl_wr && ioctl_addr[24:18] == 7'b0000010) begin
			vrom1_wr_en <= 1'b1;
			vrom2_wr_en <= 1'b1;

			if (ioctl_addr[2:0] == 3'b111) begin
				vrom1_wr_addr <= ioctl_addr[17:3];
				vrom1_wr_data <= acc_bytes[23:16];
				
				vrom2_wr_addr <= ioctl_addr[17:3];
				vrom2_wr_data <= acc_bytes[15:8];
			end
		end
		else begin
			vrom1_wr_en <= 1'b0;
			vrom2_wr_en <= 1'b0;
		end
	end
end

reg [7:0] vrom_byte5;
reg [7:0] vrom_byte6;

// Additional VROM read circuit:
// - Marble Madness and RoadBlasters use a max of 5 or 6 VROM planes,
//   and all others use a maximum of 4 VROM planes.
// - Given these only represent an extra 64KB of space, we map that 
//   into BRAM instead of SDRAM to avoid the extra complexity of
//   storing/reading the planes as quad-words. 
// This circuit maps the data stored in BRAM to the VDATA register for
// the two games that need them.
always @(posedge clk_sys) begin
	vrom_byte5 <= 8'hff;
	vrom_byte6 <= 8'hff;

	// PP PR00 0000 0000 0000
	// Where PPP == ROM bank
	//         R == MSB of bank - used for bank-select 

	if (game_id == GAME_ID_MARBLE_MADNESS) begin
		// Marble Madness uses the 2 x 32K VROMs as a single
		// contiguous 64K ROM.
		if (slv_VADDR[18:15] == 4'b0010) begin
			vrom1_rd_addr <= slv_VADDR[14:0];
			vrom_byte5 <= vrom1_rd_data;
		end
		else if (slv_VADDR[18:15] == 4'b0011) begin
			vrom2_rd_addr <= slv_VADDR[14:0];
			vrom_byte5 <= vrom2_rd_data;
		end
	end
	else if (game_id == GAME_ID_ROAD_BLASTERS) begin
		// RoadBlasters Uses 2 x 32K VROMs as a 32K x 16 bit 
		// rom.
		if (slv_VADDR[18:15] == 4'b0010) begin			
			vrom1_rd_addr <= slv_VADDR[14:0];
			vrom2_rd_addr <= slv_VADDR[14:0];

			vrom_byte5 <= vrom1_rd_data;
			vrom_byte6 <= vrom2_rd_data;
		end
	end
end

// 32 M10K blocks
// ROM0 BIOS 0x000000, 0x4000
dpram #(14,16) mp_rom0
(.clock_a(clk_sys), .enable_a(), .wren_a(sl_wr_ROM0 ), .address_a(ioctl_addr[14:1]), .data_a({acc_bytes[7:0],ioctl_dout}), .q_a(             ),
    .clock_b(clk_sys), .enable_b(), .wren_b(           ), .address_b( slv_MADEC[14:1]), .data_b(                           ), .q_b(slv_ROM0     ));

//
// Note: ROM 1, 2, 5, 6 moved to SRAM
//
// 64 M10K blocks
// ROM1 0x10000, 0x08000
//dpram #(15,16) mp_rom1
//(.clock_a(clk_sys), .enable_a(), .wren_a(sl_wr_ROM1 ), .address_a(ioctl_addr[15:1]), .data_a({acc_bytes[7:0],ioctl_dout}), .q_a(             ),
//    .clock_b(clk_sys), .enable_b(), .wren_b(           ), .address_b( slv_MADEC[15:1]), .data_b(                           ), .q_b(slv_ROM1     ));
// 64 M10K blocks
// ROM2 0x20000, 0x08000
// dpram #(15,16) mp_rom2
// (.clock_a(clk_sys), .enable_a(), .wren_a(sl_wr_ROM2 ), .address_a(ioctl_addr[15:1]), .data_a({acc_bytes[7:0],ioctl_dout}), .q_a(             ),
//    .clock_b(clk_sys), .enable_b(), .wren_b(           ), .address_b( slv_MADEC[15:1]), .data_b(                           ), .q_b(slv_ROM2     ));

// 32 M10K blocks
// ROM3 0x30000, INDIANA JONES only region, remapped to 0x070000 instead.
//	dpram #(14,16) mp_rom3
//	(.clock_a(clk_sys), .enable_a(), .wren_a(sl_wr_ROM3 ), .address_a(ioctl_addr[14:1]), .data_a({acc_bytes[7:0],ioctl_dout}), .q_a(             ),
//	 .clock_b(clk_sys), .enable_b(), .wren_b(           ), .address_b( slv_MADEC[14:1]), .data_b(                           ), .q_b(slv_ROM3     ));

// 64 M10K blocks
// ROM5 0x50000, 0x08000
//dpram #(15,16) mp_rom5
//(.clock_a(clk_sys), .enable_a(), .wren_a(sl_wr_ROM5 ), .address_a(ioctl_addr[15:1]), .data_a({acc_bytes[7:0],ioctl_dout}), .q_a(             ),
//   .clock_b(clk_sys), .enable_b(), .wren_b(           ), .address_b( slv_MADEC[15:1]), .data_b(                           ), .q_b(slv_ROM5     ));

// 64 M10K blocks
// ROM6 0x60000, 0x08000
//dpram #(15,16) mp_rom6
//(.clock_a(clk_sys), .enable_a(), .wren_a(sl_wr_ROM6 ), .address_a(ioctl_addr[15:1]), .data_a({acc_bytes[7:0],ioctl_dout}), .q_a(             ),
//   .clock_b(clk_sys), .enable_b(), .wren_b(           ), .address_b( slv_MADEC[15:1]), .data_b(                           ), .q_b(slv_ROM6     ));

// 64 M10K blocks
// ROM7 0x70000, 0x08000
dpram #(15,16) mp_rom7
(.clock_a(clk_sys), .enable_a(), .wren_a(sl_wr_ROM7 ), .address_a(ioctl_addr[15:1]), .data_a({acc_bytes[7:0],ioctl_dout}), .q_a(             ),
    .clock_b(clk_sys), .enable_b(), .wren_b(           ), .address_b( slv_MADEC[15:1]), .data_b(                           ), .q_b(slv_ROM7     ));

// 32 M10K blocks
// Slapstic 0x80000, 0x04000
dpram #(14,16) mp_rom_slap
(.clock_a(clk_sys), .enable_a(), .wren_a(sl_wr_SLAP ), .address_a(ioctl_addr[14:1]), .data_a({acc_bytes[7:0],ioctl_dout}), .q_a(             ),
    .clock_b(clk_sys), .enable_b(), .wren_b(           ), .address_b( slv_MADEC[14:1]), .data_b(                           ), .q_b(slv_SLAP     ));

// 16 M10K blocks
// Audiocpu 0x4000, 0x4000
dpram #(14, 8) ap_srom0
(.clock_a(clk_sys), .enable_a(), .wren_a(sl_wr_SROM0), .address_a(ioctl_addr[13:0]), .data_a(                ioctl_dout ), .q_a(             ),
    .clock_b(clk_sys), .enable_b(), .wren_b(           ), .address_b(   slv_SBA[13:0]), .data_b(                           ), .q_b(slv_SROM0    ));

// 16 M10K blocks
// Audiocpu 0x8000, 0x4000
dpram #(14, 8) ap_srom1
(.clock_a(clk_sys), .enable_a(), .wren_a(sl_wr_SROM1), .address_a(ioctl_addr[13:0]), .data_a(                ioctl_dout ), .q_a(             ),
    .clock_b(clk_sys), .enable_b(), .wren_b(           ), .address_b(   slv_SBA[13:0]), .data_b(                           ), .q_b(slv_SROM1    ));

// 16 M10K blocks
// Audiocpu 0xC000, 0x4000
dpram #(14, 8) ap_srom2
(.clock_a(clk_sys), .enable_a(), .wren_a(sl_wr_SROM2), .address_a(ioctl_addr[13:0]), .data_a(                ioctl_dout ), .q_a(             ),
    .clock_b(clk_sys), .enable_b(), .wren_b(           ), .address_b(   slv_SBA[13:0]), .data_b(                           ), .q_b(slv_SROM2    ));

// 8 M10K blocks
// Alphanumerics ROM
dpram  #(14,8) rom_alpha
(.clock_a(clk_sys), .enable_a(), .wren_a(sl_wr_2B   ), .address_a(ioctl_addr[13:0]), .data_a(                ioctl_dout ), .q_a(             ),
    .clock_b(clk_sys), .enable_b(), .wren_b(           ), .address_b(  slv_PA2B[13:0]), .data_b(                           ), .q_b(slv_PD2B     ));

// 1 M10K blocks
// Color PROM
dpram  #(9,8) rom_color
(.clock_a(clk_sys), .enable_a(), .wren_a(sl_wr_5A   ), .address_a(ioctl_addr[ 8:0]), .data_a(                ioctl_dout ), .q_a(             ),
    .clock_b(clk_sys), .enable_b(), .wren_b(           ), .address_b( slv_PADDR[ 8:0]), .data_b(                           ), .q_b(slv_PD4A     ));

// 1 M10K blocks
// Remap PROM
dpram  #(9,8) rom_remap
(.clock_a(clk_sys), .enable_a(), .wren_a(sl_wr_7A   ), .address_a(ioctl_addr[ 8:0]), .data_a(                ioctl_dout ), .q_a(             ),
    .clock_b(clk_sys), .enable_b(), .wren_b(           ), .address_b( slv_PADDR[ 8:0]), .data_b(                           ), .q_b(slv_PD7A     ));

// 1 M10K blocks
// EPROM
dpram  #(9,8) mp_eprom
(.clock_a(clk_sys), .enable_a(), .wren_a(sl_wr_ep1  ), .address_a(ioctl_addr[ 8:0]), .data_a(                ioctl_dout ), .q_a(             ),
    .clock_b(clk_sys), .enable_b(), .wren_b(sl_wr_ep2  ), .address_b( slv_MADEC[ 9:1]), .data_b(             slv_eprom_dout), .q_b(slv_eprom_din));

// SP-276 SP-277 SP-282 SP-286 SP-298 #########################################
// J102         J103        P104            P105               J106
//  1 +5         1 +5        1 +5           1 Self Test         1 +5
//  2 P2 Up      2 H_CLK2    2 Coin Ctr 1   2 Right Audio GND   2 N/C
//  3 P2 Down    3 H_DIR2    3 Coin Ctr 2   3 Right Audio       3 LED2
//  4 P2 Left    4 V_CLK2    4              4 Left  Audio GND   4 LED1
//  5 P1 Right   5 V_DIR2    5              5 KEY               5 KEY
//  6 P2 Right   6 H_CLK1    6              6 Left  Audio       6 SW5
//  7 P1 Left    7 H_DIR1    7                                  7 SW2   P2 Start/Whip
//  8 P1 Up      8 V_CLK1    8 KEY                              8 SW4
//  9 P1 Down    9 KEY       9 Left Coin                        9 SW1   P1 Start/Whip
// 10 KEY       10 V_DIR1   10 Right Coin                      10 SW3   Jump
// 11 GND       11 GND      11 GND                             11 GND

// Roadblasters
// Gas pedal, pot tied across +5V and GND with wiper going to J103-P
// Wheel connects CLK to P103-17 and DIR to P103-A
// Trigger button to P103-10 (SW2)
// Thumb button to P103-L (SW1)

reg [15:0] analog_0 = 0;
reg [15:0] analog_1 = 0;

reg [7:0] inputs;
reg [4:0] switches;
reg [2:0] adc_addr;
reg [7:0] adc_data;
reg wheel_mode;

reg analog_stick1_detected;
reg analog_stick2_detected;

always @(posedge clk_sys) begin

   analog_stick1_detected <= cont1_joy_s[15:0] != 0;
   analog_stick2_detected <= cont2_joy_s[15:0] != 0;

	if (game_id == GAME_ID_MARBLE_MADNESS)
	begin
		wheel_mode <= 0;
		// NC NC NC Action Action
		switches <= ({3'b111, ~(joy[4]), ~(joy[5])});
		// Directional analog type control from trackball which appears to be mounted with a 45 degree clockwise rotation
		// so U is UL, D is DR, L is LD and R is RU

		// Special condiiton -- user holds "Y" to speed up marble to "fast" speed.
		if (joy[7] == 1'b1) begin
			analog_speed_1 <= 2'b01;
		end
		else begin
			analog_speed_1 <= analog_speed;
		end
		
		if (!analog_stick1_detected || (joy[3:0] != 4'b0000)) begin
			// Digital controls
			analog_0[15:8] <= joy[0] ? 8'h80 : joy[1] ? 8'h7F : 8'h00;
			analog_0[7:0] <=  joy[2] ? 8'h80 : joy[3] ? 8'h7F : 8'h00;
		end
		else if (analog_stick1_detected) begin
			// Analog controls w/P1 left stick
			analog_0[15:8] <= cont1_joy_s[15:8] + 8'h80;
			analog_0[7:0]  <= cont1_joy_s[7:0] + 8'h80;
		end
		else begin
			analog_0 <= 16'b0;
		end

		// Special condiiton -- user holds "Y" to speed up marble to "fast" speed.
		if (joy2[7] == 1'b1) begin
			analog_speed_2 <= 2'b01;
		end
		else begin
			analog_speed_2 <= analog_speed;
		end
		
		if (!analog_stick2_detected || (joy2[3:0] != 4'b0000)) begin
			// Digital controls
			analog_1[15:8] <= joy2[0] ? 8'h80 : joy2[1] ? 8'h7F : 8'h00;
			analog_1[7:0] <=  joy2[2] ? 8'h80 : joy2[3] ? 8'h7F : 8'h00;
		end		
		else if (analog_stick2_detected) begin
			// Analog controls w/P2 left stick
			analog_1[15:8] <= cont2_joy_s[15:8] + 8'h80;
			analog_1[7:0]  <= cont2_joy_s[7:0] + 8'h80;
		end
		else begin
			analog_1 <= 16'b0;
		end
	end
	else if (game_id == GAME_ID_TEMPLE_OF_DOOM)
	begin
		// direction control inputs
		inputs <=
		// for Indy (105) shift them by one (000UDLR0) else default to (0000UDLR)
		({3'b0, joy[0],joy[1], joy[2],joy[3], 1'b0});
		// NC NC NC Whip Whip
		switches <= ({3'b111, ~(joy[4]), ~(joy[5])}); // "left" whip, "right" whip.
		// Directional on/off switch type control
		adc_data <= inputs[adc_addr]?8'hff:8'h80;
	end
	else if (game_id == GAME_ID_PETER_PACK_RAT)
	begin
		// direction control inputs
		inputs <=
		({4'b0,  joy[0],joy[1], joy[2],joy[3] });
		// NC NC Jump NC Throw
		switches <= ({2'b11, ~(joy[5]), 1'b1, ~(joy[4])});
		// Directional on/off switch type control
		adc_data <= inputs[adc_addr]?8'hff:8'h80;
	end
	else if (game_id == GAME_ID_ROAD_RUNNER)
	begin
		adc_data <= 8'h80; // default ADC value
		// direction control inputs
		inputs <=
		({4'b0,  joy[0],joy[1], joy[2],joy[3] });
		// NC NC NC Action Action
		switches <= ({3'b111, ~(joy[5]), ~(joy[4])});

		// Directional analog type control from joystick
		if (adc_addr==3'd0) begin
			if (inputs[0]) begin
				adc_data <= 8'h00; // R
			end
			else if (inputs[1]) begin
				adc_data <= 8'hFF; // L
			end
			else if (analog_stick1_detected) begin
				adc_data <= 8'hFF - cont1_joy_s[7:0];
			end
		end
		else if (adc_addr==3'd7) begin
			if (inputs[2]) begin
				adc_data <= 8'hFF; // D
			end
			else if (inputs[3]) begin
				adc_data <= 8'h00; // U
			end
			else if (analog_stick1_detected) begin
				adc_data <= cont1_joy_s[15:8];
			end			
		end 
		else begin
			adc_data <= 8'h80;
		end
	end
	else if (game_id == GAME_ID_ROAD_BLASTERS)
	begin
		wheel_mode <= 1;
		// NC NC NC Action Action
		switches <= ({3'b111, ~(joy[5] || joy[11] || joy[10]), ~(joy[7] || joy[9] || joy[8])}); // regular fire = b or r2 or L2, special fire = y or r1 or l1
		if (adc_addr == 3'd3) begin
			if (joy[4]) begin
				adc_data <= joy[4] ? 8'hFF : 8'h00;	// accelerate = a
			end
			else if (analog_stick1_detected && !cont1_joy_s[15]) begin
				adc_data <= 8'hFF - {cont1_joy_s[14:8], 1'b1}; // 0xFF == up, 0x00 == center.
			end
			else begin
				adc_data <= 8'h00;	// accelerate = a
			end
		end
		else begin
			adc_data <= 0;
		end
		
		// analog_0/1 are represented as *signed* int8 values.
		// 15:8 == v, 7:0 == h
		analog_0[15:8] <= 0;
		
		if (analog_stick1_detected) begin
			analog_0[7:0] <= cont1_joy_s[7:0] + 8'h80;
		end
		else if (!analog_stick1_detected || (joy[3:2] != 2'b00)) begin			
			analog_0[7:0] <= joy[2] ? 8'h80 : joy[3] ? 8'h7F : 8'h00;
		end 
		else begin
			analog_0[7:0] <= 8'h00;
		end

	end
end

wire [3:0]	clks, dirs;

// for Marble player 1 or Road Blasters wheel mode (analog joy or mouse only)
quad quad_p1
(
	.clk        (clk_core_7),
	.mode       (wheel_mode), // 1 = wheel, 0 = joy/mouse
	.joy        (analog_0),
	.mouse      (25'b0),
	.speed      (analog_speed_1), // Slow (00 = fastest, 11 = slowest)
	.XA         (dirs[0]),
	.XB         (clks[0]),
	.YA         (dirs[1]),
	.YB         (clks[1])
);

// only used by Marble player 2
quad quad_p2
(
	.clk        (clk_core_7),
	.mode       (1'b0),
	.joy        (analog_1),
	.mouse      (25'b0),
	.speed      (analog_speed_2),
	.XA         (dirs[2]),
	.XB         (clks[2]),
	.YA         (dirs[3]),
	.YB         (clks[3])
);

///////////////////////////////////////////////
// Video
///////////////////////////////////////////////

reg hblank_core, vblank_core;
wire hs_core, vs_core;
wire [3:0] r;
wire [3:0] g;
wire [3:0] b;
wire [3:0] i;

wire [3:0] r_conv;
wire [3:0] g_conv;
wire [3:0] b_conv;

// convert input video from 16bit IRGB to 12 bit RGB
RGBI RCONV (.ADDR({i,r}), .DATA(r_conv));
RGBI GCONV (.ADDR({i,g}), .DATA(g_conv));
RGBI BCONV (.ADDR({i,b}), .DATA(b_conv));

reg video_de_reg;
reg video_hs_reg;
reg video_vs_reg;
reg [23:0] video_rgb_reg;

reg hs_prev;
reg vs_prev;

assign video_rgb_clock = clk_core_7;
assign video_rgb_clock_90 = clk_core_7_90deg;

assign video_de = video_de_reg;
assign video_hs = video_hs_reg;
assign video_vs = video_vs_reg;
assign video_rgb = video_rgb_reg;
assign video_skip = 0;

always @(posedge clk_core_7) begin
    video_de_reg <= 0;
    
    video_rgb_reg <= 24'd0;
    if (~(~vblank_core || ~hblank_core)) begin
        video_de_reg <= 1;
        video_rgb_reg[23:16]  <= {2{r_conv}};
        video_rgb_reg[15:8]   <= {2{g_conv}};        
        video_rgb_reg[7:0]    <= {2{b_conv}};
	end    
	
    video_hs_reg <= ~hs_prev && ~hs_core;
    video_vs_reg <= ~vs_prev && ~vs_core;
    hs_prev <= ~hs_core;
    vs_prev <= ~vs_core;
end


///////////////////////////////////////////////
// Audio
///////////////////////////////////////////////

wire [15:0] audio_l;
wire [15:0] audio_r;

sound_i2s #(
    .CHANNEL_WIDTH(16),
    .SIGNED_INPUT(1)
) sound_i2s (
    .clk_74a(clk_74a),
    .clk_audio(clk_14),
    
    .audio_l(audio_l),
    .audio_r(audio_r),

    .audio_mclk(audio_mclk),
    .audio_lrck(audio_lrck),
    .audio_dac(audio_dac)
);


///////////////////////////////////////////////
// Control
///////////////////////////////////////////////

wire [15:0] joy;
wire [15:0] joy2;
wire [31:0] cont1_joy_s;
wire [31:0] cont2_joy_s;

synch_3 #(
    .WIDTH(16)
) cont1_key_s (
    cont1_key,
    joy,
    clk_14
);

synch_3 #(
    .WIDTH(16)
) cont2_key_s (
    cont2_key,
    joy2,
    clk_14
);

synch_3 #(
    .WIDTH(32)
) cont1_joy_sync (
    cont1_joy,
    cont1_joy_s,
    clk_14
);

synch_3 #(
    .WIDTH(32)
) cont2_joy_sync (
    cont2_joy,
    cont2_joy_s,
    clk_14
);


wire m_service;
assign m_service = service_mode;

wire [2:0] m_coins;
wire m_coin_1;
wire m_coin_2;

assign m_coin_1 = joy[14];
assign m_coin_2 = joy2[14];

assign m_coins = {1'b1, ~(m_coin_2), ~(m_coin_1)};

///////////////////////////////////////////////
// Board
///////////////////////////////////////////////

FPGA_ATARISYS1 atarisys1
(
	.I_SLAP_TYPE (
		game_id == GAME_ID_MARBLE_MADNESS ? 103 :
		game_id == GAME_ID_TEMPLE_OF_DOOM ? 105 : 
		game_id == GAME_ID_ROAD_RUNNER ? 108 : 
		game_id == GAME_ID_ROAD_BLASTERS ? 110 : 
		game_id == GAME_ID_PETER_PACK_RAT ? 107 : 
		103		
	),
	
	// System Clock
	.I_CLK_7M    (clk_core_7),
	.I_CLK_14M   (clk_14),

	// Active high reset
	.I_RESET     (~reset_n || manual_reset),
	
	// SELFTEST, COIN_AUX, COIN_L, COIN_R, SW[5:1] active low
	.I_SELFTESTn (~m_service),
	.I_COIN      (m_coins),
	// J106 SW5,4,3,2,1 = NC, NC, Jump (NC), Whip2/Start2, Whip1/Start1
	.I_SW        (switches),

	// Each ADC input is biased to VCC/2 with resistors and can be pulled high or low or any value in between for analog controllers
	// Some  games use an ADC channel as up/idle/down or left/idle/right control (0xF0 / 0x80 / 0x00)
	// Other games use an ADC channel as a simple on/off (0xFF / 0x00)
	.O_ADC_ADDR  (adc_addr),
	.I_ADC_DATA  (adc_data),

	// P103 LETA trackball inputs active low
	.I_CLK       (clks), // HCLK2,VCLK2,HCLK1,VCLK1
	.I_DIR       (dirs), // HDIR2,VDIR2,HDIR1,VDIR1

	.O_LEDS      (),

	.O_AUDIO_L   (audio_l),
	.O_AUDIO_R   (audio_r),

	.O_VIDEO_I   (i),
	.O_VIDEO_R   (r),
	.O_VIDEO_G   (g),
	.O_VIDEO_B   (b),
	.O_HSYNC     (hs_core),
	.O_VSYNC     (vs_core),
	.O_CSYNC     (),
	.O_HBLANK    (hblank_core),
	.O_VBLANK    (vblank_core),

	.O_ADDR2B    (slv_PA2B),
	.I_DATA2B    (slv_PD2B),

	.O_EEPDATA   (slv_eprom_dout),
	.I_EEPDATA   (slv_eprom_din),
	.O_EEPWR     (sl_wr_ep2),

	// CART memory interface
	.O_ROMn      (slv_ROMn),  // maincpu ROM selects
	.O_MA18n     (sl_MA18n),
	.I_MDATA     (slv_MDATA),
	.O_SROMn     (slv_SROMn), // sound ROM selects
	.O_SBA       (slv_SBA),
	.O_MADEC     (slv_MADEC), // SLAPSTIC decoded mem addr

	// PROMs
	.O_PADDR     (slv_PADDR),
	.I_PD4A      (slv_PD4A),
	.I_PD7A      (slv_PD7A),

	// sound ROMs
	.I_SDATA     (slv_SDATA),

	// video ROMs
	.O_VADDR     (slv_VADDR),
	.I_VDATA     (slv_VDATA_s)
);
	 

///////////////////////////////////////////////
// Clocks
///////////////////////////////////////////////

// "Intermediate" clock used to enable us to use an integer PLL for every other clock.
wire    pll_bridge_locked;
wire 	clk_root;
mf_pllbridge mp0 (
    .refclk         ( clk_74a ),
    .rst            ( 0 ),
    .outclk_0       ( clk_root ),
    .locked         ( pll_bridge_locked )
);

wire    clk_core_7;
wire    clk_core_7_90deg;
wire    clk_sys;  
wire 	clk_93;	
wire    clk_14;
wire    pll_core_locked;

assign clk_sys = clk_93;
mf_pllbase mp1 (
    .refclk         ( clk_root ),
    .rst            ( ~pll_bridge_locked ),

    .outclk_2       ( clk_core_7_90deg ),
	.outclk_1       ( clk_core_7 ),
    .outclk_0       ( clk_14 ),

    .locked         ( pll_core_locked )
);


wire    pll_sdram_locked;
mf_pllsdram mp2 (
    .refclk         ( clk_root ),
    .rst            ( ~pll_bridge_locked ),

	// .outclk_1	  	( dram_clk),
    .outclk_0	  	( clk_93 ),

	.locked         ( pll_sdram_locked )
);

endmodule
