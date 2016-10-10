`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 		 ITEP
// Engineer: 		 SvirLex
// 
// Create Date:    19:11:52 09/15/2014 
// Design Name:    fpga_maind
// Module Name:    fpga_maind
// Project Name:   wfd125
// Target Devices: xc6SLX45T-2-FGG484
//
// Revision 0.01 - File Created
// Additional Comments: 
//
//		FP connectors (top to bottom):
//			Ethernet (not used)
//			FP pairs
//			Input0, chans 0-15
//			Input1, chans 16-31
//			Input2, chans 32-47
//			Input3, chans 48-63
//		
//		LED assignments (top to bottom):
//		0 (YEL)	memory error or full or token recieving error
//		1 (GRN)	VME access, either to main FPGA or to CPLD
//		2 (GRN)	module recieved master trigger
//		3 (GRN)	lit when module feels NO inhibit
//
//		Front panel RJ 10P
//		Pin#	Net		Assignment		Term
//		1		GND
//		2,3	FP0/FP1	Triger P/N - 125/8 MHz here	Yes	
//		4,5	FP2/FP3	Inhibit P/N			Yes
//		6,7	FP4/FP5	Trigger to Prop			Yes
//		8,9	FP6/FP7	Clock P/N			Yes
//		10		GND
//
//		Back panel USERDEF (P2 connector)
//		Pin#	Net		Assignment		Term
//		A1,A2	UDEF0, UDEF2	Inhibit P/N		No
//		C1,C2	UDEF1, UDEF3	unused P/N		No
//		A4,C3 	UDEF6, UDEF5	Trigger P/N - 125/8 MHz	No
//		C4,C5	UDEF7, UDEF9	unused P/N		No
//		A3	UDEF4		unused			N/A
//		A5	UDEF8		unused			N/A
//		A24,C24	VCLKP, VCLKN	Clock P/N		Yes
//
//		CPLD to Xilinx lines (all this direction)
//		C2X[5:0]	GA[5:0] noninverted
//		C2X[6]	CPLD access from VME
//		C2X[7]	Global reset, active LOW
//
//		InterXilinx communication
//		ICX[1:0]	pseudo-differential serial trigger to channels
//		ICX[2]		SPI CS (frame, to channels)
//		ICX[3]		SPI serial data (bidir)
//		ICX[4]		SPI clocks (to channels)
//		ICX[5]		prepheral wishbone reset (to channels)
//		ICX[6]		INH to channels
//		ICX[7]   	Test pulse
//		ICX[15:8]	not used
//
//////////////////////////////////////////////////////////////////////////////////



module fpga_maind(
	// VME interface
	// Address
	inout [31:0] XA, 
	// Address modifier
	input [5:0] XAM,
	// Geographical addr (useless as now)
	input [5:0] XGA,	// active low
	// Data
	inout [31:0] XD, 
	// Strobes
	input XAS,		// active low
	input [1:0] XDS,	// active low
	input XWRITE,		// 0 = write
	output XDTACK,		// active low
	output XDTACKOE,	// active low
	output ADIR,		// 0 - inward
	output DDIR,		// 0 - inward
	// Errors/resets
	input XRESET,
	output XBERR,
	output XRETRY,
	input XRESP,
	// Interrupt handling
	input XIACK,
	input XIACKIN,
	output IACKPASS,
	// Interrupts
	output [5:0] XIRQ,
	// Interconnection to CPLD
	input [7:0] C2X,
	// Parallel lines to other FPGAs
	// 0-1, 14-15: "good pairs"
	// 10-11, 12-13: "satisfactory pair"
	inout [15:0] ICX,
	// Fast serial connections and CLK
	// Main Clock
	input [1:0] RCLK,
	// Recievers
	input [1:0] RX0,
	input [1:0] RX1,
	input [1:0] RX2,
	input [1:0] RX3,
	// Transmitters
	output [1:0] TX0,
	output [1:0] TX1,
	output [1:0] TX2,
	output [1:0] TX3,
	// Serial interfaces
	// Clock Buffer I2C
	inout CBUFSCL,
	inout CBUFSDA,
	// DAC SPI
	output BDACC,
	output BDACD,
	output BDACCS,
	// Clock selection
	output ECLKSEL,
	output OCLKSEL,
	output CLKENFP,
	output CLKENBP,
	output CLKENBFP,
	// Front Panel
	// Indication LEDS (LED0 - Yellow)
	output [3:0] LED,
	// Front panel pairs (even-odd)
	inout [5:0] FP,
	// Back panel
	// 0-2, 1-3, 7-9 : "next-by-next" BP pair, X-pair, board pair
	// 6-5 : "real" BP pair, X-pair, but NOT board pair
	inout [9:0] USRDEF,
	// FLASH/Config interface
	input INIT,
	input [1:0] M,
	input DOUT,
	input FLASHCS,
	input FLASHCLK,
	input [3:0] FDAT,
	// Ethernet PHY interface
	// Input
	input PHYRXCLK,
	input PHYRXDVLD,
	input [3:0] PHYRXD,
	// Output
	output PHYTXCLK,
	output PHYTXENB,
	output [3:0] PHYTXD,
	// Slow interface
	inout PHYMDIO,
	output PHYMDC,
	output PHYRST,
	input PHYINT,
	// SDRAM interface
	// Address
	output [14:0] MEMA,
	// Bank addr
	output [2:0] MEMBA,
	// Data
	inout [15:0] MEMD,
	// Other single ended
	output MEMRST,
	output MEMCKE,
	output MEMWE,
	output MEMODT,
	output MEMRAS,
	output MEMCAS,
	output MEMUDM,
	output MEMLDM,
	// Pairs
	output [1:0] MEMCK,
	output [1:0] MEMUDQS,
	output [1:0] MEMLDQS,
	// Impedance matching
	output MEMZIO,
	output MEMRZQ,
	// Test points
	output [5:1] TP
);

	wire		wb_clk;
	wire    	wb_rst;
	
`include "wb_intercon.vh"
`include "version.vh"

	wire		CLK125;
	wire		CLKMCB;
	reg [3:0]	ledp;
	reg 		once = 1;
	reg 		greset = 1;
	
	wire [5:0]	VME_GA_i;
	assign VME_GA_i = ~C2X[5:0];
	
	wire		VME_BERR_o;
	wire		VME_DTACK_n_o;
	wire         	VME_RETRY_n_o;
	wire         	VME_LWORD_n_i;
	wire         	VME_LWORD_n_o;
	wire [31:1]  	VME_ADDR_i;
	wire [31:1]  	VME_ADDR_o;
	wire [31:0]  	VME_DATA_i;
	wire [31:0]  	VME_DATA_o;
	wire         	VME_DTACK_OE_o;
	wire         	VME_DATA_DIR_o;
	wire         	VME_DATA_OE_N_o;
	wire         	VME_ADDR_DIR_o;
	wire         	VME_ADDR_OE_N_o;
	wire         	VME_RETRY_OE_o;
	wire [7:1]   	VME_IRQ_o;
	wire [7:0]   	debug;
	wire [1:0]   	dummy_vme_addr;
	wire         	CBUFSDA_o;
	wire		CBUFSCL_o;
	wire         	CBUFSDA_en;
	wire		CBUFSCL_en;
	
	wire [63:0]  	gtp_data_o;	// data from GTP reciever
	wire [3:0]   	gtp_kchar_o;	// k-char signature from GTP reciever
	wire [15:0]  	gtp_token;	// token data and status to channels
	wire [15:0]	trg_data;	// data from triggen to trigger block fifo in memory
	wire		trg_valid;	// valid accompanying the above
//	wire [15:0]	tok_data;	// data from token synchro module
//	wire		tok_valid;	// valid accompanying the above
	wire [14:0]  	usr_word;	// user word from CSR to be put to trigger block in memory
	wire		trigger;	// master trigger from triggen to commutation in csreg
	wire		inhibit;	// inhibit from triggen to commutation in csreg
	wire [9:0]	token;		// 10 bit token of recieved trigger
	wire		tok_rdy;	// token recieved
	wire		tok_err;	// token error
	wire		mem_status;	// memory errors
	reg [31:0]  	CNT = 0;
	reg [5:1] 	tpdebug = 0;
	wire [15:13] 	CSR_BITS;	// CSR bits 15:13
	wire         	auxtrig;	// front panel aux trigger input
	wire		clk125_8;	// synchro frequency 125/8 MHz
	reg [2:0]	clk125_div = 0;

	always @ (posedge CLK125) begin
		tpdebug[5] <= 0;
		tpdebug[4] <= gtp_data_o[63] & gtp_kchar_o[3];
		tpdebug[3] <= gtp_data_o[47] & gtp_kchar_o[2];
		tpdebug[2] <= gtp_data_o[31] & gtp_kchar_o[1];
		tpdebug[1] <= gtp_data_o[15] & gtp_kchar_o[0];
	end
	assign TP = 0;

	assign IACKPASS = 1'bz;
	assign PHYTXCLK = 1'bz;
	assign PHYTXENB = 1'bz;
	assign PHYTXD  = 4'hz;
	assign PHYMDIO = 1'bz;
	assign PHYMDC  = 1'bz;
	assign PHYRST  = 1'bz;
	assign BDACC = 1'bz;
	assign BDACD = 1'bz;
	assign BDACCS = 1'bz;

	csreg reg_csr (
		.wb_clk		(wb_clk), 
		.wb_cyc 	(wb_m2s_reg_csr_cyc), 
		.wb_stb 	(wb_m2s_reg_csr_stb), 
		.wb_adr 	(wb_m2s_reg_csr_adr[2]), 
		.wb_we  	(wb_m2s_reg_csr_we), 
		.wb_dat_i 	(wb_m2s_reg_csr_dat), 
		.wb_dat_o 	(wb_s2m_reg_csr_dat), 
		.wb_ack 	(wb_s2m_reg_csr_ack),
		//
		.gen_o   	(CSR_BITS),
		.gen_i		({22'h0, token}),
		// assigned outputs
		.pwb_rst	(ICX[5]),	// peripheral wishbone reset
	 	.usr_word	(usr_word),	// user word to be put to trigger memory block
		// inputs from triggen
		.trig		(clk125_8),	// 125/8 MHz
		.inh		(inhibit),
		.auxtrig 	(auxtrig),
		// front panel signals
		.trig_FP	(FP[1:0]),	// 125/8 MHz
		.inh_FP		(FP[3:2]),
		.trig1_FP 	(FP[5:4]),
		// back panel signals
		.trig_BP	({USRDEF[5], USRDEF[6]}),	// 125/8 MHz
		.inh_BP		({USRDEF[2], USRDEF[0]}),
		// signals to peripheral X's
		.trig_ICX	(ICX[1:0]),			// 125/8 MHz
		.inh_ICX	(ICX[6]),
		// outputs to drive CLK muxes
		.ECLKSEL	(ECLKSEL),
		.OCLKSEL	(OCLKSEL),
		.CLKENFP	(CLKENFP),
		.CLKENBP	(CLKENBP),
		.CLKENBFP	(CLKENBFP),
		.testpulse	(ICX[7])
	);
	
	assign wb_s2m_reg_csr_rty = 0;
	assign wb_s2m_reg_csr_err = 0;
	
	inoutreg reg_ver (
		.wb_clk (wb_clk), 
		.wb_cyc (wb_m2s_reg_ver_cyc), 
		.wb_stb (wb_m2s_reg_ver_stb), 
		.wb_adr (wb_m2s_reg_ver_adr[2]), 
		.wb_we  (wb_m2s_reg_ver_we), 
		.wb_dat_i (wb_m2s_reg_ver_dat), 
		.wb_dat_o (wb_s2m_reg_ver_dat), 
		.wb_ack (wb_s2m_reg_ver_ack),
		.reg_o   (),
		.reg_i	(VERSION)
	);
	assign wb_s2m_reg_ver_rty = 0;
	assign wb_s2m_reg_ver_err = 0;

	assign gtp_token = (tok_rdy) ? {5'b10000, tok_err, token} : 16'h00BC;

	gtprcv4 # (.WB_DIVIDE(2), .WB_MULTIPLY(2))	// 125 MHz for wishbone
	UGTP (
		.rxpin	({RX3, RX2, RX1, RX0}),	// input data pins
		.txpin	({TX3, TX2, TX1, TX0}),	// output data pins
		.clkpin	(RCLK),			// input clock pins - tile0 package pins A10/B10
		.clkout	(CLK125),		// output 125 MHz clock
		.clkwb  (wb_clk),		// output clock for wishbone
		.gck_o  (CLKMCB),		// clock to MCB
//			Do remapping here
		.data_o		({gtp_data_o[15:0], gtp_data_o[31:16], gtp_data_o[47:32], gtp_data_o[63:48]}),	// data received
		.charisk_o	({gtp_kchar_o[0], gtp_kchar_o[1], gtp_kchar_o[2], gtp_kchar_o[3]}), 		// K-char signature received
		.data_i		({4{gtp_token}}),		// data for sending, token with its status when token recieved
		.charisk_i	({4{~tok_rdy}}),		// K-char signature for sending, not k-char when token recieved
		.locked  ()
    );

/***************************************************************
			VME
****************************************************************/
	assign XBERR         	= ~VME_BERR_o;
	assign XDTACK        	= VME_DTACK_OE_o ? VME_DTACK_n_o : 1'bz;
	assign XDTACKOE      	= VME_DTACK_OE_o ? 1'b0 : 1'bz;
	assign XRETRY        	= VME_RETRY_n_o;
	assign XA            	= (VME_ADDR_DIR_o) ? {VME_ADDR_o, VME_LWORD_n_o} : 32'bZ;
	assign ADIR          	= VME_ADDR_DIR_o;
	assign VME_ADDR_i    	= XA[31:1];
	assign VME_LWORD_n_i 	= XA[0];
	assign XD            	= (VME_DATA_DIR_o) ? VME_DATA_o : 32'bZ;
	assign DDIR          	= (VME_DATA_DIR_o) ? 1'b1 : 1'bz;
	assign VME_DATA_i    	= XD;
	assign XIRQ     	= {VME_IRQ_o[7:5], VME_IRQ_o[3:1]};
	assign wb_rst 		= greset;	// wb_rst is now active high
	assign wb_m2s_VME64xCore_Top_adr[1:0] = 2'b00;

VME64xCore_Top #(
	.g_clock (13), 	    		// clock period (ns)
	.g_wb_data_width (32),		// WB data width:
	.g_wb_addr_width (32),		// WB address width:
	 .g_endian        (3),		// Swap Word+ Swap Byte   eg: 01234567 become 32107654
	 .g_IRQ_level     (0),		// no IRQ so far, range 1-7
	 .g_IRQ_vector    (0)		// no IRQ so far
) vme (
	.clk_i		(wb_clk),
	.rst_n_i	(~wb_rst),

	.VME_AS_n_i	(XAS),
	.VME_RST_n_i    (XRESET),
	.VME_WRITE_n_i  (XWRITE),
	.VME_AM_i       (XAM),
	.VME_DS_n_i     (XDS),
	.VME_GA_i       (VME_GA_i),
	.VME_BERR_o     (VME_BERR_o),

	.VME_DTACK_n_o  (VME_DTACK_n_o),
	.VME_RETRY_n_o  (VME_RETRY_n_o),
	.VME_LWORD_n_i  (VME_LWORD_n_i),
	.VME_LWORD_n_o  (VME_LWORD_n_o),
	.VME_ADDR_i     (VME_ADDR_i),
	.VME_ADDR_o     (VME_ADDR_o),
	.VME_DATA_i     (VME_DATA_i),
	.VME_DATA_o     (VME_DATA_o),
	.VME_IRQ_o      (VME_IRQ_o),
	.VME_IACKIN_n_i (1'b1),
	.VME_IACK_n_i   (1'b1),
	.VME_IACKOUT_n_o(),

	.VME_DTACK_OE_o (VME_DTACK_OE_o),
	.VME_DATA_DIR_o (VME_DATA_DIR_o),
	.VME_DATA_OE_N_o(VME_DATA_OE_N_o),
	.VME_ADDR_DIR_o (VME_ADDR_DIR_o),
	.VME_ADDR_OE_N_o(VME_ADDR_OE_N_o),
	.VME_RETRY_OE_o (VME_RETRY_OE_o),

	.DAT_i          (wb_s2m_VME64xCore_Top_dat),
	.DAT_o          (wb_m2s_VME64xCore_Top_dat),
	.ADR_o          ({dummy_vme_addr, wb_m2s_VME64xCore_Top_adr[31:2]}),
	.CYC_o          (wb_m2s_VME64xCore_Top_cyc),
	.ERR_i          (wb_s2m_VME64xCore_Top_err),
	.RTY_i          (wb_s2m_VME64xCore_Top_rty),
	.SEL_o          (wb_m2s_VME64xCore_Top_sel),
	.STB_o          (wb_m2s_VME64xCore_Top_stb),
	.ACK_i          (wb_s2m_VME64xCore_Top_ack),
	.WE_o           (wb_m2s_VME64xCore_Top_we),
	.STALL_i        (wb_s2m_VME64xCore_Top_stall),

	.INT_ack_o      (),
	.IRQ_i          (1'b0),
	.debug          (debug)		// debug[0] is card_sel: access from VME decoded
);

	assign wb_m2s_VME64xCore_Top_cti = 0;
	assign wb_m2s_VME64xCore_Top_bte = 0;

//		clock buffer control
i2c_master_slave UI2C (
	.wb_clk_i  (wb_clk), 
	.wb_rst_i  (wb_rst),		// active high 
	.arst_i    (1'b0), 		// active high
	.wb_adr_i  (wb_m2s_i2c_ms_cbuf_adr[4:2]), 
	.wb_dat_i  (wb_m2s_i2c_ms_cbuf_dat[7:0]), 
	.wb_dat_o  (wb_s2m_i2c_ms_cbuf_dat[7:0]),
	.wb_we_i   (wb_m2s_i2c_ms_cbuf_we),
	.wb_stb_i  (wb_m2s_i2c_ms_cbuf_stb),
	.wb_cyc_i  (wb_m2s_i2c_ms_cbuf_cyc), 
	.wb_ack_o  (wb_s2m_i2c_ms_cbuf_ack), 
	.wb_inta_o (),
	.scl_pad_i (CBUFSCL), 
	.scl_pad_o (CBUFSCL_o), 
	.scl_padoen_o (CBUFSCL_en), 	// active low ?
	.sda_pad_i (CBUFSDA), 
	.sda_pad_o (CBUFSDA_o), 
	.sda_padoen_o (CBUFSDA_en)		// active low ?
);

	assign CBUFSCL = (!CBUFSCL_en) ? (CBUFSCL_o) : 1'bz;
	assign CBUFSDA = (!CBUFSDA_en) ? (CBUFSDA_o) : 1'bz;
	assign wb_s2m_i2c_ms_cbuf_dat[31:8] = 0;		// pad high data with zeroes
	assign wb_s2m_i2c_ms_cbuf_err = 0;
	assign wb_s2m_i2c_ms_cbuf_rty = 0;
	
   gentrig UTRIG (
		// GTP reciever data and k-char info
		.gtp_clk	(CLK125),
		.gtp_dat	(gtp_data_o),
		.kchar		(gtp_kchar_o),
		.auxtrig    	(auxtrig),
		// intrface to memory fifo
		.trg_dat	(trg_data),
		.trg_vld	(trg_valid),
		// user word from CSR
		.usr_word	(usr_word),
		// WB
		.wb_clk  	(wb_clk), 
		.wb_rst  	(wb_rst), 
		.wb_cyc  	(wb_m2s_triggen_cyc), 
		.wb_stb  	(wb_m2s_triggen_stb),
		.wb_adr  	(wb_m2s_triggen_adr[3:2]), 
		.wb_we   	(wb_m2s_triggen_we),
		.wb_ack  	(wb_s2m_triggen_ack), 
		.wb_dat_i  	(wb_m2s_triggen_dat), 
		.wb_dat_o  	(wb_s2m_triggen_dat),
		// trigger and inhibit
		.trigger	(trigger),
		.inhibit	(inhibit)
	);

	assign wb_s2m_triggen_err = 0;
	assign wb_s2m_triggen_rty = 0;

//		SPI to DAC
wire [6:0] empty_spi_csa;
xspi_master  #(
	.CLK_DIV 	(49),
	.CLK_POL 	(1'b1)
) dac_spi (
	.wb_rst    	(wb_rst),
	.wb_clk    	(wb_clk),
	.wb_we     	(wb_m2s_dac_spi_we),
	.wb_dat_i  	(wb_m2s_dac_spi_dat[15:0]),
	.wb_dat_o  	(wb_s2m_dac_spi_dat[15:0]),
	.wb_cyc		(wb_m2s_dac_spi_cyc),
	.wb_stb		(wb_m2s_dac_spi_stb),
	.wb_ack		(wb_s2m_dac_spi_ack),
	.spi_dat   	(BDACD),
	.spi_clk   	(BDACC),
	.spi_cs    	({empty_spi_csa, BDACCS}),
	.wb_adr		(wb_m2s_dac_spi_adr[2])
);
	assign wb_s2m_dac_spi_err = 0;
	assign wb_s2m_dac_spi_rty = 0;	
	assign wb_s2m_dac_spi_dat[31:16] = 0;	

//		SPI to other xilinxes
wire [6:0] empty_spi_csb;
xspi_master  #(
	.CLK_DIV 	(49),
	.CLK_POL 	(1'b0)
) icx_spi (
	.wb_rst    	(wb_rst),
	.wb_clk    	(wb_clk),
	.wb_we     	(wb_m2s_icx_spi_we),
	.wb_dat_i  	(wb_m2s_icx_spi_dat[15:0]),
	.wb_dat_o  	(wb_s2m_icx_spi_dat[15:0]),
	.wb_cyc		(wb_m2s_icx_spi_cyc),
	.wb_stb		(wb_m2s_icx_spi_stb),
	.wb_ack		(wb_s2m_icx_spi_ack),
	.spi_dat   	(ICX[3]),
	.spi_clk   	(ICX[4]),
	.spi_cs    	({empty_spi_csb, ICX[2]}),
	.wb_adr		(wb_m2s_icx_spi_adr[2])
);
	assign wb_s2m_icx_spi_err = 0;
	assign wb_s2m_icx_spi_rty = 0;	
	assign wb_s2m_icx_spi_dat[31:16] = 0;	

memory #(
	.FIFO_MBITS	(12),
	.READ_BURST_LEN	(8)
) sdram (
	.wb_clk		(wb_clk),
	.wb_rst    	(wb_rst),
	// Memory WishBone
	.wbm_cyc   	(wb_m2s_sdram_cyc),
	.wbm_stb	(wb_m2s_sdram_stb),
	.wbm_we		(wb_m2s_sdram_we),
	.wbm_sel   	(wb_m2s_sdram_sel),
	.wbm_addr  	(wb_m2s_sdram_adr),	// byte address is 29 bit wide for 4 Gbits (512 Mbytes)
	.wbm_dat_i	(wb_m2s_sdram_dat),
	.wbm_ack	(wb_s2m_sdram_ack),
	.wbm_stall 	(wb_s2m_sdram_stall),
	.wbm_dat_o	(wb_s2m_sdram_dat),
	// Register WishBone
	.wbr_cyc	(wb_m2s_reg_fifo_cyc),
	.wbr_stb	(wb_m2s_reg_fifo_stb),
	.wbr_we		(wb_m2s_reg_fifo_we),
	.wbr_addr	(wb_m2s_reg_fifo_adr[3:2]),
	.wbr_dat_i	(wb_m2s_reg_fifo_dat),
	.wbr_ack	(wb_s2m_reg_fifo_ack),
	.wbr_dat_o	(wb_s2m_reg_fifo_dat),
	// GTP data
	// reciever clock 125 MHz
	.gtp_clk	(CLK125),
	// recied data from 4 recievers
	.gtp_dat	(gtp_data_o),
	// recieved data valid (not a comma)
	.gtp_vld	(~gtp_kchar_o),
	// trigger data from triggen module
	.trg_dat	(trg_data),
	// trigger data valid
	.trg_vld	(trg_valid),
	// token synchronization
	.tok_dat	(0),
	// trigger data valid
	.tok_vld	(0),
	// SDRAM interface
	.mcb_clk	(CLKMCB),
	// Address
	.MEMA		(MEMA),
	// Bank addr
	.MEMBA		(MEMBA),
	// Data
	.MEMD		(MEMD),
	// Other single ended
	.MEMRST		(MEMRST),
	.MEMCKE		(MEMCKE),
	.MEMWE		(MEMWE),
	.MEMODT		(MEMODT),
	.MEMRAS		(MEMRAS),
	.MEMCAS		(MEMCAS),
	.MEMUDM		(MEMUDM),
	.MEMLDM		(MEMLDM),
	// Pairs
	.MEMCK		(MEMCK),
	.MEMUDQS	(MEMUDQS),
	.MEMLDQS	(MEMLDQS),
	// Impedance matching
	.MEMZIO		(MEMZIO),
	.MEMRZQ		(MEMRZQ),
	// current status
	.status		(mem_status),
	.tp		()
);
	
	assign	wb_s2m_sdram_err = 0;
	assign	wb_s2m_sdram_rty = 0;
	assign	wb_s2m_reg_fifo_err = 0;
	assign	wb_s2m_reg_fifo_rty = 0;

//		Global counter and 125/8 generation
	always @(posedge CLK125) begin
		if (!once) greset <= !C2X[7];
		CNT <= CNT + 1;
		if (CNT == 1000000) once = 0;
		clk125_div <= clk125_div + 1;
	end;
	assign clk125_8 = clk125_div[2];

// serial trigger capture
trigrcv trigcap (
	.clk		(CLK125),
	.ser_trig_in	(trigger),
	.token		(token),
	.tok_rdy	(tok_rdy),
	.tok_err	(tok_err)
);

//		LEDs
	always @(posedge CLK125) begin
		ledp[0] <= mem_status | wb_rst | tok_err;	// memory errors or trigger recieving error
		ledp[1] <= debug[0] | C2X[6];			// VME access to this card or to CPLD
		ledp[2] <= tok_rdy;				// master trigger recieved
		ledp[3] <= ~ICX[6];				// not INH
	end

	genvar i;
	generate
		for (i = 0; i < 4; i = i + 1) begin : GLED
			ledengine ULED (
				.clk	(CLK125),
				.led  	(LED[i]),
				.trig 	(ledp[i])
			);
		end
	endgenerate

endmodule
