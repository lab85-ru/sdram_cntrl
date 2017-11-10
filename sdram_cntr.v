// sdram controller
// operation read/write - full page, or len
// 
//
// v1.0 2016

`timescale 1ns/1ps
`define DEBUG

module sdram_cntr
#(
    parameter SDRAM_ROW_WIDTH_BIT  = 13,
    parameter SDRAM_COL_WIDTH_BIT  = 9,
	 parameter SDRAM_BANK_WIDTH_BIT = 2,
	 parameter SDRAM_DATA_WIDTH_BIT = 16,
	 parameter SDRAM_CAS            = 3,
	 
    parameter tPOWERUP   = 16'd14285,
	 parameter tREF       = 64,    // ms
	 parameter tREF_COUNT = 8192,  // refresh counter (cycle refresh)
	 parameter tCK        = 8      // clk period 7 ns (for calcul refresh interval)
	 
)
(
    input wire i_clk,
	 
	 // interface for manager RAM
	 input wire [SDRAM_DATA_WIDTH_BIT-1:0] i_ram_data,
	 output wire [SDRAM_DATA_WIDTH_BIT-1:0] o_ram_data,

	 input wire [SDRAM_ROW_WIDTH_BIT + SDRAM_COL_WIDTH_BIT + SDRAM_BANK_WIDTH_BIT - 1:0] i_ram_addr,
	 input wire [SDRAM_COL_WIDTH_BIT:0] i_ram_len, // i_ram_len = SDRAM_COL_WIDTH_BIT + 1

	 input wire i_ram_read_req,
	 output wire o_ram_read_valid,
	 
	 input wire i_ram_write_req,
	 output wire o_ram_write_valid,
	 
	 output wire o_ready,
	 
	 // interface for SDRAM
	 input wire  [SDRAM_DATA_WIDTH_BIT-1:0] i_sdram_data,
	 output wire [SDRAM_DATA_WIDTH_BIT-1:0] o_sdram_data,
	 output wire [SDRAM_ROW_WIDTH_BIT-1:0] o_sdram_addr,
	 output wire o_sdram_clk,
	 output wire o_sdram_cke,
	 output wire o_sdram_cs_l,
	 output wire o_sdram_ras_l,
	 output wire o_sdram_cas_l,
	 output wire o_sdram_we_l,
	 output wire o_sdram_buf_we, // signal pereklucheniya buffera SDRAM na peredachu

	 output wire [SDRAM_BANK_WIDTH_BIT-1:0] o_sdram_ba,
//	 output wire [SDRAM_DATA_WIDTH_BIT-1:0] o_sdram_dq,
	 output wire o_sdram_dqml,
	 output wire o_sdram_dqmh
`ifdef DEBUG	 
	 ,
	 output wire [15:0] d_wait_cnt,
	 output wire d_wait_cnt_en,
	 output wire [7:0] d_st,
	 output wire d_ref_req
`endif
);

localparam tRP      = SDRAM_CAS;
localparam tRC      = 6 + SDRAM_CAS;
localparam tMRD     = 2;
localparam tRCD     = SDRAM_CAS;

//-----------------------------------------------------------------
//   cmd signal bits     CS RAS CAS WE
localparam CMD_DESL  = 4'b1_1_1_1; // device deselect
localparam CMD_NOP   = 4'b0_1_1_1; // no operation
localparam CMD_BST   = 4'b0_1_1_0; // burst stop
localparam CMD_READ  = 4'b0_1_0_1; // read
localparam CMD_WRITE = 4'b0_1_0_0; // write
localparam CMD_ACT   = 4'b0_0_1_1; // bank asctivate
localparam CMD_PRE   = 4'b0_0_1_0; // precharge select bank
localparam CMD_REF   = 4'b0_0_0_1; // CBR auto-refresh
localparam CMD_MRS   = 4'b0_0_0_0; // Mode register set
//-----------------------------------------------------------------

//-----------------------------------------------------------------
localparam INIT_START   = 8'd0;
localparam INIT_POWERUP = 8'd1;
localparam INIT_PRE     = 8'd2;
localparam INIT_NOP1    = 8'd3;
localparam INIT_NOP2    = 8'd4;
localparam INIT_REF1    = 8'd5;
localparam INIT_WAIT1   = 8'd6;
localparam INIT_REF2    = 8'd7;
localparam INIT_WAIT2   = 8'd8;
localparam INIT_WAIT3   = 8'd9;
localparam INIT_LMR     = 8'd10;
localparam INIT_LMR1    = 8'd11;
localparam INIT_LMR2    = 8'd12;

localparam IDLE         = 8'd13;

localparam REF          = 8'd14;
localparam REF_NOP1     = 8'd15;
localparam REF_NOP2     = 8'd16;
localparam REF_AREF     = 8'd17;
localparam REF_WAIT     = 8'd18;

localparam WRITE        = 8'd19;
localparam WRITE_ACT    = 8'd20;
localparam WRITE_NOP1   = 8'd21;
localparam WRITE_NOP2   = 8'd22;
localparam WRITE_D0     = 8'd23;
localparam WRITE_N      = 8'd24;
localparam WRITE_STOP0  = 8'd25;
localparam WRITE_STOP1  = 8'd26;
localparam WRITE_STOP_PRE= 8'd27;

localparam READ         = 8'd28;
localparam READ_ACT     = 8'd29;
localparam READ_NOP1    = 8'd30;
localparam READ_NOP2    = 8'd31;
localparam READ_START   = 8'd32;
localparam READ_WAIT1   = 8'd33;
localparam READ_WAIT2   = 8'd34;
localparam READ_WAIT3   = 8'd35;
localparam READ_DATA    = 8'd36;
localparam READ_END     = 8'd37;

//-----------------------------------------------------------------
localparam ADR_WIDTH = SDRAM_ROW_WIDTH_BIT + SDRAM_COL_WIDTH_BIT + SDRAM_BANK_WIDTH_BIT - 1;

reg [3:0] sdram_cmd                       = CMD_DESL; // cmd signal bits     CS RAS CAS WE
reg [SDRAM_DATA_WIDTH_BIT-1:0] sdram_data = 0;
reg [SDRAM_ROW_WIDTH_BIT-1:0] sdram_addr  = 0;
reg sdram_cke;
reg [SDRAM_BANK_WIDTH_BIT-1:0] sdram_ba   = 0;
//reg [SDRAM_DATA_WIDTH_BIT-1:0] sdram_dq;
reg sdram_dqml                            = 0;
reg sdram_dqmh                            = 0;

reg [15:0] wait_cnt        = 0;
reg [15:0] wait_cnt_load   = 0;
reg 		  wait_cnt_start  = 0;
reg        wait_cnt_end    = 0;


reg [SDRAM_DATA_WIDTH_BIT-1:0] ram_data_output = 0;

// refresh counter -------------------------------------------------
localparam REFRESH_INTERVAL = (((tREF * 1_000_000) / tREF_COUNT) / tCK);
reg [15:0] ref_cnt = 0;  // counter interval refresh
reg        ref_en  = 0;  // global enable refresh
reg        ref_req = 0;  // pora delat refresh
reg        ref_ok  = 0;  // refresh vipolnen

reg sdram_buf_we = 0;

reg ready = 0;
reg [7:0] st = 0;
reg [1:0] w_st = 0;


// ------------------------------------------------------------------
reg [SDRAM_ROW_WIDTH_BIT - 1:0] ram_row = 0;
reg [SDRAM_COL_WIDTH_BIT - 1:0] ram_col = 0;
reg [SDRAM_BANK_WIDTH_BIT - 1:0] ram_ba = 0;

reg [SDRAM_COL_WIDTH_BIT:0] ram_len      = 0;  // dlinna danih
reg [SDRAM_COL_WIDTH_BIT:0] ram_len_btrm = 0;  // kogda podavat cmd burst terminate
reg [SDRAM_COL_WIDTH_BIT:0] data_cnt     = 0;  // counter danih

reg ram_write_valid = 0;
reg ram_read_valid  = 0;

//reg flag_full_page = 0;  // flag R/W full page

always @(posedge i_clk)
begin
    case (w_st)
	 0:
	 begin
	     if (wait_cnt_start) begin
	         {wait_cnt_end, wait_cnt} <= wait_cnt_load;
				w_st <= 1;
		  end
	 end
	 
	 1:
	 begin
	     if (~wait_cnt_end) {wait_cnt_end, wait_cnt} <= wait_cnt - 1'b1;
		  else w_st <= 0;
	 end
	 
	 endcase
end

// refresh counter ---------------------------------------------------
always @(posedge i_clk)
begin
    if (ref_en) begin
	     if (ref_cnt == REFRESH_INTERVAL) begin
		      ref_cnt <= 0;
				ref_req <= 1; // pora delat refresh
		  end else begin
		      ref_cnt <= ref_cnt + 1'b1;
				if (ref_ok) ref_req <= 0;
		  end
	 end

end





always @(posedge i_clk)
begin
    case (st)
	 INIT_START:
	 begin
	     // synopsys translate_off
		  $display("------------ SDRAM Init start. ------------");
		  // synopsys translate_on

	     ready          <= 0;

	     sdram_cke      <= 1;
        sdram_dqml     <= 0;
        sdram_dqmh     <= 0;

		  sdram_addr     <= 0;
		  sdram_ba       <= 0;
		  sdram_cmd      <= CMD_NOP;
	     wait_cnt_load  <= tPOWERUP;
		  wait_cnt_start <= 1;
		  st             <= INIT_POWERUP;
	 end
	 
	 INIT_POWERUP:
	 begin
	     wait_cnt_start <= 0;
	     if (wait_cnt_end) begin
		      sdram_cmd <= CMD_NOP;
				st        <= INIT_PRE;
		  end
	 end
	 
	 INIT_PRE:
	 begin
        sdram_cmd      <= CMD_PRE;
		  sdram_addr[10] <= 1;        // all bank
		  st             <= INIT_NOP1;
	 end
	 
	 INIT_NOP1:
	 begin
        sdram_cmd <= CMD_NOP;
		  if (tRP == 3) st <= INIT_NOP2;
		  else st <= INIT_REF1;
	 end
	 
	 INIT_NOP2:
	 begin
        sdram_cmd <= CMD_NOP;
		  st        <= INIT_REF1;
	 end
	 	 
	 INIT_REF1:
	 begin
        sdram_cmd      <= CMD_REF;
		  st             <= INIT_WAIT1;
	     wait_cnt_load  <= tRC - 2'd3;
		  wait_cnt_start <= 1;
	 end
	 
	 INIT_WAIT1:
	 begin
	     wait_cnt_start <= 0;
	     sdram_cmd      <= CMD_NOP;
		  st             <= INIT_REF2;
	 end
	 
	 INIT_REF2:
	 begin
	     if (wait_cnt_end) begin
		      sdram_cmd <= CMD_REF;
				st        <= INIT_WAIT2;
		  end
	     wait_cnt_load  <= tRC - 3'd4;
		  wait_cnt_start <= 1;
	 end
	 
    INIT_WAIT2:
    begin
	     sdram_cmd <= CMD_NOP;
		  wait_cnt_start <= 0;
		  st <= INIT_WAIT3;
    end

    INIT_WAIT3:
    begin
		  if (wait_cnt_end) begin
		      st <= INIT_LMR;
		  end
    end
	 
	 INIT_LMR:
	 begin
	     sdram_cmd      <= CMD_MRS;
		  sdram_addr     <= (SDRAM_CAS << 4) | 3'b111; // Programmed Burst Length | CAS | Full Page
		  sdram_ba       <= 0;
		  st             <= INIT_LMR1;
	 end
	 
	 INIT_LMR1:
	 begin
	     sdram_cmd <= CMD_NOP;
		  st        <= INIT_LMR2;
	 end
	 
	 INIT_LMR2:
	 begin
	     // synopsys translate_off
		  $display("------------ SDRAM Init OK. ------------");
		  // synopsys translate_on
	     sdram_cmd <= CMD_NOP;
	     ready     <= 1;
		  ref_en    <= 1;  // enable refresh
		  st        <= IDLE;
	 end

	 // IDLE ---------------------------------------------------
	 IDLE:
	 begin
	     sdram_cmd <= CMD_NOP;
		  ready     <= 1;
	     if (ref_req) begin
		      st <= REF;
		  end else if (i_ram_read_req) begin
		      st <= READ;
		  end else if (i_ram_write_req) begin
		      st <= WRITE;
		  end
	 end
	 
	 // REFRESH ---------------------------------------------------
	 REF:
	 begin
	     ready          <= 0;
	     ref_ok         <= 1; // vipolnili refresh (upreshaushee vistavlenie !)
	     sdram_cmd      <= CMD_PRE;
		  sdram_addr[10] <= 1; // all bank
		  st             <= REF_NOP1;
	 end
	 
	 REF_NOP1:
	 begin
	     sdram_cmd      <= CMD_NOP;
	     //if (tRP == 3) st <= REF_NOP2;
		  //else st <= REF_AREF;
		  st <= REF_NOP2;
	 end

	 REF_NOP2:
	 begin
	     wait_cnt_load  <= tRC - 3'd3; // zaderka meshdu REFRESH - ACTIVE
		  wait_cnt_start <= 1;
	     sdram_cmd      <= CMD_NOP;
		  st             <= REF_AREF;
	 end
	 
	 REF_AREF:
	 begin
	     ref_ok         <= 0; // vipolnili refresh (upreshaushee vistavlenie !)
		  sdram_cmd      <= CMD_REF;
		  wait_cnt_start <= 0;
		  st             <= REF_WAIT;
	 end
	 
	 REF_WAIT:
	 begin
	     sdram_cmd      <= CMD_NOP;
		  if (wait_cnt_end) begin
		      st <= IDLE;
		  end
	 end
	 
	 // WRITE --------------------------------------
	 WRITE:
	 begin
	     ready   <= 0;
        ram_col <= i_ram_addr[SDRAM_COL_WIDTH_BIT - 1 : 0];
	     ram_row <= i_ram_addr[SDRAM_ROW_WIDTH_BIT + SDRAM_COL_WIDTH_BIT - 1 : SDRAM_COL_WIDTH_BIT];
        ram_ba  <= i_ram_addr[SDRAM_ROW_WIDTH_BIT + SDRAM_COL_WIDTH_BIT + SDRAM_BANK_WIDTH_BIT - 1 : SDRAM_ROW_WIDTH_BIT + SDRAM_COL_WIDTH_BIT];

		  ram_len  <= i_ram_len;
		  st       <= WRITE_ACT;
	 end
	 
	 WRITE_ACT:
	 begin
	     sdram_cmd  <= CMD_ACT;
		  sdram_addr <= ram_row;
		  sdram_ba   <= ram_ba;
		  st         <= WRITE_NOP1;
		  
		  if (tRCD == 2) ram_write_valid <= 1;   // vistvili na 1 takt ranche ?? ne proveral ! tRCD == 2
	 end
	 
	 WRITE_NOP1:
	 begin
	     sdram_cmd  <= CMD_NOP;
		  if (tRCD == 3) begin
		      st <= WRITE_NOP2;
				ram_write_valid <= 1;   // vistvili na 1 takt ranche
		  end else begin
		      //ram_write_valid <= 1;   // vistvili na 1 takt ranche
		      st <= WRITE_D0;
		  end
	 end

	 WRITE_NOP2:
	 begin
	     //ram_write_valid <= 1;   // vistvili na 1 takt ranche
	     sdram_cmd       <= CMD_NOP;
		  st              <= WRITE_D0;
	 end

	 WRITE_D0:
	 begin
	     // synopsys translate_off
		  $display("WRITE_D0 i_ram_data = 0x%x", i_ram_data);
		  // synopsys translate_on
		  		  
	     sdram_cmd       <= CMD_WRITE;
		  sdram_data      <= i_ram_data;
		  data_cnt        <= 1;
		  sdram_buf_we    <= 1;
		  sdram_addr[SDRAM_COL_WIDTH_BIT-1:0] <= ram_col;
		  sdram_ba        <= ram_ba;
		  st              <= WRITE_N;
	 end
	 
	 WRITE_N:
	 begin
	     // synopsys translate_off
		  $display("WRITE_N i_ram_data = 0x%x", i_ram_data);
		  // synopsys translate_on

	     sdram_data <= i_ram_data;
		  data_cnt   <= data_cnt + 1'b1;
		  
		  if (data_cnt == ram_len) begin
		      sdram_cmd       <= CMD_BST;
				ram_write_valid <= 0;
				sdram_buf_we    <= 0;
				st              <= WRITE_STOP0;
		  end else begin
		      sdram_cmd  <= CMD_NOP;
		  end
	 end
	 
	 WRITE_STOP0:
	 begin
	     sdram_cmd <= CMD_NOP;
		  st        <= WRITE_STOP1;
	 end

	 WRITE_STOP1:
	 begin
	     sdram_cmd <= CMD_NOP;
		  st        <= WRITE_STOP_PRE;
	 end

	 WRITE_STOP_PRE:
	 begin
		  sdram_cmd      <= CMD_PRE;
		  sdram_addr[10] <= 1;       // precharge all
		  st             <= IDLE;
	 end

    // READ -------------------------------------------------------------------
    READ:
    begin
	     ram_read_valid <= 0;
	     ready    <= 0;
		  ram_col  <= i_ram_addr[SDRAM_COL_WIDTH_BIT - 1 : 0];
	     ram_row  <= i_ram_addr[SDRAM_ROW_WIDTH_BIT + SDRAM_COL_WIDTH_BIT - 1 : SDRAM_COL_WIDTH_BIT];
        ram_ba   <= i_ram_addr[SDRAM_ROW_WIDTH_BIT + SDRAM_COL_WIDTH_BIT + SDRAM_BANK_WIDTH_BIT - 1 : SDRAM_ROW_WIDTH_BIT + SDRAM_COL_WIDTH_BIT];
		  ram_len  <= i_ram_len;
		  st       <= READ_ACT;
	 end
	 
	 READ_ACT:
	 begin
	     if (SDRAM_CAS == 3) begin // raschet kogda podavat BURST TERMINATE
		      //ram_len_btrm <= ram_len - 2 - 3 + 1; // -3 t.k. v konveere doplnitelnie DELAY, -1 delaem terminate na 1 clk poshe chtobi poslednii danie chut zaderhalis na hine
				ram_len_btrm <= ram_len; // -3 t.k. v konveere doplnitelnie DELAY, -1 delaem terminate na 1 clk poshe chtobi poslednii danie chut zaderhalis na hine
		  end else begin
		      //ram_len_btrm <= ram_len - 1;
				ram_len_btrm <= ram_len;
		  end
	 
	     sdram_cmd  <= CMD_ACT;
		  sdram_addr <= ram_row;
		  sdram_ba   <= ram_ba;
		  st         <= READ_NOP1;
	 end

	 READ_NOP1:
	 begin
	     sdram_cmd  <= CMD_NOP;
		  if (tRCD == 3) begin
		      st <= READ_NOP2;
		  end else begin
		      st <= READ_START;
		  end
	 end

	 READ_NOP2:
	 begin
	     sdram_cmd       <= CMD_NOP;
		  st              <= READ_START;
	 end

	 READ_START:
	 begin
	     // synopsys translate_off
		  $display("READ START.");
		  // synopsys translate_on
		  		  
	     sdram_cmd       <= CMD_READ;
		  data_cnt        <= 1;
		  sdram_buf_we    <= 0;
		  sdram_addr[SDRAM_COL_WIDTH_BIT-1:0] <= ram_col;
		  sdram_ba        <= ram_ba;
		  st              <= READ_WAIT1;
	 end

	 READ_WAIT1:
	 begin
	     sdram_cmd <= CMD_NOP;
		  
		  if (SDRAM_CAS == 3) st <= READ_WAIT2;
	     else st <= READ_WAIT3;//READ_DATA;
//	     else st <= READ_DATA;
	 end

	 READ_WAIT2:
	 begin
	     data_cnt  <= 0;
	     sdram_cmd <= CMD_NOP;
	     st        <= READ_WAIT3;//READ_DATA;
//	     st        <= READ_DATA;
	 end
	 
	 READ_WAIT3: // dopolnitelnaya zaderhka, danie budem poluchat na sleduuhem takte
	 begin
	     //data_cnt  <= 0;
		  data_cnt  <= 1;
	     sdram_cmd <= CMD_NOP;
	     st        <= READ_DATA;
	 end
	 
	 READ_DATA:
	 begin
	     // synopsys translate_off
		  $display("READ_DATA i_sdram_data = 0x%x", i_sdram_data);
		  // synopsys translate_on
		  
	     ram_read_valid  <= 1;
		  ram_data_output <= i_sdram_data;
		  
		  data_cnt        <= data_cnt + 1'b1;
		  
		  if (data_cnt == ram_len) begin
		      st <= READ_END;
		  end
		  
		  if (data_cnt == ram_len_btrm) begin
		  //if (data_cnt == ram_len) begin
		      sdram_cmd <= CMD_BST;
		  end else begin
		      sdram_cmd <= CMD_NOP;
		  end
	 end
	 
	 READ_END:
	 begin
	     ram_read_valid <= 0;
		  sdram_cmd      <= CMD_PRE;
		  sdram_addr[10] <= 1; // precharge all
	     st             <= IDLE;
	 end
	 
	 
	 default:
	 begin
	     st <= INIT_START;
	 end
	 
	 endcase
end


/*
assign #7 {o_sdram_cs_l, o_sdram_ras_l, o_sdram_cas_l, o_sdram_we_l} = sdram_cmd;
assign #7 o_sdram_data = sdram_data;
assign #7 o_sdram_addr = sdram_addr;
assign #7 o_sdram_ba   = sdram_ba;
assign #7 o_sdram_dqml = sdram_dqml;
assign #7 o_sdram_dqmh = sdram_dqmh;
assign #7 o_sdram_cke  = sdram_cke;
assign #4 o_sdram_clk  = i_clk;
assign #7 o_ready      = ready;

assign #7 o_ram_read_valid  = ram_read_valid;
assign #7 o_ram_write_valid = ram_write_valid;
assign #7 o_sdram_buf_we    = sdram_buf_we;
assign #7 o_ram_data        = ram_data_output;
*/


assign #6 {o_sdram_cs_l, o_sdram_ras_l, o_sdram_cas_l, o_sdram_we_l} = sdram_cmd;
assign #6 o_sdram_data = sdram_data;
assign #6 o_sdram_addr = sdram_addr;
assign #6 o_sdram_ba   = sdram_ba;
assign #6 o_sdram_dqml = sdram_dqml;
assign #6 o_sdram_dqmh = sdram_dqmh;
assign #6 o_sdram_cke  = sdram_cke;
assign #4 o_sdram_clk  = i_clk;
assign #6 o_ready      = ready;

assign #4 o_ram_read_valid  = ram_read_valid;
assign #4 o_ram_write_valid = ram_write_valid;
assign #5 o_sdram_buf_we    = sdram_buf_we;
assign #5 o_ram_data        = ram_data_output;




`ifdef DEBUG	 
assign d_wait_cnt    = wait_cnt;
assign d_wait_cnt_en = wait_cnt_end;
assign d_st          = st;
assign d_ref_req     = ref_req;
`endif


endmodule
