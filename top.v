// TOP

//`define DEBUG

module top
#(
    parameter SDRAM_ROW_WIDTH_BIT  = 13,
    parameter SDRAM_COL_WIDTH_BIT  = 9,
	 parameter SDRAM_BANK_WIDTH_BIT = 2,
	 parameter SDRAM_DATA_WIDTH_BIT = 16,
	 
	 parameter SDRAM_CAS            = 3,   // cas 2 or 3
//	 parameter SDRAM_CAS            = 2,   // cas 2 or 3
	 
    parameter tPOWERUP   = 16'd14285, // delay perd init 100 us
	 parameter tREF       = 64,        // ms
	 parameter tREF_COUNT = 8192,      // refresh counter (cycle refresh)
	 parameter tCK        = 8          // Period SDRAM CLK (pri 143Mhz(7ns) nehvataet refresha.)
)

(
    input wire i_clk,
	 
	 output wire o_sys_clk,
	 output wire o_clk_shift,
	 
	 // interface for manager RAM
//	 input wire [SDRAM_DATA_WIDTH_BIT-1:0] i_ram_data,
	 output wire [SDRAM_DATA_WIDTH_BIT-1:0] ram_data,

//	 input wire [SDRAM_ROW_WIDTH_BIT + SDRAM_COL_WIDTH_BIT + SDRAM_BANK_WIDTH_BIT - 1:0] i_ram_addr,
//	 input wire [SDRAM_COL_WIDTH_BIT-1:0] i_ram_len,

//	 input wire i_ram_read_req,
//	 output wire o_ram_read_valid,
	 output wire o_ram_read_req,
	 output wire o_ram_read_valid,
	 
//	 input wire i_ram_write_req,
//	 output wire o_ram_write_valid,
	 output wire o_ram_write_req,
	 output wire o_ram_write_valid,
	 
	 output wire o_ready,
	 
	 // interface for SDRAM
	 output wire o_sdram_clk,
	 output wire o_sdram_cke,
	 output wire o_sdram_cs_l,
	 output wire o_sdram_ras_l,
	 output wire o_sdram_cas_l,
	 output wire o_sdram_we_l,
    output wire [SDRAM_ROW_WIDTH_BIT-1:0] o_sdram_addr,	 
	 output wire [SDRAM_BANK_WIDTH_BIT-1:0] o_sdram_ba,
	 inout wire  [SDRAM_DATA_WIDTH_BIT-1:0] o_sdram_dq,
	 output wire o_sdram_dqml,
	 output wire o_sdram_dqmh,
	 
	 output wire o_test_ok,
	 output wire o_test_er,
	 output wire o_test_stop

`ifdef DEBUG	 
	 ,
	 output wire [15:0] d_wait_cnt,
	 output wire d_wait_cnt_en,
	 output wire [7:0] d_st,
	 output wire d_ref_req
`endif

);

wire [SDRAM_DATA_WIDTH_BIT-1:0] sdram_data_o;

reg  [SDRAM_DATA_WIDTH_BIT-1:0] i_ram_data = 0;
wire [SDRAM_DATA_WIDTH_BIT-1:0] o_ram_data;
reg [SDRAM_DATA_WIDTH_BIT-1:0] ram_data_r = 0;
reg  [SDRAM_ROW_WIDTH_BIT + SDRAM_COL_WIDTH_BIT + SDRAM_BANK_WIDTH_BIT - 1:0] ram_addr = 0;
reg  [SDRAM_COL_WIDTH_BIT:0] ram_len = 0;
reg  ram_read_req    = 0;
wire ram_read_valid;
reg  ram_write_req   = 0;
wire ram_write_valid;
wire sdram_buf_we;

wire clk_125MHz;
wire clk_125MHz_SDRAM;
wire clk_50MHz;
wire clk_25MHz;


pll PLL1
(
    .inclk0(  i_clk             ),
	 .c0(      clk_125MHz        ),
	 .c1(      clk_125MHz_SDRAM  ),
	 .c2(      clk_25MHz         )
);




sdram_cntr
#(
    .SDRAM_ROW_WIDTH_BIT(SDRAM_ROW_WIDTH_BIT),
    .SDRAM_COL_WIDTH_BIT(SDRAM_COL_WIDTH_BIT),
	 .SDRAM_BANK_WIDTH_BIT(SDRAM_BANK_WIDTH_BIT),
	 .SDRAM_DATA_WIDTH_BIT(SDRAM_DATA_WIDTH_BIT),
	 .SDRAM_CAS(SDRAM_CAS),
	 .tPOWERUP(tPOWERUP),
	 .tREF(tREF),
	 .tREF_COUNT(tREF_COUNT),
	 .tCK(tCK)
	 
)
SDRAM_CNTR0
(
    .i_clk(             clk_125MHz         ),
	 
	 // interface for manager RAM
	 .i_ram_data(        i_ram_data         ),
	 .o_ram_data(        o_ram_data         ),

	 .i_ram_addr(        ram_addr           ),
	 .i_ram_len(         ram_len            ),

	 .i_ram_read_req(    ram_read_req       ),
	 .o_ram_read_valid(  ram_read_valid     ),
	 
	 .i_ram_write_req(   ram_write_req      ),
	 .o_ram_write_valid( ram_write_valid    ),
	 
	 .o_ready(           o_ready            ),
	 
	 // interface for SDRAM
	 .i_sdram_data(      o_sdram_dq         ),
	 .o_sdram_data(      sdram_data_o       ),
	 .o_sdram_addr(      o_sdram_addr       ),

//	 .o_sdram_clk(       o_sdram_clk        ),
//	 .o_sdram_clk(       o_sys_clk          ),

	 .o_sdram_cke(       o_sdram_cke        ),
	 .o_sdram_cs_l(      o_sdram_cs_l       ),
	 .o_sdram_ras_l(     o_sdram_ras_l      ),
	 .o_sdram_cas_l(     o_sdram_cas_l      ),
	 .o_sdram_we_l(      o_sdram_we_l       ),
	 .o_sdram_buf_we(    sdram_buf_we       ),
	 .o_sdram_ba(        o_sdram_ba         ),
	 .o_sdram_dqml(      o_sdram_dqml       ),
	 .o_sdram_dqmh(      o_sdram_dqmh       )

`ifdef DEBUG	 
	 ,
	 .d_wait_cnt(        d_wait_cnt         ),
	 .d_wait_cnt_en(     d_wait_cnt_en      ),
	 .d_st(              d_st               ),
	 .d_ref_req(         d_ref_req          )
`endif

);


localparam PAGE_SIZE = 2**SDRAM_COL_WIDTH_BIT;
localparam ADDR_ROW_BA_END = 15'b111_1111_1111_1111; 
//localparam ADDR_ROW_BA_END = 15'b000_0000_0000_0000; 
//localparam ADDR_ROW_BA_END = 15'b000_0000_0000_0011; 

reg [7:0] data_num = 0; // nomer danih v tabliche

reg [SDRAM_BANK_WIDTH_BIT + SDRAM_ROW_WIDTH_BIT - 1:0] addr_row_ba = 0; // adress BANK & ROW
reg [SDRAM_DATA_WIDTH_BIT - 1:0] data_w = 0; //data for write
reg test_er   = 0;
reg test_ok   = 0;
reg test_stop = 0;

reg [15:0] st = 0;

reg [SDRAM_COL_WIDTH_BIT:0] cnt = 0;

always @(posedge clk_125MHz)
begin
    case (st)
	 
	 0:
	 begin
	     test_er     <= 0;
		  test_ok     <= 0;
		  addr_row_ba <= 0;
		  data_num    <= 0;
		  data_w      <= 16'h0000;
		  
	     if (o_ready) begin
		      st <= st + 1'b1;
		  end
	 end
	 
	 // write page ---------------------------------------------
	 1:
	 begin
	     test_er       <= 0;
		  test_ok       <= 0;
	     test_stop     <= 0;
		  cnt           <= 0;
	     ram_write_req <= 1;
	     ram_addr      <= {addr_row_ba, 9'b0_0000_0000};
		  ram_len       <= PAGE_SIZE;
		  st            <= st + 1'b1;
	 end
	 
	 2:
	 begin
		  if (cnt == PAGE_SIZE) begin
		      st <= st + 1'b1;
		  end
		  if (ram_write_valid) begin
		      i_ram_data    <= data_w;
		      ram_write_req <= 0;
		      cnt           <= cnt + 1'b1;
		  end
	 end

	 3:
	 begin
	     if (addr_row_ba != ADDR_ROW_BA_END) begin
		      addr_row_ba <= addr_row_ba + 1'b1;
				st <= 1; // write new page
		  end else begin
				addr_row_ba <= 0;
		      st          <= st + 1'b1;  // jmp to read
		  end
	 end
	 
	 // reset pered read ---------------------------------------
	 4:
	 begin
	     test_er     <= 0;
		  test_ok     <= 0;
	     test_stop   <= 1;
	     addr_row_ba <= 0;
	     st          <= st + 1'b1;
	 end
	 
    // read page -----------------------------------------------
	 5:
	 begin
		  cnt           <= 0;
	     ram_read_req  <= 1;
		  ram_addr      <= {addr_row_ba, 9'b0_0000_0000};
		  ram_len       <= PAGE_SIZE;
		  st            <= st + 1'b1;
	 end
	 
	 6:
	 begin
		  if (cnt == PAGE_SIZE) begin
		      st <= st + 1'b1;
		  end
		  if (ram_read_valid) begin
				if (o_ram_data != data_w) begin
				    //st <= 10_000; // error
					 test_ok <= 0;
					 test_er <= 1;
				end else begin
				    test_ok <= 1;
				    test_er <= 0;
				end
		      ram_data_r    <= o_ram_data;
		      ram_read_req  <= 0;
		      cnt           <= cnt + 1'b1;
		  end
	 end

	 7:
	 begin
	     if (addr_row_ba != ADDR_ROW_BA_END) begin
		      addr_row_ba <= addr_row_ba + 1'b1;
				st <= 5; // read new page
		  end else begin
				addr_row_ba <= 0;
		      st          <= st + 1'b1;  // jmp to read
		  end
	 end

	 8:
	 begin
        data_num <= data_num + 1'b1;
		  st       <= st + 1'b1;
	 end
	 
	 9:
	 begin
		  case (data_num)
		  0: data_w <= 16'h0000;
		  1: data_w <= 16'hffff;
		  2: data_w <= 16'haaaa;
		  3: data_w <= 16'h5555;
		  default: data_w <= 16'h0000;
		  endcase
	 
	     if (data_num != 4) begin
		      st <= 1;
		  end else begin
		      st <= st + 1'b1;
		  end
	 end
	 
	 // stop
	 10:
	 begin
        //test_stop <= ! test_stop;
		  st <= 0;
	 end
	 
	 default:
	 begin
	     ram_read_req  <= 0;
	     ram_write_req <= 0;
	     st            <= st + 1'b1;
	 end
	 
	 
	 endcase
end





// assign ---------------------------------------------------------
assign o_sdram_dq = sdram_buf_we ? sdram_data_o : 16'hz;

assign o_ram_read_req    = ram_read_req;
assign o_ram_read_valid  = ram_read_valid;
assign o_ram_write_req   = ram_write_req;
assign o_ram_write_valid = ram_write_valid;


assign o_sdram_clk       = clk_125MHz_SDRAM;
//assign o_sdram_clk       = clk_125MHz;

assign o_sys_clk         = clk_125MHz;
assign o_clk_shift       = clk_125MHz_SDRAM;

assign ram_data = ram_data_r;


assign o_test_er   = test_er;
assign o_test_ok   = test_ok;
assign o_test_stop = test_stop;



endmodule
