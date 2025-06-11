`include "CLOG2.v"

// cache fsm states
`define IDLE 2'b00
`define hit_OR_miss 2'b01
`define WRITEBACK 2'b10
`define ALLOCATE 2'b11

module Cache #(parameter LINE_SIZE = 16, // cache size: 256B, block size: 16B
               parameter NUM_SETS = 16, // Direct-mapped: NUM_SETS == LINE_SIZE
               parameter NUM_WAYS = 1) ( // LINE_SIZE == NUM_SETS * NUM_WAYS
    input reset,
    input clk,

    input is_input_valid,
    input [31:0] addr,
    input mem_read,
    input mem_write,
    input [31:0] din,

    output is_ready,
    output reg is_output_valid,
    output reg [31:0] dout,
    output is_hit,
    output reg [31:0] num_miss,
    output reg [31:0] num_total_access);
   
  //local parameter
  localparam SET_WIDTH = `CLOG2(NUM_SETS);
  //localparam WAY_WIDTH = `CLOG2(NUM_WAYS);
  localparam LINE_WIDTH = `CLOG2(LINE_SIZE);

  localparam TAG_WIDTH = 28 - SET_WIDTH;
  localparam TAG_LSB = 4 + SET_WIDTH;
  localparam IDX_MSB = 3 + SET_WIDTH;

  // Wire declarations
  // for DataMemory
  wire is_data_mem_ready;
  wire [127:0] data_mem_dout;
  reg [127:0] temp_data;
  reg data_mem_read;
  reg data_mem_is_output_valid;

  wire [TAG_WIDTH-1:0] cache_tag;
  wire [SET_WIDTH-1:0] cache_idx; // == set index
  wire [1:0] cache_bo; // bo: 2 bits, g: 2 bits]

  assign cache_tag = addr[31:TAG_LSB];
  assign cache_idx = addr[IDX_MSB:4];
  assign cache_bo = addr[3:2];

  wire [31:0] data_block0;
  wire [31:0] data_block1;
  wire [31:0] data_block2;
  wire [31:0] data_block3;

  assign data_block0 = data_bank[cache_idx][0][31:0]; 
  assign data_block1 = data_bank[cache_idx][0][63:32]; 
  assign data_block2 = data_bank[cache_idx][0][95:64];
  assign data_block3 = data_bank[cache_idx][0][127:96];

   Mux_4to1 mux_dout(
    .mux_in0(data_block0),   // input
    .mux_in1(data_block1),   // input
    .mux_in2(data_block2),   // input
    .mux_in3(data_block3),   // input
    .sel(cache_bo),
    .mux_out(dout)    // output
  );

  assign is_ready = is_data_mem_ready;

  // Reg declarations
  reg [TAG_WIDTH-1:0] tag_bank [0:NUM_SETS-1][0:NUM_WAYS-1];
  reg [127:0] data_bank [0:NUM_SETS-1][0:NUM_WAYS-1]; //(256/NUM_SETS)*-1:0 ***
  reg valid_bank [0:NUM_SETS-1][0:NUM_WAYS-1];
  reg dirty_bank [0:NUM_SETS-1][0:NUM_WAYS-1]; // Write-back & Write-allocate cache

  reg [NUM_WAYS-1:0] ways_hit; // idx -> a set (includes way(s)) -> hit?
  //reg [WAY_WIDTH-1:0] hit_way_idx; // hit 된 way 중에서 선택할 index
  //reg [WAY_WIDTH-1:0] evict_way_idx; // cache miss 시 evict 할 way index (by replacement policy)
  //reg [WAY_WIDTH-1:0] lru_queue [0:NUM_SETS-1][0:NUM_WAYS-1]; // array of way indices. replacement policy: LRU
  //reg [WAY_WIDTH-1:0] temp_set [0:NUM_WAYS-1];
  //reg [WAY_WIDTH-1:0] free_way_idx; // cache miss 시 way가 free(valid bit == 0)한 경우
  //reg evict_valid_way;

  reg data_mem_write;
  reg [127:0] data_mem_din; // block size: 16B //256->127
  reg [31:0] data_mem_addr;
  reg data_mem_is_input_valid;

  reg write_cache;
  reg is_valid;
  reg is_dirty;

  // reg [31:0] num_total_access;
  // reg [31:0] num_miss;

  integer i, j, k;
  integer updated_order;

  // ----------- Cache FSM -------------
  reg [1:0] current_cache_state, next_cache_state;

  always @(posedge clk) begin
    if (reset)
      current_cache_state <= `IDLE;
    else
      current_cache_state <= next_cache_state;
  end

  assign is_hit = (valid_bank[cache_idx][0] && (tag_bank[cache_idx][0] == cache_tag));

  always @(*) begin
    data_mem_write = 0;
    data_mem_read = 0;
    data_mem_is_input_valid = 0;
    data_mem_din = 0;
    data_mem_addr = 0;
    is_output_valid = 0;
    
    write_cache = 0;
    is_valid = 1'b0;
    is_dirty = 1'b0;
    temp_data = 0;

    //evict_valid_way = 1'b1;
    //hit_way_idx = 0;
    //evict_way_idx = 0;
    //next_cache_state = `IDLE;
    
    /*for (i = 0; i < NUM_WAYS; i = i + 1) begin
      ways_hit[i] = ((tag_bank[cache_idx][i] == cache_tag) && valid_bank[cache_idx][i]);
    end
    for (i = 0; i < NUM_WAYS; i = i + 1) begin
      if (ways_hit[i]) begin
        //hit_way_idx = i[WAY_WIDTH-1 : 0]; // hit 된 way 중 가장 마지막 way의 index 이용
        is_hit = 1'b1;
      end
    end*/
    

    /*
    if (!is_hit) begin // miss -> need to pick a way to put new blocks
      for (i = 0; i < NUM_WAYS; i = i + 1) begin
        if (!valid_bank[cache_idx][i]) begin
          evict_valid_way = 1'b0;
          evict_way_idx = i[WAY_WIDTH-1 : 0]; // free way idx
        end
      end
      if (evict_valid_way) begin
        evict_way_idx = lru_queue[cache_idx][NUM_WAYS-1]; // LRU queue의 마지막 요소를 way index로 가지는 block을 evict
      end
    end 
    */ 
    /*if (data_mem_is_output_valid) begin
      data_bank[cache_idx][0] = data_mem_dout;
    end*/
    
    temp_data = data_bank[cache_idx][0];
  
    case (cache_bo)
      2'b00: begin
        temp_data[31:0] = din;
      end
      2'b01: begin
        temp_data[63:32] = din;
      end
      2'b10: begin
        temp_data[95:64] = din;
      end
      2'b11: begin
        temp_data[127:96] = din;
      end
    endcase

    case(current_cache_state)
      `IDLE: begin
        //is_ready = 1'b1;
        if (is_input_valid) begin
          next_cache_state = `hit_OR_miss;
        end
        else begin
           is_output_valid = 1'b1;
          next_cache_state = `IDLE;
        end
      end

      `hit_OR_miss: begin
          if (is_hit) begin // hit
            is_output_valid = 1'b1; // whether dout and is_hit are valid(doesn’t mean True) or not
            next_cache_state = `IDLE;
            if (mem_write && !mem_read) begin
              write_cache = 1'b1;
              is_valid = 1'b1;
              is_dirty = 1'b1;
            end
          end
          else begin // miss
            if (dirty_bank[cache_idx][0]) begin 
              next_cache_state = `WRITEBACK;
            end
            else begin
              next_cache_state = `ALLOCATE;
            end
          end
      end

      `WRITEBACK: begin
        if (is_data_mem_ready) begin // newly allocated, same as hit_OR_miss & miss & not dirty
          next_cache_state = `ALLOCATE;
          data_mem_is_input_valid = 1'b1;
          data_mem_addr = {tag_bank[cache_idx][0], cache_idx, 4'b0} >> LINE_WIDTH;
          data_mem_write = 1'b1;
          data_mem_read = 1'b0;
          data_mem_din = data_bank[cache_idx][0];
        end
        else begin
          next_cache_state = `WRITEBACK; // data 전달받을 때까지 반복
        end
      end

      `ALLOCATE: begin
        data_mem_addr = addr >> LINE_WIDTH;
        data_mem_write = 1'b0; 
        data_mem_read = 1'b1;
        if (data_mem_is_output_valid) begin
          next_cache_state = `hit_OR_miss;
          data_mem_is_input_valid = 1'b0;
          write_cache = 1'b1;
          is_valid = 1'b1;
          is_dirty = 1'b0;

          temp_data = data_mem_dout;
        end
        else begin
          data_mem_is_input_valid = 1'b1;
          next_cache_state = `ALLOCATE; // data 전달받을 때까지 반복
        end
      end

      default: begin
        next_cache_state = `IDLE;
        data_mem_write = 0;
        data_mem_is_input_valid = 0;
        data_mem_din = 0;
        data_mem_addr = 0;
        is_output_valid = 0;
        is_hit = 0;
      end
    endcase
  end

  always @(posedge clk) begin
    if (reset) begin
      for (i = 0; i < NUM_SETS; i = i + 1) begin
        for (j = 0; j < NUM_WAYS; j = j + 1) begin
          tag_bank[i][j] <= 0;
          valid_bank[i][j] <= 0;
          dirty_bank[i][j] <= 0;
          data_bank[i][j] <= 0;
          //lru_queue[i][j] <= j[WAY_WIDTH-1 : 0]; // 임의
        end
      end

      num_total_access <= 0;
      num_miss <= 0;
    end
    else begin
      // miss rate calculation
      if (is_hit && (current_cache_state == `hit_OR_miss)) begin
        num_total_access <= num_total_access + 1;
      end
      else if (!is_hit && (current_cache_state == `hit_OR_miss)) begin
        num_miss <= num_miss + 1;
      end
/*
    if (!reset && current_cache_state == `hit_OR_miss) begin
      $display("[Time %0t] Hit: %b | Total: %d | Miss: %d", $time, is_hit, num_total_access, num_miss);
    end
*/
      if (write_cache) begin
        data_bank[cache_idx][0] <= temp_data;
        tag_bank[cache_idx][0] <= cache_tag;
        valid_bank[cache_idx][0] <= is_valid;
        dirty_bank[cache_idx][0] <= is_dirty;   
      end
    end
  end


  //// cache design in the lab pdf
  // async read: valid, data, is_hit
  // sync write: write to cache (+ replacement) <- CPU & data_mem
  // Write-back & Write-allocate
 
  // Instantiate data memory
  DataMemory #(.BLOCK_SIZE(LINE_SIZE)) data_mem(
    .reset(reset),    // input
    .clk(clk),    // input

    .is_input_valid(data_mem_is_input_valid),    // input
    .addr(data_mem_addr),    // input, NOTE: address must be shifted by CLOG2(LINE_SIZE)
    .mem_read(data_mem_read),    // input
    .mem_write(data_mem_write),    // input
    .din(data_mem_din),    // input

    // is output from the data memory valid?
    .is_output_valid(data_mem_is_output_valid),    // output
    .dout(data_mem_dout),    // output
    // is data memory ready to accept request?
    .mem_ready(is_data_mem_ready)   // output
  );
endmodule
