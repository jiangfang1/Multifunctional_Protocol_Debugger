module top (
    input       clk,
    input       reset_n,
    inout [7:0] fx2_fdata,  //  FX2型USB2.0芯片的SlaveFIFO的数据线
    input       fx2_flagb,  //  FX2型USB2.0芯片的端点2 OUT空标志，1为非空，0为空
    input       fx2_flagc,  //  FX2型USB2.0芯片的端点6 IN满标志，1为非满，0为满
    input       fx2_ifclk,  //  FX2型USB2.0芯片的接口时钟信号

    output [1:0] fx2_faddr,  //  FX2型USB2.0芯片的SlaveFIFO的FIFO地址线
    output fx2_sloe,  //  FX2型USB2.0芯片的SlaveFIFO的输出使能信号，低电平有效
    output fx2_slwr,  //  FX2型USB2.0芯片的SlaveFIFO的写控制信号，低电平有效
    output fx2_slrd,  //  FX2型USB2.0芯片的SlaveFIFO的读控制信号，低电平有效
    output fx2_pkt_end,  //数据包结束标志信号
    output fx2_slcs,

    input  SPI_CS,
    input  SPI_SCLK,
    input  SPI_MOSI,
    output SPI_MISO,

    //key
    input key_in0,
    //hex8 interface
    output sh_cp,
    output st_cp,
    output ds,
    //beep
    output beep,
    //UART连接到板载的USB转串口芯片   
    output uart_tx,
//    input  uart_rx
    //SPI连接到板载的ADC128S
//    output SPI_M_CS,
//    output SPI_M_SCLK,
//    output SPI_M_MOSI,
//    input  SPI_M_MISO

    output [7:0]DA0_Data,     
    output DA0_Clk,   

    output          AD0_CLK,  
    input   [7:0]   AD0,   

    //pcf8563 interface
    output i2c_sclk,
    inout i2c_sdat
);

assign fx2_slcs = 1'b0;

reg [23:0]rst_cnt;
reg rst_n;
wire clk100m;
wire clk50m;
wire clk30m;
wire rst_cmb = reset_n  & rst_n;
wire disp_data_sel;
wire set_done;
wire set_time; //设置时间使能
wire [23:0]set_time_data; //设置时间数据
wire set_date; //设置日期使能
wire [31:0]set_date_data; //设置日期数据
reg  [25:0]read_tcnt;
wire read;
wire [23:0]time_data;
wire [31:0]date_data;  
wire read_done;
wire [31:0]disp_data;
wire [47:0]date_time;
wire usb_data_valid;
wire usb_data_ready;
wire [7:0] usb_data_payload;
wire uart_tx_done;
wire [7:0]uart_tx_data;
wire uart_tx_en;
// 三态处理
wire [7:0] fx2_fdata_in;
wire [7:0] fx2_fdata_out;
assign fx2_fdata_in = fx2_fdata;
assign fx2_fdata    = fx2_slwr ? 8'hZZ : fx2_fdata_out;
reg Send_Data_Valid;
reg [7:0] Send_Data;
wire Recive_Data_Valid;
wire [7:0] Recive_Data;
wire [15:0] Trans_Cnt;
wire Trans_Done;
wire Trans_Start;
wire Trans_End;
wire pwm_en;
wire adc_cap_en;
wire freq_calc_en;
wire adc2usb_get;
wire dds_wr;
wire [15:0] dds_waddr;
wire [31:0] dds_wdata;
reg [7:0] ad_data_sync;
wire [7:0] ad_data_fifo;
wire fifo_full;
wire fifo_wr;
wire [7:0] fifo_wdata;
wire adc_empty;
reg [15:0] wr_cnt;
wire adc_done;
wire [31:0] freq_res;
wire [31:0] freq_high;
wire [31:0] freq_low;
wire [7:0] freq_res_payload;

always @(posedge clk,negedge reset_n)begin  //由于高云复位寄存器问题，增加加一个延时复位
    if(!reset_n)
        rst_cnt <= 0;
    else if(rst_cnt == 5_000_000)
        rst_cnt <= rst_cnt;
    else
        rst_cnt <= rst_cnt + 1;
end

always @(posedge clk ,negedge reset_n)begin
    if(!reset_n)
        rst_n <= 0;
    else if((rst_cnt >= 4_000_000) && (rst_cnt <= 4_500_000))
        rst_n <= 0;
    else
        rst_n <= 1;
end

Gowin_PLL Gowin_PLL(
    .clkin(clk), //input  clkin
    .clkout0(clk100m), //output  clkout0
    .clkout1(clk50m), //output  clkout1
    .clkout2(clk30m) //output  clkout2
);

/*****************************/
wire pcf_valid;
wire [7:0] pcf_data;
cmd_parse cmd_parse (   
    .sys_clk(clk50m),
    .reset_n(rst_cmb),
    
    .data_valid(usb_data_valid),
    .data_payload(usb_data_payload),
    .data_ready(usb_data_ready),

    .pcf_valid(pcf_valid),
    .pcf_data(pcf_data),

	.dds_wr(dds_wr),
    .dds_waddr(dds_waddr),
	.dds_wdata(dds_wdata),

    .pwm_en(pwm_en),

    .adc_cap_en(adc_cap_en),   
    .adc_done(adc_done),

    .freq_res(freq_res),
    .freq_high(freq_high),
    .freq_low(freq_low),
    .freq_calc_en(freq_calc_en),
    .freq_res_payload(freq_res_payload)  

);

/*****************************/

usb_fifo usb_fifo(
      .fx2_ifclk(fx2_ifclk),
      .sys_clk(clk50m),
      .reset_n(rst_cmb),
      .fx2_flagb(fx2_flagb),  // FX2型USB2.0芯片的端点2 OUT空标志，1为非空，0为空
      .fx2_flagc(fx2_flagc),  // FX2型USB2.0芯片的端点6 IN满标志，1为非满，0为满
      .fx2_faddr(fx2_faddr),  // FX2型USB2.0芯片的SlaveFIFO的FIFO地址线
      .fx2_sloe(fx2_sloe),  // FX2型USB2.0芯片的SlaveFIFO的输出使能信号，低电平有效
      .fx2_slwr(fx2_slwr),  // FX2型USB2.0芯片的SlaveFIFO的写控制信号，低电平有效
      .fx2_slrd(fx2_slrd),  // FX2型USB2.0芯片的SlaveFIFO的读控制信号，低电平有效
      .fdata_in(fx2_fdata),
      .fdata_out(fx2_fdata_out),
      .fx2_pkt_end(fx2_pkt_end),

      .fpga_data_valid(~adc_empty),
      .fpga_data_payload(ad_data_fifo),
      .fpga_data_ready(adc2usb_get),
  
      .usb_data_valid(usb_data_valid),
      .usb_data_payload(usb_data_payload),
      .usb_data_ready(usb_data_ready)
);


  SPI_Slave #(
      .CPOL(1'b0),
      .CPHA(1'b0),
      .BITS_ORDER(1'b1)
  ) SPI_Slave (
      .Clk(fx2_ifclk),
      .Rst_n(rst_cmb),
      .Send_Data_Valid(Send_Data_Valid),
      .Send_Data(Send_Data),
      .Recive_Data_Valid(Recive_Data_Valid),
      .Recive_Data(Recive_Data),
      .Trans_Cnt(Trans_Cnt),
      .Trans_Done(Trans_Done),
      .SPI_CS(SPI_CS),
      .SPI_SCK(SPI_SCLK),
      .SPI_MOSI(SPI_MOSI),
      .SPI_MISO(SPI_MISO),
      .Trans_Start(Trans_Start),
      .Trans_End(Trans_End)
  );

/*当出现Trans_Start时，先接收一个字节，判断bit[7]读(0)/写(1)位，并判断bit[6:0]字节长度len
  如果为写，则读取len个字节参数并存储到寄存器[7:0]Param_Reg[63:0]
  如果为读，且长度为len字节，则从Param_Reg连续读取len个字节，通过MISO发送出去
  */

  reg [7:0] Param_Reg[63:0];
  reg [6:0] len;
  localparam local_data = {8'h08, 8'h00, 8'h00, 8'h00, 8'h01, 8'hC2,8'h00};
  localparam S_IDLE = 7'b0000001;  // 空闲
  localparam S_CMD = 7'b0000010;  // 第一个字节完成，解析
  localparam S_READ_WAIT = 7'b0000100;  // 读等待
  localparam S_READ_REG = 7'b0001000;  // 读
  localparam S_WRITE_WAIT = 7'b0010000;  // 写等待
  localparam S_WRITE_REG = 7'b0100000;  // 写
  localparam S_END = 7'b1000000;  // 结束

  reg [6:0] SM_State;
 

  always @(posedge fx2_ifclk or negedge rst_cmb) begin
    if (~rst_cmb) begin
      SM_State <= S_IDLE;
      len <= 7'd0;
      Send_Data_Valid <= 1'b0;
      Send_Data <= 8'h00;
      Param_Reg[0] <= local_data[7:0];
      Param_Reg[1] <= local_data[15:8];
      Param_Reg[2] <= local_data[23:16];
      Param_Reg[3] <= local_data[31:24];
      Param_Reg[4] <= local_data[39:32];
      Param_Reg[5] <= local_data[47:40];
      Param_Reg[6] <= local_data[55:48];
    end else begin
      case (SM_State)
        S_IDLE: begin
          if (Trans_Start) begin
            SM_State <= S_CMD;
          end else begin
            SM_State <= S_IDLE;
          end
        end
        S_CMD: begin
          if (Recive_Data_Valid) begin
            len <= Recive_Data[6:0];
            if (Recive_Data[7]) begin  //写寄存器
              SM_State <= S_READ_REG;
            end else begin
              SM_State <= S_WRITE_REG;
            end
          end else begin
            SM_State <= S_CMD;
          end
        end
        S_READ_WAIT: begin
          if (Recive_Data_Valid) begin
            if (Trans_Cnt >= len + 1'b1) SM_State <= S_END;
            else SM_State <= S_READ_REG;
          end else begin
            Send_Data_Valid <= 1'b0;
            SM_State <= S_READ_WAIT;
          end

        end
        S_READ_REG: begin
          if (~SPI_SCLK) begin
            Send_Data <= Param_Reg[Trans_Cnt-1];  //读取寄存器数据
            Send_Data_Valid <= 1'b1;
            SM_State <= S_READ_WAIT;
          end else begin
            Send_Data <= Param_Reg[Trans_Cnt-1];  //读取寄存器数据
            Send_Data_Valid <= 1'b0;
          end
        end
        S_WRITE_WAIT: begin
            if (Trans_Cnt >= len + 1'b1) SM_State <= S_END;
            else SM_State <= S_WRITE_REG;
        end
        S_WRITE_REG: begin
          if (Recive_Data_Valid) begin
            Param_Reg[Trans_Cnt-2] <= Recive_Data;
            SM_State <= S_WRITE_WAIT;
          end else begin
            SM_State <= S_WRITE_REG;
          end
        end
        S_END: begin
          len <= 7'd0;
          Send_Data_Valid <= 1'b0;
          Send_Data <= 8'h00;
          SM_State <= S_IDLE;
        end
        default: SM_State <= S_IDLE;
      endcase
    end
  end
/*****************************/
  key_filter key_filter
  (
    .clk(clk50m),
    .reset_n(rst_cmb),
    .key_in(key_in0),
    .key_flag(),
    .key_state(disp_data_sel)
  );

  always@(posedge clk50m or negedge rst_cmb)
  begin
    if(!rst_cmb)
        read_tcnt <= 26'd0;
    else if(read_tcnt == 26'd14_999_999)
        read_tcnt <= 26'd0;
    else
        read_tcnt <= read_tcnt + 1'd1;
  end

  assign read = (read_tcnt == 26'd14_999_999);

  uart_adjust_time uart_adjust_time
  (
    //clock reset
    .clk            (clk50m          ), //系统时钟输入＿50M
    .reset_n        (rst_cmb         ), //复位信号输入，低有效
    .uart_rx_data   (pcf_data    ), //串口接收数据
    .uart_data_valid(pcf_valid ), //串口接收数据有效标识
    .set_done       (set_done        ),
    .set_time       (set_time        ), //设置时间使能
    .set_time_data  (set_time_data   ), //设置时间数据
    .set_date       (set_date        ), //设置日期使能
    .set_date_data  (set_date_data   )  //设置日期数据
  );

  pcf8563_ctrl pcf8563_ctrl(
    .Clk(clk50m),
    .Rst_n(rst_cmb),
    .Set_Time(set_time),
    .Time_to_Set(set_time_data),
    .Set_Date(set_date),
    .Date_to_Set(set_date_data),
    .Read(read),
    .Time_Read(time_data),
    .Date_Read(date_data),
    .Set_Done(set_done),
    .Read_Done(read_done),
    .i2c_sclk(i2c_sclk),
    .i2c_sdat(i2c_sdat)
  ); 

  assign disp_data = disp_data_sel ? {time_data[23:16],4'hf,time_data[15:8] ,4'hf,time_data[7:0]} :
                                     {8'h20,date_data[31:24],date_data[23:16],date_data[7:0]};

  hex_top_special_for_rtc hex_top_special_for_rtc(
    .clk (clk50m),
    .reset_n (rst_cmb),
    .disp_data (disp_data),
    .sh_cp (sh_cp),
    .st_cp (st_cp),
    .ds (ds)
  );

  uart_byte_tx uart_byte_tx
  (
    .clk(clk50m),
    .reset_n(rst_cmb),
    .data_byte(uart_tx_data),
    .send_en(uart_tx_en),
    .baud_set(3'd0),    
    .uart_tx(uart_tx),
    .tx_done(uart_tx_done),
    .uart_state()
  );

//  assign date_time = {date_data[31:16],date_data[7:0],time_data};
//  time_send_uart time_send_uart(
//    .clk(clk50m),
//    .reset(~rst_cmb),
//    .date_time_en(read_done),
//    .date_time(date_time),
//    .uart_tx_done(uart_tx_done),
//    .uart_tx_data(uart_tx_data),
//    .uart_tx_en(uart_tx_en)
//  );

/*****************************/
pwm_top pwm_top(
    .clk(clk50m),
    .reset_n(rst_cmb),
    .en(pwm_en),
    .beep(beep)
);

/*****************************/
wire [7:0] dds_dout;
assign DA0_Clk = clk50m;
assign DA0_Data[7]= ~dds_dout[7];
assign DA0_Data[6:0] = dds_dout[6:0];

	DDS_Top DDS_Top(
		.clk(clk50m), //input clk
		.rstn(rst_cmb), //input rstn
		.wr(dds_wr), //input wr
		.waddr(dds_waddr), //input [15:0] waddr
		.wdata(dds_wdata), //input [31:0] wdata
		.dout(dds_dout), //output [7:0] dout
		.out_valid() //output out_valid
	);

/*****************************/

assign AD0_CLK = clk50m;

always@(posedge clk50m) begin
    ad_data_sync <= AD0;
end


always@(posedge clk50m) begin
    if(~rst_cmb)
        wr_cnt <= 0;
    else if(adc_cap_en & (~fifo_full))
        wr_cnt <= wr_cnt + 1;
end

assign fifo_wr = (adc_cap_en | freq_calc_en )& (~fifo_full);
assign adc_done = &wr_cnt;
assign fifo_wdata = freq_calc_en ? freq_res_payload : ad_data_sync;

adc_fifo_adptor adc_fifo_adptor(
    .Data(fifo_wdata), //input [7:0] Data
    .Clk(clk50m), //input Clk
    .WrEn(fifo_wr), //input WrEn
    .RdEn(adc2usb_get & (~adc_empty)), //input RdEn
    .Reset(0), //input Reset
    .Q(ad_data_fifo), //output [7:0] Q
    .Empty(adc_empty), //output Empty
    .Full(fifo_full) //output Full
);

/*****************************/
freq_calc freq_calc(
    .clk100m(clk100m),
    .test_signal(clk30m),
    .reset_n(rst_cmb),
    .cnt_test_clk_res(freq_res),
    .pulse_high_value(freq_high),
    .pulse_low_value(freq_low)
);


/*****************************/
endmodule
