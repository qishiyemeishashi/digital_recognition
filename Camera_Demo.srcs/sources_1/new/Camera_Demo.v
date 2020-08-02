`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2020/06/25 09:56:56
// Design Name: 
// Module Name: Camera_Demo
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module Camera_Demo(
    input i_clk,
    input i_rst,
    input i_clk_rx_data_n,
    input i_clk_rx_data_p,
    input [1:0]i_rx_data_n,
    input [1:0]i_rx_data_p,
    input i_data_n,
    input i_data_p,
    inout i_camera_iic_sda,
    output o_camera_iic_scl,
    output o_camera_gpio,
    output TMDS_Tx_Clk_N,
    output TMDS_Tx_Clk_P,
    output [2:0]TMDS_Tx_Data_N,
    output [2:0]TMDS_Tx_Data_P
    );
    //时钟信号
    wire clk_100MHz_system;
    wire clk_200MHz;
    
    //HDMI信号
    wire [23:0]rgb_data_src;
    wire rgb_hsync_src;
    wire rgb_vsync_src;
    wire rgb_vde_src;
    wire clk_pixel;
    wire clk_serial;
    
    parameter NUM_ROW	 = 1'd1			  ; //需识别的图像的行数
parameter NUM_COL    = 3'd4			  ; //需识别的图像的列数
parameter H_PIXEL    = 480			  ; //图像的水平像素
parameter V_PIXEL    = 272			  ; //图像的垂直像素
parameter DEPBIT	 = 4'd10		  ; //数据位宽

//LCD ID
localparam  ID_4342 =   0;               //4寸屏幕，分辨率：480*272
localparam  ID_7084 =   1;               //7寸屏幕，分辨率：800*480
localparam  ID_7016 =   2;               //7寸屏幕，分辨率：1024*600
localparam  ID_1018 =   5;               //10.1寸屏幕，分辨率：1280*800
parameter   ID_LCD = ID_4342;            //对于不同的LCd屏幕修改，赋ID_LCD对应的值即可


wire        clk_lcd        ;  //lcd模块分频后的时钟
//wire        locked         ;
//wire        rst_n          ;

wire        rd_en          ;  //sdram_ctrl模块读使能
wire [15:0] rd_data        ;  //sdram_ctrl模块读数据

wire [15:0] ID_lcd         ;  //LCD的ID
wire [23:0]	digit		   ;
wire [15:0]	color_rgb	   ;
wire [10:0]	xpos		   ;
wire [10:0]	ypos		   ;
wire		hs_t		   ;
wire		vs_t		   ;
wire		de_t		   ;

wire        lcd_hs     ;  //LCD 行同步信号
wire        lcd_vs     ;  //LCD 场同步信号
wire        lcd_de     ;  //LCD 数据输入使能
wire [15:0] lcd_rgb    ;  //LCD RGB565颜色数据
wire        lcd_bl     ;  //LCD 背光控制信号
wire        lcd_rst    ;  //LCD 复位信号
wire        lcd_pclk   ;  //LCD 采样时钟

    //assign color_rgb={rgb_data_src[22:16],rgb_data_src[13:8],rgb_data_src[4:0]};    
    //系统时钟
    clk_wiz_0 clk_10(.clk_out1(clk_100MHz_system),.clk_out2(clk_200MHz),.clk_in1(i_clk));
    
    //HDMI驱动
    rgb2dvi_0 Mini_HDMI_Driver(
      .TMDS_Clk_p(TMDS_Tx_Clk_P),     // output wire TMDS_Clk_p
      .TMDS_Clk_n(TMDS_Tx_Clk_N),     // output wire TMDS_Clk_n
      .TMDS_Data_p(TMDS_Tx_Data_P),      // output wire [2 : 0] TMDS_Data_p
      .TMDS_Data_n(TMDS_Tx_Data_N),      // output wire [2 : 0] TMDS_Data_n
      .aRst_n(i_rst),                   // input wire aRst_n
      .vid_pData(rgb_data_src),         // input wire [23 : 0] vid_pData
      .vid_pVDE(rgb_vde_src),           // input wire vid_pVDE
      .vid_pHSync(rgb_hsync_src),       // input wire vid_pHSync
      .vid_pVSync(rgb_vsync_src),       // input wire vid_pVSync
      .PixelClk(clk_pixel)
    );
    
    //图像MIPI信号转RGB
    Driver_MIPI MIPI_Trans_Driver(
        .i_clk_200MHz(clk_200MHz),
        .i_clk_rx_data_n(i_clk_rx_data_n),
        .i_clk_rx_data_p(i_clk_rx_data_p),
        .i_rx_data_n(i_rx_data_n),
        .i_rx_data_p(i_rx_data_p),
        .i_data_n(i_data_n),
        .i_data_p(i_data_p),
        .o_camera_gpio(o_camera_gpio),
        .o_rgb_data(rgb_data_src),
        .o_rgb_hsync(rgb_hsync_src),
        .o_rgb_vsync(rgb_vsync_src),
        .o_rgb_vde(rgb_vde_src),
        .o_set_x(),
        .o_set_y(),
        .o_clk_pixel(clk_pixel)
    );
    
    //摄像头IIC的SDA线的三态节点
    wire camera_iic_sda_i;
    wire camera_iic_sda_o;
    wire camera_iic_sda_t;
    
    //Tri-state gate
    IOBUF Camera_IIC_SDA_IOBUF
       (.I(camera_iic_sda_o),
        .IO(i_camera_iic_sda),
        .O(camera_iic_sda_i),
        .T(~camera_iic_sda_t));
    
    //摄像头IIC驱动信号
    wire iic_busy;
    wire iic_mode;
    wire [7:0]slave_addr;
    wire [7:0]reg_addr_h;
    wire [7:0]reg_addr_l;
    wire [7:0]data_w;
    wire [7:0]data_r;
    wire iic_write;
    wire iic_read;
    wire ov5647_ack;
    
    //摄像头驱动
    OV5647_Init MIPI_Camera_Driver(
        .i_clk(clk_100MHz_system),
        .i_rst(i_rst),
        .i_iic_busy(iic_busy),
        .o_iic_mode(iic_mode),          
        .o_slave_addr(slave_addr),    
        .o_reg_addr_h(reg_addr_h),   
        .o_reg_addr_l(reg_addr_l),   
        .o_data_w(data_w),      
        .o_iic_write(iic_write),
        .o_ack(ov5647_ack)                 
    );
    
    //摄像头IIC驱动
    Driver_IIC MIPI_Camera_IIC(
        .i_clk(clk_100MHz_system),
        .i_rst(i_rst),
        .i_iic_sda(camera_iic_sda_i),
        .i_iic_write(iic_write),                //IIC写信号,上升沿有效
        .i_iic_read(iic_read),                  //IIC读信号,上升沿有效
        .i_iic_mode(iic_mode),                  //IIC模式,1代表双地址位,0代表单地址位,低位地址有效
        .i_slave_addr(slave_addr),              //IIC从机地址
        .i_reg_addr_h(reg_addr_h),              //寄存器地址,高8位
        .i_reg_addr_l(reg_addr_l),              //寄存器地址,低8位
        .i_data_w(data_w),                      //需要传输的数据
        .o_data_r(data_r),                      //IIC读到的数据
        .o_iic_busy(iic_busy),                  //IIC忙信号,在工作时忙,低电平忙
        .o_iic_scl(o_camera_iic_scl),           //IIC时钟线
        .o_sda_dir(camera_iic_sda_t),           //IIC数据线方向,1代表输出
        .o_iic_sda(camera_iic_sda_o)            //IIC数据线
    );
    
        
//例化LCD顶层模块
lcd u_lcd(
    .clk                (i_clk),
    .rst_n              (i_rst),
                        
    .lcd_hs             (hs_t),
    .lcd_vs             (vs_t),
    .lcd_de             (de_t),
    .lcd_rgb            (color_rgb),
    .lcd_bl             (lcd_bl),
    .lcd_rst            (lcd_rst),
    .lcd_pclk           (lcd_pclk),
            
    .pixel_data         (rd_data),
    .rd_en              (rd_en),
    .clk_lcd            (clk_lcd),          //LCD驱动时钟

    .ID_lcd             (ID_LCD),            //LCD ID
	
	//user interface
    .pixel_xpos 		(xpos  ),
    .pixel_ypos 		(ypos  )
    );

//图像处理模块
vip #(
    .NUM_ROW(NUM_ROW),
    .NUM_COL(NUM_COL),
    .H_PIXEL(H_PIXEL),
    .V_PIXEL(V_PIXEL)
)u_vip(
    //module clock
    .clk              (i_clk),  // 时钟信号
    .rst_n            (i_rst    ),  // 复位信号（低有效）
    //图像处理前的数据接口
    .pre_frame_vsync  (vs_t   ),
    .pre_frame_hsync  (hs_t   ),
    .pre_frame_de     (de_t   ),
    .pre_rgb          (color_rgb),
    .xpos             (xpos   ),
    .ypos             (ypos   ),
    //图像处理后的数据接口
    .post_frame_vsync (lcd_vs ),  // 场同步信号
    .post_frame_hsync (lcd_hs ),  // 行同步信号
    .post_frame_de    (lcd_de ),  // 数据输入使能
    .post_rgb         (lcd_rgb),  // RGB565颜色数据
    //user interface
    .digit            (digit  )   // 识别到的数字
);
    
endmodule
