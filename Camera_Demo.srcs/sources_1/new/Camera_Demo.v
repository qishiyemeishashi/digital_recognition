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
    //ʱ���ź�
    wire clk_100MHz_system;
    wire clk_200MHz;
    
    //HDMI�ź�
    wire [23:0]rgb_data_src;
    wire rgb_hsync_src;
    wire rgb_vsync_src;
    wire rgb_vde_src;
    wire clk_pixel;
    wire clk_serial;
    
    parameter NUM_ROW	 = 1'd1			  ; //��ʶ���ͼ�������
parameter NUM_COL    = 3'd4			  ; //��ʶ���ͼ�������
parameter H_PIXEL    = 480			  ; //ͼ���ˮƽ����
parameter V_PIXEL    = 272			  ; //ͼ��Ĵ�ֱ����
parameter DEPBIT	 = 4'd10		  ; //����λ��

//LCD ID
localparam  ID_4342 =   0;               //4����Ļ���ֱ��ʣ�480*272
localparam  ID_7084 =   1;               //7����Ļ���ֱ��ʣ�800*480
localparam  ID_7016 =   2;               //7����Ļ���ֱ��ʣ�1024*600
localparam  ID_1018 =   5;               //10.1����Ļ���ֱ��ʣ�1280*800
parameter   ID_LCD = ID_4342;            //���ڲ�ͬ��LCd��Ļ�޸ģ���ID_LCD��Ӧ��ֵ����


wire        clk_lcd        ;  //lcdģ���Ƶ���ʱ��
//wire        locked         ;
//wire        rst_n          ;

wire        rd_en          ;  //sdram_ctrlģ���ʹ��
wire [15:0] rd_data        ;  //sdram_ctrlģ�������

wire [15:0] ID_lcd         ;  //LCD��ID
wire [23:0]	digit		   ;
wire [15:0]	color_rgb	   ;
wire [10:0]	xpos		   ;
wire [10:0]	ypos		   ;
wire		hs_t		   ;
wire		vs_t		   ;
wire		de_t		   ;

wire        lcd_hs     ;  //LCD ��ͬ���ź�
wire        lcd_vs     ;  //LCD ��ͬ���ź�
wire        lcd_de     ;  //LCD ��������ʹ��
wire [15:0] lcd_rgb    ;  //LCD RGB565��ɫ����
wire        lcd_bl     ;  //LCD ��������ź�
wire        lcd_rst    ;  //LCD ��λ�ź�
wire        lcd_pclk   ;  //LCD ����ʱ��

    //assign color_rgb={rgb_data_src[22:16],rgb_data_src[13:8],rgb_data_src[4:0]};    
    //ϵͳʱ��
    clk_wiz_0 clk_10(.clk_out1(clk_100MHz_system),.clk_out2(clk_200MHz),.clk_in1(i_clk));
    
    //HDMI����
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
    
    //ͼ��MIPI�ź�תRGB
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
    
    //����ͷIIC��SDA�ߵ���̬�ڵ�
    wire camera_iic_sda_i;
    wire camera_iic_sda_o;
    wire camera_iic_sda_t;
    
    //Tri-state gate
    IOBUF Camera_IIC_SDA_IOBUF
       (.I(camera_iic_sda_o),
        .IO(i_camera_iic_sda),
        .O(camera_iic_sda_i),
        .T(~camera_iic_sda_t));
    
    //����ͷIIC�����ź�
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
    
    //����ͷ����
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
    
    //����ͷIIC����
    Driver_IIC MIPI_Camera_IIC(
        .i_clk(clk_100MHz_system),
        .i_rst(i_rst),
        .i_iic_sda(camera_iic_sda_i),
        .i_iic_write(iic_write),                //IICд�ź�,��������Ч
        .i_iic_read(iic_read),                  //IIC���ź�,��������Ч
        .i_iic_mode(iic_mode),                  //IICģʽ,1����˫��ַλ,0������ַλ,��λ��ַ��Ч
        .i_slave_addr(slave_addr),              //IIC�ӻ���ַ
        .i_reg_addr_h(reg_addr_h),              //�Ĵ�����ַ,��8λ
        .i_reg_addr_l(reg_addr_l),              //�Ĵ�����ַ,��8λ
        .i_data_w(data_w),                      //��Ҫ���������
        .o_data_r(data_r),                      //IIC����������
        .o_iic_busy(iic_busy),                  //IICæ�ź�,�ڹ���ʱæ,�͵�ƽæ
        .o_iic_scl(o_camera_iic_scl),           //IICʱ����
        .o_sda_dir(camera_iic_sda_t),           //IIC�����߷���,1�������
        .o_iic_sda(camera_iic_sda_o)            //IIC������
    );
    
        
//����LCD����ģ��
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
    .clk_lcd            (clk_lcd),          //LCD����ʱ��

    .ID_lcd             (ID_LCD),            //LCD ID
	
	//user interface
    .pixel_xpos 		(xpos  ),
    .pixel_ypos 		(ypos  )
    );

//ͼ����ģ��
vip #(
    .NUM_ROW(NUM_ROW),
    .NUM_COL(NUM_COL),
    .H_PIXEL(H_PIXEL),
    .V_PIXEL(V_PIXEL)
)u_vip(
    //module clock
    .clk              (i_clk),  // ʱ���ź�
    .rst_n            (i_rst    ),  // ��λ�źţ�����Ч��
    //ͼ����ǰ�����ݽӿ�
    .pre_frame_vsync  (vs_t   ),
    .pre_frame_hsync  (hs_t   ),
    .pre_frame_de     (de_t   ),
    .pre_rgb          (color_rgb),
    .xpos             (xpos   ),
    .ypos             (ypos   ),
    //ͼ���������ݽӿ�
    .post_frame_vsync (lcd_vs ),  // ��ͬ���ź�
    .post_frame_hsync (lcd_hs ),  // ��ͬ���ź�
    .post_frame_de    (lcd_de ),  // ��������ʹ��
    .post_rgb         (lcd_rgb),  // RGB565��ɫ����
    //user interface
    .digit            (digit  )   // ʶ�𵽵�����
);
    
endmodule
