// MAX10 ADC Controller REVA0
//
// NEW VERSION // WR/RD mode with flow control
//

module CIRS (CLK, CLK1, STAT,RD,WR,USBX,RXF,TXE,
FT600OE,BE0,BE1,COE,CWR,CRXF,CTXE,CCLK,DMONITOR,
ADCS0, ADCS1, ADRESET0, ADRESET1, ADPD0, ADPD1, ADCNVST0, ADCNVST1,
ADSDOUT0, ADSDOUT1, ADBUSY0, ADBUSY1,
ADSCLK0, ADSCLK1, ADRDERR0, ADRDERR1);


input CLK, CLK1; // CLK1 = FT600 CLOCK

// ADC definition start

output ADCS0, ADCS1;
output ADRESET0, ADRESET1;
output ADPD0, ADPD1;
output ADCNVST0, ADCNVST1;
output ADSCLK0, ADSCLK1;

input ADSDOUT0, ADSDOUT1;
input ADBUSY0, ADBUSY1;
input ADRDERR0, ADRDERR1;
// ADC definition end

// LED definition start

output [7:0] STAT; // LED OUTPUT

// LED definition end

// FTDI USB definition start

output RD,WR; // FT600 WR negative logic 
input RXF,TXE;
output FT600OE;
inout BE0,BE1;

output COE,CWR,CRXF,CTXE,CCLK;
inout [15:0] USBX;
// FTDI USB definition end

output [7:0] DMONITOR;

// register definition start

reg wall;

reg [15:0] dix;
reg [15:0] dox; // output data for USBX
reg [12:0] cnt; 
reg [12:0] cnt1,cnt2;
reg [26:0] refresh;
reg [13:0] usbdadrs;
reg [13:0] adrs;
reg [7:0] cntmask; // to skip data
reg adc,resad,rdad,chn;
reg [7:0] lx1,init;
reg [7:0] lstat,lstat1;
reg [17:0] waved; // waveform data
reg [20:0] overall_dat, dat_tmp;
reg renew,renew0; // internal flag
reg busyad; // AD7643 busy
reg ocbe; // BE0-1 enable // 
reg wr0,rd0;
reg ledind; // external indicator
reg [7:0] adcl;
reg be0,be1;
reg oe; //FT600OE
reg [7:0] usb;
reg coe,crd,crxf,ctxe,cwr,cclk;
reg [7:0] dmonitor;
reg [15:0] dmem [0:32767];
reg [15:0] emem [0:32767];
reg cs,pd;
//AD7643 serial slave mode readout
reg [10:0] adcounter, dat_digit; // 0 to 1024
reg [31:0] adloopcounter;
reg sclk,sdo0,sdo1,busy0,busy1;
reg [17:0] da,db;
reg [17:0] ad_cnt;
reg adcs0, adcnvst0, adsclk0, adreset0;

reg [9:0] w0,w1,w2,w3,w4,w5,w6,w7,w8,w9;
reg [23:0] wavg;

reg [12:0] timer;

reg ad_serial_clk;
reg [7:0] adclkdig;

reg [7:0] loopcounter;
// USB command -> LX1



always @(negedge CLK) begin //CLK1 or CLK?
//always @(negedge ADSCLK0) begin

cclk<=1-cclk; 
refresh<=refresh+1;
if (refresh==0) begin
	ocbe<=1;
	wr0<=1; 
	rd0<=1;
	oe<=1;
	cntmask<=0;
	cnt<=0;
	init<=12;
	cnt<=0;
	lstat<=255; // Initialization End
	renew<=0;
	renew0<=0;
	cnt2<=0;
	be0<=1;
	be1<=1;
	cs<=0;
	pd<=1;	

//	adrs<=0;
end

// DIGITAL PORT MONITOR
crxf<=RXF;
cwr<=wr0;
crd<=rd0;
ctxe<=TXE;
coe<=oe;


if (RXF==0 && cntmask==0) begin
	// DATA to be read are in the FIFO (FT600)
	oe<=0; cnt<=0; cntmask<=1;  crxf<=1;  lstat<=15;
	//dmonitor<=USBX;
end
else if (cntmask==1) begin
	rd0<=0; cntmask<=2;   coe<=1; cnt<=cnt+1; lstat<=16;
	//dmonitor<=USBX;
end
else if (cntmask==2) begin
	// DATA after 1st byte are ignored.
	cnt<=cnt+1;  cntmask<=3; lx1<=USBX; crd<=1;  lstat<=18;
	//dmonitor<=USBX;
end
else if (cntmask==3) begin
	rd0<=1; oe<=1; crxf<=0; coe<=0; crd<=0; renew<=1; cnt1<=0; cntmask<=4;
	//dmonitor<=USBX;
end
else if (cntmask==4) begin	// Command Analysis and doing actions  
   if (lx1==1) begin	//lx1 1:memory clear, line 270 of visual studio, onmcamemoryclear
	   lstat<=1;
		renew<=0;
		if(cnt1==65535) begin
			cntmask<=0; dmem[cnt1]<=0; cnt1<=0;
		end
		else begin
			cnt1<=cnt1+1;
			dmem[cnt1]<=0;	// memory reset
		end
	end
	else if(lx1==2) begin //lx1 2: pointer clear, line 286 of visual studio, onmcapointerclear
		lstat<=lx1;
		renew<=0;
		adrs<=0;
		cntmask<=0; 
		ocbe<=1;
		wr0<=1; 
		rd0<=1;
		oe<=1;
		cntmask<=0;
		cnt<=0;
		init<=12;
		cnt<=0;
		lstat<=15; // Initialization End
		renew<=0;
		renew0<=0;
		cnt2<=0;
		be0<=1;
		be1<=1;
		adc<=1;
		cs<=0;
		pd<=0;
		da<=0;
		db<=0;
		resad<=0;

		// for ADC start
		adcounter<=0;
		//adcs0 <= 1;
		adcnvst0 <= 0;
		//dmonitor[1] <= 0;
		//overall_dat <= 0;
		//adsclk0 <= 0;
		adclkdig <= 0;
		//dmonitor <= 0;
		
	end
	else if(lx1==5) begin // ADC start
		// also in AD conversion, line 305 of visual studio, onmcamemoryread
		// 125 MHz is 8ns
		dmonitor[0] <= adcs0;      //CS
		dmonitor[1] <= adcnvst0;   //CNVST
		dmonitor[2] <= ADBUSY0;    //BUSY
		dmonitor[3] <= ad_serial_clk;
		dmonitor[4] <= ADSDOUT0;    //SCLK
		dmonitor[5] <= ADRDERR0;   //SDOUT
		// dmonitor can only be observed until the fifth.
		// connect dmonitor[6] to CLK.

		adcounter <= adcounter + 1;

		if (ADBUSY0==0) begin
			ad_serial_clk <= 1 - ad_serial_clk;
			if (ad_serial_clk==1) begin
				dat_digit <= dat_digit + 1;
				if ((dat_digit<=15) && (dat_digit>=5)) begin
					//dat_tmp <= overall_dat * 2;
					overall_dat <= overall_dat * 2 + ADSDOUT0;
				end
			end
		end
		
		
		if (adcounter==0) begin adcs0 <= 1; adcnvst0 <= 1; dat_digit <= 0; end
		if (adcounter==4) begin adcnvst0 <= 0; end // from the oscilloscope, it seems like 1clk:10ns
		
		if (adcounter==70) begin adcs0 <= 0; end
		if (adcounter==210) begin
			w8 <= w7; w7 <= w6; w6 <= w5; w5 <= w4; w4 <= w3; w3 <= w2; w2 <= w1; w1 <= w0;
			w0 <= overall_dat / 4;
		end
		if (adcounter==215) begin wavg <= (w0 + w1 + w2 + w3 + w4 + w5 + w6 + w7) / 8; end
		if (adcounter==220) begin dmem[adrs] <= wavg; lstat <= wavg / 8; end
		//if (adcounter==289) begin adcounter <= 0; adrs <= adrs + 1; overall_dat <= 0; end
		// BUSY high is about 1300 ns, 1300 / 8 = 165
		if (adcounter==250) begin adcounter <= 0; adrs <= adrs + 1; overall_dat <= 0; end

	end
	else if(lx1==6) begin
		lstat <= lx1;
		renew<=0;
		renew0<=0;
		be0<=1;
		be1<=1;
		adc<=1;
		cs<=0;
		pd<=0;
		adcounter<=0;
		da<=0;
		db<=0;
		resad<=0;
	end
	else if (lx1==8) begin	// fixed pattern generator
		lstat<=18;
		renew<=0;
		if(cnt1==65535) begin
			cntmask<=0; dmem[cnt1]<=cnt1; cnt1<=0;
		end
		else begin
			cnt1<=cnt1+1;
			dmem[cnt1]<=cnt1;	// memory ramp data pattern set
		end 
	end
	else if (lx1==3) begin
		// probably OnMcaThreshold32777
		//lstat<=15; // during transfer
		adcounter <= adcounter + 1;
	end
	else begin cnt<=cnt+1;end

end
else if (TXE==0) begin
	cntmask<=5;		// GET trigger signal for read
	ocbe<=0;
	if(cnt2==3) begin wr0<=0; cnt2<=cnt2+1; lstat<=7; end //ここたぶん転送
	else if (cnt2==65535) begin
		//default transfer number 
		wr0<=1; dox<=dmem[adrs]; renew<=0; cnt2<=0; cntmask<=0; ocbe<=1; lstat<=4;
	end
	else if (cnt2>3 && cnt2<65535) begin
		dox<=dmem[adrs]; adrs<=adrs+1; cnt2<=cnt2+1;
	end
	else begin
		cnt2<=cnt2+1;
	end
end // end of TXE

else if (TXE==1) begin
end

end // end of always negedge

//always @(posedge CLK) begin
// 8ns 125MHz clock
//	sysclk <= 1 - sysclk;

//end

assign USBX = (1-wr0)?dox:16'bz;

assign STAT = lstat;
assign WR = wr0;
assign RD = rd0;
assign BE0 = (1-ocbe)?be0:1'bz;
assign BE1 = (1-ocbe)?be1:1'bz;
assign FT600OE= oe;
assign CWR=cwr;
assign CRXF=crxf;
assign CTXE=ctxe;
assign COE=coe;
assign DMONITOR = dmonitor; //sclk should appear on dmonitor
assign CCLK = cclk; // CCLK is the pin on the left of GND on the DIGITAL PORT
assign CS1=cs;
assign PD0=pd;


//assign ADRESET0 = 0;
assign ADCS0 = adcs0;
assign ADCNVST0 = adcnvst0;
assign ADSCLK0 = ad_serial_clk;

endmodule





	// {{ALTERA_ARGS_BEGIN}} DO NOT REMOVE THIS LINE!
	
	// {{ALTERA_ARGS_END}} DO NOT REMOVE THIS LINE!
	// {{ALTERA_IO_BEGIN}} DO NOT REMOVE THIS LINE!
	// {{ALTERA_IO_END}} DO NOT REMOVE THIS LINE!f