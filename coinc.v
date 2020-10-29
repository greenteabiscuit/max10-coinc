// MAX10 ADC Controller REVA0
//
// NEW VERSION // WR/RD mode with flow control
//

module CIRS (CLK, CLK1, STAT,RD,WR,USBX,RXF,TXE,
RESAD0,RESAD1,FT600OE,BE0,BE1,COE,CWR,CRXF,CTXE,CCLK,DMONITOR,
ADCS0, ADCS1, ADRESET0, ADRESET1, ADPD0, ADPD1, ADCNVST0, ADCNVST1,
ADSDOUT0, ADSDOUT1, ADBUSY0, ADBUSY1, ADSYNC0, ADSYNC1, ADSCLK0, ADSCLK1, ADSDIN0, ADSDIN1);

output RESAD0,RESAD1;


input CLK, CLK1; // CLK1 = FT600 CLOCK

// ADC definition start

output ADCS0, ADCS1;
output ADRESET0, ADRESET1;
output ADPD0, ADPD1;
output ADCNVST0, ADCNVST1;

input ADSDOUT0, ADSDOUT1;
input ADBUSY0, ADBUSY1;
input ADSYNC0, ADSYNC1;

inout ADSCLK0, ADSCLK1;
inout ADSDIN0, ADSDIN1;

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
reg [20:0] overall_dat;
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
reg [8:0] adcounter;
reg [31:0] adloopcounter;
reg sclk,sdo0,sdo1,busy0,busy1;
reg [17:0] da,db;
reg [17:0] ad_cnt;
reg adcs0;
reg adcnvst0;
reg adsclk0;

reg [7:0] loopcounter;
// USB command -> LX1

always @(negedge CLK1) begin

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
	lstat<=128; // Initialization End
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
	oe<=0; cnt<=0;  dmonitor<=USBX; cntmask<=1;  crxf<=1;  lstat<=15;
end
else if (cntmask==1) begin
	rd0<=0; cntmask<=2;   coe<=1; dmonitor<=USBX; cnt<=cnt+1; lstat<=16;
end
else if (cntmask==2) begin
	// DATA after 1st byte are ignored.
	cnt<=cnt+1;  cntmask<=3; lx1<=USBX; dmonitor<=USBX; crd<=1;  lstat<=17;
end
else if (cntmask==3) begin
	rd0<=1; oe<=1; dmonitor<=USBX; crxf<=0; coe<=0; crd<=0; renew<=1; cnt1<=0; cntmask<=4;
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
		adcs0 <= 1;
		adcnvst0 <= 1;
		overall_dat <= 0;
		
	end
	else if(lx1==5) begin // ADC start
		// also in AD conversion, line 305 of visual studio, onmcamemoryread
		// 125 MHz is 8ns
		adcounter <= adcounter + 1;

		if (adcounter==0) begin adcs0 <= 0; end

		if (adcounter==5) begin adcnvst0 <= 0; end
		if (adcounter==8) begin adcnvst0 <= 1; end

		if (ADSYNC0==1) begin
			lx1 <= 3;
			adloopcounter <= adloopcounter + 1;
			if (adloopcounter==2000000000) begin lx1 <= 0; adloopcounter<=0; end

			//if (ADSCLK0==1) begin
			//	if (ADBUSY0==1) begin
			//		adcounter <= 0;
			//	end
			//end
		end

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

		if (adcounter==0) begin adcs0 <= 0; end

		if (adcounter==5) begin adcnvst0 <= 0; end
		if (adcounter==8) begin adcnvst0 <= 1; end

		if (ADSYNC0==1) begin
			lstat <= 3;
			adloopcounter <= adloopcounter + 1;
			if (adloopcounter==2000000000) begin lx1 <= 0; adloopcounter<=0; end

			if (ADSCLK0==1) begin
				if (ADBUSY0==1) begin
					adcounter <= 0;
				end
			end
		end
		else begin
			lstat <= 1;
		end
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

always @(posedge CLK) begin
// 8ns 125MHz clock

end

assign USBX = (1-wr0)?dox:16'bz;

assign STAT = lstat;
assign WR = wr0;
assign RD = rd0;
assign SDIN0 =adc;
assign RESAD0 = resad;
assign RESAD1 =resad;
assign BE0 = (1-ocbe)?be0:1'bz;
assign BE1 = (1-ocbe)?be1:1'bz;
assign FT600OE= oe;
assign CWR=cwr;
assign CRXF=crxf;
assign CTXE=ctxe;
assign COE=coe;
assign DMONITOR = dmonitor;
assign CCLK=cclk;
assign CS1=cs;
assign PD0=pd;
assign PD1=pd;
assign SCLK0=sclk;
assign SCLK1=sclk;

assign ADCS0 = adcs0;
assign ADCNVST0 = adcnvst0;
assign ADSCLK0 = adsclk0;

endmodule





	// {{ALTERA_ARGS_BEGIN}} DO NOT REMOVE THIS LINE!
	
	// {{ALTERA_ARGS_END}} DO NOT REMOVE THIS LINE!
	// {{ALTERA_IO_BEGIN}} DO NOT REMOVE THIS LINE!
	// {{ALTERA_IO_END}} DO NOT REMOVE THIS LINE!f