// MAX10 ADC Controller REVA0
//
// NEW VERSION // WR/RD mode with flow control
//

module CIRS (CLK, CLK1, STAT,RD,WR,USBX,RXF,TXE,ADA0,ADB0,SDOUT0,
SDOUT1,SCLK0,SCLK1,ADCLK0,ADCLK1,PD0,PD1,CS0,CS1,BUSYAD0,BUSYAD1,
RESAD0,RESAD1,FT600OE,BE0,BE1,CRD,COE,CWR,CRXF,CTXE,CCLK,DMONITOR);

input ADA0,ADB0; // AD7643
output RESAD0,RESAD1;

input SDOUT0,SDOUT1;
output SCLK0,SCLK1;

inout [15:0] USBX;

input CLK, CLK1; // CLK1 = FT600 CLOCK

output [7:0] STAT; // LED OUTPUT
output RD,WR; // FT600 WR negative logic 
input RXF,TXE;
output ADCLK0,ADCLK1,PD0,PD1,CS0,CS1;
input BUSYAD0, BUSYAD1;
output FT600OE;
inout BE0,BE1;
output CRD,COE,CWR,CRXF,CTXE,CCLK;
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
reg [6:0] adcounter;
reg sclk,sdo0,sdo1,busy0,busy1;
reg [17:0] da,db;

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
		lstat<=2;
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
		lstat<=128; // Initialization End
		renew<=0;
		renew0<=0;
		cnt2<=0;
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
	else if(lx1==5) begin // ADC start
		// also in AD conversion, line 305 of visual studio, onmcamemoryread
		lstat<=lx1;
		renew<=0;
		adcounter<=adcounter+1;
		if(adcounter==0) begin adc<=0; end
		if(adcounter>2 && adcounter<40) begin
			adc<=1; sclk<=1-sclk;	// 18 SCLK mean 18 bit readout 
			if(sclk==0) begin
				da<=da*2+SDOUT0; db<=db*2+SDOUT1;
			end
		end
		if(adcounter==40) begin  dmem[adrs]<=(da/4);  lstat<=da; end	
		if(adcounter==41) begin  emem[adrs]<=(db/4);  end	
		if(adcounter==100) begin adcounter<=0; adrs<=adrs+1; da<=0; db<=0; end
	end
	else if(lx1==lstat) begin
		lstat <= 6;
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
		lstat <= 14;

	end // end of lx1 5
	else begin cnt<=cnt+1;end

end
else if (TXE==0) begin
	cntmask<=5;		// GET trigger signal for read
	ocbe<=0;
	if(cnt2==3) begin wr0<=0; cnt2<=cnt2+1; lstat<=3; end
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
end // end of TXE:0

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
assign ADCLK0 =adc;
assign ADCLK1 =adc;
assign RESAD0 = resad;
assign RESAD1 =resad;
assign BE0 = (1-ocbe)?be0:1'bz;
assign BE1 = (1-ocbe)?be1:1'bz;
assign FT600OE= oe;
assign CWR=cwr;
assign CRD=crd;
assign CRXF=crxf;
assign CTXE=ctxe;
assign COE=coe;
assign DMONITOR = dmonitor;
assign CCLK=cclk;
assign CS0=cs;
assign CS1=cs;
assign PD0=pd;
assign PD1=pd;
assign SCLK0=sclk;
assign SCLK1=sclk;

endmodule





	// {{ALTERA_ARGS_BEGIN}} DO NOT REMOVE THIS LINE!
	
	// {{ALTERA_ARGS_END}} DO NOT REMOVE THIS LINE!
	// {{ALTERA_IO_BEGIN}} DO NOT REMOVE THIS LINE!
	// {{ALTERA_IO_END}} DO NOT REMOVE THIS LINE!f