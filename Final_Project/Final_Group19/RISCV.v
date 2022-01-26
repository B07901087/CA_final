// Your code

module RISCV(clk,
            rst_n,
            // for mem_D
            mem_wen_D,
            mem_addr_D,
            mem_wdata_D,
            mem_rdata_D,
            // for mem_I
            mem_addr_I,
            mem_rdata_I);
    input         clk, rst_n        ;  
    input  [31:0] mem_rdata_I       ;
    input  [63:0] mem_rdata_D       ;
	
    output reg [31:2] mem_addr_I    ;
    reg [31:2] mem_addr_I_w;
    output   mem_wen_D  ;
    output reg [31:2] mem_addr_D ; // output address for data mem
    output wire [63:0] mem_wdata_D ; // output data for data mem
	// wire/reg  
	wire [31:0] mem_rdata_I_change;
	
	reg [63:0] imme_w;
	reg [63:0] ALU_data1; // rs1
	reg [63:0] ALU_data2; // rs2 or imme
	reg [63:0] ALU_data2_w; // store for rs2, get the value of the register
	reg [63:0] ALU_dataout;
	reg [29:0] Branch_output;
	reg [63:0] Rd_register_w; //this is data for rd register
	reg [63:0] Rd_register_data;
	reg [4:0] Rd_register_addr;
	reg [63:0] Rd_register_buffer;
	integer k;
	//{ JAL, JALR, Branch, MemRead, MemWrite, MemtoReg, RegWrite, ALUSrc, ALUOp}
	//reg JAL, JALR, ALUsrc, MemToReg, RegWrite, MemRead, Branch;
	reg [11:0] signal;
	//reg [3: 0] ALUControl;
	reg is_sll; // for immediate
	reg [4:0]instruction_type; // for immediate
// ********* register part **********
	reg [63:0] register1;
	reg [63:0] register2;
	reg [63:0] register3;
	reg [63:0] register4;
	reg [63:0] register5;
	reg [63:0] register6;
	reg [63:0] register7;
	reg [63:0] register8;
	reg [63:0] register9;
	reg [63:0] register10;
	reg [63:0] register11;
	reg [63:0] register12;
	reg [63:0] register13;
	reg [63:0] register14;
	reg [63:0] register15;
	reg [63:0] register16;
	reg [63:0] register17;
	reg [63:0] register18;
	reg [63:0] register19;
	reg [63:0] register20;
	reg [63:0] register21;
	reg [63:0] register22;
	reg [63:0] register23;
	reg [63:0] register24;
	reg [63:0] register25;
	reg [63:0] register26;
	reg [63:0] register27;
	reg [63:0] register28;
	reg [63:0] register29;
	reg [63:0] register30;
	reg [63:0] register31;
	// ********* register part **********
	
	
	//parameters
	parameter jal =  ( 23'b01 << 22 );
	parameter jalr = ( 23'b01 << 21 );
	parameter beq =  ( 23'b01 << 20 );
	parameter bne =  ( 23'b01 << 19 );
	parameter ld =   ( 23'b01 << 18 );
	parameter sd =   ( 23'b01 << 17 );
	parameter addi = ( 23'b01 << 16 );
	parameter slti = ( 23'b01 << 15 );
	parameter xori = ( 23'b01 << 14 );
	parameter ori =  ( 23'b01 << 13 );
	parameter andi = ( 23'b01 << 12 );
	parameter slli = ( 23'b01 << 11 );
	parameter srli = ( 23'b01 << 10 );
	parameter srai = ( 23'b01 << 9 );
	parameter add =  ( 23'b01 << 8 );
	parameter sub =  ( 23'b01 << 7 );
	parameter sll =  ( 23'b01 << 6 );
	parameter slt =  ( 23'b01 << 5 );
	parameter XOR =  ( 23'b01 << 4 );
	parameter srl=   ( 23'b01 << 3 );
	parameter sra =  ( 23'b01 << 2 );
	parameter OR =   ( 23'b01 << 1 );
	parameter AND =    23'b01 ;
	
	parameter jals =  {8'b10000011,4'b0000};
	parameter jalrs = {8'b01000011,4'b0000};
	parameter beqs =  {8'b00100000,4'b1000};
	parameter bnes =  {8'b00100000,4'b1000};
	parameter lds =   {8'b00010111,4'b0000}; 
	parameter sds =   {8'b00001001,4'b0000};
	parameter addis = {8'b00000011,4'b0000};
	parameter sltis = {8'b00000011,4'b0010};
	parameter xoris = {8'b00000011,4'b0100};
	parameter oris =  {8'b00000011,4'b0110}; 
	parameter andis = {8'b00000011,4'b0111};
	parameter sllis = {8'b00000011,4'b0001};
	parameter srlis = {8'b00000011,4'b0101}; 
	parameter srais = {8'b00000011,4'b1101}; 
	parameter adds =  {8'b00000010,4'b0000}; 
	parameter subs =  {8'b00000010,4'b1000};
	parameter slls =  {8'b00000010,4'b0001}; 
	parameter slts =  {8'b00000010,4'b0010}; 
	parameter XORs =  {8'b00000010,4'b0100}; 
	parameter srls =  {8'b00000010,4'b0101};
	parameter sras =  {8'b00000010,4'b1101}; 
	parameter ORs =   {8'b00000010,4'b0110};
	parameter ANDs =  {8'b00000010,4'b0111}; 

	parameter R = {5'b10000};
	parameter I = {5'b01000};
	parameter S = {5'b00100};
	parameter B = {5'b00010};
	parameter J = {5'b00001};
	// ALU Control, I have used the excel file
	parameter ALU_ADD = 4'b0000;
	parameter ALU_SUB = 4'b1000;
	parameter ALU_SLL = 4'b0001;  //should we change this?
	parameter ALU_SLT = 4'b0010;  //should we change this?
	parameter ALU_XOR = 4'b0100;
	parameter ALU_SRL = 4'b0101;
	parameter ALU_SRA = 4'b1101;
	parameter ALU_OR = 4'b0110;
	parameter ALU_AND = 4'b0111;
	
//assign mem_wen_D = mem_wen_D; // write mem
assign mem_wen_D = signal[7];
assign mem_wdata_D[63:0] = {ALU_data2_w[7:0], ALU_data2_w[15:8], ALU_data2_w[23:16], ALU_data2_w[31:24], ALU_data2_w[39:32], ALU_data2_w[47:40], ALU_data2_w[55:48], ALU_data2_w[63:56]};
assign mem_rdata_I_change = {mem_rdata_I[7: 0], mem_rdata_I[15: 8], mem_rdata_I[23: 16], mem_rdata_I[31: 24]};
always @(*) begin //mem_rdata_I or mem_rdata_D
//write to memory
	// mem_rdata_I_change = {mem_rdata_I[7: 0], mem_rdata_I[15: 8], mem_rdata_I[23: 16], mem_rdata_I[31: 24]};
	instruction_type = 5'b0;
	is_sll = 1'b0;
	//imme_w = 64'b0;
	// JAL = 1'b0;
	// JALR = 1'b0;
	// Branch = 1'b0;
	// MemRead = 1'b0;
	// mem_wen_D = 1'b0;
	// MemToReg = 1'b0;
	// RegWrite = 1'b0;
	// ALUsrc = 1'b0;
	// ALUControl[3:0] = 4'b0;
	$display("####### mem_addr_I: 0x%h #######", {mem_addr_I, 2'b0});
    $display("instruction: 0x%h", mem_rdata_I_change);
	
    case(mem_rdata_I_change[6:0])
		
		7'b1101111:begin //jal
			instruction_type = J;
			signal = jals;
		end
		
		7'b1100111:begin //jalr
			instruction_type = I;
			signal = jalrs;
		end
		
		7'b1100011:begin // SB
			instruction_type = B;
			case(mem_rdata_I_change[14:12])
				3'b000:begin// beq
					signal = beqs;
				end
				default:begin // bne
					signal = bnes;
				end
			endcase
		end
		
		7'b0000011:begin // ld
			instruction_type = I;
			signal = lds;
		end
		
		7'b0100011:begin // sd
			instruction_type = S;
			signal = sds;
		end
		7'b0010011:begin
			instruction_type = I;
			case(mem_rdata_I_change[14:12])
				3'b000:begin // addi
					signal = addis;
				end
				3'b010:begin //slti
					signal = sltis;
				end
				3'b100:begin // xori
					signal = xoris;
					
				end
				3'b110:begin // ori
					signal = oris;
				end
				3'b111:begin // andi
					signal = andis;
				end
				3'b001:begin // slli
					is_sll = 1;
					signal = sllis;
				end
				default:begin // srli and srai
					is_sll = 1;
					signal = ( mem_rdata_I_change[31:25] == 7'b0 )?srlis:srais;
				end
			endcase
		end	
		7'b0110011:begin
			instruction_type = R;
			case(mem_rdata_I_change[14:12])
				3'b000: signal = ( mem_rdata_I_change[31:25] == 7'b0 )?(adds):(subs);
				3'b001: signal = slls;
				3'b010: signal = slts;
				3'b100: signal = XORs;
				3'b101: signal = ( mem_rdata_I_change[31:25] == 7'b0 )? (srls):(sras);
				3'b110: signal = ORs;
				default: signal = ANDs;  //3'b111
				//default: ALUControl[3:0] = 4'b0;
			endcase
		end
		default:begin
			signal = 11'b0;
			instruction_type = 5'b0;
		end
	endcase
	
	// imme part
	case(instruction_type)
		I:imme_w =(is_sll==1)?{ {58{mem_rdata_I_change[31]}}, mem_rdata_I_change[25:20] }:{ {52{mem_rdata_I_change[31]}}, mem_rdata_I_change[31:20] };
		S:imme_w = { {52{mem_rdata_I_change[31]}}, mem_rdata_I_change[31: 25], mem_rdata_I_change[11: 7] };
		B:imme_w = { {52{mem_rdata_I_change[31]}}, mem_rdata_I_change[7], mem_rdata_I_change[30: 25], mem_rdata_I_change[11: 8], 1'b0 };
		default:imme_w = {{43{mem_rdata_I_change[31]}},mem_rdata_I_change[31], mem_rdata_I_change[19: 12], mem_rdata_I_change[20], mem_rdata_I_change[30: 21], 1'b0};
	endcase
	
		
		
		
		
		// ALU part
		// ********* register part ********** // read reg:
		// rs1
		case(mem_rdata_I_change[19:15])
			5'd31: ALU_data1 = register31;
			5'd30: ALU_data1 = register30;
			5'd29: ALU_data1 = register29;
			5'd28: ALU_data1 = register28;
			5'd27: ALU_data1 = register27;
			5'd26: ALU_data1 = register26;
			5'd25: ALU_data1 = register25;
			5'd24: ALU_data1 = register24;
			5'd23: ALU_data1 = register23;
			5'd22: ALU_data1 = register22;
			5'd21: ALU_data1 = register21;
			5'd20: ALU_data1 = register20;
			5'd19: ALU_data1 = register19;
			5'd18: ALU_data1 = register18;
			5'd17: ALU_data1 = register17;
			5'd16: ALU_data1 = register16;
			5'd15: ALU_data1 = register15;
			5'd14: ALU_data1 = register14;
			5'd13: ALU_data1 = register13;
			5'd12: ALU_data1 = register12;
			5'd11: ALU_data1 = register11;
			5'd10: ALU_data1 = register10;
			5'd9: ALU_data1 = register9;
			5'd8: ALU_data1 = register8;
			5'd7: ALU_data1 = register7;
			5'd6: ALU_data1 = register6;
			5'd5: ALU_data1 = register5;
			5'd4: ALU_data1 = register4;
			5'd3: ALU_data1 = register3;
			5'd2: ALU_data1 = register2;
			5'd1: ALU_data1 = register1;
			5'd0: ALU_data1 = 64'b0;
			default: ALU_data1 = 64'b0;
		endcase
		// rs2
		case(mem_rdata_I_change[24:20])
			5'd31: ALU_data2_w = register31;
			5'd30: ALU_data2_w = register30;
			5'd29: ALU_data2_w = register29;
			5'd28: ALU_data2_w = register28;
			5'd27: ALU_data2_w = register27;
			5'd26: ALU_data2_w = register26;
			5'd25: ALU_data2_w = register25;
			5'd24: ALU_data2_w = register24;
			5'd23: ALU_data2_w = register23;
			5'd22: ALU_data2_w = register22;
			5'd21: ALU_data2_w = register21;
			5'd20: ALU_data2_w = register20;
			5'd19: ALU_data2_w = register19;
			5'd18: ALU_data2_w = register18;
			5'd17: ALU_data2_w = register17;
			5'd16: ALU_data2_w = register16;
			5'd15: ALU_data2_w = register15;
			5'd14: ALU_data2_w = register14;
			5'd13: ALU_data2_w = register13;
			5'd12: ALU_data2_w = register12;
			5'd11: ALU_data2_w = register11;
			5'd10: ALU_data2_w = register10;
			5'd9: ALU_data2_w = register9;
			5'd8: ALU_data2_w = register8;
			5'd7: ALU_data2_w = register7;
			5'd6: ALU_data2_w = register6;
			5'd5: ALU_data2_w = register5;
			5'd4: ALU_data2_w = register4;
			5'd3: ALU_data2_w = register3;
			5'd2: ALU_data2_w = register2;
			5'd1: ALU_data2_w = register1;
			5'd0: ALU_data2_w = 64'b0;
			default: ALU_data2_w = 64'b0;
		endcase
		ALU_data2 = ( signal[4] == 1'b0 )? ( ALU_data2_w ):( imme_w ); // rs2 or imme
		// ********* The end of register part **********

		// perform calculation
		
		case(signal[3:0])
		ALU_ADD:ALU_dataout = ALU_data1 + ALU_data2;
		ALU_SUB:ALU_dataout = ALU_data1 - ALU_data2;
		ALU_SLL:ALU_dataout = (ALU_data1 << ALU_data2);
		ALU_SLT:ALU_dataout = (ALU_data1 < ALU_data2);
		ALU_XOR:ALU_dataout = ALU_data1 ^ ALU_data2;
		ALU_SRL:ALU_dataout = (ALU_data1 >> ALU_data2);
		ALU_SRA:ALU_dataout = (ALU_data1 >>> ALU_data2); //sign extension 可以這樣嗎?
		ALU_OR:ALU_dataout = ALU_data1 | ALU_data2;
		ALU_AND:ALU_dataout = ALU_data1 & ALU_data2;
		default:ALU_dataout = 64'b0;
		endcase
		
		$display("ALU_data1: 0x%h", ALU_data1);
		$display("ALU_data2: 0x%h", ALU_data2);
		$display("ALU_dataout: 0x%h", ALU_dataout);
		mem_addr_D = ALU_dataout[31:2];
		
		// branch part
		case(mem_rdata_I_change[6:0])
			7'b1101111:begin // jal
				Branch_output = imme_w[31:2];
				mem_addr_I_w = mem_addr_I  + Branch_output;
			end
			
			7'b1100111:begin // jalr
				Branch_output = ALU_dataout[31:2];
				mem_addr_I_w = Branch_output;
			end
			
			7'b1100011:begin // beq and bne
				case(mem_rdata_I_change[14:12])
					3'b000:begin // beq
					Branch_output = (ALU_dataout == 64'b0)? (imme_w[31:2]):(30'b01); // (beq)if not branch, just don't add
					mem_addr_I_w = mem_addr_I  + Branch_output;
					end
					default:begin // bne
					Branch_output = (ALU_dataout == 64'b0)? (30'b01):(imme_w[31:2]);// (bne)
					mem_addr_I_w = mem_addr_I  + Branch_output;
					end
				endcase
			end
				
			default:begin
				Branch_output = 1'b0;
				mem_addr_I_w = mem_addr_I + 30'b01;
			end
		endcase
		 
		
		
		
		
		// write to reg part
		Rd_register_w = (signal[6] == 1'b1)?({mem_rdata_D[7:0], mem_rdata_D[15:8], mem_rdata_D[23:16], mem_rdata_D[31:24], mem_rdata_D[39:32], mem_rdata_D[47:40], mem_rdata_D[55:48], mem_rdata_D[63:56]}):(ALU_dataout[63:0]); //Rd_register_w是可能輸入memory的資料
		// choose Rd_register_w or PC + 4 from jal or jalr
		Rd_register_data = (signal[11] == 1'b1 || signal[10] == 1'b1)? ({mem_addr_I, 2'b0} + 32'b100):(Rd_register_w);
		$display("Rd_register_data: 0x%h", Rd_register_data);
		$display("RegWrite: %b", signal[5]);
		Rd_register_addr = mem_rdata_I_change[11:7] ;
		case(signal[5])
		1'b1: Rd_register_buffer = Rd_register_data;
		default: begin
		
		case(Rd_register_addr)
			5'd31: Rd_register_buffer = register31;
			5'd30: Rd_register_buffer = register30;
			5'd29: Rd_register_buffer = register29;
			5'd28: Rd_register_buffer = register28;
			5'd27: Rd_register_buffer = register27;
			5'd26: Rd_register_buffer = register26;
			5'd25: Rd_register_buffer = register25;
			5'd24: Rd_register_buffer = register24;
			5'd23: Rd_register_buffer = register23;
			5'd22: Rd_register_buffer = register22;
			5'd21: Rd_register_buffer = register21;
			5'd20: Rd_register_buffer = register20;
			5'd19: Rd_register_buffer = register19;
			5'd18: Rd_register_buffer = register18;
			5'd17: Rd_register_buffer = register17;
			5'd16: Rd_register_buffer = register16;
			5'd15: Rd_register_buffer = register15;
			5'd14: Rd_register_buffer = register14;
			5'd13: Rd_register_buffer = register13;
			5'd12: Rd_register_buffer = register12;
			5'd11: Rd_register_buffer = register11;
			5'd10: Rd_register_buffer = register10;
			5'd9: Rd_register_buffer = register9;
			5'd8: Rd_register_buffer = register8;
			5'd7: Rd_register_buffer = register7;
			5'd6: Rd_register_buffer = register6;
			5'd5: Rd_register_buffer = register5;
			5'd4: Rd_register_buffer = register4;
			5'd3: Rd_register_buffer = register3;
			5'd2: Rd_register_buffer = register2;
			5'd1: Rd_register_buffer = register1;
			default: Rd_register_buffer = 64'b0; //only when target is x0
		endcase
		end
		endcase
	$display("Rd_register_buffer: 0x%h", Rd_register_buffer);	
		
	

	
	end

	always @(posedge clk or negedge rst_n) begin // sequential part
		if (!rst_n) begin // low trigger
			$display("Clock(rst): mem_addr_I: %d", mem_addr_I);
			$display("Clock(rst): mem_rdata_I: %h", mem_rdata_I);
			$display("Clock(rst): mem_rdata_D: %h", mem_rdata_D);
        	mem_addr_I = 1'b0;  //I have changed this
        	
		register31 = 1'b0;
		register30 = 1'b0;
		register29 = 1'b0;
		register28 = 1'b0;
		register27 = 1'b0;
		register26 = 1'b0;
		register25 = 1'b0;
		register24 = 1'b0;
		register23 = 1'b0;
		register22 = 1'b0;
		register21 = 1'b0;
		register20 = 1'b0;
		register19 = 1'b0;
		register18 = 1'b0;
		register17 = 1'b0;
		register16 = 1'b0;
		register15 = 1'b0;
		register14 = 1'b0;
		register13 = 1'b0;
		register12 = 1'b0;
		register11 = 1'b0;
		register10 = 1'b0;
		register10 = 1'b0;
		register9 = 1'b0;
		register8 = 1'b0;
		register7 = 1'b0;
		register6 = 1'b0;
		register5 = 1'b0;
		register4 = 1'b0;
		register3 = 1'b0;
		register2 = 1'b0;
		register1 = 1'b0;
    	end
    	else begin //this is for output
			// $display("Clock: mem_addr_I: %d", mem_addr_I);
			// $display("Clock: mem_rdata_I: %h", mem_rdata_I);
			// $display("Clock: mem_rdata_D: %h", mem_rdata_D);
			//$display("Rd_register_buffer: 0x%h", Rd_register_buffer);
			//$display("Rd_register_addr: %d", Rd_register_addr);
			mem_addr_I = mem_addr_I_w;  
			case(Rd_register_addr)
			5'd31:  register31 = Rd_register_buffer;
			5'd30:  register30 = Rd_register_buffer;
			5'd29:  register29 = Rd_register_buffer;
			5'd28:  register28 = Rd_register_buffer;
			5'd27:  register27 = Rd_register_buffer;
			5'd26:  register26 = Rd_register_buffer;
			5'd25:  register25 = Rd_register_buffer;
			5'd24:  register24 = Rd_register_buffer;
			5'd23:  register23 = Rd_register_buffer;
			5'd22:  register22 = Rd_register_buffer;
			5'd21:  register21 = Rd_register_buffer;
			5'd20:  register20 = Rd_register_buffer;
			5'd19:  register19 = Rd_register_buffer;
			5'd18:  register18 = Rd_register_buffer;
			5'd17:  register17 = Rd_register_buffer;
			5'd16:  register16 = Rd_register_buffer;
			5'd15:  register15 = Rd_register_buffer;
			5'd14:  register14 = Rd_register_buffer;
			5'd13:  register13 = Rd_register_buffer;
			5'd12:  register12 = Rd_register_buffer;
			5'd11:  register11 = Rd_register_buffer;
			5'd10:  register10 = Rd_register_buffer;
			5'd9:  register9 = Rd_register_buffer;
			5'd8:  register8 = Rd_register_buffer;
			5'd7:  register7 = Rd_register_buffer;
			5'd6:  register6 = Rd_register_buffer;
			5'd5:  register5 = Rd_register_buffer;
			5'd4:  register4 = Rd_register_buffer;
			5'd3:  register3 = Rd_register_buffer;
			5'd2:  register2 = Rd_register_buffer;
			5'd1:  register1 = Rd_register_buffer;
			default: ; //only when target is x0
			endcase
    	end
    	//$display("x1: 0x%h", register1);
	end
	

endmodule
