/*
(Main Module)
Organizes/calls the hierarchies of sub-modules in our final design.
Has state information and handles all the pause/setup info.

Inputs:
=======
AD - Address Data
CE - Chip Enable bit
PU - Pause
Clk1/Clk2 - Clocks
Const - Constants describing states
CFG - Configure Mode bit

Outputs:
========
AD - Address Data
ALE - Address Latch Enable
RD - Read bit
WR - Write bit

Other:
======
a1 - first feedback coefficient
a2 - second feedback coefficient
b0 - first feedforward coefficient
b1 - second feedforward coefficient
b2 - third feedforward coefficient
devAddr - address of input/output
px - first previous value of x
ppx - second previous value of x
py - first previous value of y
ppy - second previous value of y
y - PIIR Filter output value (fed back into, hence IR in PIIR)
setx - input term going into filter
config_flag - signals when to config
*/

module PIIR( inout [15:0] AD, output ALE, output RD, output WR, input CE, input PU, input Clk1, input Clk2,
            input [3:0] Const, input CFG);

wire[15:0] a1,a2,b0,b1,b2, devAddr ,px,ppx,py,ppy, y, setx;
wire config_flag;

assign AD = (ALE)? devAddr: ((WR)? y:16'bz);

configure_mode conf(Clk2, CFG, Const, AD, a1, a2, b0, b1, b2, py, ppy, px, ppx, setx, devAddr, CE, y, RD, config_flag);
run_forest_run runner(config_flag, RD, WR, ALE, PU, CFG, Clk1, Clk2, setx, y, px, ppx, py, ppy, a1, a2, b0, b1, b2, CE);
endmodule

/*
(Fixed-point Adder)
Adds two 16-bit fixed-point numbers(Signed magnitude not 2's complement.).

Inputs:
=======
in1 - Input 1
in2 - Input 2

Outputs:
========
Sum - in1+in2

Other:
======
temp1 - Holds least significant 15 bits of in1, and used for calculations.
temp2 - Hold least significant 15 bits of in2, and used for calculations.
*/

module adder(sum,in1,in2);
 
 output reg [15:0] sum;
 
 input [15:0] in1,in2;
 wire [14:0] temp1,temp2;
 
 assign temp1= in1[14:0];
 assign temp2= in2[14:0];
 
 always @( in1, in2)
 begin
   case({in1[15],in2[15]})
     
   2'b00 : begin
              sum[14:0] = temp1+temp2;
              sum = {1'b0,sum[14:0]};
           end
   2'b01 : begin
            if(temp1>=temp2)
            sum = {1'b0,(temp1-temp2)};
            else
            sum = {1'b1,(temp2-temp1)};        
           end
   2'b10:  begin
             if(temp1>temp2)
            sum = {1'b1,(temp1-temp2)};
            else
            sum = {1'b0,(temp2-temp1)};
           end        
   2'b11 : begin
              sum[14:0] = temp1+temp2;
              sum = {1'b1,sum[14:0]};
           end
   endcase
 end  

endmodule


/*
(Configure Mode)
Module is called from within PIIR to set up intial state information. 

Inputs:
=======
cfg - Configure Mode bit
CE - Chip Enable bit
RD - Read bit
config_flag - Boolean value used to determine when the PIIR should configure
y - PIIR Filter output value (fed back into, hence IR in PIIR)
constant - Constants describing state
address - Address Data
clk - Clock 2

Outputs:
========
a_one - first feedback coefficient
a_two - second feedback coefficient
b_zero - first feedforward coefficient
b_one - second feedforward coefficient
b_two - third feedforward coefficient
prev_y - first previous value of y
pprev_y - second previous value of y
prev_x - first previous value of x
pprev_x - second previous value of x
x - input term going into filter
devAddr - address of input/output
*/
module configure_mode(clk,cfg,constant,address,a_one, a_two,b_zero,b_one, b_two,prev_y,pprev_y, prev_x,pprev_x,x,devAddr, CE, y, RD, config_flag);

input cfg, CE, RD,config_flag;
input [15:0] y;
input [3:0] constant;
inout [15:0] address;
input clk;


output reg [15:0] a_one, a_two,b_zero,b_one, b_two,prev_y,pprev_y, prev_x,pprev_x,x,devAddr;

always@(posedge clk) //cfg instead of clk?
begin
  if(!cfg && CE) begin
  case(constant)
        
    //zero
     4'b0000: begin
                  a_one   <= a_one;
                  a_two   <= a_two;
                  b_zero  <= b_zero;
                  b_one   <= b_one;
                  b_two   <= b_two;
                  prev_y  <= prev_y;
                  pprev_y <= pprev_y;
                  prev_x  <= prev_x;
                  pprev_x <= pprev_x;
                  x       <= x;
                  devAddr <= devAddr;
                  end

    //one
     4'b0001: begin
                  a_one   <= {~address[15], address[14:0]};
                  //a_one   <= address;
                  a_two   <= a_two;
                  b_zero  <= b_zero;
                  b_one   <= b_one;
                  b_two   <= b_two;
                  prev_y  <= prev_y;
                  pprev_y <= pprev_y;
                  prev_x  <= prev_x;
                  pprev_x <= pprev_x;
                  x       <= x;
                  devAddr <= devAddr;
                  end
                  
    //two
    4'b0010: begin
                  a_one   <= a_one;
                  a_two   <= {~address[15], address[14:0]};
                  //a_two   <= address;
                  b_zero  <= b_zero;
                  b_one   <= b_one;
                  b_two   <= b_two;
                  prev_y  <= prev_y;
                  pprev_y <= pprev_y;
                  prev_x  <= prev_x;
                  pprev_x <= pprev_x;
                  x       <= x;
                  devAddr <= devAddr;
                  end



    // five
    4'b0101: begin
                  a_one   <= a_one;
                  a_two   <= a_two;
                  b_zero  <= b_zero;
                  b_one   <= b_one;
                  b_two   <= b_two;
                  prev_y<= address;
                  pprev_y <= pprev_y;
                  prev_x  <= prev_x;
                  pprev_x <= pprev_x;
                  x       <= x;
                  devAddr <= devAddr;
                  end

    //six
     4'b0110: begin
                  a_one   <= a_one;
                  a_two   <= a_two;
                  b_zero  <= b_zero;
                  b_one   <= b_one;
                  b_two   <= b_two;
                  prev_y  <= prev_y;
                  pprev_y <= address;
                  prev_x  <= prev_x;
                  pprev_x <= pprev_x;
                  x       <= x;
                  devAddr <= devAddr;
                  end
    
    //eight
    4'b1000: begin
                  a_one   <= a_one;
                  a_two   <= a_two;
                  b_zero  <= address;
                  b_one   <= b_one;
                  b_two   <= b_two;
                  prev_y  <= prev_y;
                  pprev_y <= pprev_y;
                  prev_x  <= prev_x;
                  pprev_x <= pprev_x;
                  x       <= x;
                  devAddr <= devAddr;
                  end
    
    //nine              
    4'b1001: begin
                  a_one   <= a_one;
                  a_two   <= a_two;
                  b_zero  <= b_zero;
                  b_one   <= address;
                  b_two   <= b_two;
                  prev_y  <= prev_y;
                  pprev_y <= pprev_y;
                  prev_x  <= prev_x;
                  pprev_x <= pprev_x;
                  x       <= x;
                  devAddr <= devAddr;
                  end

    //ten                  
    4'b1010: begin
                  a_one   <= a_one;
                  a_two   <= a_two;
                  b_zero  <= b_zero;
                  b_one   <= b_one;
                  b_two   <= address;
                  prev_y  <= prev_y;
                  pprev_y <= pprev_y;
                  prev_x  <= prev_x;
                  pprev_x <= pprev_x;
                  x       <= x;
                  devAddr <= devAddr;
                  end
                  
    // twelve
    4'b1100: begin
                  a_one   <= a_one;
                  a_two   <= a_two;
                  b_zero  <= b_zero;
                  b_one   <= b_one;
                  b_two   <= b_two;
                  prev_y  <= prev_y;
                  pprev_y <= pprev_y;
                  prev_x  <= prev_x;
                  pprev_x <= pprev_x;
                  x       <= address;
                  devAddr <= devAddr;
                  end
    
    // thirteen
    4'b1101: begin
                  a_one   <= a_one;
                  a_two   <= a_two;
                  b_zero  <= b_zero;
                  b_one   <= b_one;
                  b_two   <= b_two;
                  prev_y  <= prev_y;
                  pprev_y <= pprev_y;
                  prev_x  <= address;
                  pprev_x <= pprev_x;
                  x       <= x;
                  devAddr <= devAddr;
                  end

    // fourteen                  
    4'b1110: begin
                  a_one   <= a_one;
                  a_two   <= a_two;
                  b_zero  <= b_zero;
                  b_one   <= b_one;
                  b_two   <= b_two;
                  prev_y  <= prev_y;
                  pprev_y <= pprev_y;
                  prev_x  <= prev_x;
                  pprev_x <= address;
                  x       <= x;
                  devAddr <= devAddr;
                  end
  
    //fifteen
    4'b1111: begin
                  a_one   <= a_one;
                  a_two   <= a_two;
                  b_zero  <= b_zero;
                  b_one   <= b_one;
                  b_two   <= b_two;
                  prev_y  <= prev_y;
                  pprev_y <= pprev_y;
                  prev_x  <= prev_x;
                  pprev_x <= pprev_x;
                  x       <= x;
                  devAddr <= address;
                  end

    default     : begin
                  a_one   <= a_one;
                  a_two   <= a_two;
                  b_zero  <= b_zero;
                  b_one   <= b_one;
                  b_two   <= b_two;
                  prev_y  <= prev_y;
                  pprev_y <= pprev_y;
                  prev_x  <= prev_x;
                  pprev_x <= pprev_x;
                  x       <= x;
                  devAddr <= devAddr;
                  end           
  endcase
  end else begin
    if(RD & ~config_flag) begin
      a_one   <= a_one;
      a_two   <= a_two;
      b_zero  <= b_zero;
      b_one   <= b_one;
      b_two   <= b_two;
      prev_y  <= y;
      pprev_y <= prev_y;
      prev_x  <= x;
      pprev_x <= prev_x;
      x       <= address;
      devAddr <= devAddr;
    
    end else begin
      a_one   <= a_one;
      a_two   <= a_two;
      b_zero  <= b_zero;
      b_one   <= b_one;
      b_two   <= b_two;
      prev_y  <= prev_y;
      pprev_y <= pprev_y;
      prev_x  <= prev_x;
      pprev_x <= pprev_x;
      x       <= x;
      devAddr <= devAddr;   
      
    end
  end      
end  

endmodule


/*
(Multiply Accumulator)
Used to model PIIR Filter function.
Iterates through inputs multiplying them and accumulating the values they generate..
Used for pipelining design and reducing the overall number of hardware adders/multipliers we need.

Inputs:
=======
cfg - Configure Mode bit
CE - Chip Enable bit
RD - Read bit
config_flag - Boolean value used to determine when the PIIR should configure
y - PIIR Filter output value (fed back into, hence IR in PIIR)
constant - Constants describing state
address - Address Data
clk - Clock 2

Outputs:
========
a_one - first feedback coefficient
a_two - second feedback coefficient
b_zero - first feedforward coefficient
b_one - second feedforward coefficient
b_two - third feedforward coefficient
prev_y - first previous value of y
pprev_y - second previous value of y
prev_x - first previous value of x
pprev_x - second previous value of x
x - input term going into filter
devAddr - address of input/output
*/
module mac_work(input clk,rst,input[15:0]a1,a2,b1,b2,b0,py,ppy,x,px,ppx,output reg [15:0] answer,
                                                                  output reg done);
  
  reg [15:0]sum;
  reg[15:0] add1;
  wire [2:0] next;
  reg[2:0] state;
  reg [15:0] a, b;
  wire [15:0] mult1, add2;
 
  multiplier m1(mult1,a,b);
  adder adder2(add2, add1, mult1);
  
  
  always@(state)
  begin
	  if(rst || state == 3'd0) add1 = 15'd0;
	  else add1 = add2;  
  end  
  
  //next state logic
  assign next =(rst)? 3'd0: ((state == 3'd5)?3'd0:state+1);
  
  // laoding answer once we are done
  always@(posedge clk)
  begin
  if(state == 3'd5)
    answer <= sum;
  else 
    answer <= answer;
  end
  
  // state transitions
  always@(posedge clk)begin
    state <= next;  
    sum <= add2;
  end
  
  //output logic  
  always@(state)begin 
    
    case(state)     

      3'd0: begin 
        a = a1;
        b = py;
        done =1'b0;  
      end
      3'd1: begin 
        a = a2;
        b = ppy;
        done=1'b0;
      end
      3'd2: begin 
        a = b1; 
        b = px;
        done =1'b0; 
      end
      
      3'd3: begin
        a = b2;
        b= ppx;
      done = 1'b0;
      end
      
       3'd4: begin 
        a = x;
        b = b0;
        done =1'b0;
      end
      3'd5: begin 
	a=16'hxxxx;
        b=16'hxxxx;
        done=1'b1;
      end
          
    default: begin 
        a=16'hxxxx;
        b=16'hxxxx;
        done =1'bx;
      end  
    endcase
  end 
  
endmodule


/*
(Fixed-point Multiplier)
Multiplies two 16-bit fixed-point numbers.

Inputs:
=======
in1 - Input 1
in2 - Input 2

Outputs:
========
Product - in1*in2

Other:
======
temp - Temporary variable. Holds the value of temp1*temp2.
temp1 - Temporary variable. Value varies.
temp2 - Temporary variable. Value varies.
endTemp - Temporary variable. Used in intermediate calculation of product.
*/
module multiplier(product,in1,in2);
 
 reg [31:0] temp;
 output reg [15:0] product;
 reg [15:0] temp1, temp2, endTemp;
 input [15:0] in1,in2;
 
 always @( in1, in2)
 begin
  if(in1[15])
    temp1 = {~in1[15], in1[14:0]};
  else
    temp1 = in1;
    
    
  if(in2[15])
    temp2 = {~in2[15], in2[14:0]};
  else
    temp2 = in2;
    
  temp = temp2 * temp1;
  endTemp = temp>>15;
  if(in1[15] ^ in2[15])
    product = {~endTemp[15], endTemp[14:0]};
  else
    product = endTemp;
  
  
 end  
  
endmodule



/*
(Run-time Module) Parameters { ALE_STATE, PAUSED, READ, MATH, WRITE, CONFIG }
When PIIR Filter is in run-mode.
Performs shifting of states and handles operations necessary for PIIR Filter to function correctly.

Inputs:
=======
a1 - first feedback coefficient
a2 - second feedback coefficient
b0 - first feedforward coefficient
b1 - second feedforward coefficient
b2 - third feedforward coefficient
x - input term going into filter
prev_x - first previous value of x
pprev_x - second previous value of x
prev_y - first previous value of y
pprev_y - second previous value of y
pu - Pause Mode bit
cfg - Configure bit
clk1 - Clock 1
clk2 - Clock 2
CE - Chip Enable

Outputs:
========
rd,wr,ale,config_flag
y - PIIR Filter output value (fed back into, hence IR in PIIR)

Other:
======
state - Current state value
next_state - Next state value
done - Boolean value that signals completion of operations
outputOfAll - Output of MAC
*/

module run_forest_run(config_flag,rd,wr,ale,pu,cfg,clk1,clk2,x,y,px,ppx,py,ppy,a1,a2,b0,b1,b2,CE);

  reg[2:0] state, next_state;

  wire done;
  output reg rd,wr,ale,config_flag;
  output reg [15:0]y;
  input [15:0] a1,a2,b0,b1,b2,x,px,ppx,py,ppy;
  input pu,cfg,clk1,clk2, CE; //pu is active high, cfg is active low
  
  wire [15:0]outputOfAll;
  
  parameter ALE_STATE  = 3'd0,// Ale is high
            PAUSED =3'd1,// Paused state
            READ =3'd2,// Read is high 
            MATH =3'd3, //does math in this state 
            WRITE=3'd4,//Write is high  
            CONFIG = 3'd5; // Config state
  
  // next state and falg logic, fsm logic in report pdf
  always @(cfg,pu,rd,ale,wr,done)
  begin
  
  if(!cfg) next_state = CONFIG;
  
  else 
    begin
    if(pu) next_state = PAUSED;
    
    else
    begin
      case(state)
        3'b000: begin next_state = READ;
        if(config_flag) 
        config_flag=1'b1;
        else
        config_flag= 1'b0;
        end
         
        3'b001: begin next_state = READ;if(config_flag) 
        config_flag=1'b1;
        else
        config_flag= 1'b0;
        end
         
        3'b010: begin next_state = MATH;
                    if(config_flag) 
        config_flag=1'b1;
        else
        config_flag= 1'b0;
        end
          
        3'b011: begin if(done) next_state = WRITE; 
                      else next_state = MATH; 
                      
                      config_flag = 1'b0;
                      end
                      
        3'b100: begin next_state = ALE_STATE; 
                if(config_flag) 
                config_flag=1'b1;
                else
                config_flag= 1'b0;
                end
                
        3'b101: begin next_state = ALE_STATE;
                      config_flag =1'b1;end
                        
        default :begin next_state = 3'bxxx;
        
        if(config_flag) 
        config_flag=1'b1;
        else
        config_flag= 1'b0;end
    endcase
    end  
       
    end 
     
  end
  

  always @(posedge clk1)
  begin
    if(CE)
      state <= next_state;
  end  
  
  always @(posedge clk1)
  begin
      if(state == WRITE)
        y <= outputOfAll;
      else
        y <= y;
  end
   
  //state transitions and outputs
  always @(state,cfg) 
  begin
    case(state)  
      
      ALE_STATE: begin 
                rd=1'b0; 
                ale=1'b1; 
                wr=1'b0;
                end    
  
      CONFIG: begin wr=1'b0; rd=1'b1; ale=1'b0; end
     
      PAUSED: begin rd=1'b0; ale=1'b0; wr=1'b0;end //pause

      READ: begin ale=1'b0; rd=1'b1; wr=1'b0; end // read
      
      WRITE: begin wr=1'b1; rd=1'b0; ale=1'b0; end//write
      
      MATH : begin wr=1'b0; rd=1'b0; ale=1'b0;end //math
      
      default: begin rd=1'b0; ale=1'b0; wr=1'b0;end
    endcase  
  end
  
mac_work please_work(clk1,rd,a1,a2,b1,b2,b0,py,ppy,x,px,ppx,outputOfAll,done);
    
endmodule     


// test becnch
module PIIR_tester2();
  wire [15:0] AD;
  reg[5:0] count;
  reg [15:0] inputValue;
  wire ALE, RD, WR;
  reg CE, PU, Clk1, Clk2, CFG;
  reg [3:0] Const;
  reg [15:0] values [34:0];
  PIIR p(AD, ALE, RD, WR, CE, PU, Clk1, Clk2, Const, CFG);
  
  assign AD = (RD | ~CFG)? inputValue : 16'bz;
  
  initial begin
//  CE = 
  count = 0;
  values[0] = 16'hd609;
  values[1] = 16'h8465;
  values[2] = 16'h5212;
  values[3] = 16'he301;
  values[4] = 16'hcd0d;
  values[5] = 16'hf176;
  values[6] = 16'hcd3d;
  values[7] = 16'h57ed;
  values[8] = 16'hf78c;  
  values[9] = 16'he9f9;
  values[10]= 16'h24c6;      
  values[11]= 16'h84c5;     
  values[12]= 16'hd2aa;      
  values[13]= 16'hf7e5;      
  values[14]= 16'h7277;  
  values[15]= 16'hd612;      
  values[16]= 16'hdb8f;      
  values[17]= 16'h69f2;      
  values[18]= 16'h96ce;      
  values[19]= 16'h7ae8;      
  values[20]= 16'h4ec5;  
  values[21]= 16'h495c;      
  values[22]= 16'h28bd;      
  values[23]= 16'h582d;      
  values[24]= 16'h2665;      
  values[25]= 16'h6263;  
  values[26]= 16'h870a;   
  values[27]= 16'h2280;      
  values[28]= 16'h2120;      
  values[29]= 16'h45aa;      
  values[30]= 16'hcc9d;  
  values[31]= 16'h3e96;      
  values[32]= 16'hb813;      
  values[33]= 16'h380d;      
  values[34]= 16'hd653;      
  values[35]= 16'hdd6b;    
  

  PU = 0;
  CE = 1;
  Clk1 = 0;
  Clk2 = 0;
  CFG = 0;
  inputValue = 16'h1000;
  #20
  Const = 4'b0001; //A1
  #40 
  CFG = 0;
  Const = 4'b0010; //A2
  inputValue = 16'h9000;
  #40 
  CFG = 0; 
  Const = 4'b0101; //y(-t)
  inputValue = 16'h3524;
  #40
  CFG = 0;
  Const = 4'b0110; //y(-2t)
  inputValue = 16'h5e81;
  #40
  CFG = 0; //b0
  Const = 4'b1000;
  inputValue = 16'h9000;
  #40  
  CFG = 0;
  Const = 4'b1001; //b1
  inputValue = 16'h1000;
  #40 
  CFG = 0;
  Const = 4'b1010; //b2
  inputValue = 16'h9000;
  #40 
  CFG = 0;
  Const = 4'b1100; //x
  inputValue = 16'hd609;
  #40 
  CFG = 0;
  Const = 4'b1101; //x(-t)
  inputValue = 16'h5663;
  #40 
  CFG = 0;
  Const = 4'b1110; //x(-2t)
  inputValue = 16'h7b0d;
  #40
  CFG = 0;
  Const = 4'b1111;
  inputValue = 16'h998d;
  #40
  CFG = 1;
  end
  
  always@(posedge RD) begin
    if(RD && CFG) begin
      //inputValue = $random;
      inputValue = values[count];
      if(count < 34)
        count = count + 1;
      else
        count = 0;
    end
  end
  
  always begin
    #10 Clk1 = !Clk1;
    #10 Clk1 = !Clk1;
    #10 Clk2 = !Clk2;
    #10 Clk2 = !Clk2;
  end
  
endmodule

