-- Name: Matt Robinson
-- Submission Date: 10-22-13
-- Class: CpE315 F'13
-- E-mail Address: msrbqb@mst.edu
--
-- Creating a 32 bit Multicycle Processor
-- LW, SW, Jr, Jump, Beq, Bne, Slt, add, sub, mpy, NOT32,
-- comp, and32, or32, xor32, not32, MSLL, MSRL, MSLA, MSRA
-- use the Behavior or RTL ALU you developed previously
--extending the ALU16 to 32 bit ALU
-----------------------------------------------

-- Implement the RISC Processor described in Ch 4.
-- Avoid pipelining for the midterms
-- Use VHDL to create a testbench around the processor
-- The Processor shall be one model only
-- Create at least a dozen vector pairs to test all the functions of the processor
-- Functions should include: LW, SW, JNE, JEQ, JR, J, ADD, SUB, MPY, SLT, AND, OR, NOT, COMP, SLL, SRL, SLA, SRA
-- You can use the simulator to generate and compare vectors, or use the INFILE, OUTFILE Methodology!

library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.STD_LOGIC_ARITH.UNSIGNED;
use IEEE.STD_LOGIC_UNSIGNED.all;
use IEEE.NUMERIC_STD.UNSIGNED;
use STD.TEXTIO.all;
--
package ProcP is
	type REG32 is Array (0 to 31) of std_logic_vector(31 downto 0);
	type MEM1K is Array(0 to 1000)of std_logic_vector(31 downto 0);
	type Popcode is (LW, SW, Jr, Jump, Beq, Bne, Slt, Add, Sub, Mpy, Not32, Comp, And32, Or32, Xor32, MSLL, MSRL, MSLA, MSRA, NOP);
	type Pstate is (Fetch, Decode, Execute, Retire);
	type Tinstruction is record
		Opcode: Popcode;
		Rs: std_logic_vector (0 to 3);
		Rt: std_logic_vector (0 to 3);
		Rd: std_logic_vector (0 to 3);
		SHAMT: std_logic_vector (0 to 4);
		Funct: std_logic_vector (0 to 6);
	end record;
	function to_std_logic(c: character) return std_logic;
	function to_std_logic_vector(s: string) return std_logic_vector;
	function Conv (temp: string (1 to 32)) return Tinstruction;
	function to_record (PW: std_logic_vector (31 downto 0)) return Tinstruction;
	function to_string (PW: std_logic_vector) return string;
end package ProcP;
--
package body ProcP is
--
	function to_std_logic(c: character) return std_logic is 
		variable sl: std_logic;
	begin
	case c is
		when 'U' => 
			sl := 'U'; 
		when 'X' =>
			sl := 'X';
		when '0' => 
			sl := '0';
		when '1' => 
			sl := '1';
		when 'Z' => 
			sl := 'Z';
		when 'W' => 
			sl := 'W';
		when 'L' => 
			sl := 'L';
		when 'H' => 
			sl := 'H';
		when '-' => 
			sl := '-';
		when others =>
			sl := 'X'; 
		end case;
		return sl;
	end to_std_logic;
--
	function to_std_logic_vector(s: string) return std_logic_vector is
		variable slv: std_logic_vector(s'high-s'low downto 0);
		variable k: integer;
	begin
		k := s'high-s'low;
		for i in s'range loop
			slv(k) := to_std_logic(s(i));
			k := k - 1;
		end loop;
		return slv;
	end to_std_logic_vector;
--
	function Conv (temp: string (1 to 32)) return Tinstruction is
		variable con: std_logic_vector (31 downto 0);
		variable T: Tinstruction;
	begin
		--convert std_logic_vector to record??	
		con := to_std_logic_vector(temp);
		T 	:= to_record(con);
		return T;
	end function Conv;
--
	function to_record (PW: std_logic_vector (31 downto 0)) return Tinstruction is
		variable T: Tinstruction;
	begin
		-- code a logic_vector to record conversion
		case PW(31 downto 24) is
			when "00000000"=> T.opcode := LW;
			when "00000001"=> T.opcode := SW;
			when "00000010"=> T.opcode := Jr;
			when "00000011"=> T.opcode := Jump;
			when "00000100"=> T.opcode := Beq;
			when "00000101"=> T.opcode := Bne;
			when "00000110"=> T.opcode := Slt;
			when "00000111"=> T.opcode := Add;
			when "00001000"=> T.opcode := Sub;
			when "00001001"=> T.opcode := Mpy;
			when "00001010"=> T.opcode := Not32;
			when "00001011"=> T.opcode := Comp;
			when "00001100"=> T.opcode := And32;
			when "00001101"=> T.opcode := Or32;
			when "00001110"=> T.opcode := Xor32;
			when "00001111"=> T.opcode := MSLL;
			when "00010000"=> T.opcode := MSRL;
			when "00010001"=> T.opcode := MSLA;
			when "00010010"=> T.opcode := MSRA;
			when "00010011"=> T.opcode := NOP;
			when others=> T.opcode := NOP;
		end case;
		-- T.Opcode	:= PW(31 downto 24);
		T.Rs 		:= PW(23 downto 20);
		T.Rt 		:= PW(19 downto 16);
		T.Rd 		:= PW(15 downto 12);
		T.SHAMT 	:= PW(11 downto 7);
		T.funct 	:= PW(6 downto 0);
		
		return T;
	end function to_record;
--
	function to_string(PW: std_logic_vector) return string is
		use std.TextIO.all;
		variable bv: bit_vector(PW'range) := to_bitvector(PW);
		variable lp: line;
	begin
		write(lp, bv);
		return lp.all;
	end;
--
end package body ProcP;

--
library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.STD_LOGIC_ARITH.UNSIGNED;
use IEEE.STD_LOGIC_UNSIGNED.all;
use IEEE.NUMERIC_STD.UNSIGNED;
--
use work.ProcP.all;
use work.all;
--
entity ALU_32 is
	port (A_bus, B_bus: in std_logic_vector(31 downto 0); Q_bus: inout std_logic_vector(63 downto 0); Opcode: in Popcode; Proc_ready: out std_logic; clk, reset: in std_logic);
end entity ALU_32;
--
architecture Behavior of ALU_32 is
	signal A, B: std_logic_vector(31 downto 0);
	signal Q: std_logic_vector(63 downto 0);
	signal Overflow: std_logic;
	signal Status_code: std_logic_vector (0 to 3);
--
	procedure action (Bus_A, Bus_B: std_logic_vector (31 downto 0); Bus_Q: std_logic_vector (63 downto 0); B_Opcode: Popcode) is
	begin
		-- You do the necessary actions
	end procedure action;

begin
	A<= A_bus;
	B<= B_bus;
--
	ALU_Exec: process(clk, reset, Opcode)
	begin
		if reset='0' and clk'event and clk='1' then
			case Opcode is
				when Add=> Q <= A+B;
					if Q>16#FFFF# then
						overflow<='1';
						assert not(overflow='1') report "An overflow occurred" severity warning;
					end if;
				when Sub=> Q<=A-B;
					if (B>A) then
						overflow<='1';
						assert not(overflow='1') report "An overflow occurred" severity warning;
					end if;
				when MPY=> Q<=A*B;
					if Q>16#FFFF# then
						overflow<='1';
						assert not(overflow='1') report "An overflow occurred" severity warning;
					end if;
				when Comp=>
					if (A<B) then Status_code (0 to 1) <= "01"; -- report "Less Than";
					elsif (A=B) then Status_code (0 to 1) <= "00"; --report "Equal"
					else Status_code(0 to 1) <= "10"; --report "Greater Than"
					end if;
				when And32 => 	Q<= A and B;
				when Or32 => 	Q<= A or B;
				when Not32 => 	Q<= not A;
				when LW => 		action (A, B, Q, Opcode);
				when SW => 		action (A, B, Q, Opcode);
				when Jr => 		action (A, B, Q, Opcode);
				when JUMP => 	action (A, B, Q, Opcode);
				when Bne => 	action (A, B, Q, Opcode);
				when Beq => 	action (A, B, Q, Opcode);
				when Slt => 	action (A, B, Q, Opcode);
				when MSLL => 	action (A, B, Q, Opcode);
				when MSRL => 	action (A, B, Q, Opcode);
				when MSLA => 	action (A, B, Q, Opcode);
				when MSRA => 	action (A, B, Q, Opcode);
				when others => 	B<= Q; -- Execute a NOP
			end case;
		end if;
	Q_bus <= Q; -- Result deposited to Q_bus
	end process;
end architecture behavior; 

--
library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.STD_LOGIC_ARITH.UNSIGNED;
use IEEE.STD_LOGIC_UNSIGNED.all;
use IEEE.NUMERIC_STD.UNSIGNED;
--
use work.Procp.all;
use work.all;
--
entity MCProc is
	port (PC, PW: inout std_logic_vector (31 downto 0); CLK, Reset: in std_logic);
end entity MCProc;
--
architecture First of MCProc is
	--
	component ALU_32
		port (A_bus, B_bus: in std_logic_vector(31 downto 0); Q_bus: inout std_logic_vector(63 downto 0); Opcode: in Popcode; Proc_ready: out std_logic; clk, reset: in std_logic);
	end component ALU_32;
	--
	signal A, B: std_logic_vector (31 downto 0):= (others => '0');
	signal Q: std_logic_vector (63 downto 0):=(others => '0');
	Signal R: REG32;
	signal Memory: MEM1K;
	signal Instruction: Tinstruction;
	signal Opcode: Popcode;
	signal Proc_ready: std_logic;
	signal STATE: PSTATE;
	for ALU_32C: ALU_32 use entity work.ALU_32(Behavior);
	--
begin
	Instruction <= to_record (PW);
	Opcode <= Instruction.opcode;
	ALU_32C: ALU_32 port map (A, B, Q, Opcode, Proc_ready, CLK, Reset);
	
	PControl: Process
	begin
		wait until (Reset = '0' and CLK'event and CLK='1');
		case STATE is
			when Fetch =>
				--stuff
				-- code Fetch --Instruction
				--Instruction <= to_record(Memory(PC));
				
				STATE <=Decode;
			when Decode =>
				--stuff
				-- code Decode --Instruction
				
				-- (LW, SW, Jr, Jump, Beq, Bne, Slt, Add, Sub, Mpy, Not32, Comp, And32, Or32, Xor32, MSLL, MSRL, MSLA, MSRA, NOP)
				-- case Instruction is
					-- when '000' =>
						
					-- when '001' =>
						
					-- when '010' =>
						
					-- when '011' =>
						
					-- when '100' =>
						
					-- when '101' =>
						
					-- when '110' =>
						
					-- when '111' =>
						
					-- when others =>
						
				-- end case;
				STATE<= Execute;
			when Execute =>	
				if (Proc_ready ='0') then STATE<= Execute;
				else STATE <= Retire;
					--do more stuff -- code Execute (ALU) Instruction
				end if;
			when Retire =>
				if (Proc_ready='1') then STATE <= Fetch;
					-- code Retire (Store Register, store memory, or Load memory)
				end if;
		end case;
	end process;
end architecture First;
 
-- Test Bench for MIPS-2 Processor
library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.STD_LOGIC_ARITH.UNSIGNED;
use IEEE.STD_LOGIC_UNSIGNED.all;
use IEEE.NUMERIC_STD.UNSIGNED;
use work.ProcP.all;
use STD.TEXTIO.all;
use work.all;
--
entity MCPROC_TB is
end entity MCPROC_TB;
--
architecture TEST of MCPROC_TB is
	component MCProc
		port (PC, PW: inout std_logic_vector (31 downto 0); CLK, Reset: in std_logic);
	end component MCProc;
--	
	signal A, B, PC, PW: std_logic_Vector (31 downto 0):=(others => '0');
	signal Q: std_logic_vector (63 downto 0):=(others => '0');
	Signal Reset, CLK: std_logic;
	signal MEMORY: MEM1K;
	signal Proc_ready: std_logic;
	signal Instruction: Tinstruction;
	Signal GO: std_logic;

	file InFile  : text open read_mode  is "\\minerfiles.mst.edu\dfs\users\msrbqb\Desktop\cpe315midterm\stimulus.txt";
	file Outfile : text open write_mode is "\\minerfiles.mst.edu\dfs\users\msrbqb\Desktop\cpe315midterm\stim_out.txt";
--
	-- Potentially need this nonsense
	for MY_PROC: MCPROC use entity work.MCPROC(First);
begin
	Reset <= '1','0' after 100 ps;
	--
	CLK_P: process
	begin
		CLK <= '0';
		wait for 5 ps;
		CLK <= '1';
		wait for 5 ps;
	end process;

	MY_PROC: MCPROC port map (PC, PW, CLK, Reset);

	Inst_Stimulate: process
		variable L1: line;
		variable S1, S2, S3, S4: string (1 to 33);
		variable temp: string(1 to 32) := (others => ' ');
	begin
		wait until (Reset='0' and Proc_ready ='1' and CLK'event and CLK= '1');
		while not (EndFile(InFile)) loop
			Go <= '0';
			READLINE(InFile, L1);-- you might need to read into a	--line!
			READ(L1, temp);
			Instruction <= Conv(temp);

			case Instruction.opcode is
				when NOP => A <= A;
				when others => Go <= '1'; -- will be calling the Microp
			end case;
			
			S1 := to_string(PW);
			S2 := to_string(A);
			S3 := to_string(B);
			S4 := to_string(Q);
			WRITE(OutFile, S1 & " " & S2 & " " & S3 & " " & S4);
		end loop;
	end process;
end architecture TEST;