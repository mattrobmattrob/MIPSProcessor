-- Name: Matt Robinson
-- Submission Date: 10-22-13
-- Class: CpE315 F'13
-- E-mail Adress: msrbqb@mst.edu
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
	type Tinstruction is Record
	Opcode: Popcode;
		Rs: std_logic_vector (0 to 3);
		Rt: std_logic_vector (0 to 3);
		Rd: std_logic_vector (0 to 3);
		SHAMT: std_logic_vector (0 to 4); 
		Funct: std_logic_vector (0 to 6); 
	end record;
	function Conv (temp: string (0 to 31)) return Tinstruction;
	function to_record (PW: std_logic_vector (31 downto 0)) return Tinstruction;
	function to_string (PW: std_logic_vector (31 downto 0)) return String;
end package ProcP;
--
package body ProcP is
--
	function Conv (temp: string (0 to 31)) return Tinstruction is
	begin
		--convert std_logic_vector to record??
	end function Conv;
--
	function to_string (PW: std_logic_vector (31 downto 0)) return string is
		variable S: String(0 to 31);
	begin
		-- code a logic_vector to record conversion
		return S;
	end function to_string;
--
	function to_text (PW: std_logic_vector (31 downto 0)) return string is
		variable T: string (0 to 31);
	begin
		-- code to convert logic_vector to Text
		return T;
	end function to_text;

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
				when And32 => 	Q<=A and B;
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
				--stuff--STATE <=Decode;
				-- code Fetch --Instruction
				Instruction <= Memory(PC);
			when Decode =>
				--stuff--STATE<= Execute;
				-- code Decode --Instruction
				
				-- (LW, SW, Jr, Jump, Beq, Bne, Slt, Add, Sub, Mpy, Not32, Comp, And32, Or32, Xor32, MSLL, MSRL, MSLA, MSRA, NOP)
				case Instruction is
					when '000' =>
						
					when '001' =>
						
					when '010' =>
						
					when '011' =>
						
					when '100' =>
						
					when '101' =>
						
					when '110' =>
						
					when '111' =>
						
					when others =>
						
				end case;

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

	File InFile: text is in "C:/Work/Stimulus.txt";
	File OutFile: text is out "C:/Work/Response.txt";
--
begin
	Reset <= '1', '0' after 100ps;
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
		Variable S1, S2, S3, S4: string (0 to 32);
		variable temp: string(1 to 16) := (others => ' ');
	begin
		wait until (Reset='0' and Proc_ready ='1' and CLK'event and CLK= '1');
		while not (EndFile (InFile)) loop
			Go <= '0';
			READLINE (InFile, L1);-- you might need to read into a	--line!
			READ (L1, temp);
			Instruction <= Conv (temp);

			case Instruction.opcode is
				when NOP => A<= A; 
				when others => Go <= '1'; -- will be calling the Microp
			end case;
			S1 := to_string (PW); S2 := to_string(A); S3 := to_string(B); S4 := to_string(Q);
			WRITE (OutFile, S1, S2, S3, S4);
		end loop;
	end process;
end architecture TEST;