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
	type REG32 			is Array (0 to 31) of std_logic_vector(31 downto 0);
	type MEM1K 			is Array(0 to 1000)of std_logic_vector(31 downto 0);
	type Popcode 		is (LW, SW, Jr, Jump, Beq, Bne, Slt, Add, Sub, Mpy, Not32, Comp, And32, Or32, Xor32, MSLL, MSRL, MSLA, MSRA);
	type Pstate 		is (Fetch, Decode, Execute, Retire);
	type Tinstruction 	is record
		Opcode: Popcode;
		Rs: std_logic_vector (0 to 4);
		Rt: std_logic_vector (0 to 4);
		Rd: std_logic_vector (0 to 4);
		SHAMT: std_logic_vector (0 to 4);
		Funct: std_logic_vector (0 to 5);
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
		variable slv: 	std_logic_vector(s'high-s'low downto 0);
		variable k: 	integer;
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
		variable con: 	std_logic_vector (31 downto 0);
		variable T: 	Tinstruction;
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
		-- (LW, SW, Jr, Jump, Beq, Bne, Slt, Add, Sub, Mpy, Not32, Comp, And32, Or32, Xor32, MSLL, MSRL, MSLA, MSRA, NOP)
		-- type Tinstruction is record
		--     Opcode: Popcode;
		--     Rs: std_logic_vector (0 to 3);
		--     Rt: std_logic_vector (0 to 3);
		--     Rd: std_logic_vector (0 to 3);
		--     SHAMT: std_logic_vector (0 to 4);
		--     Funct: std_logic_vector (0 to 6);
		
		case PW(31 downto 26) is
			when "000000" =>
				case PW(5 downto 0) is
					when "001000" =>
						T.opcode := jr;
						--jr
					when "101010" =>
						T.opcode := slt;
						--slt
					when "100000" =>
						T.opcode := add;
						--add
					when "100010" =>
						T.opcode := sub;
						--sub
					when "011000" =>
						T.opcode := mpy;
						--mpy
					when "100100" =>
						T.opcode := and32;
						--and
					when "100101" =>
						T.opcode := or32;
						--or
					when "100110" =>
						T.opcode := xor32;
						--xor
					when "000000" =>
						T.opcode := msll;
						--msll (sll)
					when "000010" =>
						T.opcode := msrl;
						--msrl (srl)
					when "000001" =>
						--not on green card
						T.opcode := msla;
						--msla (sla)
					when "000011" =>
						T.opcode := msra;
						--msra (sra)
					when "101100" =>
						--not on green card
						T.opcode := comp;
						--comp
					when "101000" =>
						--not on green card
						T.opcode := not32;
						--not
					when others =>
				end case;
			when "100011" =>
				T.opcode := lw;
				--lw
			when "101011" =>
				T.opcode := sw;
				--sw
			when "000010" =>
				T.opcode := jump;
				--jump
			when "000100" =>
				T.opcode := beq;
				--beq
			when "000101" =>
				T.opcode := bne;
				--bne
			when "010001" =>
				--(2)
			when others =>
		end case;
	
		-- T.Opcode	:= PW(31 downto 26);
		
		T.Rs 		:= PW(25 downto 21);
		T.Rt 		:= PW(20 downto 16);
		T.Rd 		:= PW(15 downto 11);
		T.SHAMT 	:= PW(10 downto 6);
		T.funct 	:= PW(5 downto 0);
		
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
library ieee;
use ieee.std_logic_1164.ALL;
use ieee.numeric_std.all;
--
use work.ProcP.all;
use work.all;
--
entity ALU_32 is
	port (A_bus, B_bus: in std_logic_vector(31 downto 0); Q_bus: inout std_logic_vector(63 downto 0); Opcode: in Popcode; Proc_ready: out std_logic; clk, reset: in std_logic);
end entity ALU_32;
--
architecture Behavior of ALU_32 is
	signal A, B: 		std_logic_vector(31 downto 0):=(others => '0');
	signal Q: 			std_logic_vector(63 downto 0):=(others => '0');
	signal Overflow: 	std_logic;
	signal Status_code: std_logic_vector (0 to 3);
--
	procedure action (Bus_A, Bus_B: in std_logic_vector (31 downto 0); signal Bus_Q: inout std_logic_vector (63 downto 0); B_Opcode: in Popcode) is
		variable temp_Q: 	std_logic_vector(31 downto 0);
		variable zeros: 	std_logic_vector(31 downto 0):=(others => '0');
		variable ones: 		std_logic_vector(31 downto 0):=(others => '1');
	begin
		case B_Opcode is
			when LW => Bus_Q <= zeros(31 downto 0) & Bus_A;
				
			when SW => Bus_Q <= zeros(31 downto 0) & Bus_B;
				
			when Jr => Bus_Q <= zeros(31 downto 1) & std_logic_vector(signed('0' & Bus_A) + signed('0' & Bus_B));
				
			when JUMP => Bus_Q <= zeros(31 downto 0) & Bus_A;
				
			when Bne =>
				if Bus_A /= Bus_B then
					-- Branch
					Bus_Q <= zeros(31 downto 0) & zeros(31 downto 1) & "1";
				else
					Bus_Q <= zeros(31 downto 0) & zeros(31 downto 0);
				end if;
				
			when Beq =>
				if Bus_A = Bus_B then
					Bus_Q <= zeros(31 downto 0) & zeros(31 downto 1) & "1";
				else
					Bus_Q <= zeros(31 downto 0) & zeros(31 downto 0);
				end if;
				
			when Slt =>
				if Bus_A < Bus_B then Bus_Q <= zeros(31 downto 0) & zeros(31 downto 1) & "1";
				else Bus_Q <= zeros(31 downto 0) & zeros(31 downto 1) & "0";
				end if;
			
			when MSLL =>
				Bus_Q <= zeros(31 downto 0) & std_logic_vector(shift_left( signed(Bus_A), to_integer(signed(Bus_B)) ));
			
			when MSRL =>
				-- shift_right replaces upper new bits with sign bit
				-- need to replace those with 0s for SRA
				temp_Q := std_logic_vector(shift_right( signed(Bus_A), to_integer(signed(Bus_B)) ));
				Bus_Q <= zeros(31 downto 0) & (temp_Q and (zeros(31 downto (31 - to_integer(signed(Bus_B)))) & ones((31 - to_integer(signed(Bus_B))) downto 0)));
			
			when MSLA =>
				Bus_Q <= zeros(31 downto 0) & std_logic_vector(shift_left( signed(Bus_A), to_integer(signed(Bus_B)) ));
			
			when MSRA =>
				Bus_Q <= zeros(31 downto 0) & std_logic_vector(shift_right( signed(Bus_A), to_integer(signed(Bus_B)) ));
			
			when others => 	Bus_Q <= Bus_Q; -- Execute a NOP
		end case;
	end procedure action;

begin
	A<= A_bus;
	B<= B_bus;
--
	ALU_Exec: process(clk, reset, Opcode)
		variable zeros: std_logic_vector(31 downto 0):=(others => '0');
	begin
		if reset='0' and clk'event and clk='1' then
			case Opcode is
				when Add=> Q <= zeros(31 downto 1) & std_logic_vector(signed('0' & A) + signed('0' & B));
					if unsigned(Q) > 16#FFFF# then
						overflow<='1';
						assert not(overflow='1') report "An overflow occurred" severity warning;
					end if;
				when Sub=> Q <= zeros(31 downto 1) & std_logic_vector(signed('0' & A) - signed('0' & B));
					if signed(B) > signed(A) then
						overflow<='1';
						assert not(overflow='1') report "An overflow occurred" severity warning;
					end if;
				when MPY=> Q <= std_logic_vector(signed(A) * signed(B));
					if unsigned(Q) > 16#FFFF# then
						overflow<='1';
						assert not(overflow='1') report "An overflow occurred" severity warning;
					end if;
				when Comp=>
					if (A<B) then Status_code (0 to 1) <= "01"; 	--report "Less Than"
					elsif (A=B) then Status_code (0 to 1) <= "00"; 	--report "Equal"
					else Status_code(0 to 1) <= "10"; 				--report "Greater Than"
					end if;
					
					Q <= zeros(31 downto 0) & zeros(31 downto 2) & Status_code(0 to 1);
				when And32 => 	Q<= zeros(31 downto 0) & (A and B);
				when Or32 => 	Q<= zeros(31 downto 0) & (A or B);
				when Not32 => 	Q<= zeros(31 downto 0 ) & not A;
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
				when others => 	Q <= zeros(31 downto 0) & B; -- Execute a NOP
			end case;
		end if;
	Q_bus <= Q; -- Result deposited to Q_bus
	Proc_ready <= '1';
	end process;
end architecture behavior; 

--
library ieee;
use ieee.std_logic_1164.ALL;
use ieee.numeric_std.all;
--
use work.Procp.all;
use work.all;
--
entity MCProc is
	port (	PC, PW			: inout std_logic_vector (31 downto 0) := (others => '0');
			CLK, Reset		: in std_logic;
			Memory			: inout MEM1K := ((others=> (others=>'0'))));
end entity MCProc;
--
architecture First of MCProc is
	--
	component ALU_32
		port (	A_bus, B_bus	: in std_logic_vector(31 downto 0);
				Q_bus			: inout std_logic_vector(63 downto 0);
				Opcode			: in Popcode;
				Proc_ready		: out std_logic;
				clk, reset		: in std_logic);
	end component ALU_32;
	--
	signal A, B: std_logic_vector (31 downto 0):= (others => '0');
	signal Q: std_logic_vector (63 downto 0):=(others => '0');
	Signal R: REG32 := ((others=> (others=>'0')));
	-- signal Memory: MEM1K := ((others=> (others=>'0')));
	signal Instruction: Tinstruction;
	signal Opcode: Popcode;
	signal Proc_ready: std_logic := '0';
	signal STATE: PSTATE := Fetch;

	-- signal PC, PW: std_logic_vector (31 downto 0) := (others => '0');
	
	for ALU_32C: ALU_32 use entity work.ALU_32(Behavior);
	--
begin
	-- PC <= PC_bus;
	-- PW <= PW_bus;

	--Memory <= p_Memory;
	--Instruction <= to_record (PW);
	--Opcode <= Instruction.opcode;
	ALU_32C: ALU_32 port map (A, B, Q, Opcode, Proc_ready, CLK, Reset);
	
	PControl: Process
		variable zeros: std_logic_vector(31 downto 0):=(others => '0');
		
	begin
		if reset = '1' then
			-- p_Memory(0)	<= "10001100000000010000000001100011";
			-- p_Memory(1)	<= "10001100000000100000000001100100";
			-- p_Memory(2)	<= "00000000001000100001100000100000";
			
			-- p_Memory(100) <= "00000000000000000000000000000000";
			-- p_Memory(101) <= "00000000000000000000000000000001";
			-- p_Memory(102) <= "00000000000000000000000000000010";
			-- p_Memory(103) <= "00000000000000000000000000000011";
			-- p_Memory(104) <= "00000000000000000000000000000100";
		end if;
		
		wait until (Reset = '0' and CLK'event and CLK='1');
		
		case STATE is
			when Fetch =>
				--stuff
				-- code Fetch --Instruction
				PW <= Memory(to_integer(unsigned(PC)));
				Instruction <= to_record(PW);
				
				STATE <=Decode;
			when Decode =>
				--stuff
				-- code Decode --Instruction
				
				-- (LW, SW, Jr, Jump, Beq, Bne, Slt, Add, Sub, Mpy, Not32, Comp, And32, Or32, Xor32, MSLL, MSRL, MSLA, MSRA)

				-- Basic instruction format (R, I, J)
				if Instruction.opcode = MSLL or Instruction.opcode = MSRL or Instruction.opcode = MSLA or Instruction.opcode = MSRA then
					--ALU needs SHAMT
					A <= R(to_integer(unsigned(Instruction.Rt)));
					B <= R(to_integer(unsigned(Instruction.SHAMT)));
						
				elsif PW(31 downto 26) = "000000" then
					--interpret based upon MIPS GC column (1)
					--R type instruction
					
					A <= R(to_integer(unsigned(Instruction.Rs)));
					B <= R(to_integer(unsigned(Instruction.Rt)));
				elsif Instruction.opcode = jump then
					--interpret based upon MIPS GC column (0)
					--J type instruction
					
					A <= R(0)(31 downto 26) & PW(25 downto 0);
				else
					--interpret based upon MIPS GC column (0)
					--I type instruction
					
					A <= std_logic_vector(unsigned(R(to_integer(unsigned(Instruction.Rs)))) + unsigned(PW(15 downto 0)));
					B <= R(to_integer(unsigned(Instruction.Rt)));
					Q <= zeros(31 downto 0) & zeros(31 downto 16) & PW(15 downto 0);
				end if;

				Opcode <= Instruction.Opcode;

				Proc_ready <= '0';
				STATE<= Execute;
			when Execute =>	
				if (Proc_ready ='0') then STATE<= Execute;
				else STATE <= Retire;
					--do more stuff -- code Execute (ALU) Instruction
					
					--interpret output from BNE and BEQ and transfer to Q
					if (Instruction.opcode = bne or Instruction.opcode = beq) then
						if unsigned(Q) = 1 then
							Q <= zeros(31 downto 0) & PC(31 downto 0);
						else
							Q <= zeros(31 downto 0) & zeros(31 downto 16) & PW(15 downto 0);
						end if;
					end if;
					
				end if;
			when Retire =>
				-- code Retire (Store Register, store memory, or Load memory)

				-- Basic instruction format (R, I, J)				
				if Instruction.opcode = jump or Instruction.opcode = jr then
					--jump PC to Q and don't increment
					
					-- JUMP, JR
					PC <= Q(31 downto 0);
					
				elsif Instruction.opcode = lw then
					--store grabbed value into Rt
					
					-- LW
					R(to_integer(unsigned(Instruction.Rt))) <= Memory(to_integer(unsigned(Q(31 downto 0))));
					
					-- PC <= std_logic_vector(unsigned(PC) + 4);
					PC <= std_logic_vector(unsigned(PC) + 1);
					
				elsif Instruction.opcode = sw then
					--store value in memory(R[Rs])
					
					-- SW
					Memory(to_integer( unsigned(R(to_integer(unsigned(Instruction.Rs)))) + unsigned(PW(15 downto 0)) )) <= Q(31 downto 0);
					
					-- PC <= std_logic_vector(unsigned(PC) + 4);
					PC <= std_logic_vector(unsigned(PC) + 1);
				
				elsif PW(31 downto 26) = "000000" then
					--interpret based upon MIPS GC column (1)
					--R type instruction
					
					-- ADD, SUB, MPY, SLL, SRL, SLA, SRA, XOR, AND, OR, NOT, SLT, Comp
					
					--store Q (result) in Rd
					R(to_integer(unsigned(Instruction.Rd))) <= Q(31 downto 0);
					
					-- PC <= std_logic_vector(unsigned(PC) + 4);
					PC <= std_logic_vector(unsigned(PC) + 1);
					
				else
					--interpret based upon MIPS GC column (0)
					--I type instruction
					
					-- BEQ, BNE
					-- PC <= std_logic_vector(unsigned(Q(31 downto 0)) + 4);
					PC <= std_logic_vector(unsigned(Q(31 downto 0)) + 1);
					
				end if;
				
				STATE <= Fetch;
		end case;
	end process;
end architecture First;
 
-- Test Bench for MIPS-2 Processor
library ieee;
use ieee.std_logic_1164.ALL;
use ieee.numeric_std.all;
--
use work.ProcP.all;
use STD.TEXTIO.all;
use work.all;
--
entity MCPROC_TB is
end entity MCPROC_TB;
--
architecture TEST of MCPROC_TB is
	signal A, B, PC, PW: std_logic_Vector (31 downto 0) := (others => '0');
	signal Q: std_logic_vector (63 downto 0) := (others => '0');
	Signal reset, clk: std_logic;
	signal Memory: MEM1K := ((others => (others=>'0')));
	signal proc_ready, proc_reset: std_logic := '1';
	signal Instruction: Tinstruction;
	Signal GO: std_logic;
	
	signal get_data: std_logic := '0';
--
	file InFile  : text open read_mode  is "\\minerfiles.mst.edu\dfs\users\msrbqb\Desktop\midterm\MIPSProcessor\stimulus.txt";
	file Outfile : text open write_mode is "\\minerfiles.mst.edu\dfs\users\msrbqb\Desktop\midterm\MIPSProcessor\stim_out.txt";
--
	component MCPROC
		port (PC, PW			: inout std_logic_vector (31 downto 0):= (others => '0');
			  clk, reset		: std_logic;
			  Memory			: inout MEM1K);
	end component MCPROC;
--
	-- Potentially need this nonsense
	for MY_PROC: MCPROC use entity work.MCPROC(First);
begin
	Reset <= '1','0' after 100 ps;
	
	-- Memory <= ((others => (others=>'0')));
	-- PC_tab <= "00000000000000000000000000000000";
	-- PW <= "00000000000000000000000000000000";
	--
	CLK_P: process
	begin
		CLK <= '0';
		wait for 5 ps;
		CLK <= '1';
		wait for 5 ps;
	end process;

	MY_PROC: MCPROC port map (PC, PW, CLK, proc_reset, Memory);
	
	Inst_get_data: process(get_data)
		
	begin
		
	end process;

	Inst_Stimulate: process(clk, reset)
		variable temp_mem	: MEM1K;
		variable L1			: line;
		variable count		: integer := 0;
		variable S1, S2, S3	: string (1 to 32);
		variable S4			: string (1 to 64);
		variable temp		: string(1 to 32) := (others => ' ');
		
		variable load_mem	: std_logic := '1';
	begin
		-- wait until (Reset='0' and CLK'event and CLK= '1');
		if (Reset='0' and CLK'event and CLK= '1') then
		
			if (load_mem = '1') then
				while (not endfile(InFile) and count < 1000) loop
					-- Go <= '0';
					readline(InFile, L1);-- you might need to read into a	--line!
					read(L1, temp);

					-- Read in instructions from file into memory structure
					if count < 1000 then
						-- temp_mem(count) := to_std_logic_vector(temp)(31 downto 0);
						Memory(count) <= to_std_logic_vector(temp)(31 downto 0);
						count := count + 1;
					else
						exit;
					end if;
					
					
			-- 			Instruction <= Conv(temp);

			-- 			case Instruction.opcode is
			-- 				-- when NOP => A <= A;
			-- 				when others => Go <= '1'; -- will be calling the Microp
			-- 			end case;
				end loop;
				
				load_mem := '0';
				
				-- Memory <= temp_mem;
			end if;
			
			-- S1 := to_string(PC);
			-- S2 := to_string(PW);
			-- S3 := to_string(B);
			-- S4 := to_string(Q);
			-- write(OutFile, S1 & " " & S2 & " " & S3 & " " & S4 & LF);

			proc_reset <= '0';
			
		end if;
	end process;
end architecture TEST;