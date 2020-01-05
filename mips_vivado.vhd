-----------------------------------------------------------------------------------
--  PCS3612 - OAC1
--
--  Recuperacao 
--  MIPS Pipeline
--
--  Grupo:
--  Guilherme Rodrigues Ludescher - 9833180
--  Matheus da Silva Sato - 9833200
-----------------------------------------------------------------------------------

library IEEE; 
use IEEE.STD_LOGIC_1164.all; use IEEE.NUMERIC_STD_UNSIGNED.all;
use std.env.all;

entity testbench is
end;

architecture test of testbench is
  component top
    port(clk, reset:           in  STD_LOGIC;
         writedata, dataadr:   out STD_LOGIC_VECTOR(31 downto 0);
         memwrite:             out STD_LOGIC);
  end component;
  signal writedata, dataadr:    STD_LOGIC_VECTOR(31 downto 0);
  signal clk, reset,  memwrite: STD_LOGIC;
begin

  -- instantiate device to be tested
  dut: top port map(clk, reset, writedata, dataadr, memwrite);

  -- Generate clock with 10 ns period
  process begin
    clk <= '1';
    wait for 5 ns; 
    clk <= '0';
    wait for 5 ns;
  end process;

  -- Generate reset for first two clock cycles
  process begin
    reset <= '1';
    wait for 22 ns;
    reset <= '0';
    wait;
  end process;

  -- check that 7 gets written to address 84 at end of program
  process (clk) begin
    if (clk'event and clk = '1' and memwrite = '1') then
      -- if (to_integer(dataadr) = 84 and writedata="00000111") then -- teste com memfile.dat
      if (to_integer(dataadr) = 84 and writedata="11111111111111110111111100000010") then -- teste com memfile2.dat
        report "NO ERRORS: Simulation succeeded" severity note;
        finish(0);
      elsif (dataadr /= 80) then 
        report "Simulation failed" severity failure;
        finish(0);
      end if;
    end if;
  end process;
end;

library IEEE; 
use IEEE.STD_LOGIC_1164.all; use IEEE.NUMERIC_STD_UNSIGNED.all;

entity top is -- top-level design for testing
  port(clk, reset:           in     STD_LOGIC;
       writedata, dataadr:   buffer STD_LOGIC_VECTOR(31 downto 0);
       memwrite:             buffer STD_LOGIC);
end;

architecture test of top is
  component mips 
    port(clk, reset:        in  STD_LOGIC;
         pc:                out STD_LOGIC_VECTOR(31 downto 0);
         instr:             in  STD_LOGIC_VECTOR(31 downto 0);
         memwrite:          out STD_LOGIC;
         aluout, writedata: out STD_LOGIC_VECTOR(31 downto 0);
         readdata:          in  STD_LOGIC_VECTOR(31 downto 0));
  end component;
  component imem
    port(a:  in  STD_LOGIC_VECTOR(5 downto 0);
         rd: out STD_LOGIC_VECTOR(31 downto 0));
  end component;
  component dmem
    port(clk, we:  in STD_LOGIC;
         a, wd:    in STD_LOGIC_VECTOR(31 downto 0);
         rd:       out STD_LOGIC_VECTOR(31 downto 0));
  end component;
  signal pc, instr, 
         readdata,
         s_writedata_mem: STD_LOGIC_VECTOR(31 downto 0);
  signal s_memwrite_mem: STD_LOGIC;
begin
  -- instantiate processor and memories
  mips1: mips port map(clk, reset, pc, instr, s_memwrite_mem, dataadr,
                       writedata, readdata);
  imem1: imem port map(pc(7 downto 2), instr);
  dmem1: dmem port map(clk, s_memwrite_mem, dataadr, writedata, readdata);
  memwrite <= s_memwrite_mem;
end;

library IEEE;
use IEEE.STD_LOGIC_1164.all; use STD.TEXTIO.all;
use IEEE.NUMERIC_STD_UNSIGNED.all;

entity dmem is -- data memory
  port(clk, we:  in STD_LOGIC;
       a, wd:    in STD_LOGIC_VECTOR(31 downto 0);
       rd:       out STD_LOGIC_VECTOR(31 downto 0));
end;

architecture behave of dmem is
begin
  -- -- Para sintetizar o projeto no VIVADO, comente a partir daqui ate...
  -- process is
  --   type ramtype is array (63 downto 0) of STD_LOGIC_VECTOR(31 downto 0);
  --   variable mem: ramtype;
  -- begin
  --   -- read or write memory
  --   loop
  --     if clk'event and clk = '1' then
  --         if (we = '1') then mem(to_integer(a(7 downto 2))) := wd;
  --         end if;
  --     end if;
  --     rd <= mem(to_integer(a(7 downto 2)));
  --     wait on clk, a;
  --   end loop;
  --   end process;
  --   -- ...aqui (trecho 1/2).
end;

library IEEE;
use IEEE.STD_LOGIC_1164.all; use STD.TEXTIO.all;
use IEEE.NUMERIC_STD_UNSIGNED.all;

entity imem is -- instruction memory
  port(a:  in  STD_LOGIC_VECTOR(5 downto 0);
       rd: out STD_LOGIC_VECTOR(31 downto 0));
end;

architecture behave of imem is
begin
  -- -- Para sintetizar o projeto no VIVADO, comente a partir daqui ate...
  -- process is
  --   file mem_file: TEXT;
  --   variable L: line;
  --   variable ch: character;
  --   variable i, index, result: integer;
  --   type ramtype is array (63 downto 0) of STD_LOGIC_VECTOR(31 downto 0);
  --   variable mem: ramtype;
  -- begin
  --   -- initialize memory from file
  --   for i in 0 to 63 loop -- set all contents low
  --     mem(i) := (others => '0');
  --   end loop;
  --   index := 0;
  --   --FILE_OPEN(mem_file, "memfile.dat", READ_MODE);
  --   FILE_OPEN(mem_file, "memfile2.dat", READ_MODE);
  --   while not endfile(mem_file) loop
  --     readline(mem_file, L);
  --     result := 0;
  --     for i in 1 to 8 loop
  --       read(L, ch);
  --       if '0' <= ch and ch <= '9' then
  --           result := character'pos(ch) - character'pos('0');
  --       elsif 'a' <= ch and ch <= 'f' then
  --          result := character'pos(ch) - character'pos('a')+10;
  --       else report "Format error on line " & integer'image(index)
  --            severity error;
  --       end if;
  --       mem(index)(35-i*4 downto 32-i*4) :=to_std_logic_vector(result,4);
  --     end loop;
  --     index := index + 1;
  --   end loop;

  --   -- read memory
  --   loop
  --     rd <= mem(to_integer(a));
  --     wait on a;
  --   end loop;
  -- end process;
  -- -- ...aqui (trecho 2/2).
end;

library IEEE; use IEEE.STD_LOGIC_1164.all;

entity mips is -- single cycle MIPS processor
  port(clk, reset:        in  STD_LOGIC;
       pc:                out STD_LOGIC_VECTOR(31 downto 0);
       instr:             in  STD_LOGIC_VECTOR(31 downto 0);
       memwrite:          out STD_LOGIC;
       aluout, writedata: out STD_LOGIC_VECTOR(31 downto 0);
       readdata:          in  STD_LOGIC_VECTOR(31 downto 0));
end;

architecture struct of mips is
  component controller
    port(op, funct:          in  STD_LOGIC_VECTOR(5 downto 0);
         zero:               in  STD_LOGIC;
         memtoreg, memwrite: out STD_LOGIC;
         pcsrc, alusrc:      out STD_LOGIC;
         regdst, regwrite:   out STD_LOGIC;
         jump, signedop:     out STD_LOGIC;  
         branch, branchnot:  out STD_LOGIC;
         alucontrol:         out STD_LOGIC_VECTOR(2 downto 0));
  end component;
  component datapath
    port(clk, reset:        in  STD_LOGIC;
         memtoreg, pcsrc:   in  STD_LOGIC;
         alusrc, regdst:    in  STD_LOGIC;
         regwrite, jump:    in  STD_LOGIC;
         signedop:          in  STD_LOGIC;  
         branch, branchnot: in  STD_LOGIC;
         alucontrol:        in  STD_LOGIC_VECTOR(2 downto 0);
         zero:              out STD_LOGIC;
         pc:                buffer STD_LOGIC_VECTOR(31 downto 0);
         instr:             in STD_LOGIC_VECTOR(31 downto 0);
         aluout, writedata_mem: out STD_LOGIC_VECTOR(31 downto 0);
         readdata:          in  STD_LOGIC_VECTOR(31 downto 0);
         instr_id:          out STD_LOGIC_VECTOR(31 downto 0);
         memwrite_in:          in  STD_LOGIC;
         memwrite_mem:      out STD_LOGIC);
  end component;
  signal memtoreg, alusrc, regdst, regwrite, jump, pcsrc: STD_LOGIC;
  signal zero, signedop: STD_LOGIC; -- Signal added
  signal branch, branchnot: STD_LOGIC;
  signal alucontrol: STD_LOGIC_VECTOR(2 downto 0);
  signal s_instr_id: STD_LOGIC_VECTOR(31 downto 0);
  signal s_memwrite, s_memwrite_mem: STD_LOGIC;
begin
  cont: controller port map(s_instr_id(31 downto 26), s_instr_id(5 downto 0),
                            zero, memtoreg, s_memwrite, pcsrc, alusrc,
                            regdst, regwrite, jump, signedop, branch, branchnot, alucontrol); -- Signal added
  dp: datapath port map(clk, reset, memtoreg, pcsrc, alusrc, regdst,
                        regwrite, jump, signedop, branch, branchnot, alucontrol, zero, pc, instr,
                        aluout, writedata, readdata, s_instr_id, s_memwrite, memwrite); -- Signal added
end;

library IEEE; use IEEE.STD_LOGIC_1164.all;

entity controller is -- single cycle control decoder
  port(op, funct:          in  STD_LOGIC_VECTOR(5 downto 0);
       zero:               in  STD_LOGIC;
       memtoreg, memwrite: out STD_LOGIC;
       pcsrc, alusrc:      out STD_LOGIC;
       regdst, regwrite:   out STD_LOGIC;
       jump, signedop:     out STD_LOGIC;  -- Signal added
       branch, branchnot:  out STD_LOGIC;
       alucontrol:         out STD_LOGIC_VECTOR(2 downto 0));
end;


architecture struct of controller is
  component maindec
    port(op:                 in  STD_LOGIC_VECTOR(5 downto 0);
         memtoreg, memwrite: out STD_LOGIC;
         branch, alusrc:     out STD_LOGIC;
         regdst, regwrite:   out STD_LOGIC;
         jump:               out STD_LOGIC;
         branchnot:          out STD_LOGIC;
         signedop:           out STD_LOGIC;  -- Signal added
         aluop:              out STD_LOGIC_VECTOR(1 downto 0));
  end component;
  component aludec
    port(funct:      in  STD_LOGIC_VECTOR(5 downto 0);
         aluop:      in  STD_LOGIC_VECTOR(1 downto 0);
         alucontrol: out STD_LOGIC_VECTOR(2 downto 0));
  end component;
  signal aluop:  STD_LOGIC_VECTOR(1 downto 0);
  signal s_branch, s_branchnot: STD_LOGIC;
begin
  md: maindec port map(op, memtoreg, memwrite, s_branch,
                       alusrc, regdst, regwrite, jump, 
                       s_branchnot, signedop, aluop);
  ad: aludec port map(funct, aluop, alucontrol);

  pcsrc <= (branch and zero) or (branchnot and not zero); -- branch op
  branch <= s_branch;
  branchnot <= s_branchnot;
end;

library IEEE; use IEEE.STD_LOGIC_1164.all;

entity maindec is -- main control decoder
  port(op:                 in  STD_LOGIC_VECTOR(5 downto 0);
       memtoreg, memwrite: out STD_LOGIC;
       branch, alusrc:     out STD_LOGIC;
       regdst, regwrite:   out STD_LOGIC;
       jump:               out STD_LOGIC;
       branchnot  :        out STD_LOGIC;
       signedop:       out STD_LOGIC;  -- Signal added
       aluop:              out STD_LOGIC_VECTOR(1 downto 0));
end;

architecture behave of maindec is
  signal controls: STD_LOGIC_VECTOR(10 downto 0); -- Added extra bit (signedop)
begin
  process(all) begin
    case op is
      when "000000" => controls <= "10110000010"; -- RTYPE
      when "100011" => controls <= "10101001000"; -- LW
      when "101011" => controls <= "10001010000"; -- SW
      when "000100" => controls <= "10000100001"; -- BEQ
      when "001000" => controls <= "10101000000"; -- ADDI
      when "000010" => controls <= "10000000100"; -- J
      when "001101" => controls <= "00101000011"; -- ORI  -- aluop altered -- CORRECAO DE REGDST
      when "000101" => controls <= "11000000001"; -- BNE aqui
      when others   => controls <= "-----------"; -- illegal op
    end case;
  end process;

  (signedop, branchnot, regwrite, regdst, 
  alusrc, branch, memwrite, memtoreg, jump) <= controls(10 downto 2);
  
  aluop <= controls(1 downto 0);

end;

library IEEE; use IEEE.STD_LOGIC_1164.all;

entity aludec is -- ALU control decoder
  port(funct:      in  STD_LOGIC_VECTOR(5 downto 0);
       aluop:      in  STD_LOGIC_VECTOR(1 downto 0);
       alucontrol: out STD_LOGIC_VECTOR(2 downto 0));
end;

architecture behave of aludec is
begin
  process(all) begin
    case aluop is
      when "00" => alucontrol <= "010"; -- add (for lw/sw/addi)
      when "01" => alucontrol <= "110"; -- sub (for beq/bne)
      when "11" => alucontrol <= "001"; -- or  (for ori)
      when others => case funct is      -- R-type instructions
                         when "100000" => alucontrol <= "010"; -- add 
                         when "100010" => alucontrol <= "110"; -- sub
                         when "100100" => alucontrol <= "000"; -- and
                         when "100101" => alucontrol <= "001"; -- or
                         when "101010" => alucontrol <= "111"; -- slt
                         when others   => alucontrol <= "---"; -- ???
                     end case;
    end case;
  end process;
end;

library IEEE; use IEEE.STD_LOGIC_1164.all; use IEEE.STD_LOGIC_ARITH.all;

entity datapath is  -- MIPS datapath
  port(clk, reset:        in  STD_LOGIC;
       memtoreg, pcsrc:   in  STD_LOGIC;
       alusrc, regdst:    in  STD_LOGIC;
       regwrite, jump:    in  STD_LOGIC;
       signedop:          in  STD_LOGIC;  
       branch, branchnot: in  STD_LOGIC;       
       alucontrol:        in  STD_LOGIC_VECTOR(2 downto 0);
       zero:              out STD_LOGIC;
       pc:                buffer STD_LOGIC_VECTOR(31 downto 0);
       instr:             in  STD_LOGIC_VECTOR(31 downto 0);
       aluout, writedata_mem: buffer STD_LOGIC_VECTOR(31 downto 0);
       readdata:          in  STD_LOGIC_VECTOR(31 downto 0);
       instr_id:          out STD_LOGIC_VECTOR(31 downto 0);
       memwrite_in:       in  STD_LOGIC;
       memwrite_mem:      out STD_LOGIC);
end;

architecture struct of datapath is

  signal regwriteD, memtoregD, memwriteD: STD_LOGIC;
  signal pcsrcD, alusrcD, jumpD, regdstD: STD_LOGIC;

  signal regwriteE, memtoregE, memwriteE: STD_LOGIC;
  signal pcsrcE, alusrcE, regdstE:        STD_LOGIC;
  signal alucontrolD, alucontrolE:        STD_LOGIC_VECTOR(2 downto 0);

  signal writereg:                        STD_LOGIC_VECTOR(4 downto 0);
  signal pcjump, pcnext:                  STD_LOGIC_VECTOR(31 downto 0);
  signal pcnext_not_stall, pcnextbr:      STD_LOGIC_VECTOR(31 downto 0);
  signal pcplus4, pcbranch:               STD_LOGIC_VECTOR(31 downto 0);
  signal signimm, signimmsh:              STD_LOGIC_VECTOR(31 downto 0);
  signal srca, srcb, result:              STD_LOGIC_VECTOR(31 downto 0);
  signal s_stall:                         STD_LOGIC;

  -- Datapath
  signal pcplus4_id, s_instr_id:          STD_LOGIC_VECTOR(31 downto 0);

  signal pcplus4_ex, signimm_ex:          STD_LOGIC_VECTOR(31 downto 0); 
  signal pc_jump_ex, srca_ex:             STD_LOGIC_VECTOR(31 downto 0); 
  signal writedata_ex, signimmsh_ex:      STD_LOGIC_VECTOR(31 downto 0); 
  signal pc_branch_ex, aluout_ex:         STD_LOGIC_VECTOR(31 downto 0);
  signal writereg0_ex, writereg1_ex:      STD_LOGIC_VECTOR(4 downto 0); 
  signal writereg_ex:                     STD_LOGIC_VECTOR(4 downto 0);

  signal aluout_mem, writedata:           STD_LOGIC_VECTOR(31 downto 0);
  signal s_writedata_mem, pc_branch:      STD_LOGIC_VECTOR(31 downto 0); 
  signal pc_branch_mem, pc_jump_mem:      STD_LOGIC_vECTOR(31 downto 0);
  signal writereg_mem:                    STD_LOGIC_VECTOR(4 downto 0);
  signal zero_mem:                        STD_LOGIC;

  signal readdata_wb, aluout_wb:          STD_LOGIC_VECTOR(31 downto 0);
  signal writereg_wb:                     STD_LOGIC_VECTOR(4 downto 0);

  -- Controller
  signal wb_id, wb_ex, wb_mem, wb_wb: STD_LOGIC_VECTOR(1 downto 0);
  signal m_if, m_ex, m_mem : STD_LOGIC_VECTOR(2 downto 0);
  signal ex_if, ex_ex: STD_LOGIC_VECTOR(4 downto 0);
  signal id_flush: STD_LOGIC;
  signal selfwdmux1, selfwdmux2: STD_LOGIC_VECTOR(1 downto 0);
  signal ulain0, ulain1: STD_LOGIC_VECTOR(31 downto 0);
  signal read_register1E: STD_LOGIC_VECTOR(4 downto 0); 

  component alu
    port(a, b:       in  STD_LOGIC_VECTOR(31 downto 0);
         alucontrol: in  STD_LOGIC_VECTOR(2 downto 0);
         result:     buffer STD_LOGIC_VECTOR(31 downto 0);
         zero:       out STD_LOGIC);
  end component;
  component regfile
    port(clk:           in  STD_LOGIC;
         we3:           in  STD_LOGIC;
         ra1, ra2, wa3: in  STD_LOGIC_VECTOR(4 downto 0);
         wd3:           in  STD_LOGIC_VECTOR(31 downto 0);
         rd1, rd2:      out STD_LOGIC_VECTOR(31 downto 0));
  end component;
  component adder
    port(a, b: in  STD_LOGIC_VECTOR(31 downto 0);
         y:    out STD_LOGIC_VECTOR(31 downto 0));
  end component;
  component sl2
    port(a: in  STD_LOGIC_VECTOR(31 downto 0);
         y: out STD_LOGIC_VECTOR(31 downto 0));
  end component;
  component signext
    port(a: in  STD_LOGIC_VECTOR(15 downto 0);
         signedop: in STD_LOGIC; -- Signal added
         y: out STD_LOGIC_VECTOR(31 downto 0));
  end component;
  component flopr generic(width: integer);
    port(clk, reset: in  STD_LOGIC;
         d:          in  STD_LOGIC_VECTOR(width-1 downto 0);
         q:          out STD_LOGIC_VECTOR(width-1 downto 0));
  end component;
  component mux2 generic(width: integer);
    port(d0, d1: in  STD_LOGIC_VECTOR(width-1 downto 0);
         s:      in  STD_LOGIC;
         y:      out STD_LOGIC_VECTOR(width-1 downto 0));
  end component;
  component mux3 is -- three-input multiplexer
    generic(width: integer);
    port(d0, d1, d2: in  STD_LOGIC_VECTOR(width-1 downto 0);
         s:      in  STD_LOGIC_VECTOR(1 downto 0);
         y:      out STD_LOGIC_VECTOR(width-1 downto 0));
  end component;

  -- Registradores de Pipeline
  component if_id is
      port (clk, reset: in  STD_LOGIC; 
      -- entradas
            flush:      in  STD_LOGIC;
            pc_in:      in  STD_LOGIC_VECTOR(31 downto 0);
            instr_in:   in  STD_LOGIC_VECTOR(31 downto 0); 
            stall:      in  STD_LOGIC; 
      -- saidas
            pc_out:     out STD_LOGIC_VECTOR(31 downto 0); 
            instr_out:  out STD_LOGIC_VECTOR(31 downto 0)
           );
  end component;

  component id_ex is
      port (clk, reset:         in  STD_LOGIC;
      -- entradas
            stall:              in STD_LOGIC ;
            pc_in:              in STD_LOGIC_VECTOR(31 downto 0); 
            sign_extended_in:   in STD_LOGIC_VECTOR(31 downto 0); 
            write_register0_in: in STD_LOGIC_VECTOR(4 downto 0); 
            write_register1_in: in STD_LOGIC_VECTOR(4 downto 0); 
            rd1_in, rd2_in:     in STD_LOGIC_VECTOR(31 downto 0);
            regwrite_in:        in STD_LOGIC;
            memtoreg_in:        in STD_LOGIC;
            memwrite_in:        in STD_LOGIC;
            regdst_in:          in STD_LOGIC;
            alusrc_in:          in STD_LOGIC;
            alucontrol_in:      in STD_LOGIC_VECTOR(2 downto 0);
            read_reg1_in:       in STD_LOGIC_VECTOR(4 downto 0);
      -- saidas
            pc_out:              out STD_LOGIC_VECTOR(31 downto 0); 
            sign_extended_out:   out STD_LOGIC_VECTOR(31 downto 0);
            write_register0_out: out STD_LOGIC_VECTOR(4 downto 0);
            write_register1_out: out STD_LOGIC_VECTOR(4 downto 0);
            rd1_out, rd2_out:    out STD_LOGIC_VECTOR(31 downto 0);
            regwrite_out:        out STD_LOGIC;
            memtoreg_out:        out STD_LOGIC;
            memwrite_out:        out STD_LOGIC;
            regdst_out:          out STD_LOGIC;
            alusrc_out:          out STD_LOGIC;
            read_reg1_out:       out STD_LOGIC_VECTOR(4 downto 0);
            alucontrol_out:      out STD_LOGIC_VECTOR(2 downto 0));
  end component;

  component ex_mem is
      port (clk, reset:        in  STD_LOGIC;
      -- entradas
            wb_in:             in  STD_LOGIC_VECTOR(1 downto 0); 
            m_in:              in  STD_LOGIC_VECTOR(2 downto 0); 
            aluresult_in:      in  STD_LOGIC_VECTOR(31 downto 0);
            rd2_in:            in  STD_LOGIC_VECTOR(31 downto 0);
            pcbranch_in:       in  STD_LOGIC_VECTOR(31 downto 0);
            write_register_in: in  STD_LOGIC_VECTOR(4 downto 0);
            pcjump_in:         in  STD_LOGIC_VECTOR(31 downto 0);
      -- saidas
            wb_out:             out STD_LOGIC_VECTOR(1 downto 0);
            m_out:              out STD_LOGIC_VECTOR(2 downto 0);
            aluresult_out:      out STD_LOGIC_VECTOR(31 downto 0);
            rd2_out:            out STD_LOGIC_VECTOR(31 downto 0);
            pcbranch_out:       out STD_LOGIC_VECTOR(31 downto 0);
            write_register_out: out STD_LOGIC_VECTOR(4 downto 0);
            pcjump_out:         out STD_LOGIC_VECTOR(31 downto 0)
           );
  end component;

  component mem_wb is -- registrador MEM/WB
      port (clk, reset:         in STD_LOGIC;
      -- entradas
            wb_in:              in  STD_LOGIC_VECTOR(1 downto 0); 
            read_mem_data_in:   in  STD_LOGIC_VECTOR(31 downto 0); 
            aluresult_in:       in  STD_LOGIC_VECTOR(31 downto 0); 
            write_register_in:  in  STD_LOGIC_VECTOR(4 downto 0); 
      -- saidas
            wb_out:             out STD_LOGIC_VECTOR(1 downto 0);
            read_mem_data_out:  out STD_LOGIC_VECTOR(31 downto 0);
            aluresult_out:      out STD_LOGIC_VECTOR(31 downto 0);
            write_register_out: out STD_LOGIC_VECTOR(4 downto 0)
          );
  end component;

  component hazard_unit is
          port(clk, reset:      in STD_LOGIC;
          -- entradas
               ID_EX_rd1:       in STD_LOGIC_VECTOR(4 downto 0);
               ID_EX_rd2:       in STD_LOGIC_VECTOR(4 downto 0);
               ID_EX_writeReg:  in STD_LOGIC_VECTOR(4 downto 0);
               ID_EX_branch:    in STD_LOGIC;
               ID_EX_branchnot: in STD_LOGIC;
               ID_EX_jump:      in STD_LOGIC;
               ID_EX_regWrite:  in STD_LOGIC;
               ID_EX_memToReg:  in STD_LOGIC;
               ID_EX_memwrite:  in STD_LOGIC;
               EX_MEM_writeReg: in STD_LOGIC_VECTOR(4 downto 0);
               EX_MEM_memToReg: in STD_LOGIC;
               EX_MEM_regWrite: in STD_LOGIC;
               MEM_WB_writeReg: in STD_LOGIC_VECTOR(4 downto 0);
               MEM_WB_regWrite: in STD_LOGIC;
               regdst:          in STD_LOGIC;
               alusrc:          in STD_LOGIC;
          -- saidas
               stall:           out STD_LOGIC);
  end component;

  component forwarding_control is
      port (
      -- entradas
          regwriteE:       in std_logic; 
          regwriteM:       in std_logic; 
          regwriteW:       in std_logic;
          memtoregM:       in std_logic;
          read_register1E: in std_logic_vector(4 downto 0);
          read_register2E: in std_logic_vector(4 downto 0);
          write_registerM: in std_logic_vector(4 downto 0); 
          write_registerW: in std_logic_vector(4 downto 0);
      -- saidas
          forward_a:       out std_logic_vector(1 downto 0);
          forward_b:       out std_logic_vector(1 downto 0)
      );
  end component;

  component jump_unit is
    port (
      register1, register2: in STD_LOGIC_VECTOR(31 downto 0);
      equal : out STD_LOGIC);
  end component jump_unit;
  
begin

  wb_ex <= regwriteE & memtoregE;
  m_ex <= "00" & memwriteE;
  writedata_mem <= s_writedata_mem;

  -- ---- hazard unit ----
  hazard_detect: hazard_unit 
    port map(clk => clk,
            reset => reset, 
            ID_EX_rd1 => s_instr_id(25 downto 21), 
            ID_EX_rd2 => s_instr_id(20 downto 16), 
            ID_EX_writeReg => writereg_ex, 
            ID_EX_branch => branch,
            ID_EX_branchnot => branchnot,
            ID_EX_jump => jump,
            ID_EX_regWrite => wb_ex(1),
            ID_EX_memToReg => wb_id(0),
            ID_EX_memwrite => memwrite_in,
            EX_MEM_writeReg => writereg_mem, 
            EX_MEM_memToReg => wb_mem(0),
            EX_MEM_regWrite => wb_mem(1),
            MEM_WB_writeReg => writereg_wb,
            MEM_WB_regWrite => wb_wb(1),
            regdst => regdst, 
            alusrc => alusrc, 
            stall => s_stall
    );
  
    ---- unidade de forwarding ----
    forwarding: forwarding_control port map
    (
      regwriteE => regwriteE,                                                 
      regwriteM => wb_mem(1),                                                 
      regwriteW => wb_wb(1),
      memtoregM => wb_mem(0),
      read_register1E => read_register1E,
      read_register2E => writereg0_ex,
      write_registerM => writereg_mem,
      write_registerW => writereg_wb,
      forward_a => selfwdmux1,
      forward_b => selfwdmux2
    );                  

  -- ---- instruction_fetch ----
  reg_PC: flopr generic map(32) port map(clk, reset, pcnext, pc);

  mux_branch: mux2 generic map(32) port map(pcplus4, pc_branch, pcsrc, pcnextbr);

  mux_jump: mux2 generic map(32) port map(pcnextbr, x"00000044", jump, pcnext_not_stall);

  mux_stall: mux2 generic map(32) port map(pcnext_not_stall, pc, s_stall, pcnext);

  add4_PC: adder port map(pc, X"00000004", pcplus4);

  id_flush <= (pcsrc or jump) and not s_stall;

  ifid: if_id port map(clk, reset, id_flush, pcplus4, instr, s_stall, pcplus4_id, s_instr_id);

  -- ---- instruction_decode ----
  pcjump <= pcplus4_id(31 downto 28) & s_instr_id(25 downto 0) & "00";
  
  sign_ext: signext port map(s_instr_id(15 downto 0), signedop, signimm);
  
  shift_imm: sl2 port map(signimm, signimmsh);
  
  add_PCbranch: adder port map(pcplus4_id, signimmsh, pc_branch);
    
  reg_file: regfile port map(clk, wb_wb(1), s_instr_id(25 downto 21), 
                       s_instr_id(20 downto 16), writereg_wb, result, srca, writedata);

  ju: jump_unit port map(srca, writedata, zero);

  instr_id <= s_instr_id;

  -- ---- logica do stall ----
  regwriteD <= regwrite   when s_stall = '0' else '0';
  memtoregD <= memtoreg   when s_stall = '0' else '0';
  pcsrcD  <= pcsrc    when s_stall = '0' else '0';
  memwriteD <= memwrite_in  when s_stall = '0' else '0';
  regdstD   <= regdst     when s_stall = '0' else '0';
  alusrcD <= alusrc     when s_stall = '0' else '0';
  alucontrolD <= alucontrol   when s_stall = '0' else "000";
    
  idex: id_ex 
    port map(
        clk => clk,
        reset => reset,
        stall => s_stall,
        pc_in => pcplus4_id,
        sign_extended_in => signimm,
        read_reg1_in => s_instr_id(25 downto 21),
        write_register0_in => s_instr_id(20 downto 16),
        write_register1_in => s_instr_id(15 downto 11),
        rd1_in => srca,
        rd2_in => writedata,
        regwrite_in => regwrite,
        memtoreg_in => memtoreg,
        memwrite_in => memwrite_in,
        regdst_in => regdst,
        alusrc_in => alusrc,
        alucontrol_in => alucontrol,
        pc_out => pcplus4_ex,
        sign_extended_out => signimm_ex,
        read_reg1_out => read_register1E, 
        write_register0_out => writereg0_ex,
        write_register1_out => writereg1_ex,
        rd1_out => srca_ex,
        rd2_out => writedata_ex,
        regwrite_out => regwriteE,    
        memtoreg_out => memtoregE,   
        memwrite_out => memwriteE,   
        regdst_out => regdstE,   
        alusrc_out => alusrcE,     
        alucontrol_out => alucontrolE
      ); 
      
  -- ---- execute ----
  wrmux: mux2 generic map(5) port map(writereg0_ex, writereg1_ex, regdstE, writereg_ex);                          
  
  mux_forw1: mux3 generic map(32) port map (srca_ex, aluout_mem, result, selfwdmux1, ulain0);
  
  mux_forw2: mux3 generic map(32) port map (writedata_ex, aluout_mem, result, selfwdmux2, srcb);

  mux_ALU: mux2 generic map(32) port map (srcb, signimm_ex, alusrcE, ulain1);

  ULA_main: alu port map
    (
      a => ulain0, 
      b => ulain1, 
      alucontrol => alucontrolE, 
      result => aluout_ex, 
      zero => open
    );

  exmem: ex_mem port map
    (
      clk => clk,
      reset => reset,
      wb_in => wb_ex,
      wb_out => wb_mem,
      m_in => m_ex,
      m_out => m_mem,
      aluresult_in => aluout_ex,
      aluresult_out => aluout_mem,
      rd2_in => srcb,
      rd2_out => s_writedata_mem,
      pcbranch_in => x"00000000",
      pcbranch_out => pc_branch_mem,
      write_register_in => writereg_ex,
      write_register_out => writereg_mem,
      pcjump_in => pc_jump_ex,
      pcjump_out => pc_jump_mem
    );
 
  -- ---- memory_access ----
  aluout <= aluout_mem;
  memwrite_mem <= m_mem(0);
  
  memwb: mem_wb port map (clk => clk, 
                            reset => reset,
                            wb_in => wb_mem,
                            read_mem_data_in => readdata,
                            aluresult_in => aluout_mem,
                            write_register_in => writereg_mem,
                            wb_out => wb_wb,
                            read_mem_data_out => readdata_wb,
                            aluresult_out => aluout_wb,
                            write_register_out => writereg_wb
                          );

  -- ---- write_back ----
  mux_result: mux2 generic map(32) port map(aluout_wb, readdata_wb, wb_wb(0), result);

end;

library IEEE; use IEEE.STD_LOGIC_1164.all; use IEEE.STD_LOGIC_ARITH.all;

entity if_id is
    port (clk, reset, flush : in  STD_LOGIC;
          pc_in, instr_in   : in  STD_LOGIC_VECTOR(31 downto 0); 
          stall             : in  STD_LOGIC;
          pc_out, instr_out : out STD_LOGIC_VECTOR(31 downto 0)
         );
end;
architecture arch_if_id of if_id is
  begin
    process(clk, reset) begin
      if reset then
          pc_out <= (others => '0'); 
          instr_out <= x"0000F800"; -- 000000 00000 00000 11111 00000000000
      elsif rising_edge(clk) then
        if flush = '1' then
          pc_out <= (others => '0'); 
          instr_out <= x"0000F800"; -- 000000 00000 00000 11111 00000000000
        elsif stall ='0' then
          pc_out <= pc_in;
          instr_out <= instr_in;
        end if;
      end if;
    end process;
  end;

library IEEE; use IEEE.STD_LOGIC_1164.all; use IEEE.STD_LOGIC_ARITH.all;

entity id_ex is
    port (clk, reset    : in  STD_LOGIC;
          stall       : in STD_LOGIC ;
          pc_in         : in STD_LOGIC_VECTOR(31 downto 0);
          sign_extended_in    : in STD_LOGIC_VECTOR(31 downto 0);
          write_register0_in  : in STD_LOGIC_VECTOR(4 downto 0);  
          write_register1_in  : in STD_LOGIC_VECTOR(4 downto 0);  
          rd1_in, rd2_in    : in STD_LOGIC_VECTOR(31 downto 0); 
          regwrite_in     : in STD_LOGIC;
          memtoreg_in     : in STD_LOGIC;
          memwrite_in     : in STD_LOGIC;
          regdst_in     : in STD_LOGIC;
          alusrc_in     : in STD_LOGIC;
          alucontrol_in   : in STD_LOGIC_VECTOR(2 downto 0);
          read_reg1_in        : in STD_LOGIC_VECTOR(4 downto 0);
          -- outputs
          pc_out        : out STD_LOGIC_VECTOR(31 downto 0); 
          sign_extended_out   : out STD_LOGIC_VECTOR(31 downto 0);
          write_register0_out : out STD_LOGIC_VECTOR(4 downto 0);
          write_register1_out : out STD_LOGIC_VECTOR(4 downto 0);
          rd1_out, rd2_out  : out STD_LOGIC_VECTOR(31 downto 0);
          regwrite_out    : out STD_LOGIC;
          memtoreg_out    : out STD_LOGIC;
          memwrite_out    : out STD_LOGIC;
          regdst_out      : out STD_LOGIC;
          alusrc_out      : out STD_LOGIC;
          alucontrol_out    : out STD_LOGIC_VECTOR(2 downto 0);
          read_reg1_out       : out STD_LOGIC_VECTOR(4 downto 0)
        
    );
end;
architecture arch_id_ex of id_ex is
begin
  process(clk, reset) begin
    if reset then
        pc_out <= (others => '0');
        sign_extended_out <= (others => '0');
        write_register0_out <= (others => '1');
        write_register1_out <= (others => '1');
        rd1_out   <= (others => '0');
        rd2_out   <= (others => '0');
        regwrite_out <= '0';
        memtoreg_out <= '0';
        memwrite_out <= '0';
        regdst_out <= '0';
        alusrc_out <= '0';
        alucontrol_out <= (others => '0');
        read_reg1_out <= (others => '0');
    elsif rising_edge(clk) then
      if stall = '0' then
        pc_out <= pc_in;
        sign_extended_out <= sign_extended_in;
        write_register0_out <= write_register0_in;
        write_register1_out <= write_register1_in;
        rd1_out <= rd1_in;
        rd2_out <= rd2_in;
        regwrite_out <= regwrite_in;
        memtoreg_out <= memtoreg_in;
        memwrite_out <= memwrite_in;
        regdst_out <= regdst_in;
        alusrc_out <= alusrc_in;
        alucontrol_out <= alucontrol_in;
        read_reg1_out <= read_reg1_in;
      else
        write_register0_out <= "11111";
        write_register1_out <= "11111";
        regwrite_out <= '0';
        memtoreg_out <= '0';
      end if;
    end if;
  end process;
end; 

library IEEE; use IEEE.STD_LOGIC_1164.all; use IEEE.STD_LOGIC_ARITH.all;

entity ex_mem is -- EX/MEM pipeline register
port (clk, reset            : in  STD_LOGIC;
-- entradas
      wb_in                 : in  STD_LOGIC_VECTOR(1 downto 0);
      m_in                  : in  STD_LOGIC_VECTOR(2 downto 0); 
      aluresult_in          : in  STD_LOGIC_VECTOR(31 downto 0);
      rd2_in                : in  STD_LOGIC_VECTOR(31 downto 0);
      pcbranch_in           : in  STD_LOGIC_VECTOR(31 downto 0);
      write_register_in     : in  STD_LOGIC_VECTOR(4 downto 0);
      pcjump_in             : in  STD_LOGIC_VECTOR(31 downto 0);
-- saidas
      wb_out                : out STD_LOGIC_VECTOR(1 downto 0);
      m_out                 : out STD_LOGIC_VECTOR(2 downto 0);
      aluresult_out         : out STD_LOGIC_VECTOR(31 downto 0);
      rd2_out               : out STD_LOGIC_VECTOR(31 downto 0);
      pcbranch_out          : out STD_LOGIC_VECTOR(31 downto 0);
      write_register_out    : out STD_LOGIC_VECTOR(4 downto 0);
      pcjump_out            : out STD_LOGIC_VECTOR(31 downto 0)
     );
end;
architecture arch_ex_mem of ex_mem is
begin
  process(clk, reset) begin
    if reset then
        pcbranch_out <= (others => '0');
        wb_out <= (others => '0');
        m_out <= (others => '0');
        aluresult_out <= (others => '0');
        rd2_out <= (others => '0');
        write_register_out <= (others => '1');
    elsif rising_edge(clk) then
        wb_out <= wb_in;
        m_out <= m_in;
        aluresult_out <= aluresult_in;
        rd2_out <= rd2_in;
        pcbranch_out <= pcbranch_in;
        write_register_out <= write_register_in;
    end if;
  end process;
end;

library IEEE; use IEEE.STD_LOGIC_1164.all; use IEEE.STD_LOGIC_ARITH.all;
 
entity mem_wb is -- MEM/WB pipeline register
    port (clk, reset: in STD_LOGIC;
          wb_in                 : in  STD_LOGIC_VECTOR(1 downto 0); 
          wb_out                : out STD_LOGIC_VECTOR(1 downto 0);
          read_mem_data_in      : in  STD_LOGIC_VECTOR(31 downto 0);
          read_mem_data_out     : out STD_LOGIC_VECTOR(31 downto 0);
          aluresult_in          : in  STD_LOGIC_VECTOR(31 downto 0);
          aluresult_out         : out STD_LOGIC_VECTOR(31 downto 0);
          write_register_in     : in  STD_LOGIC_VECTOR(4 downto 0);
          write_register_out    : out STD_LOGIC_VECTOR(4 downto 0)
        );
end;
architecture arch_wb_reg of mem_wb is
begin
  process(clk, reset) begin
    if reset then
        aluresult_out <= (others => '0');
        wb_out <= (others => '0');
        read_mem_data_out <= (others => '0');
        write_register_out <= (others => '0');
    elsif rising_edge(clk) then
        wb_out <= wb_in;
        read_mem_data_out <= read_mem_data_in;
        aluresult_out <= aluresult_in;
        write_register_out <= write_register_in;
    end if;
  end process;
end;

library IEEE; use IEEE.STD_LOGIC_1164.all; 
use IEEE.NUMERIC_STD_UNSIGNED.all;

entity adder is -- adder
  port(a, b: in  STD_LOGIC_VECTOR(31 downto 0);
       y:    out STD_LOGIC_VECTOR(31 downto 0));
end;

architecture behave of adder is
begin
  y <= a + b;
end;

library IEEE; use IEEE.STD_LOGIC_1164.all; 
use IEEE.NUMERIC_STD_UNSIGNED.all;

entity alu is 
  port(a, b:       in  STD_LOGIC_VECTOR(31 downto 0);
       alucontrol: in  STD_LOGIC_VECTOR(2 downto 0);
       result:     buffer STD_LOGIC_VECTOR(31 downto 0);
       zero:       out STD_LOGIC);
end;

architecture behave of alu is
  signal condinvb, sum: STD_LOGIC_VECTOR(31 downto 0);
begin
  condinvb <= not b when alucontrol(2) else b;
  sum <= a + condinvb + alucontrol(2);

  process(all) begin
    case alucontrol(1 downto 0) is
      when "00"   => result <= a and b; 
      when "01"   => result <= a or b; 
      when "10"   => result <= sum; 
      when "11"   => result <= (0 => sum(31), others => '0'); 
      when others => result <= (others => 'X'); 
    end case;
  end process;

  zero <= '1' when result = X"00000000" else '0';
end;


library IEEE; use IEEE.STD_LOGIC_1164.all;  use IEEE.STD_LOGIC_ARITH.all;

entity flopr is -- flip-flop with synchronous reset
  generic(width: integer);
  port(clk, reset: in  STD_LOGIC;
       d:          in  STD_LOGIC_VECTOR(width-1 downto 0);
       q:          out STD_LOGIC_VECTOR(width-1 downto 0));
end;

architecture asynchronous of flopr is
begin
  process(clk, reset) begin
    if reset then  q <= (others => '0');
    elsif rising_edge(clk) then
      q <= d;
    end if;
  end process;
end;

library IEEE; 
use IEEE.STD_LOGIC_1164.all; 
use IEEE.NUMERIC_STD_UNSIGNED.all;

entity forwarding_control is
  port (
    -- entradas
        regwriteE:       in std_logic; 
        regwriteM:       in std_logic; 
        regwriteW:       in std_logic;
        memtoregM:       in std_logic;
        read_register1E: in std_logic_vector(4 downto 0);
        read_register2E: in std_logic_vector(4 downto 0);
        write_registerM: in std_logic_vector(4 downto 0); 
        write_registerW: in std_logic_vector(4 downto 0);
    -- saidas
        forward_a:       out std_logic_vector(1 downto 0);
        forward_b:       out std_logic_vector(1 downto 0)
    );
end;

architecture control of forwarding_control is
 
 signal selA , selB : std_logic_vector(2 downto 0);
 signal equal, rs_not_zero, rs_eq_rt : std_logic;
begin

  process(all) begin
    -- conditions for alu first source
    if read_register1E = write_registerM and regwriteM = '1' then
      forward_a <= "01";
    elsif read_register1E = write_registerW and regwriteW = '1' then
      forward_a <= "10";
    else forward_a <= "00";
    end if;
    -- conditions for alu second source
    if read_register2E = write_registerM and regwriteM = '1' then
      forward_b <= "01";
    elsif read_register2E = write_registerW and regwriteW = '1' then
      forward_b <= "10";
    else forward_b <= "00";
    end if;
  end process;
end;

-- hazard detection unit. Based on section 4.7 of Patterson & Hennessy 4? Ed.

library IEEE; use IEEE.STD_LOGIC_1164.all; use IEEE.STD_LOGIC_ARITH.all;

entity hazard_unit is
    port(clk: in STD_LOGIC;
         reset : in STD_LOGIC;
         ID_EX_rd1:  in STD_LOGIC_VECTOR(4 downto 0);
         ID_EX_rd2:  in STD_LOGIC_VECTOR(4 downto 0);
         ID_EX_writeReg:  in STD_LOGIC_VECTOR(4 downto 0);
         ID_EX_branch:     in STD_LOGIC;
         ID_EX_branchnot: in STD_LOGIC;
         ID_EX_jump:     in STD_LOGIC;
         ID_EX_regWrite:   in STD_LOGIC;
         ID_EX_memToReg: in STD_LOGIC;
         ID_EX_memwrite:    in STD_LOGIC;
         EX_MEM_writeReg: in STD_LOGIC_VECTOR(4 downto 0);
         EX_MEM_memToReg:  in STD_LOGIC;
         EX_MEM_regWrite:   in STD_LOGIC;
         MEM_WB_writeReg: in STD_LOGIC_VECTOR(4 downto 0);
         MEM_WB_regWrite:   in STD_LOGIC;
         regdst: in STD_LOGIC;
         alusrc: in STD_LOGIC;
         stall: out STD_LOGIC);
end;
architecture arch_hazard_unit of hazard_unit is
  -- stall for each condition
    --signal aluStall, 
    signal branchStall, lwStall, swStall : STD_LOGIC;
    -- D = decode, E = ex, M = mem, W = wb. rs and rt are read regs, rd is read.
    signal rsD, rtD, rdE, rdM, rdW : STD_LOGIC_VECTOR(4 downto 0);
    -- branch signal
    signal branchD : STD_LOGIC;

begin

    rsD <= ID_EX_rd1;
    rdE <= ID_EX_writeReg;
    rdM <= EX_MEM_writeReg;
    rdW <= MEM_WB_writeReg;
    branchD <= (ID_EX_branch or ID_EX_branchnot or ID_EX_jump);
    
    stall <= lwStall or branchStall; --or aluStall;

    process (all) begin
        if regdst = '1' or (regdst='0' and alusrc = '0') or ID_EX_memwrite = '1' then -- there is no second operand
            rtD <= ID_EX_rd2;
        else rtD <= ID_EX_rd1;
        end if;

        -- stall logic for branch
        if branchD = '1' and ID_EX_regWrite ='1' and (rdE = rsD or rdE = rtD) then
          branchStall <= '1';
        elsif branchD = '1' and EX_MEM_regWrite ='1' and (rdM = rsD or rdM = rtD) then
          branchStall <= '1';
        else branchStall <= '0';
        end if;

        -- stall logic for lw
        if EX_MEM_memToReg = '1' and ((rsD = rdE) or (rtD = rdE)) then
            lwStall <= '1';
        else lwStall <= '0';
        end if;
   end process;
end;

library IEEE; use IEEE.STD_LOGIC_1164.all; use IEEE.STD_LOGIC_ARITH.all;

entity jump_unit is
  port(
    register1:  in  STD_LOGIC_VECTOR(31 downto 0); 
    register2:  in  STD_LOGIC_VECTOR(31 downto 0);
    equal: out STD_LOGIC);
end entity jump_unit;

architecture behavioral of jump_unit is

begin
  equal <= '1' when (register1 = register2) 
        else '0';
end architecture behavioral;


library IEEE; use IEEE.STD_LOGIC_1164.all;

entity mux2 is -- two-input multiplexer
  generic(width: integer);
  port(d0, d1: in  STD_LOGIC_VECTOR(width-1 downto 0);
       s:      in  STD_LOGIC;
       y:      out STD_LOGIC_VECTOR(width-1 downto 0));
end;

architecture behave of mux2 is
begin
  y <= d1 when s else d0;
end;

library IEEE;use IEEE.STD_LOGIC_1164.all;

entity mux3 is -- three-input multiplexer
    generic (width : INTEGER);
    port (d0, d1, d2 : in STD_LOGIC_VECTOR(width - 1 downto 0);
          s : in STD_LOGIC_VECTOR(1 downto 0);
          y : out STD_LOGIC_VECTOR(width - 1 downto 0)
    );
end;

architecture choose1 of mux3 is
begin
    with s select
    y <= d0 when "00", 
         d1 when "01", 
         d2 when others;
end choose1;

library IEEE; use IEEE.STD_LOGIC_1164.all; 
use IEEE.NUMERIC_STD_UNSIGNED.all;

entity regfile is -- three-port register file
  port(clk:           in  STD_LOGIC;
       we3:           in  STD_LOGIC;
       ra1, ra2, wa3: in  STD_LOGIC_VECTOR(4 downto 0);
       wd3:           in  STD_LOGIC_VECTOR(31 downto 0);
       rd1, rd2:      out STD_LOGIC_VECTOR(31 downto 0));
end;

architecture behave of regfile is
  type ramtype is array (31 downto 0) of STD_LOGIC_VECTOR(31 downto 0);
  signal mem: ramtype;
begin
  -- three-ported register file
  -- read two ports combinationally
  -- write third port on rising edge of clock
  -- register 0 hardwired to 0
  -- note: for pipelined processor, write third port
  -- on falling edge of clk
  process(clk) begin
    if falling_edge(clk) then
       if we3 = '1' then mem(to_integer(wa3)) <= wd3;
       end if;
    end if;
  end process;
  process(all) begin
    if (to_integer(ra1) = 0) then rd1 <= X"00000000"; -- register 0 holds 0
    else rd1 <= mem(to_integer(ra1));
    end if;
    if (to_integer(ra2) = 0) then rd2 <= X"00000000"; 
    else rd2 <= mem(to_integer(ra2));
    end if;
  end process;
end;

library IEEE; use IEEE.STD_LOGIC_1164.all;

entity signext is -- sign extender
  port(a: in  STD_LOGIC_VECTOR(15 downto 0);
       signedop: in STD_LOGIC; -- Signal added
       y: out STD_LOGIC_VECTOR(31 downto 0));
end;

architecture behave of signext is
begin
-- Adding unsigned operation
  y <= X"ffff" & a when (a(15) AND signedop) else X"0000" & a; 
end;

library IEEE; use IEEE.STD_LOGIC_1164.all;

entity sl2 is -- shift left by 2
  port(a: in  STD_LOGIC_VECTOR(31 downto 0);
       y: out STD_LOGIC_VECTOR(31 downto 0));
end;

architecture behave of sl2 is
begin
  y <= a(29 downto 0) & "00";
end;