use crate::memory::Memory;
use crate::opcodes::{OPCODE_MACHINE_CYCLES, PREFIX_OPCODE_MACHINE_CYCLES};
use crate::binary_utils;

const MACHINE_CYCLE: u8 = 4;
const PREFIX_OPCODE: u8 = 0xCB;

pub struct Cpu {
    pub a: u8,              //Accumulator Register
    pub b: u8,              //General Purpose Register
    pub c: u8,              //General Purpose Register
    pub d: u8,              //General Purpose Register
    pub e: u8,              //General Purpose Register
    pub f: u8,              //Flags Register
    pub h: u8,              //General Purpose Register
    pub l: u8,              //General Purpose Register
    pub sp: u16,            //Stack Pointer Register
    pub pc: u16,            //Program Counter Register
    cpu_state: CpuState,    //Let's us know the current state of the CPU
    cpu_clk_cycles: u8,     //Keeps track of how many cpu clk cycles have gone by
    current_opcode: u8,     //Keeps track of the current worked on opcode
}

impl Cpu {
    pub fn new() -> Cpu {
        //Note these initial values depend on the gameboy model.
        //We are assuming DMG
        Cpu { 
            a: 0x01, 
            b: 0x00, 
            c: 0x13, 
            d: 0x00, 
            e: 0xD8, 
            f: 0xB0, 
            h: 0x01, 
            l: 0x4D, 
            sp: 0xFFFE, 
            pc: 0x0100,
            cpu_state: CpuState::Fetch,
            cpu_clk_cycles: 0,
            current_opcode: 0x00,
        }
    }

    /**
     * This mimicks 1 cpu clk cycle. NOT a machine cycle, which is 4 cpu clk cycles.
     */
    pub fn cycle(&mut self, memory: &mut Memory) {
        //Because the entire system is memory limited. All instructions take at least
        //One machine cycles to complete. So we won't start doing work until the last cpu cycle (=4)
        self.cpu_clk_cycles += 1;
        if self.cpu_clk_cycles >= MACHINE_CYCLE {
            self.cpu_clk_cycles = 0;
        } else {
            return;
        }

        //Essentially can only by here if 4 cpu clk cycles have passed equaling 1 machine cycle

        match self.cpu_state {
            CpuState::Fetch => {    //1 machine cycle (4 cpu clks)
                self.current_opcode = self.fetch(memory);  

                if self.current_opcode == PREFIX_OPCODE {
                    self.cpu_state = CpuState::FetchPrefix;
                } else {
                    if OPCODE_MACHINE_CYCLES[self.current_opcode as usize] == 1 {       //Running the execute step in the same cycle b/c the instruction only takes 1 machine cycle and the fetch already takes 1 machine cycle
                        self.cpu_state = CpuState::Execute { status: ExecuteStatus::Running, machine_cycles: 1, temp_reg: 0 };
                        self.exexute(self.current_opcode, memory, 1, &mut 0);   
                    } else {
                        self.cpu_state = CpuState::Execute { status: ExecuteStatus::NotStarted, machine_cycles: 0, temp_reg: 0 };
                    }       
                }
            }
            CpuState::FetchPrefix => {  //1 machine cycle (4 cpu clks)
                self.current_opcode = self.fetch(memory);
                self.cpu_state = CpuState::Execute { status: ExecuteStatus::NotStarted, machine_cycles: 0, temp_reg: 0 };
            }
            CpuState::Execute { status, mut machine_cycles, mut temp_reg } => {
                machine_cycles += 1;
                self.exexute(self.current_opcode, memory, machine_cycles, &mut temp_reg);

                match status {
                    ExecuteStatus::Completed => self.cpu_state = CpuState::InterruptHandle,
                    _ => ()
                }
            },
            CpuState::InterruptHandle => {

            }
        }
    }

    

    /**
     * Going into memory and reading what instruction the programmer
     * wants to execute. This function does not do the actual execution.
     */
    pub fn fetch(&mut self, memory: &Memory) -> u8 {
        let opcode = memory.read_byte(self.pc);
        self.pc += 1;
        return opcode;
    }

    /**
    * Given an opcode it will execute the instruction of the opcode
    */
    pub fn exexute(&mut self, opcode: u8, memory: &mut Memory, machine_cycle: u8, temp_reg: &mut u16) {
        match opcode {
            0x00 => (),                                                                 //NOP
            0x01 => self.ld_r16_u16(memory, &mut self.b, &mut self.c, machine_cycle),   //LD_BC_U16
            0x02 => self.ld_r16_a(memory, &mut self.b, &mut self.c, machine_cycle),     //LD_BC_A
            0x03 => self.inc_r16(memory, &mut self.b, &mut self.c, machine_cycle),      //INC_BC
            0x04 => self.inc_r8(&mut self.b, machine_cycle),                            //INC_B
            0x05 => self.dec_r8(&mut self.b, machine_cycle),                            //DEC_B
            0x06 => self.ld_r8_u8(memory, &mut self.b, machine_cycle),                  //LD_B_U8
            0x07 => self.rlca(machine_cycle),                                           //RLCA
            0x08 => self.ld_u16_sp(memory, machine_cycle, temp_reg),                    //LD_U16_SP
            0x09 => self.add_hl_r16(&mut self.b, &mut self.c, machine_cycle),           //ADD_HL_BC may not be the most cycle accurate
            0x0A => self.ld_a_r16(memory, &mut self.b, &mut self.c, machine_cycle),     //LD_A_(BC)
            0x0B => self.dec_r16(&mut self.b, &mut self.c, machine_cycle),              //DEC_BC may not be the most accurate in cycles,
            0x0C => self.inc_r8(&mut self.c, machine_cycle),                            //INC_C
            0x0D => self.dec_r8(&mut self.c, machine_cycle),                            //DEC_C
            0x0E => self.ld_r8_u8(memory, &mut self.c, machine_cycle),                  //LD_C_U8
            0x0F => self.rrca(machine_cycle),                                           //RRCA         
            0x10 => self.stop(),                                                        //STOP
            0x11 => self.ld_r16_u16(memory, &mut self.d, &mut self.e, machine_cycle),   //LD_DE_U16
            0x12 => self.ld_r16_a(memory, &mut self.d, &mut self.e, machine_cycle),     //LD_(DE)_A
            0x13 => self.inc_r16(memory, &mut self.d, &mut self.e, machine_cycle),      //INC_DE
            0x14 => self.inc_r8(&mut self.d, machine_cycle),                            //INC_D
            0x15 => self.dec_r8(&mut self.d, machine_cycle),                            //DEC_D
            0x16 => self.ld_r8_u8(memory, &mut self.d, machine_cycle),                  //LD_D_U8
            0x17 => self.rla(machine_cycle),                                            //RLA
            0x18 => self.jr_i8(memory, machine_cycle, temp_reg),                        //JR_i8
            0x19 => self.add_hl_r16(&mut self.d, &mut self.e, machine_cycle),           //ADD_HL_DE
            0x1A => self.ld_a_r16(memory, &mut self.d, &mut self.e, machine_cycle),     //LD_A_R16
            0x1B => self.dec_r16(&mut self.d, &mut self.e, machine_cycle),              //DEC_DE
            0x1C => self.inc_r8(&mut self.e, machine_cycle),                            //INC_E
            0x1D => self.dec_r8(&mut self.e, machine_cycle),                            //DEC_E
            0x1E => self.ld_r8_u8(memory, &mut self.e, machine_cycle),                  //LD_E_U8
            0x1F => self.rra(machine_cycle),                                            //RRA
            0x20 => self.jr_cc_i8(memory, !self.get_zero_flag()),
            0x21 => self.ld_r16_u16(memory, Register::H, Register::L),
            0x22 => self.ld_hli_a(memory),
            0x23 => self.inc_r16(Register::H, Register::L),
            0x24 => self.inc_r8(Register::H),
            0x25 => self.dec_r8(Register::H),
            0x26 => self.ld_r8_u8(memory, Register::H),
            0x27 => self.daa(),
            0x28 => self.jr_cc_i8(memory, self.get_zero_flag()),
            0x29 => self.add_hl_r16(Register::H, Register::L),
            0x2A => self.ld_a_hli(memory),
            0x2B => self.dec_r16(Register::H, Register::L),
            0x2C => self.inc_r8(Register::L),
            0x2D => self.dec_r8(Register::L),
            0x2E => self.ld_r8_u8(memory, Register::L),
            0x2F => self.cpl(),
            0x30 => self.jr_cc_i8(memory, !self.get_carry_flag()),
            0x31 => self.ld_sp_u16(memory),
            0x32 => self.ld_hld_a(memory),
            0x33 => self.inc_sp(),
            0x34 => self.inc_hl(memory),
            0x35 => self.dec_hl(memory),
            0x36 => self.ld_hl_u8(memory),
            0x37 => self.scf(),
            0x38 => self.jr_cc_i8(memory, self.get_carry_flag()),
            0x39 => self.add_hl_sp(),
            0x3A => self.ld_a_hld(memory),
            0x3B => self.dec_sp(),
            0x3C => self.inc_r8(Register::A),
            0x3D => self.dec_r8(Register::A),
            0x3E => self.ld_r8_u8(memory, Register::A),
            0x3F => self.ccf(),
            0x40 => self.ld_r8_r8(Register::B, Register::B),
            0x41 => self.ld_r8_r8(Register::B, Register::C),
            0x42 => self.ld_r8_r8(Register::B, Register::D),
            0x43 => self.ld_r8_r8(Register::B, Register::E),
            0x44 => self.ld_r8_r8(Register::B, Register::H),
            0x45 => self.ld_r8_r8(Register::B, Register::L),
            0x46 => self.ld_r8_hl(memory, Register::B),
            0x47 => self.ld_r8_r8(Register::B, Register::A),
            0x48 => self.ld_r8_r8(Register::C, Register::B),
            0x49 => self.ld_r8_r8(Register::C, Register::C),
            0x4A => self.ld_r8_r8(Register::C, Register::D),
            0x4B => self.ld_r8_r8(Register::C, Register::E),
            0x4C => self.ld_r8_r8(Register::C, Register::H),
            0x4D => self.ld_r8_r8(Register::C, Register::L),
            0x4E => self.ld_r8_hl(memory, Register::C),
            0x4F => self.ld_r8_r8(Register::C, Register::A),
            0x50 => self.ld_r8_r8(Register::D, Register::B),
            0x51 => self.ld_r8_r8(Register::D, Register::C),
            0x52 => self.ld_r8_r8(Register::D, Register::D),
            0x53 => self.ld_r8_r8(Register::D, Register::E),
            0x54 => self.ld_r8_r8(Register::D, Register::H),
            0x55 => self.ld_r8_r8(Register::D, Register::L),
            0x56 => self.ld_r8_hl(memory, Register::D),
            0x57 => self.ld_r8_r8(Register::D, Register::A),
            0x58 => self.ld_r8_r8(Register::E, Register::B),
            0x59 => self.ld_r8_r8(Register::E, Register::C),
            0x5A => self.ld_r8_r8(Register::E, Register::D),
            0x5B => self.ld_r8_r8(Register::E, Register::E),
            0x5C => self.ld_r8_r8(Register::E, Register::H),
            0x5D => self.ld_r8_r8(Register::E, Register::L),
            0x5E => self.ld_r8_hl(memory, Register::E),
            0x5F => self.ld_r8_r8(Register::E, Register::A),
            0x60 => self.ld_r8_r8(Register::H, Register::B),
            0x61 => self.ld_r8_r8(Register::H, Register::C),
            0x62 => self.ld_r8_r8(Register::H, Register::D),
            0x63 => self.ld_r8_r8(Register::H, Register::E),
            0x64 => self.ld_r8_r8(Register::H, Register::H),
            0x65 => self.ld_r8_r8(Register::H, Register::L),
            0x66 => self.ld_r8_hl(memory, Register::H),
            0x67 => self.ld_r8_r8(Register::H, Register::A),
            0x68 => self.ld_r8_r8(Register::L, Register::B),
            0x69 => self.ld_r8_r8(Register::L, Register::C),
            0x6A => self.ld_r8_r8(Register::L, Register::D),
            0x6B => self.ld_r8_r8(Register::L, Register::E),
            0x6C => self.ld_r8_r8(Register::L, Register::H),
            0x6D => self.ld_r8_r8(Register::L, Register::L),
            0x6E => self.ld_r8_hl(memory, Register::L),
            0x6F => self.ld_r8_r8(Register::L, Register::A),
            0x70 => self.ld_hl_r8(memory, Register::B),
            0x71 => self.ld_hl_r8(memory, Register::C),
            0x72 => self.ld_hl_r8(memory, Register::D),
            0x73 => self.ld_hl_r8(memory, Register::E),
            0x74 => self.ld_hl_r8(memory, Register::H),
            0x75 => self.ld_hl_r8(memory, Register::L),
            0x76 => self.halt(),
            0x77 => self.ld_hl_r8(memory, Register::A),
            0x78 => self.ld_r8_r8(Register::A, Register::B),
            0x79 => self.ld_r8_r8(Register::A, Register::C),
            0x7A => self.ld_r8_r8(Register::A, Register::D),
            0x7B => self.ld_r8_r8(Register::A, Register::E),
            0x7C => self.ld_r8_r8(Register::A, Register::H),
            0x7D => self.ld_r8_r8(Register::A, Register::L),
            0x7E => self.ld_r8_hl(memory, Register::A), 
            0x7F => self.ld_r8_r8(Register::A, Register::A),
            0x80 => self.add_a_r8(Register::B),
            0x81 => self.add_a_r8(Register::C),
            0x82 => self.add_a_r8(Register::D),
            0x83 => self.add_a_r8(Register::E),
            0x84 => self.add_a_r8(Register::H),
            0x85 => self.add_a_r8(Register::L),
            0x86 => self.add_a_hl(memory),
            0x87 => self.add_a_r8(Register::A),
            0x88 => self.adc_a_r8(Register::B),
            0x89 => self.adc_a_r8(Register::C),
            0x8A => self.adc_a_r8(Register::D),
            0x8B => self.adc_a_r8(Register::E),
            0x8C => self.adc_a_r8(Register::H),
            0x8D => self.adc_a_r8(Register::L),
            0x8E => self.adc_a_hl(memory),
            0x8F => self.adc_a_r8(Register::A),
            0x90 => self.sub_a_r8(Register::B),
            0x91 => self.sub_a_r8(Register::C),
            0x92 => self.sub_a_r8(Register::D),
            0x93 => self.sub_a_r8(Register::E),
            0x94 => self.sub_a_r8(Register::H),
            0x95 => self.sub_a_r8(Register::L),
            0x96 => self.sub_a_hl(memory),
            0x97 => self.sub_a_r8(Register::A),
            0x98 => self.sbc_a_r8(Register::B),
            0x99 => self.sbc_a_r8(Register::C),
            0x9A => self.sbc_a_r8(Register::D),
            0x9B => self.sbc_a_r8(Register::E),
            0x9C => self.sbc_a_r8(Register::H),
            0x9D => self.sbc_a_r8(Register::L),
            0x9E => self.sbc_a_hl(memory),
            0x9F => self.sbc_a_r8(Register::A),
            0xA0 => self.and_a_r8(Register::B),
            0xA1 => self.and_a_r8(Register::C),
            0xA2 => self.and_a_r8(Register::D),
            0xA3 => self.and_a_r8(Register::E),
            0xA4 => self.and_a_r8(Register::H),
            0xA5 => self.and_a_r8(Register::L),
            0xA6 => self.and_a_hl(memory),
            0xA7 => self.and_a_r8(Register::A),
            0xA8 => self.xor_a_r8(Register::B),
            0xA9 => self.xor_a_r8(Register::C),
            0xAA => self.xor_a_r8(Register::D),
            0xAB => self.xor_a_r8(Register::E),
            0xAC => self.xor_a_r8(Register::H),
            0xAD => self.xor_a_r8(Register::L),
            0xAE => self.xor_a_hl(memory),
            0xAF => self.xor_a_r8(Register::A),
            0xB0 => self.or_a_r8(Register::B),
            0xB1 => self.or_a_r8(Register::C),
            0xB2 => self.or_a_r8(Register::D),
            0xB3 => self.or_a_r8(Register::E),
            0xB4 => self.or_a_r8(Register::H),
            0xB5 => self.or_a_r8(Register::L),
            0xB6 => self.or_a_hl(memory),
            0xB7 => self.or_a_r8(Register::A),
            0xB8 => self.cp_a_r8(Register::B),
            0xB9 => self.cp_a_r8(Register::C),
            0xBA => self.cp_a_r8(Register::D),
            0xBB => self.cp_a_r8(Register::E),
            0xBC => self.cp_a_r8(Register::H),
            0xBD => self.cp_a_r8(Register::L),
            0xBE => self.cp_a_hl(memory),
            0xBF => self.cp_a_r8(Register::A),
            0xC0 => self.ret_cc(memory, !self.get_zero_flag()),
            0xC1 => self.pop(memory, Register::B, Register::C),
            0xC2 => self.jp_cc_u16(memory, !self.get_zero_flag()),
            0xC3 => self.jp_u16(memory),
            0xC4 => self.call_cc_u16(memory, !self.get_zero_flag()),
            0xC5 => self.push_r16(memory, Register::B, Register::C),
            0xC6 => self.add_a_u8(memory),
            0xC7 => self.rst_vec(memory, 0x00),
            0xC8 => self.ret_cc(memory, self.get_zero_flag()),
            0xC9 => self.ret(memory),
            0xCA => self.jp_cc_u16(memory, self.get_zero_flag()),
            0xCB => self.prefix(memory),    //Need to pass the self.current opcode here as that will
            0xCC => self.call_cc_u16(memory, self.get_zero_flag()),
            0xCD => self.call_u16(memory),
            0xCE => self.adc_a_u8(memory),
            0xCF => self.rst_vec(memory, 0x08),
            0xD0 => self.ret_cc(memory, !self.get_carry_flag()),
            0xD1 => self.pop(memory, Register::D, Register::E),
            0xD2 => self.jp_cc_u16(memory, !self.get_carry_flag()),
            0xD3 => panic!("0xD3 is an unused opcode"),
            0xD4 => self.call_cc_u16(memory, !self.get_carry_flag()),
            0xD5 => self.push_r16(memory, Register::D, Register::E),
            0xD6 => self.sub_a_u8(memory),
            0xD7 => self.rst_vec(memory, 0x10),
            0xD8 => self.ret_cc(memory, self.get_carry_flag()),
            0xD9 => self.reti(memory),
            0xDA => self.jp_cc_u16(memory, self.get_carry_flag()),
            0xDB => panic!("0xDB is an unused opcode"),
            0xDC => self.call_cc_u16(memory, self.get_carry_flag()),
            0xDE => panic!("0xDE is an unused opcode"),
            0xDF => self.rst_vec(memory, 0x18),
            0xE0 => self.ldh_u8_a(memory),
            0xE1 => self.pop(memory, Register::H, Register::L),
            0xE2 => self.ldh_c_a(memory),
            0xE3 => panic!("0xE3 is an unused opcode"),
            0xE4 => panic!("0xE4 is an unused opcode"),
            0xE5 => self.push_r16(memory, Register::H, Register::L),
            0xE6 => self.and_a_u8(memory),
            0xE7 => self.rst_vec(memory, 0x20),
            0xE8 => self.add_sp_i8(memory),
            0xE9 => self.jp_hl(),
            0xEA => self.ld_u16_a(memory),
            0xEB => panic!("0xEB is an unused opcode"),
            0xEC => panic!("0xEC is an unused opcode"),
            0xED => panic!("0xED is an unused opcode"),
            0xEE => self.xor_a_u8(memory),
            0xEF => self.rst_vec(memory, 0x28),
            0xF0 => self.ldh_a_u8(memory),
            0xF1 => self.pop(memory, Register::A, Register::F),
            0xF2 => self.ldh_a_c(memory),
            0xF3 => self.di(memory),
            0xF4 => panic!("0xF4 is an unused opcode"),
            0xF5 => self.push_r16(memory, Register::A, Register::F),
            0xF6 => self.or_a_u8(memory),
            0xF7 => self.rst_vec(memory, 0x30),
            0xF8 => self.ld_hl_sp_i8(memory),
            0xF9 => self.ld_sp_hl(),
            0xFA => self.ld_a_u16(memory),
            0xFB => self.ei(memory),
            0xFC => panic!("0xFC is an unused opcode"),
            0xFD => panic!("0xFD is an unused opcode"),
            0xFE => self.cp_a_u8(memory),
            0xFF => self.rst_vec(memory, 0x38),
            _ => panic!("Unknown opcode"),
        }
    }

    /**
     * Returns the A and F registers as a 16-bit register
     */
    pub fn af(&self) -> u16 {
        let mut af: u16; 

        af = (self.a as u16) << 8;
        af |= self.f as u16;

        return af;
    }

    /**
     * Returns the B and C registers as a 16-bit register
     */
    pub fn bc(&self) -> u16 {
        let mut bc: u16; 

        bc = (self.b as u16) << 8;
        bc |= self.c as u16;

        return bc;
    }

    pub fn set_bc(&mut self, data: u16) {
        self.b = (data >> 8) as u8;
        self.c = data as u8;
    }

    /**
    * Returns the D and E registers as a 16-bit register
    */
    pub fn de(&self) -> u16 {
        let mut de: u16; 

        de = (self.d as u16) << 8;
        de |= self.e as u16;

        return de;
    }

    pub fn set_de(&mut self, data: u16) {
        let d: u8 = (data >> 8) as u8;
        let e: u8 = data as u8;

        self.d = d;
        self.e = e;
    }

    /**
    * Returns the H and L registers as a 16-bit register
    */
    pub fn hl(&self) -> u16 {
        let mut hl: u16; 

        hl = (self.h as u16) << 8;
        hl |= self.l as u16;

        return hl;
    }

    pub fn set_hl(&mut self, data: u16) {
        let h: u8 = (data >> 8) as u8;
        let l: u8 = data as u8;

        self.h = h;
        self.l = l
    }

    pub fn get_carry_flag(&self) -> u8 {
        binary_utils::get_bit(self.f, 4)
    }

    /**
     * Sets cpu flag register
     */
    fn set_flags(&mut self, zero_flag: Option<bool>, negative_flag: Option<bool>, half_carry_flag: Option<bool>, carry_flag: Option<bool>) {
        match zero_flag {
            None => (),
            Some(flag) => {
                if flag {
                    self.f |= 0b10000000;
                } else {
                    self.f &= 0b01111111;
                }
            },
        }

        match negative_flag {
            None => (),
            Some(flag) => {
                if flag {
                    self.f |= 0b01000000;
                } else {
                    self.f &= 0b10111111;
                }
            }
        }

        match half_carry_flag {
            None => (),
            Some(flag) => {
                if flag {
                    self.f |= 0b00100000;
                } else {
                    self.f &= 0b11011111;
                }
            }
        }

        match carry_flag {
            None => (),
            Some(flag) => {
                if flag {
                    self.f |= 0b00010000;
                } else {
                    self.f &= 0b11101111;
                }
            }
        }
    }

    /**
     * Does absolutely nothing but consume a machine cycle and increment the pc
     * 
     * MACHINE CYCLES: 1
     * INSTRUCTION LENGTH: 1
     */
    pub fn nop(&mut self) {
        match self.cpu_state {
            CpuState::Execute { mut status, machine_cycles, temp_reg } => status = ExecuteStatus::Completed,
            _ => panic!("How are you executing when you're not in a execution state"),
        }
    }

    /**
     * Loads the unsigned 16 bit value into the given registers
     * 
     * MACHINE CYCLES: 3
     * INSTRUCTION LENGTH: 3
     */
    pub fn ld_r16_u16(&mut self, memory: &Memory, upper_reg: &mut u8, lower_reg: &mut u8, machine_cycle: u8) {
        match machine_cycle {
            1 => *lower_reg = memory.read_byte(self.pc),
            2 => *upper_reg = memory.read_byte(self.pc),
            _ => panic!("1 to many cycles on ld_r16_u16"),
        }
        self.pc += 1;
    }

    /**
     * Store the value in register A into the byte pointed to by register r16.
     * 
     * MACHINE CYCLES: 4
     * INSTRUCTION LENGTH: 3
     */
    pub fn ld_r16_a(&mut self, memory: &Memory, upper_reg: &mut u8, lower_reg: &mut u8, machine_cycle: u8) {
        let address: u16 = (*upper_reg as u16) << 8 | *lower_reg as u16;
        match machine_cycle {
            1 => memory.write_byte(address, self.a),
            _ => panic!("1 to many cycles on ld_r16_a") 
        }
    }

    /**
     * Increment value in register r16 by 1.
     * 
     * MACHINE CYCLES: 2
     * INSTRUCTION LENGTH: 1
     */
    pub fn inc_r16(&mut self, memory: &Memory, upper_reg: &mut u8, lower_reg: &mut u8, machine_cycle: u8) {
        match machine_cycle {
            1 => {
                let r16 = binary_utils::build_16bit_num(*upper_reg, *lower_reg) + 1;
                *upper_reg = (r16 >> 8) as u8;
                *lower_reg = r16 as u8;
            }
            _ => panic!("1 to many cycles on inc_r16"),
        }  
    }

    /**
     * Increment value in register r8 by 1.
     * 
     * MACHINE CYCLES: 1
     * INSTRUCTION LENGTH: 1
     */
    pub fn inc_r8(&mut self, reg: &mut u8, machine_cycle: u8) {
        match machine_cycle {
            1 => {
                *reg += 1;
                self.set_flags(Some(*reg == 0), Some(false), Some((*reg & 0xF) + 1 > 0xF), None);
            }
            _ => panic!("1 to many cycles on inc_r8"),
        }
    }

        /**
     * Decrement value in register r8 by 1.
     * 
     * MACHINE CYCLES: 1
     * INSTRUCTION LENGTH: 1
     */
    pub fn dec_r8(&mut self, reg: &mut u8, machine_cycle: u8) {
        match machine_cycle {
            1 => {
                *reg -= 1;
                self.set_flags(Some(*reg == 0), Some(true), Some(*reg & 0xF == 0xF), None);
            }
            _ => panic!("1 to many cycles on dec_r8"),
        }
    }

    /**
     * Load value u8 into register r8
     * 
     * MACHINE CYCLES: 2
     * INSTRUCTION LENGTH: 2
     */
    pub fn ld_r8_u8(&mut self, memory: &Memory, reg: &mut u8, machine_cycle: u8) {
        match machine_cycle {
            1 => *reg = memory.read_byte(self.pc),
            _ => panic!("1 to many cycles on ld_r8_u8"),
        }
        self.pc += 1;
    }

    /**
     * Rotate register A left.
     * 
     * MACHINE CYCLES: 1
     * INSTRUCTION LENGTH: 1
     */
    pub fn rlca(&mut self, machine_cycle: u8) {
        match machine_cycle {
            1 => {
                self.set_flags(Some(false), Some(false), Some(false), Some(binary_utils::get_bit(self.a, 7) != 0));
                self.a = self.a.rotate_left(1);
            }
            _ => panic!("1 to many cycles on RLCA"),
        }
    }

    /**
     * Store SP & $FF at address n16 and SP >> 8 at address n16 + 1.
     * 
     * MACHINE CYCLES: 5
     * INSTRUCTION LENGTH: 3
     */
    pub fn ld_u16_sp(&mut self, memory: &mut Memory, machine_cycle: u8, temp_reg: &mut u16) {
        match machine_cycle {
            1 => { *temp_reg |= memory.read_byte(self.pc) as u16; self.pc += 1; },        //read lower byte ASSUMING TEMP REG TO BE 0
            2 => { *temp_reg |= (memory.read_byte(self.pc) as u16) << 8; self.pc += 1; }, //read upper byte 
            3 => memory.write_byte(*temp_reg, self.sp as u8),
            4 => memory.write_byte(*temp_reg + 1, (self.sp >> 8) as u8),
            _ => panic!("1 to many cycles on LD_U16_SP"),
        }
    }

    /**
     * Add the value in r16 to HL.
     * 
     * MACHINE CYCLES: 2
     * INSTRUCTION LENGTH: 1 
     */
    pub fn add_hl_r16(&mut self, upper_reg: &mut u8, lower_reg: &mut u8, machine_cycle: u8) {
        match machine_cycle {
            1 => {
                let reg_16 = binary_utils::build_16bit_num(*upper_reg, *lower_reg);           
                let (result, overflow) = self.hl().overflowing_add(reg_16);
                let half_carry_overflow = (self.hl() & 0x0FFF) + (reg_16 & 0x0FFF) > 0x0FFF;
                self.set_hl(result);
                self.set_flags(None, Some(false), Some(half_carry_overflow), Some(overflow));
            }
            _ => panic!("1 to many cycles on add_hl_r16"),
        }
    }

    /**
     * Load value in register A from the byte pointed to by register r16.
     * 
     * MACHINE CYCLES: 2
     * INSTRUCTION LENGTH: 1
     */
    pub fn ld_a_r16(&mut self, memory: &Memory, upper_reg: &mut u8, lower_reg: &mut u8, machine_cycle: u8) {
        match machine_cycle {
            1 => self.a = memory.read_byte(binary_utils::build_16bit_num(*upper_reg, *lower_reg)),
            _ => panic!("1 to many cycles on ld_a_r16"),
        }
    }

    /**
    * Decrement value in register r16 by 1.
    * 
    * MACHINE CYCLES: 2
    * INSTRUCTION LENGTH: 1
    */
    pub fn dec_r16(&mut self, upper_reg: &mut u8, lower_reg: &mut u8, machine_cycle: u8) {
        match machine_cycle {
            1 => {
                let r16 = binary_utils::build_16bit_num(*upper_reg, *lower_reg) - 1;
                *upper_reg = (r16 >> 8) as u8;
                *lower_reg = r16 as u8;
            }
            _ => panic!("1 to many cycles on dec_r16"),
        }  
    }

    /**
    * Rotate register A right.
    * 
    * MACHINE CYCLE: 1
    * INSTRUCTION LENGTH: 1
    */
    pub fn rrca(&mut self, machine_cycle: u8) {
        match machine_cycle {
            1 => {
                self.set_flags(Some(false), Some(false), Some(false), Some(binary_utils::get_bit(self.a, 0) != 0));
                self.a = self.a.rotate_right(1);
            }
            _ => panic!("1 to many cycles on RLCA"),
        }
    }

    /**
     * THIS IS VERY SPECIAL NEED TO KNOW MORE ABOUT IT. Helps the gameboy
     * get into a very low power state, but also turns off a lot of peripherals
     */
    pub fn stop(&mut self) {
        //Need to reset the Timer divider register
        //timer begins ticking again once stop mode ends
        todo!("NEED TO IMPLEMENT THE STOP INSTRUCTION");
    }

    /**
     * Rotate register A left through carry.
     * 
     * MACHINE CYCLES: 1
     * INSTRUCTION LENGTH: 1 
     */
    pub fn rla(&mut self, machine_cycle: u8) {
        match machine_cycle {
            1 => {
                self.a = (self.a << 1) | self.get_carry_flag();
                self.set_flags(Some(false), Some(false), Some(false), Some(binary_utils::get_bit(self.a, 7) != 0));
            }
            _ => panic!("1 to many machine cycles in rla")
        }
    }

    /**
     * Jump by i8 to a different address relative to the pc 
     * 
     * MACHINE CYCLES: 3
     * INSTRUCTION LENGTH: 2
     */
    pub fn jr_i8(&mut self, memory: &Memory, machine_cycle: u8, temp_reg: &mut u16) {
        match machine_cycle {
            1 => { *temp_reg = memory.read_byte(self.pc); self.pc += 1; },
            2 =>  self.pc = self.pc.wrapping_add_signed(*temp_reg as i16),
            _ => panic!("1 to many machine cycles in jr_i8")
        }
    }

    /**
     * Rotate register A right through carry.
     * 
     * MACHINE CYCLES: 1
     * INSTRUCTION LENGTH: 1
     */
    pub fn rra(&mut self, machine_cycle: u8) {
        match machine_cycle {
            1 => {
                self.a = (self.a >> 1) | (self.get_carry_flag() << 7);
                self.set_flags(Some(false), Some(false), Some(false), Some(binary_utils::get_bit(self.a, 0) != 0));
            }
            _ => panic!("1 to many machine cycles in rla")
        }
    }

    /**
     * Relative Jump by i8 if condition cc is met.
     * 
     * MACHINE CYCLES: 3 IF TAKEN/ 2 IF NOT TAKEN
     * INSTRUCTION LENGTH: 2
     */
    pub fn jr_cc_i8(&mut self, memory: &Memory, mut condition: u8, machine_cycle: u8, temp_reg: &mut u16) {
        match machine_cycle {
            1 => { *temp_reg = memory.read_byte(self.pc); self.pc += 1 },
            2 => self.pc = self.pc.wrapping_add_signed(*temp_reg as i16),
            _ => panic!("1 to many machine cycles in jr_cc_i8"),
        }
        self.pc += 1;
        condition &= 0x1;   //doing this so we only see the first bit

        if condition != 0 {
            self.pc = self.pc.wrapping_add_signed(relative_address as i16);
        }
    }
}