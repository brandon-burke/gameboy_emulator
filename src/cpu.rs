use crate::memory::{Memory, self};
use crate::opcodes::{OPCODE_MACHINE_CYCLES, PREFIX_OPCODE_MACHINE_CYCLES};

const MACHINE_CYCLE: u8 = 4;

pub enum Conditions {
    Zero,
    Subtraction,
    HalfCarry,
    Carry,
    NotZero,
    NotCarry,
}

#[derive(Debug)]
pub enum Register {
    A,
    B,
    C,
    D,
    E,
    F,
    H,
    L,
    SP,
    PC,
}

pub enum CpuState {
    Fetch,          //Indicates the stage where we are getting the next opcode
    PreExecute,     //Indicates the stage where we are waiting a certain length of M-Cycles before excuting the instruction
    Execute,        //Indicated the stage where we are doing that actual work of the instruction
}

pub struct Cpu {
    pub a: u8,      //Accumulator
    pub b: u8,      //General Purpose register
    pub c: u8,      //General Purpose register
    pub d: u8,      //General Purpose register
    pub e: u8,      //General Purpose register
    pub f: u8,      //Flags register              Bit 7: zero flag(Z), Bit 6: add/sub flag(N), Bit 5: half carry flag(H), Bit 4: carry flag(C), Bits 3-0: NOT USED
    pub h: u8,
    pub l: u8,
    pub sp: u16,    //stack pointer
    pub pc: u16,    //program counter
    pub clk_cycle_count: u8,
    pub cpu_state: CpuState,
    pub current_opcode: u8,
    pub machine_cycles_left: u8,
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
            clk_cycle_count: 0,
            cpu_state: CpuState::Fetch,
            current_opcode: 0,
            machine_cycles_left: 0,
        }
    }

    /**
     * Represents 1 clk cycle and NOT a machine cycle. Since all
     * instructions take 1 machine cycle.
     */
    pub fn cycle(&mut self, memory: &mut Memory) {
        if self.clk_cycle_count == MACHINE_CYCLE {
            self.clk_cycle_count = 0;
        } else {
            self.clk_cycle_count += 1;
            return;
        }

        match self.cpu_state {
            CpuState::Fetch => { 
                self.current_opcode = self.fetch(memory); 
                self.machine_cycles_left = OPCODE_MACHINE_CYCLES[self.current_opcode as usize];
                self.cpu_state = CpuState::PreExecute;
            },
            CpuState::PreExecute =>  {
                self.machine_cycles_left -= 1;
                if 
            }
            CpuState::Execute =>todo!(), //Need to figure out how many machine cycles the opcode has,
        }
    }

    /**
     * Command will retrieve the next instruction
     */
    pub fn fetch(&mut self, memory: &Memory) -> u8 {
        let opcode = memory.read_byte(self.pc);
        self.pc += 1;

        return opcode;
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
        let b: u8 = (data >> 8) as u8;
        let c: u8 = data as u8;

        self.b = b;
        self.c = c;
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

    pub fn get_zero_flag(&self) -> u8 {
        return (self.f >> 7) & 0x1;
    }

    pub fn set_zero_flag(&mut self) {
        let mask = 0b10000000;
        self.f |= mask;
    }

    pub fn reset_zero_flag(&mut self) {
        let mask = 0b01111111;
        self.f &= mask;
    }

    pub fn get_add_sub_flag(&self) -> u8 {
        return (self.f >> 6) & 0x1;
    }

    pub fn set_add_sub_flag(&mut self) {
        let mask = 0b01000000;
        self.f |= mask;
    }

    pub fn reset_add_sub_flag(&mut self) {
        let mask = 0b10111111;
        self.f &= mask;
    }

    pub fn get_half_carry_flag(&self) -> u8 {
        return (self.f >> 5) & 0x1;
    }

    pub fn set_half_carry_flag(&mut self) {
        let mask = 0b00100000;
        self.f |= mask;
    }

    pub fn reset_half_carry_flag(&mut self) {
        let mask = 0b11011111;
        self.f &= mask;
    }

    pub fn get_carry_flag(&self) -> u8 {
        return (self.f >> 4) & 0x1;
    }

    pub fn set_carry_flag(&mut self) {
        let mask = 0b00010000;
        self.f |= mask;
    }

    pub fn reset_carry_flag(&mut self) {
        let mask = 0b11101111;
        self.f &= mask;
    }

    pub fn get_bit(value: u8, bit_position: u8) -> u8 {
        return (value >> bit_position) & 0x1;
    }

    pub fn get_bit_16(value: u16, bit_position: u8) -> u16 {
        return (value >> bit_position) & 0x1;
    }

    /**
     * Returns the upper byte from a 16bit number
     */
    pub fn get_upper_byte(value: u16) -> u8 {
        return (value >> 8) as u8;
    }

    /**
     * Returns the lower byte from a 16bit number
     */
    pub fn get_lower_byte(value: u16) -> u8  {
        return value as u8;
    }

    /**
     * Does absolutely nothing but consume a machine cycle and increment the pc
     * 
     * MACHINE CYCLES: 1
     * INSTRUCTION LENGTH: 1
     */
    pub fn nop(&self) {
        return;
    }

    /**
     * Loads the unsigned 16 bit value into the given registers
     * 
     * MACHINE CYCLES: 3
     * INSTRUCTION LENGTH: 3
     */
    pub fn ld_r16_u16(&mut self, memory: &Memory, high_reg: Register, low_reg: Register) {
        let lower_byte: u8 = memory.read_byte(self.pc);
        self.pc += 1;
        let upper_byte: u8 = memory.read_byte(self.pc);
        self.pc += 1;
        
        match (&high_reg, &low_reg) {
            (Register::B, Register::C) => self.set_bc((upper_byte as u16) << 8 | lower_byte as u16),
            (Register::D, Register::E) => self.set_de((upper_byte as u16) << 8 | lower_byte as u16),
            (Register::H, Register::L) => self.set_hl((upper_byte as u16) << 8 | lower_byte as u16),
            _ => panic!("Cannot use {:?} and {:?} as a 16bit register", high_reg, low_reg),
        }
    }

    /**
     * Store the value in register A into the byte pointed to by register r16.
     * 
     * MACHINE CYCLES: 4
     * INSTRUCTION LENGTH: 3
     */
    pub fn ld_r16_a(&mut self, memory: &mut Memory, high_reg: Register, low_reg: Register) {
        let address = match (&high_reg, &low_reg) {
            (Register::B, Register::C) => self.bc(),
            (Register::D, Register::E) => self.de(),
            (Register::H, Register::L) => self.hl(),
            _ => panic!("Cannot use {:?} and {:?} as a 16bit register", high_reg, low_reg),
        };
        memory.write_byte(address, self.a);
    }

    /**
     * Increment value in register r16 by 1.
     * 
     * MACHINE CYCLES: 2
     * INSTRUCTION LENGTH: 1
     */
    pub fn inc_r16(&mut self, high_reg: Register, low_reg: Register) {
        match (&high_reg, &low_reg) {
            (Register::B, Register::C) => self.set_bc(self.bc() + 1),
            (Register::D, Register::E) => self.set_de(self.de() + 1),
            (Register::H, Register::L) => self.set_hl(self.hl() + 1),
            _ => panic!("Cannot use {:?} and {:?} as a 16bit register", high_reg, low_reg),
        };    
    }

    /**
     * Increment value in register r8 by 1.
     * 
     * MACHINE CYCLES: 1
     * INSTRUCTION LENGTH: 1
     */
    pub fn inc_r8(&mut self, reg: Register) {
        let mut result: u8 = 0;
        match reg {
            Register::A => { result = self.a + 1; self.a = result; }, 
            Register::B => { result = self.b + 1; self.b = result; },
            Register::C => { result = self.c + 1; self.c = result; },
            Register::D => { result = self.d + 1; self.d = result; },
            Register::E => { result = self.e + 1; self.e = result; },
            Register::F => panic!("Cannot increment the f register"),
            Register::H => { result = self.h + 1; self.h = result; },
            Register::L => { result = self.l + 1; self.l = result; },
            _ => panic!("Cannot increment the sp and pc as this is a 8bit instruction")
        };

        self.reset_add_sub_flag();

        if result == 0 {
            self.set_zero_flag();
        } else {
            self.reset_zero_flag();
        }

        if (result & 0xF) + 1 > 0xF {
            self.set_half_carry_flag();
        } else {
            self.reset_half_carry_flag();
        }
    }

    /**
     * Decrement value in register r8 by 1.
     * 
     * MACHINE CYCLES: 1
     * INSTRUCTION LENGTH: 1
     */
    pub fn dec_r8(&mut self, reg: Register) {
        let mut result: u8 = 0;
        match reg {
            Register::A => { result = self.a - 1; self.a = result; }, 
            Register::B => { result = self.b - 1; self.b = result; },
            Register::C => { result = self.c - 1; self.c = result; },
            Register::D => { result = self.d - 1; self.d = result; },
            Register::E => { result = self.e - 1; self.e = result; },
            Register::F => panic!("Cannot decrement the f register"),
            Register::H => { result = self.h - 1; self.h = result; },
            Register::L => { result = self.l - 1; self.l = result; },
            _ => panic!("Cannot decrement the sp and pc as this is a 8bit instruction")
        };

        if result == 0 {
            self.set_zero_flag();
        } else {
            self.reset_zero_flag();
        }

        if result & 0xF == 0xF {
            self.set_half_carry_flag();
        } else {
            self.reset_half_carry_flag();
        }

        self.set_add_sub_flag();
    }

    /**
     * Load value u8 into register r8
     * 
     * MACHINE CYCLES: 2
     * INSTRUCTION LENGTH: 2
     */
    pub fn ld_r8_u8(&mut self, memory: &Memory, reg: Register) {
        match reg {
            Register::A => self.a = memory.read_byte(self.pc),
            Register::B => self.b = memory.read_byte(self.pc),
            Register::C => self.c = memory.read_byte(self.pc),
            Register::D => self.d = memory.read_byte(self.pc),
            Register::E => self.e = memory.read_byte(self.pc),
            Register::F => panic!("Cannot load into the f register"),
            Register::H => self.h = memory.read_byte(self.pc),
            Register::L => self.l = memory.read_byte(self.pc),
            _ => panic!("Cannot use sp and pc as this is a 8 bit instruction"),
        }
        self.pc += 1;
    }

    /**
     * Rotate register A left.
     * 
     * MACHINE CYCLES: 1
     * INSTRUCTION LENGTH: 1
     */
    pub fn rlca(&mut self) {
        let rotated_bit: u8 = Self::get_bit(self.a, 7);
        self.a = (self.a << 1) | rotated_bit;

        self.reset_zero_flag();
        self.reset_add_sub_flag();
        self.reset_half_carry_flag();
        if rotated_bit != 0 {
            self.set_carry_flag();
        } else {
            self.reset_carry_flag();
        }
    }

    /**
     * Store SP & $FF at address n16 and SP >> 8 at address n16 + 1.
     * 
     * MACHINE CYCLES: 5
     * INSTRUCTION LENGTH: 3
     */
    pub fn ld_u16_sp(&mut self, memory: &mut Memory) {
        let lower_byte = memory.read_byte(self.pc);
        self.pc += 1;
        let upper_byte = memory.read_byte(self.pc);
        self.pc += 1;

        let address: u16 = ((upper_byte as u16) << 8) | lower_byte as u16;

        memory.write_byte(address, Self::get_lower_byte(self.sp));
        memory.write_byte(address + 1, Self::get_upper_byte(self.sp));
    }

    /**
     * Add the value in r16 to HL.
     * 
     * MACHINE CYCLES: 2
     * INSTRUCTION LENGTH: 1 
     */
    pub fn add_hl_r16(&mut self, high_reg: Register, low_reg: Register) {
        let reg = match (&high_reg, &low_reg) {
            (Register::B, Register::C) => self.bc(),
            (Register::D, Register::E) => self.de(),
            (Register::H, Register::L) => self.hl(),
            _ => panic!("Cannot use {:?} and {:?} as a 16bit register", high_reg, low_reg),
        };  
        
        let (result, overflow) = self.hl().overflowing_add(reg);
        let half_carry_overflow = (self.hl() & 0x0FFF) + (reg & 0x0FFF) > 0x0FFF;

        self.set_hl(result);

        self.reset_add_sub_flag();

        if half_carry_overflow {
            self.set_half_carry_flag();
        } else {
            self.reset_half_carry_flag();
        }
        
        if overflow {
            self.set_carry_flag();
        } else {
            self.reset_carry_flag();
        }
    }

    /**
     * Load value in register A from the byte pointed to by register r16.
     * 
     * MACHINE CYCLES: 2
     * INSTRUCTION LENGTH: 1
     */
    pub fn ld_a_r16(&mut self, memory: &Memory, high_reg: Register, low_reg: Register) {
        let address = match (&high_reg, &low_reg) {
            (Register::B, Register::C) => self.bc(),
            (Register::D, Register::E) => self.de(),
            (Register::H, Register::L) => self.hl(),
            _ => panic!("Cannot use {:?} and {:?} as a 16bit register", high_reg, low_reg),
        };  

        self.a = memory.read_byte(address);
    }

    /**
    * Decrement value in register r16 by 1.
    * 
    * MACHINE CYCLES: 2
    * INSTRUCTION LENGTH: 1
    */
    pub fn dec_r16(&mut self, high_reg: Register, low_reg: Register) {
        let value: u16;
        match (&high_reg, &low_reg) {
            (Register::B, Register::C) => { value = self.bc() - 1; self.set_bc(value); },
            (Register::D, Register::E) => { value = self.de() - 1; self.set_de(value); },
            (Register::H, Register::L) => { value = self.hl() - 1; self.set_hl(value); },
            _ => panic!("Cannot use {:?} and {:?} as a 16bit register", high_reg, low_reg),
        };  
    }

    /**
    * Rotate register A right.
    * 
    * MACHINE CYCLE: 1
    * INSTRUCTION LENGTH: 1
    */
    pub fn rrca(&mut self) {
        let rotated_bit: u8 = Self::get_bit(self.a, 0);

        self.a = (self.a >> 1) | (rotated_bit << 7);

        self.reset_zero_flag();
        self.reset_add_sub_flag();
        self.reset_half_carry_flag();

        if rotated_bit != 0 {
            self.set_carry_flag();
        } else {
            self.reset_carry_flag();
        }
    }

    /**
     * THIS IS VERY SPECIAL NEED TO KNOW MORE ABOUT IT
     */
    pub fn stop(&mut self) {
        todo!();
    }

    /**
     * Rotate register A left through carry.
     * 
     * MACHINE CYCLES: 1
     * INSTRUCTION LENGTH: 1 
     */
    pub fn rla(&mut self) {
        let rotated_bit: u8 = Self::get_bit(self.a, 7);

        self.a = (self.a << 1) | self.get_carry_flag();

        self.reset_zero_flag();
        self.reset_add_sub_flag();
        self.reset_half_carry_flag();
        if rotated_bit != 0 {
            self.set_carry_flag();
        } else {
            self.reset_carry_flag();
        }
    }

    /**
     * Jump by i8 to a different address relative to the pc 
     * 
     * MACHINE CYCLES: 3
     * INSTRUCTION LENGTH: 2
     */
    pub fn jr_i8(&mut self, memory: &mut Memory) {
        let relative_address: i8 = memory.read_byte(self.pc) as i8;
        self.pc += 1;
        self.pc = self.pc.wrapping_add_signed(relative_address as i16);
    }

    /**
     * Rotate register A right through carry.
     * 
     * MACHINE CYCLES: 1
     * INSTRUCTION LENGTH: 1
     */
    pub fn rra(&mut self) {
        let rotated_bit: u8 = Self::get_bit(self.a, 0);

        self.a = (self.a >> 1) | (self.get_carry_flag() << 7);

        self.reset_zero_flag();
        self.reset_add_sub_flag();
        self.reset_half_carry_flag();

        if rotated_bit != 0 {
            self.set_carry_flag();
        } else {
            self.reset_carry_flag();
        }
    }

    /**
     * Relative Jump by i8 if condition cc is met.
     * 
     * MACHINE CYCLES: 3 IF TAKEN/ 2 IF NOT TAKEN
     * INSTRUCTION LENGTH: 2
     */
    pub fn jr_cc_i8(&mut self, memory: &Memory, mut condition: u8) {
        condition &= 0x1;   //doing this so we only see the first bit

        if condition != 0 {
            let relative_address = memory.read_byte(self.pc);
            self.pc += 1;
            self.pc = self.pc.wrapping_add_signed(relative_address as i16);
        } else {
            self.pc += 1;
        }
    }

    /**
     * Store value in register A into the byte pointed by HL and increment HL afterwards.
     * 
     * MACHINE CYCLES: 2
     * INSTRUCTION LENGTH: 1
     */
    pub fn ld_hli_a(&mut self, memory: &mut Memory) {
        memory.write_byte(self.hl(), self.a);
        self.set_hl(self.hl() + 1);
    }

    /**
     * Decimal Adjust Accumulator to get a correct BCD representation after an arithmetic instruction.
     * 
     * MACHINE CYCLES: 1
     * INSTRUCTION LENGTH: 1
     */
    pub fn daa(&mut self) {
        if self.get_add_sub_flag() == 0 {
            if self.get_carry_flag() != 0 || self.a > 0x99 {
                self.a += 0x60;
                self.set_carry_flag();
            }
            if self.get_half_carry_flag() != 0 || self.a & 0x0F > 0x09 {
                self.a += 0x6;
            }
        }

        if self.a == 0 {
            self.set_zero_flag();
        } else {
            self.reset_zero_flag();
        }

        self.reset_half_carry_flag();
    }

    /**
     * Load value into register A from the byte pointed by HL and increment HL afterwards.
     * 
     * MACHINE CYCLES: 2
     * INSTRUCTION LENGTH: 1
     */
    pub fn ld_a_hli(&mut self, memory: &Memory) {
        let hl_data = memory.read_byte(self.hl());
        self.a = hl_data;
        self.set_hl(self.hl() + 1);
    }

    /**
     * Store the complement of the A register into the A register
     * 
     * MACHINE CYCLES: 1
     * INSTRUCTION LENGTH: 1
     */
    pub fn cpl(&mut self) {
        self.a = !self.a;

        self.set_add_sub_flag();
        self.set_half_carry_flag();
    }

    /**
     * Load value n16 into register SP.
     * 
     * MACHINE CYCLES: 3
     * INSTRUCTION LENGTH: 3
     */
    pub fn ld_sp_u16(&mut self, memory: &Memory) {
        let lower_byte = memory.read_byte(self.pc);
        self.pc += 1;
        let upper_byte = memory.read_byte(self.pc);
        self.pc += 1;
        
        self.sp = ((upper_byte as u16) << 8) | lower_byte as u16;
    }

    /**
     * Store value in register A into the byte pointed by HL and decrement HL afterwards.
     * 
     * MACHINE CYCLES: 2
     * INSTRUCTION LENGTH: 1
     */
    pub fn ld_hld_a(&mut self, memory: &mut Memory) {
        memory.write_byte(self.hl(), self.a);
        self.set_hl(self.hl() - 1);
    }

    /**
     * Increment value in register SP by 1.
     * 
     * MACHINE CYCLES: 2
     * INSTRUCTION LENGTH: 1
     */
    pub fn inc_sp(&mut self) {
        self.sp += 1;
    }

    /**
     * Increment the byte pointed to by HL by 1.
     * 
     * MACHINE CYCLES: 3
     * INSTRUCTION LENGTH: 1
     */
    pub fn inc_hl(&mut self, memory: &mut Memory) {
        let mut hl_data = memory.read_byte(self.hl());
        hl_data += 1;
        memory.write_byte(self.hl(), hl_data);

        self.reset_add_sub_flag();

        if hl_data == 0 {
            self.set_zero_flag();
        } else {
            self.reset_zero_flag();
        }

        if (hl_data & 0xF) + 1 > 0xF {
            self.set_half_carry_flag();
        } else {
            self.reset_half_carry_flag();
        }
    }

    /**
    * Decrement the byte pointed to by HL by 1.
    * 
    * MACHINE CYCLES: 3
    * INSTRUCTION LENGTH: 1
    */
    pub fn dec_hl(&mut self, memory: &mut Memory) {
        let mut hl_data = memory.read_byte(self.hl());
        hl_data -= 1;
        memory.write_byte(self.hl(), hl_data);

        if hl_data == 0 {
            self.set_zero_flag();
        } else {
            self.reset_zero_flag();
        }

        if hl_data & 0xF == 0xF {
            self.set_half_carry_flag();
        } else {
            self.reset_half_carry_flag();
        }

        self.set_add_sub_flag();
    }

    /**
     * Store value n8 into the byte pointed to by register HL.
     * 
     * MACHINE CYCLES: 3
     * INSTRUCTION LENGTH: 2
     */
    pub fn ld_hl_u8(&mut self, memory: &mut Memory) {
        let immediate = memory.read_byte(self.pc);
        self.pc += 1;
        memory.write_byte(self.hl(), immediate);
    }

    /**
     * Set the carry flag
     * 
     * MACHINE CYCLE: 1
     * INSTRUCTION LENGTH: 1
     */
    pub fn scf(&mut self) {
        self.set_carry_flag();
        self.reset_add_sub_flag();
        self.reset_half_carry_flag();
    }

    /**
     * Add the value in sp to hl
     * 
     * MACHINE CYCLES: 2
     * INSTRUCTION LENGTH: 1
     */
    pub fn add_hl_sp(&mut self) {
        let (result, overflow) = self.hl().overflowing_add(self.sp);
        let half_carry_overflow = (self.hl() & 0x0FFF) + (self.sp & 0x0FFF) > 0x0FFF;

        self.set_hl(result);

        self.reset_add_sub_flag();

        if half_carry_overflow {
            self.set_half_carry_flag();
        } else {
            self.reset_half_carry_flag();
        }

        if overflow {
            self.set_carry_flag();
        } else {
            self.reset_carry_flag();
        }
    }

    /**
     * Load value into register A from the byte pointed by HL and decrement HL afterwards.
     * 
     * MACHINE CYCLES: 2
     * INSTRUCTION LENGTH: 1
     */
    pub fn ld_a_hld(&mut self, memory: &Memory) {
        let hl_data = memory.read_byte(self.hl());
        self.a = hl_data;
        self.set_hl(self.hl() - 1);
    }

    /**
    * Decrement value in register SP by 1.
    * 
    * MACHINE CYCLES: 2
    * INSTRUCTION LENGTH: 1
    */
    pub fn dec_sp(&mut self) {
        self.sp -= 1;
    }

    /**
    * Complement the carry flag
    */
    pub fn ccf(&mut self) {
        self.reset_add_sub_flag();
        self.reset_half_carry_flag();

        if self.get_carry_flag() == 0 {
            self.set_carry_flag();
        } else {
            self.reset_carry_flag();
        }
    }

    /**
     * Load (copy) value in register on the right into register on the left.
     * 
     * MACHINE CYCLES: 1
     * INSTRUCTION LENGTH: 1
     */
    pub fn ld_r8_r8(&mut self, reg_left: Register, reg_right: Register) {
        let value = match reg_right {
            Register::A => self.a,
            Register::B => self.b,
            Register::C => self.c,
            Register::D => self.d,
            Register::E => self.e,
            Register::F => panic!("Cannot use the f register in (ld_r8_r8)"),
            Register::H => self.h,
            Register::L => self.l,
            _ => panic!("Cannot use sp or pc register this is a 8 bit operation (ld_r8_r8)")
        };

        match reg_left {
            Register::A => self.a = value,
            Register::B => self.b = value,
            Register::C => self.c = value,
            Register::D => self.d = value,
            Register::E => self.e = value,
            Register::F => panic!("Cannot use the f register in (ld_r8_r8)"),
            Register::H => self.h = value,
            Register::L => self.l = value,
            _ => panic!("Cannot use sp or pc register this is a 8 bit operation (ld_r8_r8)")
        }
    }
    
    /**
     * Load value into register r8 from the byte pointed to by register HL.
     * 
     * MACHINE CYCLES: 2
     * INSTRUCTION LENGTH: 1
     */
    pub fn ld_r8_hl(&mut self, memory: &Memory, reg: Register) {
        let hl_data = memory.read_byte(self.hl());

        match reg {
            Register::A => self.a = hl_data,
            Register::B => self.b = hl_data,
            Register::C => self.c = hl_data,
            Register::D => self.d = hl_data,
            Register::E => self.e = hl_data,
            Register::F => panic!("Cannot use f register (ld_r8_hl)"),
            Register::H => self.f = hl_data,
            Register::L => self.l = hl_data,
            _ => panic!("Cannot use sp or pc as this is a 8 bit operation (ld_r8_hl)"),
        }
    }

    /**
     * Store value in register r8 into the byte pointed to by register HL.
     * 
     * MACHINE CYCLES: 2
     * INSTRUCTION LENGTH: 1
     */
    pub fn ld_hl_r8(&mut self, memory: &mut Memory, reg: Register) {
        let value = match reg {
            Register::A => self.a,
            Register::B => self.b,
            Register::C => self.c,
            Register::D => self.d,
            Register::E => self.e,
            Register::F => panic!("Cannot use f register (ld_hl_r8)"),
            Register::H => self.f,
            Register::L => self.l,
            _ => panic!("Cannot use sp or pc as this is a 8 bit operation (ld_hl_r8)"),
        };

        memory.write_byte(self.hl(), value);
    }

    /**
     * Enter CPU low-power consumption mode until an interrupt occurs. 
     * The exact behavior of this instruction depends on the state of the IME flag.
     * 
     * MACHINE CYCLES: -
     * INSTRUCTION LENGTH: 1
     */
    pub fn halt(&mut self) {
        todo!();
    }

    /**
     * Add the value in r8 to A.
     * 
     * MACHINE CYCLES: 1
     * INSTRUCTION LENGTH: 1 
     */
    pub fn add_a_r8(&mut self, reg: Register) {
        let value = match reg {
            Register::A => self.a,
            Register::B => self.b,
            Register::C => self.c,
            Register::D => self.d,
            Register::E => self.e,
            Register::F => panic!("Cannot use f register in (add_a_r8)"),
            Register::H => self.h,
            Register::L => self.l,
            _ => panic!("cannot use sp or pc in a this 8 bit operation (add_a_r8)")
        };

        let (result, overflow) = self.a.overflowing_add(value);
        let half_carry_overflow = (self.a & 0xF) + (value & 0xF) > 0xF;

        self.a = result;
        self.arithmetic_8bit_flag_update(result, half_carry_overflow, overflow);
    }

    /**
    * Add the byte pointed to by HL to A.
    * 
    * MACHINE CYCLES: 2
    * INSTRUCTION LENGTH: 1 
    */
    pub fn add_a_hl(&mut self, memory: &Memory) {
        let hl_data = memory.read_byte(self.hl());
        let (result, overflow) = self.a.overflowing_add(hl_data);
        let half_carry_overflow = (self.a & 0xF) + (hl_data & 0xF) > 0xF;

        self.a = result;
        self.arithmetic_8bit_flag_update(result, half_carry_overflow, overflow);
    }

    /**
     * Add the value in r8 plus the carry flag to A.
     * 
     * MACHINE CYCLES: 1
     * INSTRUCTION LENGTH: 1 
     */
    pub fn adc_a_r8(&mut self, reg: Register) {
        let value = match reg {
            Register::A => self.a,
            Register::B => self.b,
            Register::C => self.c,
            Register::D => self.d,
            Register::E => self.e,
            Register::F => panic!("Cannot use f register in (adc_a_r8)"),
            Register::H => self.h,
            Register::L => self.l,
            _ => panic!("cannot use sp or pc in a this 8 bit operation (adc_a_r8)")
        };

        let (partial_result, first_overflow) = self.a.overflowing_add(value);
        let (result, second_overflow) = partial_result.overflowing_add(self.get_carry_flag());
        let half_carry_overflow = (self.a & 0xF) + (value & 0xF) + self.get_carry_flag() > 0xF;

        self.a = result;
        self.arithmetic_8bit_flag_update(result, half_carry_overflow, first_overflow || second_overflow);
    }

    
    /**
     * Add the byte pointed to by HL plus the carry flag to A.
     * 
     * MACHINE CYCLES: 2
     * INSTRUCTION LENGTH: 1 
     */
    pub fn adc_a_hl(&mut self, memory: &Memory) {
        let hl_data = memory.read_byte(self.hl());
        let (partial_result, first_overflow) = self.a.overflowing_add(hl_data);
        let (result, second_overflow) = partial_result.overflowing_add(self.get_carry_flag());
        let half_carry_overflow = (self.a & 0xF) + (hl_data & 0xF) + self.get_carry_flag() > 0xF;

        self.a = result;
        self.arithmetic_8bit_flag_update(result, half_carry_overflow, first_overflow || second_overflow);
    }

    /**
    * Subtract the value in r8 from A.
    * 
    * MACHINE CYCLES: 1
    * INSTRUCTION LENGTH: 1
    */
    pub fn sub_a_r8(&mut self, reg: Register) {
        let reg_value = match reg {
            Register::A => self.a,
            Register::B => self.b,
            Register::C => self.c,
            Register::D => self.d,
            Register::E => self.e,
            Register::F => panic!("Cannot use f register in (sub_a_r8)"),
            Register::H => self.h,
            Register::L => self.l,
            _ => panic!("cannot use sp or pc in a this 8 bit operation (sub_a_r8)")
        };

        let result = self.a - reg_value;
        self.a = result;
        
        self.set_add_sub_flag();

        if result == 0 {
            self.set_zero_flag();
        } else {
            self.reset_zero_flag();
        }

        if reg_value > self.a & 0xF {
            self.set_half_carry_flag();
        } else {
            self.reset_half_carry_flag();
        }

        if reg_value > self.a {
            self.set_carry_flag();
        } else {
            self.reset_carry_flag();
        }
    }

    /**
     * Subtract the byte pointed to by HL from A.
     * 
     * MACHINE CYCLES: 2
     * INSTRUCTION LENGTH: 1
     */
    pub fn sub_a_hl(&mut self, memory: &Memory) {
        let hl_data = memory.read_byte(self.hl());
        let result = self.a - hl_data;

        self.a = result;

        self.set_add_sub_flag();

        if result == 0 {
            self.set_zero_flag();
        } else {
            self.reset_zero_flag();
        }

        if hl_data > self.a & 0xF {
            self.set_half_carry_flag();
        } else {
            self.reset_half_carry_flag();
        }

        if hl_data > self.a {
            self.set_carry_flag();
        } else {
            self.reset_carry_flag();
        }
    }

    /**
     * Subtract the value in r8 and the carry flag from A.
     * 
     * MACHINE CYCLES: 1
     * INSTRUCTION LENGTH: 1
     */
    pub fn sbc_a_r8(&mut self, reg: Register) {
        let reg_value = match reg {
            Register::A => self.a,
            Register::B => self.b,
            Register::C => self.c,
            Register::D => self.d,
            Register::E => self.e,
            Register::F => panic!("Cannot use f register in (sbc_a_r8)"),
            Register::H => self.h,
            Register::L => self.l,
            _ => panic!("cannot use sp or pc in a this 8 bit operation (sbc_a_r8)")
        };

        let result = self.a - reg_value - self.get_carry_flag();
        self.a = result;

        if result == 0 {
            self.set_zero_flag();
        } else {
            self.reset_zero_flag();
        }

        self.set_add_sub_flag();

        if (reg_value - self.get_carry_flag()) > self.a & 0xF {
            self.set_half_carry_flag();
        } else {
            self.reset_half_carry_flag();
        }

        if reg_value + self.get_carry_flag() > self.a {
            self.set_carry_flag();
        } else {
            self.reset_carry_flag();
        }
    } 

    /**
     * Subtract the byte pointed to by HL and the carry flag from A.
     * 
     * MACHINE CYCLES: 2
     * INSTRUCTION LENGTH: 1
     */
    pub fn sbc_a_hl(&mut self, memory: &Memory) {
        let hl_data = memory.read_byte(self.hl());
        let result = self.a - hl_data - self.get_carry_flag();

        self.a = result;

        self.set_add_sub_flag();

        if result == 0 {
            self.set_zero_flag();
        } else {
            self.reset_zero_flag();
        }

        if (hl_data - self.get_carry_flag()) > self.a & 0xF {
            self.set_half_carry_flag();
        } else {
            self.reset_half_carry_flag();
        }

        if hl_data > self.a {
            self.set_carry_flag();
        } else {
            self.reset_carry_flag();
        }
    }

    /**
     * Bitwise AND between the value in r8 and A.
     * 
     * MACHINE CYCLES: 1
     * INSTRUCTION LENGTH: 1
     */
    pub fn and_a_r8(&mut self, reg: Register) {
        let reg_value = match reg {
            Register::A => self.a,
            Register::B => self.b,
            Register::C => self.c,
            Register::D => self.d,
            Register::E => self.e,
            Register::F => panic!("Cannot use f register in (and_a_r8)"),
            Register::H => self.h,
            Register::L => self.l,
            _ => panic!("cannot use sp or pc in a this 8 bit operation (and_a_r8)")
        };

        let result = reg_value & self.a;
        self.a = result;

        if result == 0 {
            self.set_zero_flag();
        } else {
            self.reset_zero_flag();
        }

        self.reset_add_sub_flag();
        self.set_half_carry_flag();
        self.reset_carry_flag();
    }

    /**
     * Bitwise AND between the byte pointed to by HL and A.
     * 
     * MACHINE CYCLE: 2
     * INSTRUCTION LENGTH: 1
     */
    pub fn and_a_hl(&mut self, memory: &Memory) {
        let hl_data = memory.read_byte(self.hl());
        let result = hl_data & self.a;

        self.a = result;

        if result == 0 {
            self.set_zero_flag();
        } else {
            self.reset_zero_flag();
        }

        self.reset_add_sub_flag();
        self.set_half_carry_flag();
        self.reset_carry_flag();
    }

    /**
     * Bitwise XOR between the value in r8 and A.
     * 
     * MACHINE CYCLE: 1
     * INSTRUCTION LENGTH: 1
     */
    pub fn xor_a_r8(&mut self, reg: Register) {
        let reg_value = match reg {
            Register::A => self.a,
            Register::B => self.b,
            Register::C => self.c,
            Register::D => self.d,
            Register::E => self.e,
            Register::F => panic!("Cannot use f register in (xor_a_r8)"),
            Register::H => self.h,
            Register::L => self.l,
            _ => panic!("cannot use sp or pc in a this 8 bit operation (xor_a_r8)")
        };

        let result = reg_value ^ self.a;
        self.a = result;

        if result == 0 {
            self.set_zero_flag();
        } else {
            self.reset_zero_flag();
        }
        self.reset_add_sub_flag();
        self.reset_half_carry_flag();
        self.reset_carry_flag();
    }

    /**
     * Bitwise XOR between the byte pointed to by HL and A.
     * 
     * MACHINE CYCLES: 2
     * INSTRUCTION LENGTH: 1
     */
    pub fn xor_a_hl(&mut self, memory: &Memory) {
        let hl_data = memory.read_byte(self.hl());
        let result = hl_data ^ self.a;

        self.a = result;

        if result == 0 {
            self.set_zero_flag();
        } else {
            self.reset_zero_flag();
        }
        self.reset_add_sub_flag();
        self.reset_half_carry_flag();
        self.reset_carry_flag();
    } 

    /**
     * Bitwise OR between the value in r8 and A. 
     * 
     * MACHINE CYCLES: 1
     * INSTRUCTION LENGTH: 1
     */
    pub fn or_a_r8(&mut self, reg: Register) {
        let reg_value = match reg {
            Register::A => self.a,
            Register::B => self.b,
            Register::C => self.c,
            Register::D => self.d,
            Register::E => self.e,
            Register::F => panic!("Cannot use f register in (or_a_r8)"),
            Register::H => self.h,
            Register::L => self.l,
            _ => panic!("cannot use sp or pc in a this 8 bit operation (or_a_r8)")
        };

        let result = reg_value | self.a;
        self.a = result;

        if result == 0 {
            self.set_zero_flag();
        } else {
            self.reset_zero_flag();
        }
        self.reset_add_sub_flag();
        self.reset_half_carry_flag();
        self.reset_carry_flag();
    }

    /**
    * Bitwise OR between the byte pointed to by HL and A.
    * 
    * MACHINE CYCLES: 2
    * INSTRUCTION LENGTH: 1
    */
    pub fn or_a_hl(&mut self, memory: &Memory) {
        let hl_data = memory.read_byte(self.hl());
        let result = hl_data | self.a;

        self.a = result;

        if result == 0 {
            self.set_zero_flag();
        } else {
            self.reset_zero_flag();
        }
        self.reset_add_sub_flag();
        self.reset_half_carry_flag();
        self.reset_carry_flag();
    } 

    /**
     * Subtract the value in r8 from A and set flags accordingly, but don't store the result. This is useful for ComParing values.
     * 
     * MACHINE CYCLES: 1
     * INSTRUCTION LENGTH: 1    
     */
    pub fn cp_a_r8(&mut self, reg: Register) {
        let reg_value = match reg {
            Register::A => self.a,
            Register::B => self.b,
            Register::C => self.c,
            Register::D => self.d,
            Register::E => self.e,
            Register::F => panic!("Cannot use f register in (cp_a_r8)"),
            Register::H => self.h,
            Register::L => self.l,
            _ => panic!("cannot use sp or pc in a this 8 bit operation (cp_a_r8)")
        };

        let result = self.a - reg_value;

        if result == 0 {
            self.set_zero_flag();
        } else {
            self.reset_zero_flag();
        }

        self.set_add_sub_flag();

        if reg_value > self.a & 0xF {
            self.set_half_carry_flag();
        } else {
            self.reset_half_carry_flag();
        }

        if reg_value > self.a {
            self.set_carry_flag();
        } else {
            self.reset_carry_flag();
        }
    }

    /**
     * Subtract the byte pointed to by HL from A and set flags accordingly, but don't store the result.
     * 
     * MACHINE CYCLES: 2
     * INSTRUCTION LENGTH: 1
     */
    pub fn cp_a_hl(&mut self, memory: &Memory) {
        let hl_data = memory.read_byte(self.hl());
        let result = self.a - hl_data;

        if result == 0 {
            self.set_zero_flag();
        } else {
            self.reset_zero_flag();
        }

        self.set_add_sub_flag();

        if hl_data > self.a & 0xF {
            self.set_half_carry_flag();
        } else {
            self.reset_half_carry_flag();
        }

        if hl_data > self.a {
            self.set_carry_flag();
        } else {
            self.reset_carry_flag();
        }
    }

    /**
     * Return from subroutine if condition cc is met.
     * 
     * MACHINE CYCLES: 5 IF TAKEN/ 2 IF NOT TAKEN
     * INSTRUCTION LENGTH: 1
     */
    pub fn ret_cc(&mut self, memory: &Memory, mut condition: u8) {
        condition &= 0x1;   //doing this so we only see the first bit

        if condition != 0 {
            let lower_byte = memory.read_byte(self.sp);
            self.sp += 1;
            let upper_byte = memory.read_byte(self.sp);
            self.sp += 1;

            self.pc = ((upper_byte as u16) << 8) | lower_byte as u16;
        } 
    }

    /**
     * Pop to register r16 from the stack.
     * 
     * MACHINE CYCLES: 3
     * INSTRUCTION LENGTH: 1
     */
    pub fn pop(&mut self, memory: &mut Memory, high_reg: Register, low_reg: Register) {
        let lower_byte = memory.read_byte(self.sp);
        self.sp += 1;
        let upper_byte = memory.read_byte(self.sp);
        self.sp += 1;

        match (&high_reg, &low_reg) {
            (Register::B, Register::C) => self.set_bc((upper_byte as u16) << 8 | lower_byte as u16),
            (Register::D, Register::E) => self.set_de((upper_byte as u16) << 8 | lower_byte as u16),
            (Register::H, Register::L) => self.set_hl((upper_byte as u16) << 8 | lower_byte as u16),
            _ => panic!("Cannot use {:?} and {:?} as a 16bit register", high_reg, low_reg),
        }
    }

    /**
    * Jump to address u16 if the condition is met
    * 
    * MACHINE CYCLES: 4
    * INSTRUCTION LENGTH: 3
    */
    pub fn jp_cc_u16(&mut self, memory: &Memory, mut condition: u8) {
        let lower_byte = memory.read_byte(self.pc);
        self.pc += 1;
        let upper_byte = memory.read_byte(self.pc);
        self.pc += 1;

        condition &= 0x1;   //doing this so we only see the first bit
        if condition != 0 {
            self.pc = ((upper_byte as u16) << 8) | lower_byte as u16;
        }
    }
    /**
    * Jump to address u16
    * 
    * MACHINE CYCLES: 4
    * INSTRUCTION LENGTH: 3
    */
    pub fn jp_u16(&mut self, memory: &Memory) {
        let lower_byte = memory.read_byte(self.pc);
        self.pc += 1;
        let upper_byte = memory.read_byte(self.pc);
        self.pc += 1;

        self.pc = ((upper_byte as u16) << 8) | lower_byte as u16;
    }

    /**
     * Call address u16 if condition cc is met.
     * 
     * MACHINE CYCLES: 6 IF TAKEN/ 3 IF NOT TAKEN
     * INSTRUCTION LENGTH: 3
     */
    pub fn call_cc_u16(&mut self, memory: &mut Memory, mut condition: u8) {
        let lower_byte = memory.read_byte(self.pc);
        self.pc += 1;
        let upper_byte = memory.read_byte(self.pc);
        self.pc += 1;

        condition &= 0x1;   //doing this so we only see the first bit
        if condition != 0 {
            self.sp -= 1;
            memory.write_byte(self.sp, Self::get_upper_byte(self.pc));
            self.sp -= 1;
            memory.write_byte(self.sp, Self::get_lower_byte(self.pc));

            self.pc = ((upper_byte as u16) << 8) | lower_byte as u16;      
        }
    }

    /**
     * Push register r16 into the stack
     * 
     * MACHINE CYCLES: 4
     * INSTRUCTION LENGTH: 1
     */
    pub fn push_r16(&mut self, memory: &mut Memory, high_reg: Register, low_reg: Register) {
        let mut lower_byte: u8 = 0;
        let mut upper_byte: u8 = 0;
        match (&high_reg, &low_reg) {
            (Register::A, Register::F) => { upper_byte = Self::get_upper_byte(self.af()); lower_byte = Self::get_lower_byte(self.af()); },
            (Register::B, Register::C) => { upper_byte = Self::get_upper_byte(self.bc()); lower_byte = Self::get_lower_byte(self.bc()); },
            (Register::D, Register::E) => { upper_byte = Self::get_upper_byte(self.de()); lower_byte = Self::get_lower_byte(self.de()); },
            (Register::H, Register::L) => { upper_byte = Self::get_upper_byte(self.hl()); lower_byte = Self::get_lower_byte(self.hl()); },
            _ => panic!("Cannot use {:?} and {:?} as a 16bit register", high_reg, low_reg),
        }

        self.sp -= 1;
        memory.write_byte(self.sp, upper_byte);
        self.sp -= 1;
        memory.write_byte(self.sp, lower_byte);
    }
    
    /**
     * Add the value u8 to A.
     * 
     * MACHINE CYCLE: 2
     * INSTRUCTION LENGTH: 2
     */
    pub fn add_a_u8(&mut self, memory: &Memory) {
        let value = memory.read_byte(self.pc);
        self.pc += 1;

        let (result, overflow) = self.a.overflowing_add(value);
        let half_carry_overflow = (self.a & 0xF) + (value & 0xF) > 0xF;

        self.a = result;
        self.arithmetic_8bit_flag_update(result, half_carry_overflow, overflow);
    }

    /**
     * Call address vec. This is a shorter and faster equivalent to CALL for suitable values of vec.
     * 
     * MACHINE CYCLE: 4
     * INSTRUCTION LENGTH: 1
     */
    pub fn rst_vec(&mut self, memory: &mut Memory, rst_address: u16) {
        self.sp -= 1;
        memory.write_byte(self.sp, Self::get_upper_byte(self.pc));
        self.sp -= 1;
        memory.write_byte(self.sp, Self::get_lower_byte(self.pc));

        self.pc = rst_address;
    }

    /**
     * Return from subroutine. This is basically a POP PC (if such an instruction existed). See POP r16 for an explanation of how POP works.
     * 
     * MACHINE CYCLES: 4
     * INSTRUCTION LENGTH: 1
     */
    pub fn ret(&mut self, memory: &Memory) {
        let lower_byte = memory.read_byte(self.sp);
        self.sp += 1;
        let upper_byte = memory.read_byte(self.sp);
        self.sp += 1;

        self.pc = ((upper_byte as u16) << 8) | lower_byte as u16;      
    }

    /**
    * Call address n16. This pushes the address of the instruction after the CALL on the stack, 
    * such that RET can pop it later; then, it executes an implicit JP n16.
    * 
    * MACHINE CYCLES: 6
    * INSTRUCTION LENGTH: 3
    */
    pub fn call_u16(&mut self, memory: &mut Memory) {
        let lower_byte = memory.read_byte(self.pc);
        self.pc += 1;
        let upper_byte = memory.read_byte(self.pc);
        self.pc += 1;

        self.sp -= 1;
        memory.write_byte(self.sp, Self::get_upper_byte(self.pc));
        self.sp -= 1;
        memory.write_byte(self.sp, Self::get_lower_byte(self.pc));

        self.pc = ((upper_byte as u16) << 8) | lower_byte as u16;      
    }

    /**
     * Add the value u8 plus the carry flag to A.
     * 
     * MACHINE CYCLES: 2
     * INSTRUCTION LENGTH: 2 
     */
    pub fn adc_a_u8(&mut self, memory: &Memory) {
        let immediate = memory.read_byte(self.pc);
        self.pc += 1;

        let (partial_result, first_overflow) = self.a.overflowing_add(immediate);
        let (result, second_overflow) = partial_result.overflowing_add(self.get_carry_flag());
        let half_carry_overflow = (self.a & 0xF) + (immediate & 0xF) + self.get_carry_flag() > 0xF;

        self.a = result;
        self.arithmetic_8bit_flag_update(result, half_carry_overflow, first_overflow || second_overflow);
    }

    /**
    * Subtract the value u8 from A.
    * 
    * MACHINE CYCLES: 2
    * INSTRUCTION LENGTH: 2
    */
    pub fn sub_a_u8(&mut self, memory: &Memory) {
        let value = memory.read_byte(self.pc);
        self.pc += 1;

        let result = self.a - value;
        self.a = result;
        
        self.set_add_sub_flag();

        if result == 0 {
            self.set_zero_flag();
        } else {
            self.reset_zero_flag();
        }

        if value > self.a & 0xF {
            self.set_half_carry_flag();
        } else {
            self.reset_half_carry_flag();
        }

        if value > self.a {
            self.set_carry_flag();
        } else {
            self.reset_carry_flag();
        }
    }

    /**
     * Return from subroutine and enable interrupts. 
     * This is basically equivalent to executing EI then RET, meaning that IME is set right after this instruction.
     * 
     * MACHINE CYCLES: 4
     * INSTRUCTION LENGTH: 1
     */
    pub fn reti(&mut self, memory: &mut Memory) {
        let lower_byte = memory.read_byte(self.sp);
        self.sp += 1;
        let upper_byte = memory.read_byte(self.sp);
        self.sp += 1;

        self.pc = ((upper_byte as u16) << 8) | lower_byte as u16;    
        memory.write_byte(0xFFFF, 0x1);
    }
    
    /**
     * Store value in register A into the byte at address $FF00+C.
     * 
     * MACHINE CYCLES: 3
     * INSTRUCTION LENGTH: 2
     */
    pub fn ldh_u8_a(&mut self, memory: &mut Memory) {
        let offset = memory.read_byte(self.pc);
        self.pc += 1;

        let address: u16 = 0xFF00 + offset as u16;
        memory.write_byte(address, self.a);
    } 

    /**
    * Store value in register A into the byte at address $FF00+C.
    * 
    * MACHINE CYCLES: 2
    * INSTRUCTION LENGTH: 1
    */
    pub fn ldh_c_a(&mut self, memory: &mut Memory) {
        let address: u16 = 0xFF00 + self.c as u16;
        memory.write_byte(address, self.a);
    } 

    /**
     * Bitwise AND between u8 immediate and 8-bit A register
     * 
     * MACHINE CYCLES: 2
     * INSTRUCTION CYCLES: 2
     */
    pub fn and_a_u8(&mut self, memory: &Memory) {
        let immediate = memory.read_byte(self.pc);
        self.pc += 1;

        let result = self.a & immediate;
        self.a = result;

        self.reset_add_sub_flag();
        self.set_half_carry_flag();
        self.reset_carry_flag();

        if result == 0 {
            self.set_zero_flag();
        } else {
            self.reset_zero_flag();
        }
    }  

    /**
     * Add the 8-bit signed value i8 to 16-bit SP register.
     * 
     * MACHINE CYCLES: 4
     * INSTRUCTION LENGTH: 2
     */
    pub fn add_sp_i8(&mut self, memory: &Memory) {
        let immediate = memory.read_byte(self.pc);
        self.pc += 1;

        let result: u16 = self.sp.wrapping_add_signed(immediate as i16);
        let half_carry_overflow = (self.sp & 0xF) + (immediate as u16 & 0xF) > 0xF;
        let carry_overflow = (self.sp & 0xFF) + (immediate as u16 & 0xFF) > 0xFF;
        
        self.sp = result;

        //N = 0
        self.reset_add_sub_flag();

        //Z
        self.reset_zero_flag();

        //H
        if half_carry_overflow {
            self.set_half_carry_flag();
        } else {
            self.reset_half_carry_flag();
        }

        //C
        if carry_overflow {
            self.set_carry_flag();
        } else {
            self.reset_carry_flag();
        }
    }

    /**
     * Jump to address in HL; effectively, load PC with value in register HL.
     * 
     * MACHINE CYCLES: 1
     * INSTRUCTION LENGTH: 1
     */
    pub fn jp_hl(&mut self) {
        self.pc = self.hl();
    }

    /**
     * Store value in register A into the byte at address u16.
     * 
     * MACHINE CYCLES: 4
     * INSTRUCTION LENGTH: 3
     */
    pub fn ld_u16_a(&mut self, memory: &mut Memory) {
        let lower_byte = memory.read_byte(self.pc);
        self.pc += 1;
        let upper_byte = memory.read_byte(self.pc);
        self.pc += 1;

        let address: u16 = (upper_byte as u16) << 8 | lower_byte as u16;
        memory.write_byte(address, self.a);
    }

    /**
     * Bitwise XOR between the value in n8 and A.
     * 
     * MACHINE CYCLES: 2
     * INSTRUCTION LENGTH: 2
     */
    pub fn xor_a_u8(&mut self, memory: &Memory) {
        let immediate = memory.read_byte(self.pc);
        self.pc += 1;

        let result = immediate ^ self.a;
        self.a = result;
    }

    /**
     * Load value in register A from the byte at address n16, provided the address is between $FF00 and $FFFF.
     * 
     * MACHINE CYCLES: 3
     * INSTRUCTION LENGTH: 2
     */
    pub fn ldh_a_u8(&mut self, memory: &Memory) {
        let offset = memory.read_byte(self.pc);
        self.pc += 1;

        let address: u16 = 0xFF00 + offset as u16;
        let result = memory.read_byte(address);
        self.a = result
    }

    /**
     * Load value in register A from the byte at address $FF00+C.
     * 
     * MACHINE CYCLES: 2
     * INSTRUCTION LENGTH: 1
     */
    pub fn ldh_a_c(&mut self, memory: &Memory) {
        let address: u16 = 0xFF00 + self.c as u16;
        let result = memory.read_byte(address);
        self.a = result;
    }

    /**
     * Disable Interrupts by clearing the IME flag.
     * 
     * MACHINE CYCLES: 1
     * INSTRUCTION LENGTH: 1
     */
    pub fn di(&mut self, memory: &mut Memory) {
        memory.write_byte(0xFFFF, 0);
    }

    /**
     * Store into A the bitwise OR of n8 and A.
     * 
     * MACHINE CYCLES: 2
     * INSTRUCTION LENGTH: 2
     */
    pub fn or_a_u8(&mut self, memory: &Memory) {
        let immediate = memory.read_byte(self.pc);
        self.pc += 1;

        let result = immediate | self.a;
        self.a = result;

        if result == 0 {
            self.set_zero_flag();
        } else {
            self.reset_zero_flag();
        }
        self.reset_add_sub_flag();
        self.reset_half_carry_flag();
        self.reset_carry_flag();
    }

    /**
     * Add the signed value e8 to SP and store the result in HL. CARRY HERE IS SO WRONG
     * 
     * MACHINE CYCLES: 3
     * INSTRUCTION LENGTH: 2
     */
    pub fn ld_hl_sp_i8(&mut self, memory: &Memory) {
        let offset = memory.read_byte(self.pc);
        self.pc += 1;

        let result = self.sp.wrapping_add_signed(offset as i16);
        self.set_hl(result);

        let carry = (self.sp & 0xFF) + offset as u16 & 0xFF > 0xFF;
        let half_carry = (self.sp & 0xF) + offset as u16 & 0xF > 0xF;

        self.reset_zero_flag();
        self.reset_add_sub_flag();

        if half_carry {
            self.set_half_carry_flag();
        } else {
            self.reset_half_carry_flag();
        }

        if carry {
            self.set_carry_flag();
        } else {
            self.reset_carry_flag();
        }
    }

    /**
     * Load register HL into register SP.
     * 
     * MACHINE CYCLES: 2
     * INSTRUCTION LENGTH: 1
     */
    pub fn ld_sp_hl(&mut self) {
        self.sp = self.hl();
    }

    /**
     * Load value in register A from the byte at address n16.
     * 
     * MACHINE CYCLES: 4
     * INSTRUCTION LENGTH: 3
     */
    pub fn ld_a_u16(&mut self, memory: &Memory) {
        let lower_byte = memory.read_byte(self.pc);
        self.pc += 1;
        let upper_byte = memory.read_byte(self.pc);
        self.pc += 1;

        let address: u16 = (upper_byte as u16) << 8 | lower_byte as u16;
        let result = memory.read_byte(address);
        self.a = result;
    }

    /**
     * Enable Interrupts by setting the IME flag. The flag is only set after the instruction following EI.\
     * 
     * MACHINE CYCLE: 1
     * INSTRUCTION LENGTH: 1
     */
    pub fn ei(&mut self, memory: &mut Memory) {
        memory.write_byte(0xFFFF, 1);
    }

    /**
     * Subtract the value n8 from A and set flags accordingly, but don't store the result.
     * 
     * MACHINE CYCLE: 2
     * INSTRUCTION LENGTH: 2
     */
    pub fn cp_a_u8(&mut self, memory: &Memory) {
        let immediate = memory.read_byte(self.pc);
        self.pc += 1;

        let result = self.a - immediate;

        if result == 0 {
            self.set_zero_flag();
        } else {
            self.reset_zero_flag();
        }

        self.set_add_sub_flag();

        if immediate > self.a & 0xF {
            self.set_half_carry_flag();
        } else {
            self.reset_half_carry_flag();
        }

        if immediate > self.a {
            self.set_carry_flag();
        } else {
            self.reset_carry_flag();
        }
    }

    /**
    * Given an opcode it will execute the instruction of the opcode
    */
    pub fn exexute(&mut self, opcode: u8, memory: &mut Memory) {
        match opcode {
            0x00 => self.nop(),
            0x01 => self.ld_r16_u16(memory, Register::B, Register::C),
            0x02 => self.ld_r16_a(memory, Register::B, Register::C),
            0x03 => self.inc_r16(Register::B, Register::C),
            0x04 => self.inc_r8(Register::B),
            0x05 => self.dec_r8(Register::B),
            0x06 => self.ld_r8_u8(memory, Register::B),
            0x07 => self.rlca(),
            0x08 => self.ld_u16_sp(memory),
            0x09 => self.add_hl_r16(Register::B, Register::C),
            0x0A => self.ld_a_r16(memory, Register::B, Register::C),
            0x0B => self.dec_r16(Register::B, Register::C),
            0x0C => self.inc_r8(Register::C),
            0x0D => self.dec_r8(Register::C),
            0x0E => self.ld_r8_u8(memory, Register::C),
            0x0F => self.rrca(),
            0x10 => self.stop(),
            0x11 => self.ld_r16_u16(memory, Register::D, Register::E),
            0x12 => self.ld_r16_a(memory, Register::D, Register::E),
            0x13 => self.inc_r16(Register::D, Register::E),
            0x14 => self.inc_r8(Register::D),
            0x15 => self.dec_r8(Register::D),
            0x16 => self.ld_r8_u8(memory, Register::D),
            0x17 => self.rla(),
            0x18 => self.jr_i8(memory),
            0x19 => self.add_hl_r16(Register::D, Register::E),
            0x1A => self.ld_a_r16(memory, Register::D, Register::E),
            0x1B => self.dec_r16(Register::D, Register::E),
            0x1C => self.inc_r8(Register::E),
            0x1D => self.dec_r8(Register::E),
            0x1E => self.ld_r8_u8(memory, Register::E),
            0x1F => self.rra(),
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
            0xCB => self.prefix(memory),
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
     * Rotate register r8 left.
     * 
     * MACHINE CYCLES: 2
     * INSTRUCTION LENGTH: 2
     */
    pub fn rlc_r8(&mut self, reg: Register) {
        let reg_value = match reg {
            Register::A => &mut self.a,
            Register::B => &mut self.b,
            Register::C => &mut self.c,
            Register::D => &mut self.d,
            Register::E => &mut self.e,
            Register::F => panic!("Cannot use register f (rlc_r8)"),
            Register::H => &mut self.h,
            Register::L => &mut self.l,
            _ => panic!("Cannot use register sp and pc (rlc_r8)"),
        };

        let rotated_bit: u8 = Self::get_bit(*reg_value, 7);
        *reg_value = (*reg_value << 1) | rotated_bit;

        if *reg_value == 0 {
            self.set_zero_flag();
        } else {
            self.reset_zero_flag();
        }

        self.reset_add_sub_flag();
        self.reset_half_carry_flag();

        if rotated_bit != 0 {
            self.set_carry_flag();
        } else {
            self.reset_carry_flag();
        }
    }

    /**
     * Rotate the byte pointed to by HL left.
     * 
     * MACHINE CYCLES: 4
     * INSTRUCTION LENGTH: 2
     */
    pub fn rlc_hl(&mut self, memory: &mut Memory) {
        let hl_data = memory.read_byte(self.hl());
        let rotated_bit: u8 = Self::get_bit(hl_data, 7);
        let result = (hl_data << 1) | rotated_bit;

        memory.write_byte(self.hl(), result);

        if result == 0 {
            self.set_zero_flag();
        } else {
            self.reset_zero_flag();
        }

        self.reset_add_sub_flag();
        self.reset_half_carry_flag();

        if rotated_bit != 0 {
            self.set_carry_flag();
        } else {
            self.reset_carry_flag();
        }
    }

    /**
    * Rotate register r8 right.
    * 
    * MACHINE CYCLE: 2
    * INSTRUCTION LENGTH: 2
    */
    pub fn rrc_r8(&mut self, reg: Register) {
        let reg_value = match reg {
            Register::A => &mut self.a,
            Register::B => &mut self.b,
            Register::C => &mut self.c,
            Register::D => &mut self.d,
            Register::E => &mut self.e,
            Register::F => panic!("Cannot use register f (rrc_r8)"),
            Register::H => &mut self.h,
            Register::L => &mut self.l,
            _ => panic!("Cannot use register sp and pc (rrc_r8)"),
        };

        let rotated_bit: u8 = Self::get_bit(*reg_value, 0);

        *reg_value = (*reg_value >> 1) | (rotated_bit << 7);

        if *reg_value == 0 {
            self.set_zero_flag();
        } else {
            self.reset_zero_flag();
        }

        self.reset_add_sub_flag();
        self.reset_half_carry_flag();

        if rotated_bit != 0 {
            self.set_carry_flag();
        } else {
            self.reset_carry_flag();
        }
    }

    /**
     * Rotate the byte pointed to by HL right.
     * 
     * MACHINE CYCLES: 4
     * INSTRUCTION LENGTH: 2
     */
    pub fn rrc_hl(&mut self, memory: &mut Memory) {
        let mut hl_data = memory.read_byte(self.hl());
        let rotated_bit: u8 = Self::get_bit(hl_data, 0);
        let result = (hl_data >> 1) | (rotated_bit << 7);

        memory.write_byte(self.hl(), result);

        if result == 0 {
            self.set_zero_flag();
        } else {
            self.reset_zero_flag();
        }

        self.reset_add_sub_flag();
        self.reset_half_carry_flag();

        if rotated_bit != 0 {
            self.set_carry_flag();
        } else {
            self.reset_carry_flag();
        }
    }

    /**
     * Rotate register r8 left through carry.
     * 
     * MACHINE CYCLES: 2
     * INSTRUCTION LENGTH: 2 
     */
    pub fn rl_r8(&mut self, reg: Register) {
        let carry = self.get_carry_flag();
        let reg_value = match reg {
            Register::A => &mut self.a,
            Register::B => &mut self.b,
            Register::C => &mut self.c,
            Register::D => &mut self.d,
            Register::E => &mut self.e,
            Register::F => panic!("Cannot use register f (rl_r8)"),
            Register::H => &mut self.h,
            Register::L => &mut self.l,
            _ => panic!("Cannot use register sp and pc (rl_r8)"),
        };

        let rotated_bit: u8 = Self::get_bit(*reg_value, 7);
        let result = (*reg_value << 1) | carry;
        *reg_value = result;

        if result == 0 {
            self.set_zero_flag();
        } else {
            self.reset_zero_flag();
        }

        self.reset_add_sub_flag();
        self.reset_half_carry_flag();

        if rotated_bit != 0 {
            self.set_carry_flag();
        } else {
            self.reset_carry_flag();
        }
    }

    /**
     * Rotate the byte pointed to by HL left through carry.
     * 
     * MACHINE CYCLES: 4
     * INSTRUCTION LENGTH: 2
     */
    pub fn rl_hl(&mut self, memory: &mut Memory) {
        let hl_data = memory.read_byte(self.hl());
        let rotated_bit: u8 = Self::get_bit(hl_data, 7);
        let result = (hl_data << 1) | self.get_carry_flag();

        memory.write_byte(self.hl(), result);

        if result == 0 {
            self.set_zero_flag();
        } else {
            self.reset_zero_flag();
        }

        self.reset_add_sub_flag();
        self.reset_half_carry_flag();

        if rotated_bit != 0 {
            self.set_carry_flag();
        } else {
            self.reset_carry_flag();
        }
    }

    /**
     * Rotate register r8 right through carry.
     * 
     * MACHINE CYCLES: 2
     * INSTRUCTION LENGTH: 2
     */
    pub fn rr_r8(&mut self, reg: Register) {
        let carry = self.get_carry_flag();
        let reg_value = match reg {
            Register::A => &mut self.a,
            Register::B => &mut self.b,
            Register::C => &mut self.c,
            Register::D => &mut self.d,
            Register::E => &mut self.e,
            Register::F => panic!("Cannot use register f (rr_r8)"),
            Register::H => &mut self.h,
            Register::L => &mut self.l,
            _ => panic!("Cannot use register sp and pc (rr_r8)"),
        };

        let rotated_bit: u8 = Self::get_bit(*reg_value, 0);
        let result = (*reg_value >> 1) | (carry << 7);
        *reg_value = result;

        if result != 0 {
            self.set_zero_flag();
        } else {
            self.reset_zero_flag();
        }

        self.reset_add_sub_flag();
        self.reset_half_carry_flag();

        if rotated_bit != 0 {
            self.set_carry_flag();
        } else {
            self.reset_carry_flag();
        }
    }

    /**
     * Rotate the byte pointed to by HL right through carry.
     * 
     * MACHINE CYCLE: 4
     * INSTRUCTION LENGTH: 2
     */
    pub fn rr_hl(&mut self, memory: &mut Memory) {
        let hl_data = memory.read_byte(self.hl());
        let rotated_bit: u8 = Self::get_bit(hl_data, 0);
        let result = (hl_data >> 1) | (self.get_carry_flag() << 7);

        memory.write_byte(self.hl(), result);

        if result != 0 {
            self.set_zero_flag();
        } else {
            self.reset_zero_flag();
        }

        self.reset_add_sub_flag();
        self.reset_half_carry_flag();

        if rotated_bit != 0 {
            self.set_carry_flag();
        } else {
            self.reset_carry_flag();
        }
    }

    /**
     * Shift Left Arithmetically register r8.
     * 
     * MACHINE CYCLES: 2
     * INSTRUCTION LENGTH: 2
     */
    pub fn sla_r8(&mut self, reg: Register) {
        let reg_value = match reg {
            Register::A => &mut self.a,
            Register::B => &mut self.b,
            Register::C => &mut self.c,
            Register::D => &mut self.d,
            Register::E => &mut self.e,
            Register::F => panic!("Cannot use register f (rr_r8)"),
            Register::H => &mut self.h,
            Register::L => &mut self.l,
            _ => panic!("Cannot use register sp and pc (rr_r8)"),
        };

        let rotated_bit = Self::get_bit(*reg_value, 7);
        let result = *reg_value << 1;
        *reg_value = result;

        if result != 0 {
            self.set_zero_flag();
        } else {
            self.reset_zero_flag();
        }

        self.reset_half_carry_flag();
        self.reset_add_sub_flag();

        if rotated_bit != 0 {
            self.set_carry_flag();
        } else {
            self.reset_carry_flag();
        }
    } 

    /**
     * Shift Left Arithmetically the byte pointed to by HL.
     * 
     * MACHINE CYCLES: 4
     * INSTRUCTION LENGTH: 2
     */
    pub fn sla_hl(&mut self, memory: &mut Memory) {
        let hl_data = memory.read_byte(self.hl());
        let rotated_bit = Self::get_bit(hl_data, 7);
        let result = hl_data << 1;

        memory.write_byte(self.hl(), result);

        if result != 0 {
            self.set_zero_flag();
        } else {
            self.reset_zero_flag();
        }

        self.reset_half_carry_flag();
        self.reset_add_sub_flag();

        if rotated_bit != 0 {
            self.set_carry_flag();
        } else {
            self.reset_carry_flag();
        }
    }

    /**
     * Shift Right Arithmetically register r8.
     * 
     * MACHINE CYCLES: 2
     * INSTRUCTION LENGTH: 2
     */
    pub fn sra_r8(&mut self, reg: Register) {
        let reg_value = match reg {
            Register::A => &mut self.a,
            Register::B => &mut self.b,
            Register::C => &mut self.c,
            Register::D => &mut self.d,
            Register::E => &mut self.e,
            Register::F => panic!("Cannot use register f (sra_r8)"),
            Register::H => &mut self.h,
            Register::L => &mut self.l,
            _ => panic!("Cannot use register sp and pc (sra_r8)"),
        };

        let rotated_bit = Self::get_bit(*reg_value, 0);
        let first_bit = Self::get_bit(*reg_value, 7);
        let result = (*reg_value >> 1) | first_bit << 7;
        
        *reg_value = result;
        
        if result != 0 {
            self.set_zero_flag();
        } else {
            self.reset_zero_flag();
        }

        self.reset_add_sub_flag();
        self.reset_half_carry_flag();

        if rotated_bit != 0 {
            self.set_carry_flag();
        } else {
            self.reset_carry_flag();
        }
    }

    /**
     * Shift Right Arithmetically the byte pointed to by HL.
     * 
     * MACHINE CYCLES: 4
     * INSTRUCTION CYCLES: 2
     */
    pub fn sra_hl(&mut self, memory: &mut Memory) {
        let hl_data = memory.read_byte(self.hl());
        let rotated_bit = Self::get_bit(hl_data, 0);
        let first_bit = Self::get_bit(hl_data, 7);
        let result = (hl_data >> 1) | first_bit << 7;

        memory.write_byte(self.hl(), hl_data);
        
        if result != 0 {
            self.set_zero_flag();
        } else {
            self.reset_zero_flag();
        }

        self.reset_add_sub_flag();
        self.reset_half_carry_flag();

        if rotated_bit != 0 {
            self.set_carry_flag();
        } else {
            self.reset_carry_flag();
        }
    }

    /**
     * Swap the upper 4 bits in register r8 and the lower 4 ones.
     * 
     * MACHINE CYCLES: 2
     * INSTRUCTION LENGTH: 2
     */
    pub fn swap_r8(&mut self, reg: Register) {
        let reg_value = match reg {
            Register::A => &mut self.a,
            Register::B => &mut self.b,
            Register::C => &mut self.c,
            Register::D => &mut self.d,
            Register::E => &mut self.e,
            Register::F => panic!("Cannot use register f (sra_r8)"),
            Register::H => &mut self.h,
            Register::L => &mut self.l,
            _ => panic!("Cannot use register sp and pc (sra_r8)"),
        };

        let lower_nibble = *reg_value >> 4;
        let result = (*reg_value << 4) | lower_nibble;

        *reg_value = result;
        
        if result != 0 {
            self.set_zero_flag();
        } else {
            self.reset_zero_flag();
        }

        self.reset_add_sub_flag();
        self.reset_half_carry_flag();
        self.reset_carry_flag();
    }

    /**
     * Swap the upper 4 bits in the byte pointed by HL and the lower 4 ones.
     * 
     * MACHINE CYCLE: 4
     * INSTRUCTION LENGTH: 2
     */
    pub fn swap_hl(&mut self, memory: &mut Memory) {
        let hl_data = memory.read_byte(self.hl());
        let lower_nibble = hl_data >> 4;
        let result = (hl_data << 4) | lower_nibble;

        memory.write_byte(self.hl(), result);
        
        if result != 0 {
            self.set_zero_flag();
        } else {
            self.reset_zero_flag();
        }

        self.reset_add_sub_flag();
        self.reset_half_carry_flag();
        self.reset_carry_flag();
    }

    /**
     * Shift Right Logically register r8.
     * 
     * MACHINE CYCLES: 2
     * INSTRUCTION LENGTH: 2
     */
    pub fn srl_r8(&mut self, reg: Register) {
        let reg_value = match reg {
            Register::A => &mut self.a,
            Register::B => &mut self.b,
            Register::C => &mut self.c,
            Register::D => &mut self.d,
            Register::E => &mut self.e,
            Register::F => panic!("Cannot use register f (srl_r8)"),
            Register::H => &mut self.h,
            Register::L => &mut self.l,
            _ => panic!("Cannot use register sp and pc (srl_r8)"),
        };

        let rotated_bit = Self::get_bit(*reg_value, 0);
        let result = *reg_value >> 1;

        *reg_value = result;
        
        if result != 0 {
            self.set_zero_flag();
        } else {
            self.reset_zero_flag();
        }

        self.reset_add_sub_flag();
        self.reset_half_carry_flag();

        if rotated_bit != 0 {
            self.set_carry_flag();
        } else {
            self.reset_carry_flag();
        }
    }

    /**
     * Shift Right Logically the byte pointed to by HL.
     * 
     * MACHINE CYCLES: 2
     * INSTRUCTION LENGTH: 2
     */
    pub fn srl_hl(&mut self, memory: &mut Memory) {
        let hl_data = memory.read_byte(self.hl());
        let rotated_bit = Self::get_bit(hl_data, 0);
        let result = hl_data >> 1;

        memory.write_byte(self.hl(), result);
        
        if result != 0 {
            self.set_zero_flag();
        } else {
            self.reset_zero_flag();
        }

        self.reset_add_sub_flag();
        self.reset_half_carry_flag();

        if rotated_bit != 0 {
            self.set_carry_flag();
        } else {
            self.reset_carry_flag();
        }
    }

    /**
     * Test bit u3 in register r8, set the zero flag if bit not set.
     * 
     * MACHINE CYCLES: 2
     * INSTRUCTION LENGTH: 2
     */
    pub fn bit_u3_r8(&mut self, bit_to_test: u8, reg: Register) {
        let reg_value = match reg {
            Register::A => &mut self.a,
            Register::B => &mut self.b,
            Register::C => &mut self.c,
            Register::D => &mut self.d,
            Register::E => &mut self.e,
            Register::F => panic!("Cannot use register f (srl_r8)"),
            Register::H => &mut self.h,
            Register::L => &mut self.l,
            _ => panic!("Cannot use register sp and pc (srl_r8)"),
        };

        let result = Self::get_bit(*reg_value, bit_to_test);

        if result != 0 {
            self.set_zero_flag();
        } else {
            self.reset_zero_flag();
        }
        
        self.reset_add_sub_flag();
        self.set_half_carry_flag();
    }

    /**
     * Test bit u3 in the byte pointed by HL, set the zero flag if bit not set.
     * 
     * MACHINE CYCLES: 3
     * INSTRUCTION LENGTH: 2
     */
    pub fn bit_u3_hl(&mut self, bit_to_test: u8, memory: &Memory) {
        let hl_data = memory.read_byte(self.hl());

        let result = Self::get_bit(hl_data, bit_to_test);

        if result != 0 {
            self.set_zero_flag();
        } else {
            self.reset_zero_flag();
        }
        
        self.reset_add_sub_flag();
        self.set_half_carry_flag();
    }

    /**
     * Set bit u3 in register r8 to 0. Bit 0 is the rightmost one, bit 7 the leftmost one.
     * 
     * MACHINE CYCLES: 2
     * INSTRUCTION LENGTH: 2
     */
    pub fn res_u3_r8(&mut self, bit_to_reset: u8, reg: Register) {
        let reg_value = match reg {
            Register::A => &mut self.a,
            Register::B => &mut self.b,
            Register::C => &mut self.c,
            Register::D => &mut self.d,
            Register::E => &mut self.e,
            Register::F => panic!("Cannot use register f (res_u3_r8)"),
            Register::H => &mut self.h,
            Register::L => &mut self.l,
            _ => panic!("Cannot use register sp and pc (res_u3_r8)"),
        };

        let result = *reg_value & !(0x1 << bit_to_reset);
        *reg_value = result;
    }

    /**
     * Set bit u3 in the byte pointed by HL to 0. Bit 0 is the rightmost one, bit 7 the leftmost one.
     * 
     * MACHINE CYCLES: 4
     * INSTRUCTION LENGTH: 2
     */
    pub fn res_u3_hl(&mut self, bit_to_reset: u8, memory: &mut Memory) {
        let hl_data = memory.read_byte(self.hl());
        let result = hl_data & !(0x1 << bit_to_reset);
        memory.write_byte(self.hl(), result);
    }

    /**
     * Set bit u3 in register r8 to 1. Bit 0 is the rightmost one, bit 7 the leftmost one.
     */
    pub fn set_u3_r8(&mut self, bit_to_set: u8, reg: Register) {
        let reg_value = match reg {
            Register::A => &mut self.a,
            Register::B => &mut self.b,
            Register::C => &mut self.c,
            Register::D => &mut self.d,
            Register::E => &mut self.e,
            Register::F => panic!("Cannot use register f (set_u3_r8)"),
            Register::H => &mut self.h,
            Register::L => &mut self.l,
            _ => panic!("Cannot use register sp and pc (set_u3_r8)"),
        };

        let result = *reg_value | (0x1 << bit_to_set);
        *reg_value = result;
    }

    /**
     * Set bit u3 in register hl to 1. Bit 0 is the rightmost one, bit 7 the leftmost one.
     */
    pub fn set_u3_hl(&mut self, bit_to_set: u8, memory: &mut Memory) {
        let hl_data = memory.read_byte(self.hl());
        let result = hl_data | (0x1 << bit_to_set);
        memory.write_byte(self.hl(), result);
    }

    /**
     * Function to take care of the prefix instructions
     */
    pub fn prefix(&mut self, memory: &mut Memory) {
        let opcode = memory.read_byte(self.pc);
        self.pc += 1; 

        match opcode {
            0x00 => self.rlc_r8(Register::B),
            0x01 => self.rlc_r8(Register::C),
            0x02 => self.rlc_r8(Register::D),
            0x03 => self.rlc_r8(Register::E),
            0x04 => self.rlc_r8(Register::H),
            0x05 => self.rlc_r8(Register::L),
            0x06 => self.rlc_hl(memory),
            0x07 => self.rlc_r8(Register::A),
            0x08 => self.rrc_r8(Register::B),
            0x09 => self.rrc_r8(Register::C),
            0x0A => self.rrc_r8(Register::D),
            0x0B => self.rrc_r8(Register::E),
            0x0C => self.rrc_r8(Register::H),
            0x0D => self.rrc_r8(Register::L),
            0x0E => self.rrc_hl(memory),
            0x0F => self.rrc_r8(Register::A),
            0x10 => self.rl_r8(Register::B),
            0x11 => self.rl_r8(Register::C),
            0x12 => self.rl_r8(Register::D),
            0x13 => self.rl_r8(Register::E),
            0x14 => self.rl_r8(Register::H),
            0x15 => self.rl_r8(Register::L),
            0x16 => self.rl_hl(memory),
            0x17 => self.rl_r8(Register::A),
            0x18 => self.rr_r8(Register::B),
            0x19 => self.rr_r8(Register::C),
            0x1A => self.rr_r8(Register::D),
            0x1B => self.rr_r8(Register::E),
            0x1C => self.rr_r8(Register::H),
            0x1D => self.rr_r8(Register::L),
            0x1E => self.rr_hl(memory),
            0x1F => self.rr_r8(Register::A),
            0x20 => self.sla_r8(Register::B),
            0x21 => self.sla_r8(Register::C),
            0x22 => self.sla_r8(Register::D),
            0x23 => self.sla_r8(Register::E),
            0x24 => self.sla_r8(Register::H),
            0x25 => self.sla_r8(Register::L),
            0x26 => self.sla_hl(memory),
            0x27 => self.sla_r8(Register::A),
            0x28 => self.sra_r8(Register::B),
            0x29 => self.sra_r8(Register::C),
            0x2A => self.sra_r8(Register::D),
            0x2B => self.sra_r8(Register::E),
            0x2C => self.sra_r8(Register::H),
            0x2D => self.sra_r8(Register::L),
            0x2E => self.sra_hl(memory),
            0x2F => self.sra_r8(Register::A),
            0x30 => self.swap_r8(Register::B),
            0x31 => self.swap_r8(Register::C),
            0x32 => self.swap_r8(Register::D),
            0x33 => self.swap_r8(Register::E),
            0x34 => self.swap_r8(Register::H),
            0x35 => self.swap_r8(Register::L),
            0x36 => self.swap_hl(memory),
            0x37 => self.swap_r8(Register::A),
            0x38 => self.srl_r8(Register::B),
            0x39 => self.srl_r8(Register::B),
            0x3A => self.srl_r8(Register::B),
            0x3B => self.srl_r8(Register::B),
            0x3C => self.srl_r8(Register::B),
            0x3D => self.srl_r8(Register::B),
            0x3E => self.srl_hl(memory),
            0x3F => self.srl_r8(Register::A),
            0x40 => self.bit_u3_r8(0, Register::B),
            0x41 => self.bit_u3_r8(0, Register::C),
            0x42 => self.bit_u3_r8(0, Register::D),
            0x43 => self.bit_u3_r8(0, Register::E),
            0x44 => self.bit_u3_r8(0, Register::H),
            0x45 => self.bit_u3_r8(0, Register::L),
            0x46 => self.bit_u3_hl(0, memory),
            0x47 => self.bit_u3_r8(0, Register::A),
            0x48 => self.bit_u3_r8(1, Register::B),
            0x49 => self.bit_u3_r8(1, Register::C),
            0x4A => self.bit_u3_r8(1, Register::D),
            0x4B => self.bit_u3_r8(1, Register::E),
            0x4C => self.bit_u3_r8(1, Register::H),
            0x4D => self.bit_u3_r8(1, Register::L),
            0x4E => self.bit_u3_hl(1, memory),
            0x4F => self.bit_u3_r8(1, Register::A),
            0x50 => self.bit_u3_r8(2, Register::B),
            0x51 => self.bit_u3_r8(2, Register::C),
            0x52 => self.bit_u3_r8(2, Register::D),
            0x53 => self.bit_u3_r8(2, Register::E),
            0x54 => self.bit_u3_r8(2, Register::H),
            0x55 => self.bit_u3_r8(2, Register::L),
            0x56 => self.bit_u3_hl(2, memory),
            0x57 => self.bit_u3_r8(2, Register::A),
            0x58 => self.bit_u3_r8(3, Register::B),
            0x59 => self.bit_u3_r8(3, Register::C),
            0x5A => self.bit_u3_r8(3, Register::D),
            0x5B => self.bit_u3_r8(3, Register::E),
            0x5C => self.bit_u3_r8(3, Register::H),
            0x5D => self.bit_u3_r8(3, Register::L),
            0x5E => self.bit_u3_hl(3, memory),
            0x5F => self.bit_u3_r8(3, Register::A),
            0x60 => self.bit_u3_r8(4, Register::B),
            0x61 => self.bit_u3_r8(4, Register::C),
            0x62 => self.bit_u3_r8(4, Register::D),
            0x63 => self.bit_u3_r8(4, Register::E),
            0x64 => self.bit_u3_r8(4, Register::H),
            0x65 => self.bit_u3_r8(4, Register::L),
            0x66 => self.bit_u3_hl(4, memory),
            0x67 => self.bit_u3_r8(4, Register::A),
            0x68 => self.bit_u3_r8(5, Register::B),
            0x69 => self.bit_u3_r8(5, Register::C),
            0x6A => self.bit_u3_r8(5, Register::D),
            0x6B => self.bit_u3_r8(5, Register::E),
            0x6C => self.bit_u3_r8(5, Register::H),
            0x6D => self.bit_u3_r8(5, Register::L),
            0x6E => self.bit_u3_hl(5, memory),
            0x6F => self.bit_u3_r8(5, Register::A),
            0x70 => self.bit_u3_r8(6, Register::B),
            0x71 => self.bit_u3_r8(6, Register::C),
            0x72 => self.bit_u3_r8(6, Register::D),
            0x73 => self.bit_u3_r8(6, Register::E),
            0x74 => self.bit_u3_r8(6, Register::H),
            0x75 => self.bit_u3_r8(6, Register::L),
            0x76 => self.bit_u3_hl(6, memory),
            0x77 => self.bit_u3_r8(6, Register::A),
            0x78 => self.bit_u3_r8(7, Register::B),
            0x79 => self.bit_u3_r8(7, Register::C),
            0x7A => self.bit_u3_r8(7, Register::D),
            0x7B => self.bit_u3_r8(7, Register::E),
            0x7C => self.bit_u3_r8(7, Register::H),
            0x7D => self.bit_u3_r8(7, Register::L),
            0x7E => self.bit_u3_hl(7, memory),
            0x7F => self.bit_u3_r8(7, Register::A),
            0x80 => self.res_u3_r8(0, Register::B),
            0x81 => self.res_u3_r8(0, Register::C),
            0x82 => self.res_u3_r8(0, Register::D),
            0x83 => self.res_u3_r8(0, Register::E),
            0x84 => self.res_u3_r8(0, Register::H),
            0x85 => self.res_u3_r8(0, Register::L),
            0x86 => self.res_u3_hl(0, memory),
            0x87 => self.res_u3_r8(0, Register::A),
            0x88 => self.res_u3_r8(1, Register::B),
            0x89 => self.res_u3_r8(1, Register::C),
            0x8A => self.res_u3_r8(1, Register::D),
            0x8B => self.res_u3_r8(1, Register::E),
            0x8C => self.res_u3_r8(1, Register::H),
            0x8D => self.res_u3_r8(1, Register::L),
            0x8E => self.res_u3_hl(1, memory),
            0x8F => self.res_u3_r8(1, Register::A),
            0x90 => self.res_u3_r8(2, Register::B),
            0x91 => self.res_u3_r8(2, Register::C),
            0x92 => self.res_u3_r8(2, Register::D),
            0x93 => self.res_u3_r8(2, Register::E),
            0x94 => self.res_u3_r8(2, Register::H),
            0x95 => self.res_u3_r8(2, Register::L),
            0x96 => self.res_u3_hl(2, memory),
            0x97 => self.res_u3_r8(2, Register::A),
            0x98 => self.res_u3_r8(3, Register::B),
            0x99 => self.res_u3_r8(3, Register::C),
            0x9A => self.res_u3_r8(3, Register::D),
            0x9B => self.res_u3_r8(3, Register::E),
            0x9C => self.res_u3_r8(3, Register::H),
            0x9D => self.res_u3_r8(3, Register::L),
            0x9E => self.res_u3_hl(3, memory),
            0x9F => self.res_u3_r8(3, Register::A),
            0xA0 => self.res_u3_r8(4, Register::B),
            0xA1 => self.res_u3_r8(4, Register::C),
            0xA2 => self.res_u3_r8(4, Register::D),
            0xA3 => self.res_u3_r8(4, Register::E),
            0xA4 => self.res_u3_r8(4, Register::H),
            0xA5 => self.res_u3_r8(4, Register::L),
            0xA6 => self.res_u3_hl(4, memory),
            0xA7 => self.res_u3_r8(4, Register::A),
            0xA8 => self.res_u3_r8(5, Register::B),
            0xA9 => self.res_u3_r8(5, Register::C),
            0xAA => self.res_u3_r8(5, Register::D),
            0xAB => self.res_u3_r8(5, Register::E),
            0xAC => self.res_u3_r8(5, Register::H),
            0xAD => self.res_u3_r8(5, Register::L),
            0xAE => self.res_u3_hl(5, memory),
            0xAF => self.res_u3_r8(5, Register::A),
            0xB0 => self.res_u3_r8(6, Register::B),
            0xB1 => self.res_u3_r8(6, Register::C),
            0xB2 => self.res_u3_r8(6, Register::D),
            0xB3 => self.res_u3_r8(6, Register::E),
            0xB4 => self.res_u3_r8(6, Register::H),
            0xB5 => self.res_u3_r8(6, Register::L),
            0xB6 => self.res_u3_hl(6, memory),
            0xB7 => self.res_u3_r8(6, Register::A),
            0xB8 => self.res_u3_r8(7, Register::B),
            0xB9 => self.res_u3_r8(7, Register::C),
            0xBA => self.res_u3_r8(7, Register::D),
            0xBB => self.res_u3_r8(7, Register::E),
            0xBC => self.res_u3_r8(7, Register::H),
            0xBD => self.res_u3_r8(7, Register::L),
            0xBE => self.res_u3_hl(7, memory),
            0xBF => self.res_u3_r8(7, Register::A),
            0xC0 => self.set_u3_r8(0, Register::B),
            0xC1 => self.set_u3_r8(0, Register::C),
            0xC2 => self.set_u3_r8(0, Register::D),
            0xC3 => self.set_u3_r8(0, Register::E),
            0xC4 => self.set_u3_r8(0, Register::H),
            0xC5 => self.set_u3_r8(0, Register::L),
            0xC6 => self.set_u3_hl(0, memory),
            0xC7 => self.set_u3_r8(0, Register::A),
            0xC8 => self.set_u3_r8(1, Register::B),
            0xC9 => self.set_u3_r8(1, Register::C),
            0xCA => self.set_u3_r8(1, Register::D),
            0xCB => self.set_u3_r8(1, Register::E),
            0xCC => self.set_u3_r8(1, Register::H),
            0xCD => self.set_u3_r8(1, Register::L),
            0xCE => self.set_u3_hl(1, memory),
            0xCF => self.set_u3_r8(1, Register::A),
            0xD0 => self.set_u3_r8(2, Register::B),
            0xD1 => self.set_u3_r8(2, Register::C),
            0xD2 => self.set_u3_r8(2, Register::D),
            0xD3 => self.set_u3_r8(2, Register::E),
            0xD4 => self.set_u3_r8(2, Register::H),
            0xD5 => self.set_u3_r8(2, Register::L),
            0xD6 => self.set_u3_hl(2, memory),
            0xD7 => self.set_u3_r8(2, Register::A),
            0xD8 => self.set_u3_r8(3, Register::B),
            0xD9 => self.set_u3_r8(3, Register::C),
            0xDA => self.set_u3_r8(3, Register::D),
            0xDB => self.set_u3_r8(3, Register::E),
            0xDC => self.set_u3_r8(3, Register::H),
            0xDD => self.set_u3_r8(3, Register::L),
            0xDE => self.set_u3_hl(3, memory),
            0xDF => self.set_u3_r8(3, Register::A),
            0xE0 => self.set_u3_r8(4, Register::B),
            0xE1 => self.set_u3_r8(4, Register::C),
            0xE2 => self.set_u3_r8(4, Register::D),
            0xE3 => self.set_u3_r8(4, Register::E),
            0xE4 => self.set_u3_r8(4, Register::H),
            0xE5 => self.set_u3_r8(4, Register::L),
            0xE6 => self.set_u3_hl(4, memory),
            0xE7 => self.set_u3_r8(4, Register::A),
            0xE8 => self.set_u3_r8(5, Register::B),
            0xE9 => self.set_u3_r8(5, Register::C),
            0xEA => self.set_u3_r8(5, Register::D),
            0xEB => self.set_u3_r8(5, Register::E),
            0xEC => self.set_u3_r8(5, Register::H),
            0xED => self.set_u3_r8(5, Register::L),
            0xEE => self.set_u3_hl(5, memory),
            0xEF => self.set_u3_r8(5, Register::A),
            0xF0 => self.set_u3_r8(6, Register::B),
            0xF1 => self.set_u3_r8(6, Register::C),
            0xF2 => self.set_u3_r8(6, Register::D),
            0xF3 => self.set_u3_r8(6, Register::E),
            0xF4 => self.set_u3_r8(6, Register::H),
            0xF5 => self.set_u3_r8(6, Register::L),
            0xF6 => self.set_u3_hl(6, memory),
            0xF7 => self.set_u3_r8(6, Register::A),
            0xF8 => self.set_u3_r8(7, Register::B),
            0xF9 => self.set_u3_r8(7, Register::C),
            0xFA => self.set_u3_r8(7, Register::D),
            0xFB => self.set_u3_r8(7, Register::E),
            0xFC => self.set_u3_r8(7, Register::H),
            0xFD => self.set_u3_r8(7, Register::L),
            0xFE => self.set_u3_hl(7, memory),
            0xFF => self.set_u3_r8(7, Register::A),
            _ => panic!("opcode is not recognized"),
        }
    }

    /**
     * Test bit u3 in register r8, set the zero flag if bit not set.
     * 
     * MACHINE CYCLES: 2
     * INSTRUCTION LENGTH: 2
     */
    pub fn BIT_u3_r8(&mut self, reg: u8, bit_to_test: u8) {
        if Self::get_bit(reg, bit_to_test) == 0 {
            self.set_zero_flag();
        } else {
            self.reset_zero_flag();
        }

        self.reset_add_sub_flag();
        self.set_half_carry_flag();
    }   

    /**
     * Test bit u3 in the byte pointed by HL, set the zero flag if bit not set.
     * 
     * MACHINE CYCLES: 3
     * INSTRUCTION LENGTH: 2
     */
    pub fn BIT_u3_HL(&mut self, memory: &Memory, bit_to_test: u8) {
        let hl_data = memory.read_byte(self.hl());
        
        if Self::get_bit(hl_data, bit_to_test) == 0 {
            self.set_zero_flag();
        } else {
            self.reset_carry_flag();
        }

        self.reset_add_sub_flag();
        self.set_half_carry_flag();
    }


    fn arithmetic_8bit_flag_update(&mut self, result: u8, did_half_carry: bool, did_carry: bool) {
        //N = 0
        self.reset_add_sub_flag();

        //Z
        if result == 0 {
            self.set_zero_flag();
        } else {
            self.reset_zero_flag();
        }

        //H
        if did_half_carry {
            self.set_half_carry_flag();
        } else {
            self.reset_half_carry_flag();
        }

        //C
        if did_carry {
            self.set_carry_flag();
        } else {
            self.reset_carry_flag();
        }
    }
}


