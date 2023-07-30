use crate::memory::{Memory, self};

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
     * Command will retrieve the next instruction
     */
    pub fn fetch(&mut self, memory: &Memory) -> u8 {
        let opcode = memory.read_byte(self.pc);
        self.pc += 1;

        return opcode;
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
        self.arithmetic_16bit_flag_update(result, half_carry_overflow, overflow);
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
        todo!();
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
            0x39 => 
            _ => (),
        }
    }

    //00000001 -> 11111110
    //00000000 -> 11111111


























    /**
     * Add the value in r8 plus the carry flag to A register.
     * 
     * INSTRUCTION LENGTH: 1 BYTE
     * MACHINE CYCLES: 1
     */
    pub fn ADC_A_r8(&mut self, reg: u8) {   
        let (partial_result, first_overflow) = self.a.overflowing_add(reg);
        let (result, second_overflow) = partial_result.overflowing_add(self.get_carry_flag());
        let half_carry_overflow = (self.a & 0xF) + (reg & 0xF) + self.get_carry_flag() > 0xF;

        self.a = result;
        self.arithmetic_8bit_flag_update(result, half_carry_overflow, first_overflow || second_overflow);
    }

    /**
     * Adds to the 8-bit A register, the carry flag and data from the absolute address 
     * specified by the 16-bit register HL, and stores the result back into the A register.
     * 
     * MACHINE CYCLES: 2
     * INSTRUCTION LENGTH: 1 BYTE
     */
    pub fn ADC_A_HL(&mut self, memory: &Memory) {
        let hl_data = memory.read_byte(self.hl());
        let (partial_result, first_overflow) = self.a.overflowing_add(hl_data);
        let (result, second_overflow) = partial_result.overflowing_add(self.get_carry_flag());
        let half_carry_overflow = (self.a & 0xF) + (hl_data & 0xF) + self.get_carry_flag() > 0xF;

        self.a = result;
        self.arithmetic_8bit_flag_update(result, half_carry_overflow, first_overflow || second_overflow);
    }

    /**
     * Adds to the 8-bit A register, the carry flag and the immediate data n, 
     * and stores the result back into the A register.
     * 
     * MACHINE CYCLES: 2
     * INSTRUCTION LENGTH: 2 BYTE
     */
    pub fn ADC_A_u8(&mut self, immediate: u8) {
        let (partial_result, first_overflow) = self.a.overflowing_add(immediate);
        let (result, second_overflow) = partial_result.overflowing_add(self.get_carry_flag());
        let half_carry_overflow = (self.a & 0xF) + (immediate & 0xF) + self.get_carry_flag() > 0xF;

        self.a = result;
        self.arithmetic_8bit_flag_update(result, half_carry_overflow, first_overflow || second_overflow);
    }

    /**
     * Adds to the 8-bit A register, the 8-bit register r, and stores the result 
     * back into the A register.
     * 
     * MACHINE CYCLES: 1
     * INSTRUCTION LENGTH: 1 BYTE
     */
    pub fn ADD_A_r8(&mut self, reg: u8) {
        let (result, overflow) = self.a.overflowing_add(reg);
        let half_carry_overflow = (self.a & 0xF) + (reg & 0xF) > 0xF;

        self.a = result;
        self.arithmetic_8bit_flag_update(result, half_carry_overflow, overflow);
    }

    /**
     * Adds to the 8-bit A register, data from the absolute address specified by 
     * the 16-bit register HL, and stores the result back into the A register.
     * 
     * MACHINE CYCLES: 2
     * INSTRUCTION LENGTH: 1 BYTE
     */
    pub fn ADD_A_HL(&mut self, memory: &Memory) {
        let hl_data = memory.read_byte(self.hl());
        let (result, overflow) = self.a.overflowing_add(hl_data);
        let half_carry_overflow = (self.a & 0xF) + (hl_data & 0xF) > 0xF;

        self.a = result;
        self.arithmetic_8bit_flag_update(result, half_carry_overflow, overflow);
    }

    /**
     * Adds to the 8-bit A register, the immediate data n, and stores the result 
     * back into the A register
     * 
     * MACHINE CYCLES: 2
     * INSTRUCTION LENGTH: 2 BYTES
     */
    pub fn ADD_A_u8(&mut self, immediate: u8) {
        let (result, overflow) = self.a.overflowing_add(immediate);
        let half_carry_overflow = (self.a & 0xF) + (immediate & 0xF) > 0xF;

        self.a = result;
        self.arithmetic_8bit_flag_update(result, half_carry_overflow, overflow);
    }



    /**
     * Add the 16-bit value in SP to 16-bit HL register.
     * 
     * MACHINE CYCLES: 2
     * INSTRUCTION LENGTH: 1
     */
    pub fn ADD_HL_SP(&mut self) {
        let (result, overflow) = self.hl().overflowing_add(self.sp);
        let half_carry_overflow = (self.hl() & 0x0FFF) + (self.sp & 0x0FFF) > 0x0FFF;

        self.set_hl(result);
        self.arithmetic_16bit_flag_update(result, half_carry_overflow, overflow);
    }

    /**
     * Add the 8-bit signed value i8 to 16-bit SP register.
     * 
     * MACHINE CYCLES: 4
     * INSTRUCTION LENGTH: 2
     */
    pub fn ADD_SP_i8(&mut self, immediate: i8) {        //FIXME I SWEAR THIS SHIT IS NOT RIGHT. SOMETHING FISHY WITH THE CARRY OVERFLOWS
        let result: u16 = self.sp.wrapping_add_signed(immediate as i16);
        let half_carry_overflow = (self.sp & 0xF) + (immediate as u16 & 0xF) > 0xF;
        let carry_overflow = (self.sp & 0xFF) + (immediate as u16 & 0xFF) > 0xFF;
        
        self.sp = result;
        self.arithmetic_16bit_flag_update(result, half_carry_overflow, carry_overflow);
    }

    /**
     * Bitwise AND between the value in r8 and A.
     * 
     * MACHINE CYCLES: 1
     * INSTRUCTION LENGTH: 1 BYTE
     */
    pub fn AND_A_r8(&mut self, reg: u8) {
        let result = self.a & reg;
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
     * Bitwise AND between the byte pointed to by HL and A.
     * 
     * MACHINE CYCLES: 2
     * INSTRUCTION LENGTH: 1 BYTE
     */

    pub fn AND_A_HL(&mut self, memory: &Memory) {
        let hl_data = memory.read_byte(self.hl());
        let result = self.a & hl_data;

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
     * Bitwise AND between u8 immediate and 8-bit A register
     * 
     * MACHINE CYCLES: 2
     * INSTRUCTION CYCLES: 2
     */
    pub fn AND_A_n8(&mut self, immediate: u8) {
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
    
    /**
     * Call address n16. This pushes the address of the instruction after the CALL on the stack, 
     * such that RET can pop it later; then, it executes an implicit JP n16.
     * 
     * MACHINE CYCLES: 6
     * INSTRUCTION LENGTH: 3
     */
    pub fn CALL_n16(&mut self, address: u16, memory: &mut Memory) {
        //Assuming the pc is always pointing to the next instruction and the sp is always point to the last written address
        self.sp -= 1;
        memory.write_byte(self.sp, Self::get_upper_byte(self.pc));
        self.sp -= 1;
        memory.write_byte(self.sp, Self::get_lower_byte(self.pc));

        //Call
        self.JP_n16(address);
    }

    /**
     * Call address n16 if condition cc is met.
     * 
     * MACHINE CYCLES: 6 if taken, 3 if not taken
     * INSTRUCTION LENGTH: 3
     */
    // pub fn CALL_cc_u16(&mut self, flag_type: Flags, address: u16, memory: &mut Memory) {
    //     match flag_type {
    //         Flags::Zero => if self.get_zero_flag() != 0 { self.CALL_n16(address, memory) },
    //         Flags::Subtraction => if self.get_add_sub_flag() != 0 { self.CALL_n16(address, memory) },
    //         Flags::HalfCarry => if self.get_half_carry_flag() != 0 { self.CALL_n16(address, memory) },
    //         Flags::Carry => if self.get_carry_flag() != 0 { self.CALL_n16(address, memory) },
    //     }
    // }

    /**
     * Jump to address n16; effectively, store n16 into PC.
     * 
     * MACHINE CYCLES: 4
     * INSTRUCTION LENGTH: 3
     */
    pub fn JP_n16(&mut self, address: u16) {
        self.pc = address;
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

    fn arithmetic_16bit_flag_update(&mut self, result: u16, did_half_carry: bool, did_carry: bool) {
        //N = 0
        self.reset_add_sub_flag();

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


