use crate::memory::Memory;

enum Flags {
    Zero,
    Subtraction,
    HalfCarry,
    Carry,
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
        return (self.f >> 7) & 1;
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
        return (self.f >> 6) & 1;
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
        return (self.f >> 5) & 1;
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
        return (self.f >> 4) & 1;
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
     * Given an opcode it will execute the instruction of the opcode
     */
    pub fn exexute(&mut self, opcode: u8, memory: &mut Memory) {
        match opcode {
            0x00 => self.NOP(),
            0x01 => self.LD_r16_u16(memory, Register::B, Register::C),
            0x02 => self.LD_r16_A(memory, Register::B, Register::C),
            0x03 => self.INC_r16(Register::B, Register::C),
            0x04 => self.INC_r8(Register::B),
            0x05 => self.DEC_r8(Register::B),
            _ => (),
        }
    }

    /**
     * Does absolutely nothing but consume a machine cycle and increment the pc
     * 
     * MACHINE CYCLES: 1
     * INSTRUCTION LENGTH: 1
     */
    pub fn NOP(&self) {
        return;
    }

    /**
     * Loads the unsigned 16 bit value into the given registers
     * 
     * MACHINE CYCLES: 3
     * INSTRUCTION LENGTH: 3
     */
    pub fn LD_r16_u16(&mut self, memory: &Memory, high_reg: Register, low_reg: Register) {
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
     * Store value in register A into the byte pointed to by register r16.
     * 
     * MACHINE CYCLES: 4
     * INSTRUCTION LENGTH: 3
     */
    pub fn LD_r16_A(&mut self, memory: &mut Memory, high_reg: Register, low_reg: Register) {
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
    pub fn INC_r16(&mut self, high_reg: Register, low_reg: Register) {
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
    pub fn INC_r8(&mut self, reg: Register) {
        match reg {
            Register::A => self.a += 1,
            Register::B => self.a += 1,
            Register::C => self.a += 1,
            Register::D => self.a += 1,
            Register::E => self.a += 1,
            Register::F => panic!("Cannot increment the f register"),
            Register::H => self.a += 1,
            Register::L => self.a += 1,
            _ => panic!("Cannot increment the sp and pc as this is a 8bit instruction")
        }

        self.reset_add_sub_flag();

        if self.a == 0 {
            self.set_zero_flag();
        } else {
            self.reset_zero_flag();
        }

        if (self.a & 0xF) + 1 > 0xF {
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
    pub fn DEC_r8(&mut self, reg: Register) {
        self.a -= 1;
    }


























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
     * Adds the value in the passed 16-bit register to the 16-bit HL register
     * and stores the result back in the HL register
     * 
     * MACHINE CYCLES: 2
     * INSTRUCTION LENGTH: 1 BYTE
     */
    pub fn ADD_HL_r16(&mut self, reg: u16) {
        let (result, overflow) = self.hl().overflowing_add(reg);
        let half_carry_overflow = (self.hl() & 0x0FFF) + (reg & 0x0FFF) > 0x0FFF;

        self.set_hl(result);
        self.arithmetic_16bit_flag_update(result, half_carry_overflow, overflow);
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
    pub fn CALL_cc_u16(&mut self, flag_type: Flags, address: u16, memory: &mut Memory) {
        match flag_type {
            Flags::Zero => if self.get_zero_flag() != 0 { self.CALL_n16(address, memory) },
            Flags::Subtraction => if self.get_add_sub_flag() != 0 { self.CALL_n16(address, memory) },
            Flags::HalfCarry => if self.get_half_carry_flag() != 0 { self.CALL_n16(address, memory) },
            Flags::Carry => if self.get_carry_flag() != 0 { self.CALL_n16(address, memory) },
        }
    }

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


