use crate::memory::Memory;

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

    pub fn af(&self) -> u16 {
        let mut af: u16; 

        af = (self.a as u16) << 8;
        af |= self.f as u16;

        return af;
    }

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
        return (value >> bit_position) & 1;
    }

    pub fn get_bit_16(value: u16, bit_position: u8) -> u16 {
        return (value >> bit_position) & 1;
    }

    /**
     * Add the value in r8 plus the carry flag to A register.
     * 
     * INSTRUCTION LENGTH: 1 BYTE
     * MACHINE CYCLES: 1
     */
    pub fn ADC_A_r8(&mut self, reg: u8) {
        let result = self.a + reg + self.get_carry_flag();
        self.a = result;
        self.arithmetic_8bit_flag_update(result);
    }

    /**
     * Adds to the 8-bit A register, the carry flag and data from the absolute address 
     * specified by the 16-bit register HL, and stores the result back into the A register.
     * 
     * INSTRUCTION LENGTH: 1 BYTE
     * MACHINE CYCLES: 2
     */
    pub fn ADC_A_HL(&mut self, memory: &Memory) {
        let hl_data = memory.read_byte(self.hl());
        let result = self.a + self.get_carry_flag() + hl_data;

        self.a = result;
        self.arithmetic_8bit_flag_update(result);
    }

    /**
     * Adds to the 8-bit A register, the carry flag and the immediate data n, 
     * and stores the result back into the A register.
     * 
     * INSTRUCTION LENGTH: 2 BYTE
     * MACHINE CYCLES: 2
     */
    pub fn ADC_A_u8(&mut self, immediate: u8) {
        let result = self.a + self.get_carry_flag() + immediate;

        self.a = result;
        self.arithmetic_8bit_flag_update(result);
    }

    /**
     * Adds to the 8-bit A register, the 8-bit register r, and stores the result 
     * back into the A register.
     * 
     * INSTRUCTION LENGTH: 1 BYTE
     * MACHINE CYCLES: 1
     */
    pub fn ADD_A_r8(&mut self, reg: u8) {
        let result = self.a + reg;

        self.a = result;
        self.arithmetic_8bit_flag_update(result);
    }

    /**
     * Adds to the 8-bit A register, data from the absolute address specified by 
     * the 16-bit register HL, and stores the result back into the A register.
     * 
     * INSTRUCTION LENGTH: 1 BYTE
     * MACHINE CYCLES: 2
     */
    pub fn ADD_A_HL(&mut self, memory: &Memory) {
        let hl_data = memory.read_byte(self.hl());
        let result = self.a + hl_data;

        self.a = result;
        self.arithmetic_8bit_flag_update(result);
    }

    /**
     * Adds to the 8-bit A register, the immediate data n, and stores the result 
     * back into the A register
     * 
     * INSTRUCTION LENGTH: 2 BYTES
     * MACHINE CYCLES: 2
     */
    pub fn ADD_A_u8(&mut self, immediate: u8) {
        let result = self.a + immediate;

        self.a = result;
        self.arithmetic_8bit_flag_update(result);
    }

    /**
     * Adds the value in the passed 16-bit register to the 16-bit HL register
     * and stores the result back in the HL register
     * 
     * INSTRUCTION LENGTH: 1 BYTE
     * MACHINE CYCLES: 2
     */
    pub fn ADD_HL_r16(&mut self, reg: u16) {
        let result = self.hl() + reg;

        self.set_hl(result);
        self.arithmetic_16bit_flag_update(result);
    }

    /**
     * Add the 16-bit value in SP to 16-bit HL register.
     * 
     * INSTRUCTION LENGTH: 1
     * MACHINE CYCLES: 2
     */
    pub fn ADD_HL_SP(&mut self) {
        let result = self.sp + self.hl();

        self.set_hl(result);
        self.arithmetic_16bit_flag_update(result);
    }

    /**
     * Add the 8-bit signed value i8 to 16-bit SP register.
     */
    pub fn ADD_SP_i8(&mut self, immediate: i8) {
        let result: u16;

        if immediate < 0 {
            result = self.sp 
        } else {
            result = self.sp + immediate as u16;
        }
    }

























    fn arithmetic_8bit_flag_update(&mut self, result: u8) {
        //N = 0
        self.reset_add_sub_flag();

        //Z
        if result == 0 {
            self.set_zero_flag();
        } else {
            self.reset_zero_flag();
        }

        //H
        if Self::get_bit(result, 3) != 0 {
            self.set_half_carry_flag();
        } else {
            self.reset_half_carry_flag();
        }

        //C
        if Self::get_bit(result, 7) != 0 {
            self.set_carry_flag();
        } else {
            self.reset_carry_flag();
        }
    }

    fn arithmetic_16bit_flag_update(&mut self, result: u16) {
        //N = 0
        self.reset_add_sub_flag();

        //H
        if Self::get_bit_16(result, 11) != 0 {
            self.set_half_carry_flag();
        } else {
            self.reset_half_carry_flag();
        }

        //C
        if Self::get_bit_16(result, 15) != 0 {
            self.set_carry_flag();
        } else {
            self.reset_carry_flag();
        }
    }

}


