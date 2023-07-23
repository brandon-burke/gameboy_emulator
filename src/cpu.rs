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

    pub fn de(&self) -> u16 {
        let mut de: u16; 

        de = (self.d as u16) << 8;
        de |= self.e as u16;

        return de;
    }
    pub fn hl(&self) -> u16 {
        let mut hl: u16; 

        hl = (self.h as u16) << 8;
        hl |= self.l as u16;

        return hl;
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
}


