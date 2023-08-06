use core::panic;

const ROM_BANK_0_START: u16 = 0x0000;
const ROM_BANK_0_END: u16 = 0x3FFF;
const ROM_BANK_X_START: u16 = 0x4000;
const ROM_BANK_X_END: u16 = 0x7FFF;
const VRAM_START: u16 = 0x8000;
const VRAM_END: u16 = 0x9FFF;
const SRAM_START: u16 = 0xA000;
const SRAM_END: u16 = 0xBFFF;
const WRAM_0_START: u16 = 0xC000;
const WRAM_0_END: u16 = 0xCFFF;
const WRAM_X_START: u16 = 0xD000;
const WRAM_X_END: u16 = 0xDFFF;
const ECHO_START: u16 = 0xE000;
const ECHO_END: u16 = 0xFDFF;
const OAM_START: u16 = 0xFE00;
const OAM_END: u16 = 0xFE9F;
const UNUSED_START: u16 = 0xFEA0;
const UNUSED_END: u16 = 0xFEFF;
const IO_START: u16 = 0xFF00;
const IO_END: u16 = 0xFF7F;
const HRAM_START: u16 = 0xFF80;
const HRAM_END: u16 = 0xFFFE;
const INTERRUPT_ENABLE_START: u16 = 0xFFFF;

pub struct Memory {
    rom_bank_0: [u8; 0x4000],   //16KB -> 0000h – 3FFFh (Non-switchable ROM bank)
    rom_bank_x: [u8; 0x4000],   //16KB -> 4000h – 7FFFh (Switchable ROM bank)
    vram: [u8; 0x2000],         //8KB  -> 8000h – 9FFFh (Video RAM)
    sram: [u8; 0x2000],         //8KB  -> A000h – BFFFh (External RAM in cartridge)
    wram_0: [u8; 0x1000],       //1KB  -> C000h – CFFFh (Work RAM)
    wram_x: [u8; 0x1000],       //1KB  -> D000h – DFFFh (Work RAM)
    echo: [u8; 0x1E00],         //     -> E000h – FDFFh (ECHO RAM) Mirror of C000h-DDFFh
    oam: [u8; 0xA0],            //     -> FE00h – FE9Fh (Object Attribute Table) Sprite information table
    unused: [u8; 0x60],         //     -> FEA0h – FEFFh (Unused)
    io: [u8; 0x80],             //     -> FF00h – FF7Fh (I/O registers)
    hram: [u8; 0x7F],           //     -> FF80h – FFFEh (HRAM)
    ie_reg: [u8; 0x1],          //     -> FFFFh         (Interrupt enable flags)
}

impl Memory {
    pub fn new() -> Self {
        Self {
            rom_bank_0: [0; 0x4000],   
            rom_bank_x: [0; 0x4000],   
            vram: [0; 0x2000],         
            sram: [0; 0x2000],        
            wram_0: [0; 0x1000],       
            wram_x: [0; 0x1000],   
            echo: [0; 0x1E00],        
            oam: [0; 0xA0],            
            unused: [0; 0x60],         
            io: [0; 0x80],             
            hram: [0; 0x7F],           
            ie_reg: [0; 0x1],          
        }
    }

    pub fn read_byte(&self, address: u16) -> u8 {
        match address {
            ROM_BANK_0_START ..= ROM_BANK_0_END => self.rom_bank_0[address as usize],
            ROM_BANK_X_START ..= ROM_BANK_X_END => self.rom_bank_x[(address - ROM_BANK_0_START) as usize],
            VRAM_START ..= VRAM_END => self.vram[(address - VRAM_START) as usize],
            SRAM_START ..= SRAM_END => self.sram[(address - SRAM_START) as usize],
            WRAM_0_START ..= WRAM_0_END => self.wram_0[(address - WRAM_0_START) as usize],
            WRAM_X_START ..= WRAM_X_END => self.wram_x[(address - WRAM_X_START) as usize],
            ECHO_START ..= ECHO_END => panic!("I don't think we should be accessing echo memory"),
            UNUSED_START ..= UNUSED_END => panic!("I don't think we should be accessing unused memory"),
            IO_START ..= IO_END => self.io[(address - IO_START) as usize],
            HRAM_START ..= HRAM_END => self.hram[(address - HRAM_START) as usize],
            INTERRUPT_ENABLE_START => self.ie_reg[(address - INTERRUPT_ENABLE_START) as usize],
            _ => panic!("MEMORY ACCESS OUT OF BOUNDS"),
        } 
    }

    pub fn write_byte(&mut self, address: u16, data_to_write: u8) {
        match address {
            ROM_BANK_0_START ..= ROM_BANK_0_END => self.rom_bank_0[address as usize] = data_to_write,
            ROM_BANK_X_START ..= ROM_BANK_X_END => self.rom_bank_x[(address - ROM_BANK_0_START) as usize] = data_to_write,
            VRAM_START ..= VRAM_END => self.vram[(address - VRAM_START) as usize] = data_to_write,
            SRAM_START ..= SRAM_END => self.sram[(address - SRAM_START) as usize] = data_to_write,
            WRAM_0_START ..= WRAM_0_END => self.wram_0[(address - WRAM_0_START) as usize] = data_to_write,
            WRAM_X_START ..= WRAM_X_END => self.wram_x[(address - WRAM_X_START) as usize] = data_to_write,
            ECHO_START ..= ECHO_END => panic!("I don't think we should be accessing echo memory"),
            UNUSED_START ..= UNUSED_END => panic!("I don't think we should be accessing unused memory"),
            IO_START ..= IO_END => self.io[(address - IO_START) as usize] = data_to_write,
            HRAM_START ..= HRAM_END => self.hram[(address - HRAM_START) as usize] = data_to_write,
            INTERRUPT_ENABLE_START => self.ie_reg[(address - INTERRUPT_ENABLE_START) as usize] = data_to_write,
            _ => panic!("MEMORY ACCESS OUT OF BOUNDS"),
        } 
    }
}