struct Memory {
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