use crate::cpu::Cpu;
use crate::memory::Memory;

struct Gameboy {
    cpu: Cpu,
    memory: Memory,
}

impl Gameboy {
    pub fn new() -> Self {
        Self {
            cpu: Cpu::new(),
            memory: Memory::new(),
        }
    }

    pub fn run(&mut self) {
         
    }

    /**
     * Represents 1 clk cycle NOT a machine cycle
     */
    fn clk_cycle(&mut self) {
        self.cpu.cycle(&mut self.memory);
    }
}