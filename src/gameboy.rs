use crate::cpu::Cpu;
use crate::memory::Memory;
use crate::timer::Timer;

struct Gameboy {
    cpu: Cpu,
    memory: Memory,
    timer: Timer,
}

impl Gameboy {
    pub fn new() -> Self {
        Self {
            cpu: Cpu::new(),
            memory: Memory::new(),
            timer: Timer::new(),
        }
    }

    pub fn run(&mut self) {
         
    }

    /**
     * Represents 1 clk cycle NOT a machine cycle
     */
    fn clk_cycle(&mut self) {
        self.cpu.cycle(&mut self.memory);
        self.timer.cycle(&mut self.memory);
    }
}