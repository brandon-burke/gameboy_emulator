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

    //main func
    pub fn main(&mut self) {
        let gameboy = Gameboy::new();
        while true {
            self.clk_cycle();
        }

        
    }

    /**
     * Represents 1 clk cycle NOT a machine cycle
     */
    fn clk_cycle(&mut self) {
        self.timer.cycle();
        //time_cycle
        //cpu_cycle
        //gpu_cycle
        //LCD_cycle
    }
}