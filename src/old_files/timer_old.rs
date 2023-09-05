use crate::memory::Memory;

const TIMER_DIV_REG: u16 = 0xFF04;
const TIMER_TIMA_REG: u16 = 0xFF05;
const TIMER_TMA_REG: u16 = 0xFF06;
const TIMER_TAC_REG: u16 = 0xFF07;
const IO_START: u16 = 0xFF00;
const IO_END: u16 = 0xFF7F;
const INTERRUPT_FLAG: u16 = 0xFF0F;
const MACHINE_CYCLE: u8 = 4;

 /**
  * NOTE THIS STRUCT DOESN'T REALLY USE THE MEMORY FUNCTIONS, BUT RATHER DIRECTLY MESSES
  * WITH THE MEMORY
  */

pub struct Timer {
    div_reg: u16,           //Note the actual DIV reg is the upper 8 bits. But in reality it is a system counter
    prev_mux_input: u8,     //This is the value of the previous msbit according to what the tac frquency was
    overflowed: bool,       
    clk_since_overflow: u8,
    ignore_tima_write: bool,
    clk_ignore_tima_write: u8,
    delayed_update: bool,

    tima_clk_cycles: u16,
    old_tma_value: u8,
    tma_write_occurred: bool,
}

impl Timer {
    pub fn new() -> Self {
        Timer {
            div_reg: 0,
            prev_mux_input: 0,
            overflowed: false,
            clk_since_overflow: 0,
            ignore_tima_write: false,
            clk_ignore_tima_write: 0,
            delayed_update: false,
            


            tima_clk_cycles: 0,
            old_tma_value: 0,
            tma_write_occurred: false,
        }
    }

    pub fn cycle(&mut self, memory: &mut Memory) {
        if self.overflowed {
            self.clk_since_overflow += 1;

            if self.clk_since_overflow == MACHINE_CYCLE {
                if !memory.tima_write {
                    self.delayed_update = true;
                }
                
                self.ignore_tima_write = true;
                self.clk_since_overflow = 0;
                self.overflowed = false;
                memory.tima_write = false; //probably should have the memory class do this an reset this on every write at the beginning
                memory.tima_write_ignore = true;
            }
        }   

        if self.delayed_update {
            self.tima_clk_cycles += 1;

            if self.tima_clk_cycles == MACHINE_CYCLE {
                let mut timer_interrupt_flag = memory.io[(INTERRUPT_FLAG - IO_START) as usize];
                timer_interrupt_flag |= 0b00000100;
                memory.io[(INTERRUPT_FLAG - IO_START) as usize] = timer_interrupt_flag;
                memory.io[(TIMER_TIMA_REG - IO_START) as usize] = memory.io[(TIMER_TMA_REG - IO_START) as usize];   //banking that the tma register would already be changed by the time it gets here

                self.delayed_update = false;
            }

        }

        self.inc_divder_register(memory);

        memory.timer_div_reg_write = false;
        self.prev_mux_input = Self::get_msb_of_current_clk_freq(&self, memory);
        todo!("1.) Need to have register reset if STOP instruction\n2.) Need to have register reset if STOP instruction\n 3.) This also happens during a speed switch");
    }

    fn inc_divder_register(&mut self, memory: &mut Memory) {
        if memory.timer_div_reg_write {
            self.div_reg = 0;
        } else {
            self.div_reg += 1;
            memory.io[(TIMER_DIV_REG - IO_START) as usize] = (self.div_reg >> 8) as u8;
        } 
        if ((Self::is_timer_enabled(memory) && self.did_falling_edge_occur(memory)) || 
            self.get_msb_of_current_clk_freq(memory) == 1 && memory.timer_enable_falling_edge) && !self.overflowed {

            self.inc_timer_counter(memory);
        }
    }

    fn did_falling_edge_occur(&self, memory: &Memory) -> bool {
        if self.prev_mux_input == 1 && self.get_msb_of_current_clk_freq(memory) == 0 {
            return true;
        }
        
        return false;
    }

    fn inc_timer_counter(&mut self, memory: &mut Memory) {
        let (incremented_time, overflowed) = memory.io[(TIMER_TIMA_REG - IO_START) as usize].overflowing_add(1);
        self.overflowed = overflowed;

        memory.io[(TIMER_TIMA_REG - IO_START) as usize] = incremented_time;
    }

    fn is_timer_enabled(memory: &Memory) -> bool {
        let time_enable_bit = (memory.io[(TIMER_TAC_REG - IO_START) as usize] >> 2) & 0x1;

        if time_enable_bit == 0 {
            return false;
        } else {
            return true;
        }
    }

    /**
     * Returns the value to divide the main clk speed by for the timer counter
     */
    fn get_clk_frequency(memory: &Memory) -> u16 {
        let clk_frequency = memory.io[(TIMER_TAC_REG - IO_START) as usize] & 0x3;
        match clk_frequency {
            0x00 => 1024,
            0x01 => 16,
            0x02 => 64,
            0x03 => 256,
            _ => panic!("reading an unknown clk frequency"),
        }
    }

    fn get_msb_of_current_clk_freq(&self, memory: &Memory) -> u8 {
        match Self::get_clk_frequency(memory) {
            0x00 => ((self.div_reg >> 9) & 0x1) as u8,
            0x01 => ((self.div_reg >> 3) & 0x1) as u8,
            0x02 => ((self.div_reg >> 5) & 0x1) as u8,
            0x03 => ((self.div_reg >> 7) & 0x1) as u8,
            _ => panic!("reading an unknown clk frequency"),
        }
    }

    /**
     * Let's you know if a glitch occurred to increase the TIMA register
     */
    fn did_glitch_occur(&mut self, memory: &mut Memory) -> bool {
        if memory.timer_div_reg_write || memory.timer_enable_falling_edge {
            if self.tima_clk_cycles >= Self::get_clk_frequency(memory)/2 {
                return true
            }
        }

        memory.timer_div_reg_write = false;
        memory.timer_enable_falling_edge = false;
        return false;
    }
}




