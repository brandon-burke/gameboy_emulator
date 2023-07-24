mod cpu;
mod memory;

use crate::cpu::Cpu;

fn main() {
    let mut cpu = Cpu::new();

    // println!("a={:#02X}, f={:#02X}, af={:#02X}", cpu.a, cpu.f, cpu.af());
    // println!("b={:#02X}, c={:#02X}, bc={:#02X}", cpu.b, cpu.c, cpu.bc());
    // println!("d={:#02X}, e={:#02X}, de={:#02X}", cpu.d, cpu.e, cpu.de());
    // println!("h={:#02X}, l={:#02X}, hl={:#02X}", cpu.h, cpu.l, cpu.hl());
    
    cpu.f = 0xF0;
    println!("f={:#b}", cpu.f); 
    cpu.reset_zero_flag();
    println!("f={:#b}", cpu.f); 
    cpu.set_zero_flag();
    println!("f={:#b}", cpu.f);
    println!(" ");

    println!("f={:#b}", cpu.f); 
    cpu.reset_add_sub_flag();
    println!("f={:#b}", cpu.f); 
    cpu.set_add_sub_flag();
    println!("f={:#b}", cpu.f); 
    println!(" ");

    println!("f={:#b}", cpu.f); 
    cpu.reset_half_carry_flag();
    println!("f={:#b}", cpu.f); 
    cpu.set_half_carry_flag();
    println!("f={:#b}", cpu.f); 
    println!(" ");

    println!("f={:#b}", cpu.f); 
    cpu.reset_carry_flag();
    println!("f={:#b}", cpu.f); 
    cpu.set_carry_flag();
    println!("f={:#b}", cpu.f); 


}



