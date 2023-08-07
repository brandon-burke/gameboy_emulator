pub mod cpu;
pub mod memory;
pub mod gameboy;
pub mod opcodes;
pub mod timer;

use std::env;
use std::fs;

fn main() {
    let args: Vec<String> = env::args().collect();
    let rom_dump = fs::read(&args[1]).expect("Error reading rom binary file");
    let mut cpu = cpu::Cpu::new();
    let mut memory = memory::Memory::new();
    let mut address: u32 = 0x0;

    //Loading the ROM
    for byte in rom_dump {
        println!("Address: {:04X}, Value: {:04X}", address, byte);
        address += 1;
    }
}

const LOOKUP_TABLE: [Opcode; 256] = [
    //0                //1             //2             //3               //4               //5              //6             //7             //8             //9                 //A             //B                 //C             //D                 //E            //F
Opcode::NOP(1),  Opcode::LD(3),  Opcode::LD(1),  Opcode::INC(1),   Opcode::INC(1),   Opcode::DEC(1),  Opcode::LD(2),   Opcode::RLCA(1), Opcode::LD(3),  Opcode::ADD(1),  Opcode::LD(1),  Opcode::DEC(1),    Opcode::INC(1),   Opcode::DEC(1),   Opcode::LD(2),  Opcode::RRCA(1),
Opcode::STOP(2), Opcode::LD(3),  Opcode::LD(1),  Opcode::INC(1),   Opcode::INC(1),   Opcode::DEC(1),  Opcode::LD(2),   Opcode::RLA(1),  Opcode::JR(2),  Opcode::ADD(1),  Opcode::LD(1),  Opcode::DEC(1),    Opcode::INC(1),   Opcode::DEC(1),   Opcode::LD(2),  Opcode::RRA(1),
Opcode::JR(2),   Opcode::LD(3),  Opcode::LD(1),  Opcode::INC(1),   Opcode::INC(1),   Opcode::DEC(1),  Opcode::LD(2),   Opcode::DAA(1),  Opcode::JR(2),  Opcode::ADD(1),  Opcode::LD(1),  Opcode::DEC(1),    Opcode::INC(1),   Opcode::DEC(1),   Opcode::LD(2),  Opcode::CPL(1),
Opcode::JR(2),   Opcode::LD(3),  Opcode::LD(1),  Opcode::INC(1),   Opcode::INC(1),   Opcode::DEC(1),  Opcode::LD(2),   Opcode::SCF(1),  Opcode::JR(2),  Opcode::ADD(1),  Opcode::LD(1),  Opcode::DEC(1),    Opcode::INC(1),   Opcode::DEC(1),   Opcode::LD(2),  Opcode::CCF(1),
Opcode::LD(1),   Opcode::LD(1),  Opcode::LD(1),  Opcode::LD(1),    Opcode::LD(1),    Opcode::LD(1),   Opcode::LD(1),   Opcode::LD(1),   Opcode::LD(1),  Opcode::LD(1),   Opcode::LD(1),  Opcode::LD(1),     Opcode::LD(1),    Opcode::LD(1),    Opcode::LD(1),  Opcode::LD(1),
Opcode::LD(1),   Opcode::LD(1),  Opcode::LD(1),  Opcode::LD(1),    Opcode::LD(1),    Opcode::LD(1),   Opcode::LD(1),   Opcode::LD(1),   Opcode::LD(1),  Opcode::LD(1),   Opcode::LD(1),  Opcode::LD(1),     Opcode::LD(1),    Opcode::LD(1),    Opcode::LD(1),  Opcode::LD(1),
Opcode::LD(1),   Opcode::LD(1),  Opcode::LD(1),  Opcode::LD(1),    Opcode::LD(1),    Opcode::LD(1),   Opcode::LD(1),   Opcode::LD(1),   Opcode::LD(1),  Opcode::LD(1),   Opcode::LD(1),  Opcode::LD(1),     Opcode::LD(1),    Opcode::LD(1),    Opcode::LD(1),  Opcode::LD(1),
Opcode::LD(1),   Opcode::LD(1),  Opcode::LD(1),  Opcode::LD(1),    Opcode::LD(1),    Opcode::LD(1),   Opcode::HALT(1), Opcode::LD(1),   Opcode::LD(1),  Opcode::LD(1),   Opcode::LD(1),  Opcode::LD(1),     Opcode::LD(1),    Opcode::LD(1),    Opcode::LD(1),  Opcode::LD(1),
Opcode::ADD(1),  Opcode::ADD(1), Opcode::ADD(1), Opcode::ADD(1),   Opcode::ADD(1),   Opcode::ADD(1),  Opcode::ADD(1),  Opcode::ADD(1),  Opcode::ADC(1), Opcode::ADC(1),  Opcode::ADC(1), Opcode::ADC(1),    Opcode::ADC(1),   Opcode::ADC(1),   Opcode::ADC(1), Opcode::ADC(1),
Opcode::SUB(1),  Opcode::SUB(1), Opcode::SUB(1), Opcode::SUB(1),   Opcode::SUB(1),   Opcode::SUB(1),  Opcode::SUB(1),  Opcode::SUB(1),  Opcode::SBC(1), Opcode::SBC(1),  Opcode::SBC(1), Opcode::SBC(1),    Opcode::SBC(1),   Opcode::SBC(1),   Opcode::SBC(1), Opcode::SBC(1),
Opcode::AND(1),  Opcode::AND(1), Opcode::AND(1), Opcode::AND(1),   Opcode::AND(1),   Opcode::AND(1),  Opcode::AND(1),  Opcode::AND(1),  Opcode::XOR(1), Opcode::XOR(1),  Opcode::XOR(1), Opcode::XOR(1),    Opcode::XOR(1),   Opcode::XOR(1),   Opcode::XOR(1), Opcode::XOR(1),
Opcode::OR(1),   Opcode::OR(1),  Opcode::OR(1),  Opcode::OR(1),    Opcode::OR(1),    Opcode::OR(1),   Opcode::OR(1),   Opcode::OR(1),   Opcode::CP(1),  Opcode::CP(1),   Opcode::CP(1),  Opcode::CP(1),     Opcode::CP(1),    Opcode::CP(1),    Opcode::CP(1),  Opcode::CP(1),
Opcode::RET(1),  Opcode::POP(1), Opcode::JP(3),  Opcode::JP(3),    Opcode::CALL(3),  Opcode::PUSH(1), Opcode::ADD(2),  Opcode::RST(1),  Opcode::RET(1), Opcode::RET(1),  Opcode::JP(3),  Opcode::PREFIX(2), Opcode::CALL(3),  Opcode::CALL(3),  Opcode::ADC(2), Opcode::RST(1),
Opcode::RET(1),  Opcode::POP(1), Opcode::JP(3),  Opcode::EMPTY(1), Opcode::CALL(3),  Opcode::PUSH(1), Opcode::SUB(2),  Opcode::RST(1),  Opcode::RET(1), Opcode::RETI(1), Opcode::JP(3),  Opcode::EMPTY(1),  Opcode::CALL(3),  Opcode::EMPTY(1), Opcode::SBC(2), Opcode::RST(1),
Opcode::LDH(2),  Opcode::POP(1), Opcode::LD(1),  Opcode::EMPTY(1), Opcode::EMPTY(1), Opcode::PUSH(1), Opcode::AND(2),  Opcode::RST(1),  Opcode::ADD(2), Opcode::JP(1),   Opcode::LD(3),  Opcode::EMPTY(1),  Opcode::EMPTY(1), Opcode::EMPTY(1), Opcode::XOR(2), Opcode::RST(1),
Opcode::LDH(2),  Opcode::POP(1), Opcode::LD(1),  Opcode::DI(1),    Opcode::EMPTY(1), Opcode::PUSH(1), Opcode::OR(2),   Opcode::RST(1),  Opcode::LD(2),  Opcode::LD(1),   Opcode::LD(3),  Opcode::EI(1),     Opcode::EMPTY(1), Opcode::EMPTY(1), Opcode::CP(2),  Opcode::RST(1),
];

const LOOKUP_TABLE_PREFIX: [Opcode; 256] = [
Opcode::RLC(2),  Opcode::RLC(2),  Opcode::RLC(2),  Opcode::RLC(2),   Opcode::RLC(2),   Opcode::RLC(2),  Opcode::RLC(2),  Opcode::RLC(2),  Opcode::RRC(2), Opcode::RRC(2), Opcode::RRC(2), Opcode::RRC(2), Opcode::RRC(2), Opcode::RRC(2), Opcode::RRC(2), Opcode::RRC(2),
Opcode::RL(2),   Opcode::RL(2),   Opcode::RL(2),   Opcode::RL(2),    Opcode::RL(2),    Opcode::RL(2),   Opcode::RL(2),   Opcode::RL(2),   Opcode::RR(2),  Opcode::RR(2),  Opcode::RR(2),  Opcode::RR(2),  Opcode::RR(2),  Opcode::RR(2),  Opcode::RR(2),  Opcode::RR(2),
Opcode::SLA(2),  Opcode::SLA(2),  Opcode::SLA(2),  Opcode::SLA(2),   Opcode::SLA(2),   Opcode::SLA(2),  Opcode::SLA(2),  Opcode::SLA(2),  Opcode::SRA(2), Opcode::SRA(2), Opcode::SRA(2), Opcode::SRA(2), Opcode::SRA(2), Opcode::SRA(2), Opcode::SRA(2), Opcode::SRA(2),
Opcode::SWAP(2), Opcode::SWAP(2), Opcode::SWAP(2), Opcode::SWAP(2),  Opcode::SWAP(2),  Opcode::SWAP(2), Opcode::SWAP(2), Opcode::SWAP(2), Opcode::SRL(2), Opcode::SRL(2), Opcode::SRL(2), Opcode::SRL(2), Opcode::SRL(2), Opcode::SRL(2), Opcode::SRL(2), Opcode::SRL(2),
Opcode::BIT(2),  Opcode::BIT(2),  Opcode::BIT(2),  Opcode::BIT(2),   Opcode::BIT(2),   Opcode::BIT(2),  Opcode::BIT(2),  Opcode::BIT(2),  Opcode::BIT(2), Opcode::BIT(2), Opcode::BIT(2), Opcode::BIT(2), Opcode::BIT(2), Opcode::BIT(2), Opcode::BIT(2), Opcode::BIT(2),
Opcode::BIT(2),  Opcode::BIT(2),  Opcode::BIT(2),  Opcode::BIT(2),   Opcode::BIT(2),   Opcode::BIT(2),  Opcode::BIT(2),  Opcode::BIT(2),  Opcode::BIT(2), Opcode::BIT(2), Opcode::BIT(2), Opcode::BIT(2), Opcode::BIT(2), Opcode::BIT(2), Opcode::BIT(2), Opcode::BIT(2),
Opcode::BIT(2),  Opcode::BIT(2),  Opcode::BIT(2),  Opcode::BIT(2),   Opcode::BIT(2),   Opcode::BIT(2),  Opcode::BIT(2),  Opcode::BIT(2),  Opcode::BIT(2), Opcode::BIT(2), Opcode::BIT(2), Opcode::BIT(2), Opcode::BIT(2), Opcode::BIT(2), Opcode::BIT(2), Opcode::BIT(2),
Opcode::BIT(2),  Opcode::BIT(2),  Opcode::BIT(2),  Opcode::BIT(2),   Opcode::BIT(2),   Opcode::BIT(2),  Opcode::BIT(2),  Opcode::BIT(2),  Opcode::BIT(2), Opcode::BIT(2), Opcode::BIT(2), Opcode::BIT(2), Opcode::BIT(2), Opcode::BIT(2), Opcode::BIT(2), Opcode::BIT(2),
Opcode::RES(2),  Opcode::RES(2),  Opcode::RES(2),  Opcode::RES(2),   Opcode::RES(2),   Opcode::RES(2),  Opcode::RES(2),  Opcode::RES(2),  Opcode::RES(2), Opcode::RES(2), Opcode::RES(2), Opcode::RES(2), Opcode::RES(2), Opcode::RES(2), Opcode::RES(2), Opcode::RES(2),
Opcode::RES(2),  Opcode::RES(2),  Opcode::RES(2),  Opcode::RES(2),   Opcode::RES(2),   Opcode::RES(2),  Opcode::RES(2),  Opcode::RES(2),  Opcode::RES(2), Opcode::RES(2), Opcode::RES(2), Opcode::RES(2), Opcode::RES(2), Opcode::RES(2), Opcode::RES(2), Opcode::RES(2),
Opcode::RES(2),  Opcode::RES(2),  Opcode::RES(2),  Opcode::RES(2),   Opcode::RES(2),   Opcode::RES(2),  Opcode::RES(2),  Opcode::RES(2),  Opcode::RES(2), Opcode::RES(2), Opcode::RES(2), Opcode::RES(2), Opcode::RES(2), Opcode::RES(2), Opcode::RES(2), Opcode::RES(2),
Opcode::RES(2),  Opcode::RES(2),  Opcode::RES(2),  Opcode::RES(2),   Opcode::RES(2),   Opcode::RES(2),  Opcode::RES(2),  Opcode::RES(2),  Opcode::RES(2), Opcode::RES(2), Opcode::RES(2), Opcode::RES(2), Opcode::RES(2), Opcode::RES(2), Opcode::RES(2), Opcode::RES(2),
Opcode::SET(2),  Opcode::SET(2),  Opcode::SET(2),  Opcode::SET(2),   Opcode::SET(2),   Opcode::SET(2),  Opcode::SET(2),  Opcode::SET(2),  Opcode::SET(2), Opcode::SET(2), Opcode::SET(2), Opcode::SET(2), Opcode::SET(2), Opcode::SET(2), Opcode::SET(2), Opcode::SET(2),
Opcode::SET(2),  Opcode::SET(2),  Opcode::SET(2),  Opcode::SET(2),   Opcode::SET(2),   Opcode::SET(2),  Opcode::SET(2),  Opcode::SET(2),  Opcode::SET(2), Opcode::SET(2), Opcode::SET(2), Opcode::SET(2), Opcode::SET(2), Opcode::SET(2), Opcode::SET(2), Opcode::SET(2),
Opcode::SET(2),  Opcode::SET(2),  Opcode::SET(2),  Opcode::SET(2),   Opcode::SET(2),   Opcode::SET(2),  Opcode::SET(2),  Opcode::SET(2),  Opcode::SET(2), Opcode::SET(2), Opcode::SET(2), Opcode::SET(2), Opcode::SET(2), Opcode::SET(2), Opcode::SET(2), Opcode::SET(2),
Opcode::SET(2),  Opcode::SET(2),  Opcode::SET(2),  Opcode::SET(2),   Opcode::SET(2),   Opcode::SET(2),  Opcode::SET(2),  Opcode::SET(2),  Opcode::SET(2), Opcode::SET(2), Opcode::SET(2), Opcode::SET(2), Opcode::SET(2), Opcode::SET(2), Opcode::SET(2), Opcode::SET(2)
];

#[derive(Debug)]
enum Opcode {
ADC(u8),
ADD(u8), 
AND(u8),
CP(u8),
DEC(u8),
INC(u8),
OR(u8),
SBC(u8),
SUB(u8),
XOR(u8),
BIT(u8),
RES(u8),
SET(u8),
SWAP(u8),
RL(u8),
RLA(u8),
RLC(u8),
RLCA(u8),
RR(u8),
RRA(u8),
RRC(u8),
RRCA(u8),
SLA(u8),
SRA(u8),
SRL(u8),
LD(u8),
LDH(u8),
CALL(u8),
JP(u8),
JR(u8),
RET(u8),
RETI(u8),
RST(u8),
POP(u8),
PUSH(u8),
CCF(u8),
CPL(u8),
DAA(u8),
DI(u8),
EI(u8),
HALT(u8),
NOP(u8),
SCF(u8),
STOP(u8),
PREFIX(u8),
EMPTY(u8),
}

impl Opcode {
    fn get_value(&self) -> u8 {
        match self {
            Opcode::ADC(num_of_bytes) => *num_of_bytes,
            Opcode::ADD(num_of_bytes) => *num_of_bytes,
            Opcode::AND(num_of_bytes) => *num_of_bytes,
            Opcode::CP(num_of_bytes) => *num_of_bytes,
            Opcode::DEC(num_of_bytes) => *num_of_bytes,
            Opcode::INC(num_of_bytes) => *num_of_bytes,
            Opcode::OR(num_of_bytes) => *num_of_bytes,
            Opcode::SBC(num_of_bytes) => *num_of_bytes,
            Opcode::SUB(num_of_bytes) => *num_of_bytes,
            Opcode::XOR(num_of_bytes) => *num_of_bytes,
            Opcode::BIT(num_of_bytes) => *num_of_bytes,
            Opcode::RES(num_of_bytes) => *num_of_bytes,
            Opcode::SET(num_of_bytes) => *num_of_bytes,
            Opcode::SWAP(num_of_bytes) => *num_of_bytes,
            Opcode::RL(num_of_bytes) => *num_of_bytes,
            Opcode::RLA(num_of_bytes) => *num_of_bytes,
            Opcode::RLC(num_of_bytes) => *num_of_bytes,
            Opcode::RLCA(num_of_bytes) => *num_of_bytes,
            Opcode::RR(num_of_bytes) => *num_of_bytes,
            Opcode::RRA(num_of_bytes) => *num_of_bytes,
            Opcode::RRC(num_of_bytes) => *num_of_bytes,
            Opcode::RRCA(num_of_bytes) => *num_of_bytes,
            Opcode::SLA(num_of_bytes) => *num_of_bytes,
            Opcode::SRA(num_of_bytes) => *num_of_bytes,
            Opcode::SRL(num_of_bytes) => *num_of_bytes,
            Opcode::LD(num_of_bytes) => *num_of_bytes,
            Opcode::LDH(num_of_bytes) => *num_of_bytes,
            Opcode::CALL(num_of_bytes) => *num_of_bytes,
            Opcode::JP(num_of_bytes) => *num_of_bytes,
            Opcode::JR(num_of_bytes) => *num_of_bytes,
            Opcode::RET(num_of_bytes) => *num_of_bytes,
            Opcode::RETI(num_of_bytes) => *num_of_bytes,
            Opcode::RST(num_of_bytes) => *num_of_bytes,
            Opcode::POP(num_of_bytes) => *num_of_bytes,
            Opcode::PUSH(num_of_bytes) => *num_of_bytes,
            Opcode::CCF(num_of_bytes) => *num_of_bytes,
            Opcode::CPL(num_of_bytes) => *num_of_bytes,
            Opcode::DAA(num_of_bytes) => *num_of_bytes,
            Opcode::DI(num_of_bytes) => *num_of_bytes,
            Opcode::EI(num_of_bytes) => *num_of_bytes,
            Opcode::HALT(num_of_bytes) => *num_of_bytes,
            Opcode::NOP(num_of_bytes) => *num_of_bytes,
            Opcode::SCF(num_of_bytes) => *num_of_bytes,
            Opcode::STOP(num_of_bytes) => *num_of_bytes,
            Opcode::PREFIX(num_of_bytes) => *num_of_bytes,
            Opcode::EMPTY(num_of_bytes) => *num_of_bytes,
        }
    }

}



