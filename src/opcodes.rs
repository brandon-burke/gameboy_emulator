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

impl Opcode {
    //Load/Copy value in right register to left register
    fn ld_r8_r8(right: &u8, left: &u8) {

    }

}