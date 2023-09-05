pub fn get_bit(value: u8, bit_position: u8) -> u8 {
    return (value >> bit_position) & 0x1;
}

pub fn get_bit_16(value: u16, bit_position: u8) -> u16 {
    return (value >> bit_position) & 0x1;
}

pub fn build_16bit_num(upper_byte: u8, lower_byte: u8) -> u16 {
    return (upper_byte as u16) << 8 | lower_byte as u16;
}