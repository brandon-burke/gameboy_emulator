struct Joypad {
    input: u8,
}

/**
 * Bit 7 - Not used
 * Bit 6 - Not used
 * Bit 5 - P15 Select Action buttons     (0=Select)
 * Bit 4 - P14 Select Direction buttons  (0=Select)
 * Bit 3 - P13 Input: Down  or Start     (0=Pressed) (Read Only)
 * Bit 2 - P12 Input: Up    or Select    (0=Pressed) (Read Only)
 * Bit 1 - P11 Input: Left  or B         (0=Pressed) (Read Only)
 * Bit 0 - P10 Input: Right or A         (0=Pressed) (Read Only)
 */ 
