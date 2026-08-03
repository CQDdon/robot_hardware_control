# Operator-level permission. Hardware-side HSH permission is still mandatory.
bool motion_enable
# True holds a safe command until a false command arrives.
bool emergency_stop
# Bit mask for firmware digital outputs Q0..Q7. Bit 0 controls Q0; bit 7 controls Q7.
uint8 io_output_mask
