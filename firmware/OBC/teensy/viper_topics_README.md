# Viper-LoRa Payload Format

This document describes the payload string/packet that gets sent over LoRa, combining GPS, VIPER card voltages, and  overall health status.


### Card Output Voltages

- **C0 OVA**  
  - Description: Output Voltage “A” for Card 0.  
  - Type: `float` (two decimal places)  
  - Format: C0 OVA: <value>;

- **C1 OVA OVB**  
  - Description: Output Voltages “A” and “B” for Card 1.  
  - Type: two `float` values (each two decimal places)   
  - Format: `C1 OVA OVB: <valueA> <valueB>;`

- **C2 OVA OVB**  
  -  Description: Output Voltage “A” for Card 2.  
  - Type: `float` (two decimal places)  
  - Format: `C2 OVA OVB: <valueA> <valueB>;`

- **C3 OVA**  
  - Description: Output Voltage “A” for Card 0.  
  - Type: `float` (two decimal places)  
  - *Format*: `C3 OVA: <value>;`

### 2.2 Health Summary

- **EEPROM**  
  - Description: EEPROM status code (`uint8_t`)  
  - Format: `EEPROM: <code>,`

- **MUX**  
  - Description: Multiplexer status code (`uint8_t`)  
  - Format: `MUX: <code>,`

- **C0 C1 C2 C3**  
  - Description: Fault status for each card 
  - Format: `C0: <code>, C1: <code>, C2: <code>, C3: <code>,`

- **Input V**  
  - Description: System input voltage ( two decimal places)  
  - Format: `Input V: <value>`



