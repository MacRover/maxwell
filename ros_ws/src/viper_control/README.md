# VIPER Bringup

## Launching VIPER Status Nodes

1. Build and source workspace, if necessary
2. Run 
```bash
ros2 launch viper_control viper_status_main.launch.py
```
This will launch 5 topics that will periodically update. 
```bash
/viper_status/card_0
/viper_status/card_1
/viper_status/card_2
/viper_status/card_3
/viper_status/health
```
**Note:** The node will continuously publish regardless of if VIPER is publishing data or not. If a VIPER card is disabled, the status message will publish the last received value


## Configuring VIPER Settings

1. In a separate terminal, run
```bash
ros2 run spidercan writer.py
```
2. In a separate terminal run
```bash
ros2 run viper_control viper_tool
```

This will allow you to modify multiple parameters.
Notable Parameters include
- `ENABLE_CARD`/`DISABLE_CARD`: Will Enable/Disable the card of choice. **NOTE:** Disabling Card 0 will shut off the jetson, ending the remote session
- `FREEZE`/`UNFREEZE`: Freezing the VIPER will prevent it from cycling through cards - it will only report data from the card the mux is set to. This increases the reporting frequency of one particular card 4x. 
- `SET_MUX_VALUE`: Sets the current value of the mux. Run this after `FREEZE` to select the card to get data from
- `SET_CARD_INTERVAL`: Sets the reporting interval in ms. If `FREEZE` is set, it is the reporting frequency of one card. If not set, it is frequency at which the next card reports. 


### Elec - VIPER Load Testing
After completing the following steps, run the following in `viper_tool`

1. `FREEZE`
2. `SET_MUX_VALUE`, followed by the desired card #
3. `SET_CARD_INTERVAL`. I've been able to get this to 100ms with no issues on a high power card (fewer messages sent)

These parameters are not stored in EEPROM, so will need to be set after each power cycle.
