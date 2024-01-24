std_msgs/Header header
uint8 board_address         # board address
float32 temperature         # board temp in degrees C
uint32 bad_crcs_received    # bad checksums recieved by board
float32 main_supply_voltage # main supply voltage (24VDC)
bool leak_detect            # water detected on board sensor
float32[] current_feedback  # current feedback in amps
