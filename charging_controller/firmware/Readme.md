# Charging Controller Firmware

First 4 bits of a CAN message ID correspond to Command ID. Next 7 bits correspond to NodeID of the board, which is currently hardcoded to `0x60`. All messages except `RESP Status` have no data.

| Command                   | Command ID | Description                                                                                                                                                                                                                       |
|---------------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| CMD Attempt charging      | 0x00       | Requests the controller to signal the docking station to start charging.                                                                                                                                                          |
| CMD Stop charging         | 0x01       | Requests the controller to stop charging (cut off batteries).                                                                                                                                                                     |
| RESP Charging success     | 0x08       | Sent after detecting sufficient current on the charging probes during the charging sequence.                                                                                                                                      |
| RESP Charging fail        | 0x09       | Sent after a timeout during the charging sequence if no current is detected on the charging probes.                                                                                                                               |
| RESP Bus off recovery     | 0x0A       | Sent after recovering from a CAN "bus off" error.                                                                                                                                                                                 |
| RESP Overheated           | 0x0B       | Sent after measuring a temperature above the safe threshold. (Note: after overheating all commands will be ignored.)                                                                                     |
| RESP Overheat recovery    | 0x0C       | Sent after the temperature drops below the safe threshold.                                                                                                                                                                        |
| RESP Fatal error recovery | 0x0D       | Sent after a reboot caused by an unhandled error.                                                                                                                                                                                 |
| RESP Status               | 0x0F       | Sent after receiving an RTR message. First byte encodes the temperature in full degrees (int8_t). Second byte encodes CHARGE_SENSE pin status (0x00 for SENSE_NOT_CHARGING 0xFF for SENSE_CHARGING) |

## Improvements and Considerations

It would be beneficial to adopt a standardized configuration format for defining CAN messages rather than hard-coding them.

Currently, CAN packet losses are not detected or handled. The existing CAN ACK is insufficient because it only confirms that the frame was received by any device, not necessarily the intended recipient. There is no mechanism to manually acknowledge receipt or to request the current state of the charging station. A potential workaround could involve adding flags and CAN messages to request and respond with the current state of the charging controller.