# SDK calibration snapshot

`joint_calib.txt` is copied unchanged from the local sibling
`r1-7a_sdk/config/joint_calib.txt` at Git commit
`2c89fe4f60f0c98f1bc06795cb2477cc73d0c92c`.

It contains SDK coordinate conversion parameters, not a new calibration of the
user's physical arm. Verify them against that arm's actual calibration before
hardware deployment. The offline command codec follows `src/arm_motor_bus.cpp`
and `thirdparty/motor_controller/src/MotorProtocol.cpp` at the same revision.

The default motor IDs are 0 through 6. Their ratios are 32, 32, 38/3, 38/3, 38/3,
16, 16. A changed motor-ID mapping requires changing the codec's ratio mapping.

The calibration snapshot and SDK-derived coordinate conventions retain the SDK BSD-3-Clause terms; see [SDK_LICENSE](SDK_LICENSE). `sdk_protocol.py` is an offline Python implementation of those conventions, not the SDK serial driver.
