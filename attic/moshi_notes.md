
PS:
& "C:/Program Files (x86)/Steam/steamapps/common/SteamVR/bin/win64/vrpathreg.exe" adddriver "C:/dev/driver_leap/build/leap"



CLeapController::UpdateTransformation seems to just be getting the controller pose and no finger poses.
    Uses CDriverConfig::GetHandsRotationOffset


driver_leap was originally setting the wrist pose directly to the open pose wrist pose, which is correct for Index controllers 



    {{-0.000632f, 0.026866f, 0.015002f, 1.000000f}, {0.421979f, -0.644251f, 0.422133f, 0.478202f}}, // Index
    {{0.000632f, 0.026866f, 0.015002f, 1.000000f}, {0.644251f, 0.421979f, -0.478202f, 0.422133f}}, // Index

6442: left - right +
4219: left + right +
4782: left + right -
4221: left + right +



    {{-0.000632f, 0.026866f, 0.015002f, 1.000000f}, {0.421979f, -0.644251f, 0.422133f, 0.478202f}}, // Index
    {{-0.002177f, 0.007120f, 0.016319f, 1.000000f}, {0.541276f, -0.546723f, 0.460749f, 0.442520f}}, // Middle
    {{-0.000513f, -0.006545f, 0.016348f, 1.000000f}, {0.550143f, -0.516692f, 0.429888f, 0.495548f}}, // Ring
    {{0.002478f, -0.018981f, 0.015214f, 1.000000f}, {0.523940f, -0.526918f, 0.326740f, 0.584025f}}, // Pinky