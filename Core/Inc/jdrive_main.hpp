#ifdef __cplusplus
extern "C" {
#endif

#ifndef INC_JDRIVE_MAIN_HPP_
#define INC_JDRIVE_MAIN_HPP_

enum ControlStatus
{
	STATUS_NONE,
	STATUS_MOTORCONTROL,
	STATUS_CALIBRATION
};

void JDriveMain();

#endif

#ifdef __cplusplus
}
#endif
