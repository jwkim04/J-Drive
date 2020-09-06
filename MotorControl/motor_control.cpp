#include <MotorControl/motor_control.hpp>
#include <BoardConfig/board_config.h>

void MotorControl::Init()
{
	motorParam.encoderOffset = *(float*)&controlTable->controlTableData[26].data;
	ADC1Offset = controlTable->controlTableData[24].data;
	ADC2Offset = controlTable->controlTableData[25].data;
	motorParam.polePair = controlTable->controlTableData[27].data;

	Encoder.encoderOffset = motorParam.encoderOffset;
	Encoder.polePair = motorParam.polePair;
	Encoder.filter_vel.SetParam(VELOCITY_FILTER_CUTOFF_FREQ, DELTA_T);
	Encoder.filter_pos.SetParam(POSITION_FILTER_CUTOFF_FREQ, DELTA_T);

	Encoder.UpdateEncoderPoll();

	filter_d.SetParam(CURRENT_FILTER_CUTOFF_FREQ, DELTA_T);
	filter_q.SetParam(CURRENT_FILTER_CUTOFF_FREQ, DELTA_T);
	filter_acceleration.SetParam(ACCEKERATION_FILTER_CUTOFF_FREQ, DELTA_T);
}

void MotorControl::SetControlTable(ControlTable *_controlTable)
{
	controlTable = _controlTable;
}

void MotorControl::ControlUpdate()
{
	Encoder.UpdateEncoder();

	jointPosition = Encoder.GetJointPosition();
	rotorPosition = Encoder.GetRotorPosition();
	extendedJointPosition = Encoder.GetExtendedJointPosition();
	jointVelocity = Encoder.GetJointVelocity();

	ia = (float) ((int32_t) GetSO1() - ADC1Offset) * (1.0f / (float) 0x7FF);
	ib = (float) ((int32_t) GetSO2() - ADC2Offset) * (1.0f / (float) 0x7FF);
	ic = (-ia - ib);

	DQZTrans(ia, ib, ic, rotorPosition, &id, &iq);
	id = filter_d.Update(id);
	iq = filter_q.Update(iq);

	presentCurrent = id + iq;
	presentVoltage = vd + vq;
	controlTable->controlTableData[43].data = *(uint32_t*)&presentVoltage;
	controlTable->controlTableData[44].data = *(uint32_t*)&presentCurrent;
	controlTable->controlTableData[45].data = *(uint32_t*)&jointVelocity;
	controlTable->controlTableData[46].data = *(uint32_t*)&extendedJointPosition;

	if(abs(jointVelocity) > *(float*)&controlTable->controlTableData[9].data)
	{
		controlTable->controlTableData[41].data = 1;
	}
	else
	{
		controlTable->controlTableData[41].data = 0;
	}

	controlParam.goalVoltage = *(float*)&controlTable->controlTableData[33].data;
	controlParam.goalCurrent = *(float*)&controlTable->controlTableData[34].data;
	controlParam.goalVelocity = *(float*)&controlTable->controlTableData[35].data;
	controlParam.goalPosition = *(float*)&controlTable->controlTableData[36].data;

	dampedOscillationParam.k = *(float*)&controlTable->controlTableData[37].data;
	dampedOscillationParam.b = *(float*)&controlTable->controlTableData[38].data;
	dampedOscillationParam.m = *(float*)&controlTable->controlTableData[39].data;

	positionPIDParam.Kp = *(float*)&controlTable->controlTableData[22].data;
	positionPIDParam.Ki = *(float*)&controlTable->controlTableData[21].data;
	positionPIDParam.Ka = 1.0f / positionPIDParam.Kp;
	if(positionPIDParam.Kp == 0.0f)
	{
		positionPIDParam.Ka = 0;
	}

	velocityPIDParam.Kp = *(float*)&controlTable->controlTableData[20].data;
	velocityPIDParam.Ki = *(float*)&controlTable->controlTableData[19].data;
	velocityPIDParam.Ka = 1.0 / velocityPIDParam.Kp;
	if(velocityPIDParam.Ka == 0.0f)
	{
		velocityPIDParam.Ka = 0;
	}

	currentPIDParam_d.Kp = *(float*)&controlTable->controlTableData[18].data;
	currentPIDParam_q.Kp = *(float*)&controlTable->controlTableData[18].data;
	currentPIDParam_d.Ki = *(float*)&controlTable->controlTableData[17].data;
	currentPIDParam_q.Ki = *(float*)&controlTable->controlTableData[17].data;
	currentPIDParam_d.Ka = 1.0f / currentPIDParam_d.Kp;
	currentPIDParam_q.Ka = 1.0f / currentPIDParam_q.Kp;
	if(currentPIDParam_d.Ka == 0.0f)
	{
		currentPIDParam_d.Ka = 0.0f;
		currentPIDParam_q.Ka = 0.0f;
	}

	controlParam.goalVoltage = Limiter(controlParam.goalVoltage, 0.9);

	controlMode = controlTable->controlTableData[6].data;

	if (controlMode == VOLTAGE_CONTROL_MODE)
	{
		VoltageControl();
	}
	else if (controlMode == CURRENT_CONTROL_MODE)
	{
		CurrentControl();
	}
	else if (controlMode == VELOCITY_CONTROL_MODE)
	{
		VelocityControl();
	}
	else if (controlMode == POSITION_CONTROL_MODE)
	{
		PositionControl();
	}
	else
	{
		DampedOscillationPositionControl();
	}
}

void MotorControl::DampedOscillationPositionControl()
{
	dampedOscillationParam.x = (controlParam.goalPosition + *(float*)&controlTable->controlTableData[8].data) - extendedJointPosition;
	dampedOscillationParam.xdot = jointVelocity;

	dampedOscillationParam.xddot = (dampedOscillationParam.xdot - dampedOscillationParam._xddotBuff[dampedOscillationParam._buffIdx]) / (DELTA_T * 10.0f);
	dampedOscillationParam.xddot = filter_acceleration.Update(dampedOscillationParam.xddot);
	dampedOscillationParam._xddotBuff[dampedOscillationParam._buffIdx++] = dampedOscillationParam.xdot;
	if (dampedOscillationParam._buffIdx >= 10)
	{
		dampedOscillationParam._buffIdx = 0;
	}

	dampedOscillationParam.F = dampedOscillationParam.x * dampedOscillationParam.k + dampedOscillationParam.xdot * -dampedOscillationParam.b + dampedOscillationParam.xddot * -dampedOscillationParam.m;

	currentCommand = dampedOscillationParam.F;

	currentCommand = Limiter(currentCommand, controlParam.goalCurrent);

	currentPIDParam_d.error = 0.0f - id;
	currentPIDParam_q.error = currentCommand - iq;

	currentPIDParam_d.p = currentPIDParam_d.error * currentPIDParam_d.Kp;
	currentPIDParam_q.p = currentPIDParam_q.error * currentPIDParam_q.Kp;

	currentPIDParam_d.i += (currentPIDParam_d.error - currentPIDParam_d.a) * currentPIDParam_d.Ki * DELTA_T;
	currentPIDParam_q.i += (currentPIDParam_q.error - currentPIDParam_q.a) * currentPIDParam_q.Ki * DELTA_T;

	vd = currentPIDParam_d.p + currentPIDParam_d.i;
	vq = currentPIDParam_q.p + currentPIDParam_q.i;

	currentPIDParam_d.a = vd;
	currentPIDParam_q.a = vq;
	vd = Limiter(vd, controlParam.goalVoltage);
	vq = Limiter(vq, controlParam.goalVoltage);
	currentPIDParam_d.a = (currentPIDParam_d.a - vd) * currentPIDParam_d.Ka;
	currentPIDParam_q.a = (currentPIDParam_q.a - vq) * currentPIDParam_q.Ka;

	DQZTransInv(vd, vq, rotorPosition, &va, &vb, &vc);

	va = va + 0.5f;
	vb = vb + 0.5f;
	vc = vc + 0.5f;
	aPWM = (uint16_t) (va * ((float) (0xFFF)));
	bPWM = (uint16_t) (vb * ((float) (0xFFF)));
	cPWM = (uint16_t) (vc * ((float) (0xFFF)));

	SetInverterPWMDuty(aPWM, bPWM, cPWM);
}

void MotorControl::PositionControl()
{
	positionPIDParam.error = (controlParam.goalPosition + *(float*)&controlTable->controlTableData[8].data) - extendedJointPosition;

	positionPIDParam.p = positionPIDParam.error * positionPIDParam.Kp;

	positionPIDParam.i += (positionPIDParam.error - positionPIDParam.a) * positionPIDParam.Ki * DELTA_T;

	velocityCommand = positionPIDParam.p + positionPIDParam.i;

	positionPIDParam.a = velocityCommand;
	velocityCommand = Limiter(velocityCommand, controlParam.goalVelocity);
	positionPIDParam.a = (positionPIDParam.a - velocityCommand) * positionPIDParam.Ka;

	velocityPIDParam.error = velocityCommand - jointVelocity;

	velocityPIDParam.p = velocityPIDParam.error * velocityPIDParam.Kp;

	velocityPIDParam.i += (velocityPIDParam.error - velocityPIDParam.a) * velocityPIDParam.Ki * DELTA_T;

	currentCommand = velocityPIDParam.p + velocityPIDParam.i;

	velocityPIDParam.a = currentCommand;
	currentCommand = Limiter(currentCommand, controlParam.goalCurrent);
	velocityPIDParam.a = (velocityPIDParam.a - currentCommand) * velocityPIDParam.Ka;

	currentPIDParam_d.error = 0.0f - id;
	currentPIDParam_q.error = currentCommand - iq;

	currentPIDParam_d.p = currentPIDParam_d.error * currentPIDParam_d.Kp;
	currentPIDParam_q.p = currentPIDParam_q.error * currentPIDParam_q.Kp;

	currentPIDParam_d.i += (currentPIDParam_d.error - currentPIDParam_d.a) * currentPIDParam_d.Ki * DELTA_T;
	currentPIDParam_q.i += (currentPIDParam_q.error - currentPIDParam_q.a) * currentPIDParam_q.Ki * DELTA_T;

	vd = currentPIDParam_d.p + currentPIDParam_d.i;
	vq = currentPIDParam_q.p + currentPIDParam_q.i;

	currentPIDParam_d.a = vd;
	currentPIDParam_q.a = vq;
	vd = Limiter(vd, controlParam.goalVoltage);
	vq = Limiter(vq, controlParam.goalVoltage);
	currentPIDParam_d.a = (currentPIDParam_d.a - vd) * currentPIDParam_d.Ka;
	currentPIDParam_q.a = (currentPIDParam_q.a - vq) * currentPIDParam_q.Ka;

	DQZTransInv(vd, vq, rotorPosition, &va, &vb, &vc);

	va = va + 0.5f;
	vb = vb + 0.5f;
	vc = vc + 0.5f;
	aPWM = (uint16_t) (va * ((float) (0xFFF)));
	bPWM = (uint16_t) (vb * ((float) (0xFFF)));
	cPWM = (uint16_t) (vc * ((float) (0xFFF)));

	SetInverterPWMDuty(aPWM, bPWM, cPWM);
}

void MotorControl::VelocityControl()
{
	velocityPIDParam.error = controlParam.goalVelocity - jointVelocity;

	velocityPIDParam.p = velocityPIDParam.error * velocityPIDParam.Kp;

	velocityPIDParam.i += (velocityPIDParam.error - velocityPIDParam.a) * velocityPIDParam.Ki * DELTA_T;

	currentCommand = velocityPIDParam.p + velocityPIDParam.i;

	velocityPIDParam.a = currentCommand;
	currentCommand = Limiter(currentCommand, controlParam.goalCurrent);
	velocityPIDParam.a = (velocityPIDParam.a - currentCommand) * velocityPIDParam.Ka;

	currentPIDParam_d.error = 0.0f - id;
	currentPIDParam_q.error = currentCommand - iq;

	currentPIDParam_d.p = currentPIDParam_d.error * currentPIDParam_d.Kp;
	currentPIDParam_q.p = currentPIDParam_q.error * currentPIDParam_q.Kp;

	currentPIDParam_d.i += (currentPIDParam_d.error - currentPIDParam_d.a) * currentPIDParam_d.Ki * DELTA_T;
	currentPIDParam_q.i += (currentPIDParam_q.error - currentPIDParam_q.a) * currentPIDParam_q.Ki * DELTA_T;

	vd = currentPIDParam_d.p + currentPIDParam_d.i;
	vq = currentPIDParam_q.p + currentPIDParam_q.i;

	currentPIDParam_d.a = vd;
	currentPIDParam_q.a = vq;
	vd = Limiter(vd, controlParam.goalVoltage);
	vq = Limiter(vq, controlParam.goalVoltage);
	currentPIDParam_d.a = (currentPIDParam_d.a - vd) * currentPIDParam_d.Ka;
	currentPIDParam_q.a = (currentPIDParam_q.a - vq) * currentPIDParam_q.Ka;

	DQZTransInv(vd, vq, rotorPosition, &va, &vb, &vc);

	va = va + 0.5f;
	vb = vb + 0.5f;
	vc = vc + 0.5f;
	aPWM = (uint16_t) (va * ((float) (0xFFF)));
	bPWM = (uint16_t) (vb * ((float) (0xFFF)));
	cPWM = (uint16_t) (vc * ((float) (0xFFF)));

	SetInverterPWMDuty(aPWM, bPWM, cPWM);
}

void MotorControl::CurrentControl()
{
	currentPIDParam_d.error = 0.0f - id;
	currentPIDParam_q.error = controlParam.goalCurrent - iq;

	currentPIDParam_d.p = currentPIDParam_d.error * currentPIDParam_d.Kp;
	currentPIDParam_q.p = currentPIDParam_q.error * currentPIDParam_q.Kp;

	currentPIDParam_d.i += (currentPIDParam_d.error - currentPIDParam_d.a) * currentPIDParam_d.Ki * DELTA_T;
	currentPIDParam_q.i += (currentPIDParam_q.error - currentPIDParam_q.a) * currentPIDParam_q.Ki * DELTA_T;

	vd = currentPIDParam_d.p + currentPIDParam_d.i;
	vq = currentPIDParam_q.p + currentPIDParam_q.i;

	currentPIDParam_d.a = vd;
	currentPIDParam_q.a = vq;
	vd = Limiter(vd, controlParam.goalVoltage);
	vq = Limiter(vq, controlParam.goalVoltage);
	currentPIDParam_d.a = (currentPIDParam_d.a - vd) * currentPIDParam_d.Ka;
	currentPIDParam_q.a = (currentPIDParam_q.a - vq) * currentPIDParam_q.Ka;

	DQZTransInv(vd, vq, rotorPosition, &va, &vb, &vc);

	va = va + 0.5f;
	vb = vb + 0.5f;
	vc = vc + 0.5f;
	aPWM = (uint16_t) (va * ((float) (0xFFF)));
	bPWM = (uint16_t) (vb * ((float) (0xFFF)));
	cPWM = (uint16_t) (vc * ((float) (0xFFF)));

	SetInverterPWMDuty(aPWM, bPWM, cPWM);
}

void MotorControl::VoltageControl()
{
	DQZTransInv(0.0f, controlParam.goalVoltage, rotorPosition, &va, &vb, &vc);

	va = va + 0.5f;
	vb = vb + 0.5f;
	vc = vc + 0.5f;
	aPWM = (uint16_t) (va * ((float) (0xFFF)));
	bPWM = (uint16_t) (vb * ((float) (0xFFF)));
	cPWM = (uint16_t) (vc * ((float) (0xFFF)));

	SetInverterPWMDuty(aPWM, bPWM, cPWM);
}

void MotorControl::DQZTrans(float a, float b, float c, float theta, float *d, float *q)
{
	float cf = FastCos(theta);
	float sf = FastSin(theta);

	*d = 0.6666667f * (cf * a + (0.86602540378f * sf - 0.5f * cf) * b + (-0.86602540378f * sf - 0.5f * cf) * c);
	*q = 0.6666667f * (-sf * a - (-0.86602540378f * cf - 0.5f * sf) * b - (0.86602540378f * cf - 0.5f * sf) * c);
}

void MotorControl::DQZTransInv(float d, float q, float theta, float *a, float *b, float *c)
{
	if (d > 0.9f)
		d = 0.9f;
	if (d < -0.9f)
		d = -0.9f;

	if (q > 0.9f)
		q = 0.9f;
	if (q < -0.9f)
		q = -0.9f;

	float cf = FastCos(theta);
	float sf = FastSin(theta);

	*a = cf * d - sf * q;
	*b = (0.86602540378f * sf - 0.5f * cf) * d - (-0.86602540378f * cf - 0.5f * sf) * q;
	*c = (-0.86602540378f * sf - 0.5f * cf) * d - (0.86602540378f * cf - 0.5f * sf) * q;
}
