#include <MotorControl/motor_control.hpp>
#include <BoardConfig/board_config.h>

void MotorControl::Init()
{
	Encoder.encoderOffset = motorParam.encoderOffset;
	Encoder.polePair = motorParam.polePair;
	Encoder.filter_vel.SetParam(80.0f, DELTA_T);
	Encoder.filter_pos.SetParam(100.0f, DELTA_T);

	Encoder.UpdateEncoderPool();

	filter_d.SetParam(100.0f, DELTA_T);
	filter_q.SetParam(100.0f, DELTA_T);
	filter_acceleration.SetParam(2.5f, DELTA_T);
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

	controlParam.goalVoltage = Limiter(controlParam.goalVoltage, 0.9);

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
	else if(controlMode == DAMPED_OSCILLATION_POSITION_CONTROL_MODE)
	{
		DampedOscillationPositionControl();
	}
}

void MotorControl::DampedOscillationPositionControl()
{
	dampedOscillationParam.x = controlParam.goalPosition - extendedJointPosition;
	dampedOscillationParam.xdot = jointVelocity;

	dampedOscillationParam.xddot = (dampedOscillationParam.xdot - dampedOscillationParam._xddotBuff[dampedOscillationParam._buffIdx]) / (delta * 10.0f);
	dampedOscillationParam.xddot = filter_acceleration.Update(dampedOscillationParam.xddot);
	dampedOscillationParam._xddotBuff[dampedOscillationParam._buffIdx++] = dampedOscillationParam.xdot;
	if(dampedOscillationParam._buffIdx >= 10)
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

	currentPIDParam_d.i += (currentPIDParam_d.error - currentPIDParam_d.a) * currentPIDParam_d.Ki * delta;
	currentPIDParam_q.i += (currentPIDParam_q.error - currentPIDParam_q.a) * currentPIDParam_q.Ki * delta;

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
	//Position Controller
	positionPIDParam.error = controlParam.goalPosition - extendedJointPosition;

	positionPIDParam.p = positionPIDParam.error * positionPIDParam.Kp;

	positionPIDParam.i += (positionPIDParam.error - positionPIDParam.a) * positionPIDParam.Ki * delta;

	velocityCommand = positionPIDParam.p + positionPIDParam.i;

	positionPIDParam.a = velocityCommand;
	velocityCommand = Limiter(velocityCommand, controlParam.goalVelocity);
	positionPIDParam.a = (positionPIDParam.a - velocityCommand) * positionPIDParam.Ka;

	//Velocity Controller
	velocityPIDParam.error = velocityCommand - jointVelocity;

	velocityPIDParam.p = velocityPIDParam.error * velocityPIDParam.Kp;

	velocityPIDParam.i += (velocityPIDParam.error - velocityPIDParam.a) * velocityPIDParam.Ki * delta;

	currentCommand = velocityPIDParam.p + velocityPIDParam.i;

	velocityPIDParam.a = currentCommand;
	currentCommand = Limiter(currentCommand, controlParam.goalCurrent);
	velocityPIDParam.a = (velocityPIDParam.a - currentCommand) * velocityPIDParam.Ka;

	//Current Controller
	currentPIDParam_d.error = 0.0f - id;
	currentPIDParam_q.error = currentCommand - iq;

	currentPIDParam_d.p = currentPIDParam_d.error * currentPIDParam_d.Kp;
	currentPIDParam_q.p = currentPIDParam_q.error * currentPIDParam_q.Kp;

	currentPIDParam_d.i += (currentPIDParam_d.error - currentPIDParam_d.a) * currentPIDParam_d.Ki * delta;
	currentPIDParam_q.i += (currentPIDParam_q.error - currentPIDParam_q.a) * currentPIDParam_q.Ki * delta;

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
	//Velocity Controller
	velocityPIDParam.error = controlParam.goalVelocity - jointVelocity;

	velocityPIDParam.p = velocityPIDParam.error * velocityPIDParam.Kp;

	velocityPIDParam.i += (velocityPIDParam.error - velocityPIDParam.a) * velocityPIDParam.Ki * delta;

	currentCommand = velocityPIDParam.p + velocityPIDParam.i;

	velocityPIDParam.a = currentCommand;
	currentCommand = Limiter(currentCommand, controlParam.goalCurrent);
	velocityPIDParam.a = (velocityPIDParam.a - currentCommand) * velocityPIDParam.Ka;

	//Current Controller
	currentPIDParam_d.error = 0.0f - id;
	currentPIDParam_q.error = currentCommand - iq;

	currentPIDParam_d.p = currentPIDParam_d.error * currentPIDParam_d.Kp;
	currentPIDParam_q.p = currentPIDParam_q.error * currentPIDParam_q.Kp;

	currentPIDParam_d.i += (currentPIDParam_d.error - currentPIDParam_d.a) * currentPIDParam_d.Ki * delta;
	currentPIDParam_q.i += (currentPIDParam_q.error - currentPIDParam_q.a) * currentPIDParam_q.Ki * delta;

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
	//Current Controller
	currentPIDParam_d.error = 0.0f - id;
	currentPIDParam_q.error = controlParam.goalCurrent - iq;

	currentPIDParam_d.p = currentPIDParam_d.error * currentPIDParam_d.Kp;
	currentPIDParam_q.p = currentPIDParam_q.error * currentPIDParam_q.Kp;

	currentPIDParam_d.i += (currentPIDParam_d.error - currentPIDParam_d.a) * currentPIDParam_d.Ki * delta;
	currentPIDParam_q.i += (currentPIDParam_q.error - currentPIDParam_q.a) * currentPIDParam_q.Ki * delta;

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
	//Voltage Controller
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
	float cf = FastCos(theta);
	float sf = FastSin(theta);

	*a = cf * d - sf * q;
	*b = (0.86602540378f * sf - 0.5f * cf) * d - (-0.86602540378f * cf - 0.5f * sf) * q;
	*c = (-0.86602540378f * sf - 0.5f * cf) * d - (0.86602540378f * cf - 0.5f * sf) * q;
}
