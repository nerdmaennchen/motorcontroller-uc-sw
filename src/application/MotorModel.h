#pragma once

struct MotorState {
	float lastKnownPos; // where was the rotor at the last tick
	float lastKnownSpeed;
	float estPhase; // angle as number [0..6]
	float estSpeed; // tick frequency in Hz
};
