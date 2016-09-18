#include <application/PWMLookupTableGenerator.h>

#include <math.h>

constexpr float pi = 3.14159265359f;

PWMLookupTableGenerator::PWMLookupTableGenerator()
{}

PWMLookupTableGenerator::~PWMLookupTableGenerator()
{}
void PWMLookupTableGenerator::generateLookupTable(
		uint8_t stepsPerPhase,
		uint32_t numSteps,
		Array<uint16_t, pwmdriver::TicksPerStep> *pattern) {
	float phaseOffset = 2.f * pi / float(stepsPerPhase);
	const float scaleFactor = pi * 2.f / float(numSteps);
	for (size_t sIdx = 0; sIdx < numSteps; ++sIdx) {
		float phase0 = (float)sIdx * scaleFactor;
		for (size_t i = 0; i < pwmdriver::TicksPerStep; ++i) {
			float phase = phase0 + phaseOffset * i;
			float s = std::sin(phase);
			s = s * .5f + .5f;
			const float intermediate = s * pwmdriver::PwmAmplitude + pwmdriver::PwmOffTimer / 2;
			const uint16_t sVal = uint32_t(roundf(intermediate));
			(*pattern)[i] = sVal;
		}
		++pattern;
	}
}

void PWMLookupTableGenerator::generateLookupTableReversed(
		uint8_t stepsPerPhase,
		uint32_t numSteps,
		Array<uint16_t, pwmdriver::TicksPerStep> *pattern) {
	float phaseOffset = 2.f * pi / float(stepsPerPhase);
	const float scaleFactor = -pi * 2.f / float(numSteps);
	for (size_t sIdx = 0; sIdx < numSteps; ++sIdx) {
		float phase0 = (float)sIdx * scaleFactor;
		for (size_t i = 0; i < pwmdriver::TicksPerStep; ++i) {
			float phase = phase0 + phaseOffset * i;
			float s = std::sin(phase);
			s = s * .5f + .5f;
			const float intermediate = s * pwmdriver::PwmAmplitude + pwmdriver::PwmOffTimer / 2;
			const uint16_t sVal = uint32_t(roundf(intermediate));
			(*pattern)[i] = sVal;
		}
		++pattern;
	}
}

