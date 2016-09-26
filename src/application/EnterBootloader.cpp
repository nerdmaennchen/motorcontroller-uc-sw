#include <flawless/applicationConfig/ApplicationConfig.h>
#include <flawless/platform/system.h>

extern "C" {
extern unsigned _ramBegin;
}

namespace
{

class : flawless::Callback<void> {
public:
	void callback() override {
		_ramBegin = 0xdeadbeef;
		system_reset();
	}
private:
flawless::ApplicationConfig<void> enableConfig{"reset", this};
} enableCallback;

}
