#pragma once

#include <flawless/platform/system.h>

namespace flawless
{
namespace util
{

template<class T>
class Singleton {
public:
	static T& get() {
		LockGuard lock;
		static T instance;
		return instance;
	}
protected:
	~Singleton() {}
};

}
}

