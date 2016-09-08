#pragma once

namespace flawless
{
namespace util
{

template<class T>
class Singleton {
public:
	static T& get() {
		static T instance;
		return instance;
	}
protected:
	~Singleton() {}
};

}
}

