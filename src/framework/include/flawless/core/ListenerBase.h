#pragma once


namespace flawless
{

class ListenerBase {
public:
	virtual void invoke() = 0;

protected:
	virtual ~ListenerBase() {}
};

}
