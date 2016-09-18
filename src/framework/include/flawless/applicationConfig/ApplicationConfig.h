#pragma once

#include <flawless/stdtypes.h>
#include <flawless/util/Callback.h>
#include <flawless/util/LinkedList.h>
#include <string.h>

namespace flawless
{

class ApplicationConfigBase : public flawless::util::LinkedListNode<ApplicationConfigBase> {
public:
	ApplicationConfigBase(const char* _name) : name(_name) {}
	virtual ~ApplicationConfigBase() {}

	char const* getName() const { return (name); }

	virtual void setValue(void const* vals) = 0;
	virtual void const* getValue() = 0;
	virtual uint16_t getSize() const = 0;

private:
	const char* name;
};

template<typename T>
class ApplicationConfig final : public ApplicationConfigBase
{
	T value;
	using callbackType = Callback<T&, bool>;
	callbackType* mCB {nullptr};
public:

	ApplicationConfig(const char* _name) : ApplicationConfigBase(_name) {}
	ApplicationConfig(const char* _name, callbackType* cb) : ApplicationConfigBase(_name), mCB(cb) {}
	ApplicationConfig(const char* _name, T const& val) : ApplicationConfigBase(_name), value(val) {}
	ApplicationConfig(const char* _name, callbackType* cb, T const& val) : ApplicationConfigBase(_name), value(val), mCB(cb) {}

	uint16_t getSize() const override {
		return sizeof(T);
	};

	ApplicationConfig<T>& operator=(T const& rhs) {
		value = rhs;
		return *this;
	}

	T& get() {
		return value;
	}

	operator T&() {
		return value;
	}

	operator T const&() const {
		return value;
	}

	T* operator->() {
		return &value;
	}

	void setValue(void const* vals) {
		memcpy(&value, vals, getSize());
		if (mCB) {
			mCB->callback(value, true);
		}
	}

	void const* getValue() override {
		if (mCB) {
			mCB->callback(value, false);
		}
		return &value;
	}
};


template<>
class ApplicationConfig<void> final : public ApplicationConfigBase
{
	using callbackType = Callback<void>;
	callbackType* mCB {nullptr};
public:

	ApplicationConfig(const char* _name) : ApplicationConfigBase(_name) {}
	ApplicationConfig(const char* _name, callbackType* cb) : ApplicationConfigBase(_name), mCB(cb) {}

	void const* getValue() override { return nullptr; };
	uint16_t getSize() const override { return 0; };

	void setValue(void const*) override {
		if (mCB) {
			mCB->callback();
		}
	}
};

} /* namespace flawless */

