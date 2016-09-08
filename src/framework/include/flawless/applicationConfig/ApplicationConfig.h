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
	ApplicationConfigBase(const char* _name, Callback<void>* cb) : mOnValueChangedCB(cb), name(_name) {}
	virtual ~ApplicationConfigBase() {}

	char const* getName() const { return (name); }

	virtual void setValue(void const* vals) = 0;
	virtual void* getDataPtr() = 0;
	virtual uint16_t getSize() const = 0;

	void setCallback(Callback<void>* cb) {
		mOnValueChangedCB = cb;
	}

	Callback<void>* mOnValueChangedCB {nullptr};
private:
	const char* name;
};

template<typename T>
class ApplicationConfig final : public ApplicationConfigBase
{
	T value;
public:

	ApplicationConfig(const char* _name) : ApplicationConfigBase(_name) {}
	ApplicationConfig(const char* _name, T const& val) : ApplicationConfigBase(_name), value(val) {}
	ApplicationConfig(const char* _name, Callback<void>* cb, T const& val) : ApplicationConfigBase(_name, cb), value(val) {}

	void* getDataPtr() override {
		return &value;
	};
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
	}
};


template<>
class ApplicationConfig<void> final : public ApplicationConfigBase
{
public:

	ApplicationConfig(const char* _name) : ApplicationConfigBase(_name) {}
	ApplicationConfig(const char* _name, Callback<void>* cb) : ApplicationConfigBase(_name, cb) {}

	void* getDataPtr() override {
		return nullptr;
	};
	uint16_t getSize() const override {
		return 0;
	};

	void setValue(void const*) override {}
};

} /* namespace flawless */

