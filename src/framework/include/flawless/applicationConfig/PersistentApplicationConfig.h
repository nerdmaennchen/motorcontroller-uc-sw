#pragma once

#include "ApplicationConfig.h"


namespace flawless
{

class PersistentConfigBase : public flawless::util::LinkedListNode<PersistentConfigBase> {
public:
	virtual ~PersistentConfigBase() {}

	virtual char const* getName() const = 0;
	virtual char const* getFormat() const = 0;

	virtual void setValue(void const* vals) = 0;
	virtual void const* getValue() = 0;
	virtual uint16_t getSize() const = 0;
};

struct PersistentConfigurationManager : public flawless::util::Singleton<PersistentConfigurationManager> {
	bool loadFromStorage(PersistentConfigBase *config);
	bool updateStorage();
	bool dirty() const;
};

template<typename T>
struct PersistentConfiguration final : public PersistentConfigBase {
	const char* name;
	const char* format;
	T value;
	using callbackType = Callback<T&, bool>;
	callbackType* mCB {nullptr};

	PersistentConfiguration(const char* _name, const char* _format, T const& val) : name(_name), format(_format), value(val) {
		PersistentConfigurationManager::get().loadFromStorage(this);
	}
	PersistentConfiguration(const char* _name, const char* _format, callbackType* cb, T const& val) : name(_name), format(_format), value(val), mCB(cb) {
		PersistentConfigurationManager::get().loadFromStorage(this);
	}

	char const* getName() const override { return name; }
	char const* getFormat() const override { return format; }

	uint16_t getSize() const override { return sizeof(T);}
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

	T& get() {
		return value;
	}

	operator T&() {
		return value;
	}

	operator T const&() const {
		return value;
	}
};

}
