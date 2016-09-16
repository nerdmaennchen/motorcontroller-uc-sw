#pragma once

/**
 * a simple substitute implementation of std::array that consumes less memory
 */
using size_t = unsigned int;
template<typename T, size_t N>
class Array final {
public:
	T& operator[](size_t idx) {
		return mElements[idx];
	}
	T const& operator[](size_t idx) const {
		return mElements[idx];
	}

	constexpr size_t size() {
		return N;
	}

	T const* begin() const {
		return &(mElements[0]);
	}
	T const* end() const {
		return &(mElements[N]);
	}

	T* begin() {
		return &(mElements[0]);
	}
	T* end() {
		return &(mElements[N]);
	}

	T* data() const {
		return mElements;
	}

	T* data() {
		return mElements;
	}

	T mElements[N];
};
