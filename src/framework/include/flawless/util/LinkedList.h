#pragma once

#include "Singleton.h"

namespace flawless
{
namespace util
{

template<typename T>
class LinkedListNode {
public:
	LinkedListNode();
	T* mNext {nullptr};
protected:
	virtual ~LinkedListNode();
};

template<typename T>
class LinkedList final : public Singleton<LinkedList<T>>{
public:
	void add(T* element) {
		element->mNext = this->mFirst;
		this->mFirst = element;
	}

	void remove(T* element) {
		T **iter = &(this->mFirst);
		while (*iter) {
			T **next = &((*iter)->mNext);
			if (*next == element) {
				(*iter)->mNext = element->mNext;
				return;
			}
			iter = next;
		}
	}
	T* mFirst {nullptr};
};


template<typename T>
LinkedListNode<T>::LinkedListNode() {
	LinkedList<T>::get().add((T*)this);
}

template<typename T>
LinkedListNode<T>::~LinkedListNode() {
	LinkedList<T>::get().remove((T*)this);
}

template<typename T>
class SortedListNode {
public:
	SortedListNode(int value);
	T* mNext {nullptr};
	int mValue {0};
protected:
	virtual ~SortedListNode();
};

template<typename T>
class SortedList final : public Singleton<SortedList<T>>{
public:
	void add(T* element) {
		T** head = &mFirst;
		do {
			if (not *head) {
				*head = element;
				break;
			}
			if ((*head)->mValue > element->mValue) {
				element->mNext = *head;
				*head = element;
				break;
			}
			head = &((*head)->mNext);
		} while (1);
	}

	void remove(T* element) {
		T **iter = &(this->mFirst);
		while (*iter) {
			T **next = &((*iter)->mNext);
			if (*next == element) {
				(*iter)->mNext = element->mNext;
				return;
			}
			iter = next;
		}
	}
	T* mFirst {nullptr};
};


template<typename T>
SortedListNode<T>::SortedListNode(int value) : mValue{value} {
	SortedList<T>::get().add((T*)this);
}

template<typename T>
SortedListNode<T>::~SortedListNode() {
	SortedList<T>::get().remove((T*)this);
}

}
}
