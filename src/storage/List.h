#ifndef LIST_H
#define LIST_H
/*
	abstract base class for lists to be used on avr
*/

template<typename T>
class List{
protected:
	List(){};
public:
	virtual uint16_t size();
	virtual uint16_t maxSize();
	virtual bool add(uint16_t index, T item);
	virtual bool pushTop(T item);
	virtual bool pushBottom(T item);
	virtual bool set(uint16_t index, T item);
	virtual T get(uint16_t index);
	virtual T remove(uint16_t index);
	virtual T popTop();
	virtual T popBottom();
	virtual void clear();
};
#endif
