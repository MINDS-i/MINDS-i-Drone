#ifndef LIST_H
#define LIST_H

// T must have a default constructor
template<typename T>
class List{
protected:
	List(){};
public:
	/**
	 * The current number of items in this liste
	 */
	virtual uint16_t size();
	/**
	 * The maximum number of items this list can store
	 */
	virtual uint16_t maxSize();
	/**
	 * Add an item at the index `index` such that the item previously at
	 * `index` now occupies position `index+1`. Returns false if the operation
	 * fails.
	 */
	virtual bool add(uint16_t index, T item);
	/**
	 * Add an item as the new first item in the list.
	 * returns false if the operation fails.
	 */
	virtual bool add(T item);
	/**
	 * Add an item to the top of the list (index 0)
	 * returns false if the operation fails.
	 */
	virtual bool pushTop(T item);
	/**
	 * Add an item to the end of the list
	 * returns false if the operation fails.
	 */
	virtual bool pushBottom(T item);
	/**
	 * Set the data at position `index` to `item`
	 * returns false if the operation fails.
	 */
	virtual bool set(uint16_t index, T item);
	/**
	 * Return the item at position `index` or T() if not present
	 */
	virtual T get(uint16_t index);
	/**
	 * Remove and return the item at position `index` or T() if not present
	 */
	virtual T remove(uint16_t index);
	/**
	 * Remove and return the first item or T() if not present
	 */
	virtual T popTop();
	/**
	 * Remove and return the last item or T() if not present
	 */
	virtual T popBottom();
	/**
	 * Remove all data items from the list
	 */
	virtual void clear();
};
#endif
