#ifndef AVRLinkedList_H
#define AVRLinkedList_H
/*
	AVR's are very susceptable to memory fragmentation due to the very limited
	amount of RAM, so a Linked List Implementation using a constant number of
	Nodes and a free list is necessary for stability
*/
template<typename T>
struct LN{
	T data;
	LN<T> *next;
};
template<typename T>
class AVRLinkedList{
public:
	AVRLinkedList(uint16_t numberOfNodes);
	~AVRLinkedList();
	uint16_t size();
	uint16_t maxSize();
	bool add(uint16_t index, T item);
	bool add(T item);
	bool pushTop(T item);
	bool pushBottom(T item);
	bool set(uint16_t index, T item);
	T get(uint16_t index);
	T remove(uint16_t index);
	T popTop();
	T popBottom();
	void clear();
private:
	AVRLinkedList(const AVRLinkedList&);
	int curSize, maxNodes;
	LN<T> *root, *last;
	LN<T> *freeRoot;
	void* raw;

	void pushFree(LN<T>* node);
	LN<T>* popFree();
	LN<T>* getNode(uint16_t index);
};
template<typename T>
AVRLinkedList<T>::AVRLinkedList(uint16_t numberOfNodes)
		:curSize(0), maxNodes(numberOfNodes), root(0), last(0){
	raw = malloc( maxNodes*sizeof(LN<T>) );
	if(raw == 0) maxNodes = 0; //not enough memory; disable the list
	freeRoot = (LN<T>*) raw;
	for (int i = 0; i < maxNodes-1; i++){
		(freeRoot+i)->next = (freeRoot+(i+1));
	}
}
template<typename T>
AVRLinkedList<T>::~AVRLinkedList(){
	free(raw);
}
template<typename T>
void AVRLinkedList<T>::pushFree(LN<T>* node){
	node->next = freeRoot;
	freeRoot = node;
}
template<typename T>
LN<T>* AVRLinkedList<T>::popFree(){
	LN<T>* freed = freeRoot;
	freeRoot = freeRoot->next;
	return freed;
}
template<typename T> inline
LN<T>* AVRLinkedList<T>::getNode(uint16_t index){
	if(index > curSize) return false;
	LN<T> *cur = root;
	for(int i=0; i<index; i++) cur = cur->next;
	return cur;
}
//-- public from here on --//
template<typename T>
uint16_t AVRLinkedList<T>::size(){
	return curSize;
}
template<typename T>
uint16_t AVRLinkedList<T>::maxSize(){
	return maxNodes;
}
template<typename T>
bool AVRLinkedList<T>::add(uint16_t index, T item){
	if(curSize >= maxNodes) return false;

	if(index == 0) return pushTop(item);
	else if(index == curSize) return pushBottom(item);
	else if(index > curSize) return false;

	LN<T>* cur = getNode(index-1);
	LN<T>* nw = popFree();
	nw->next = cur->next;
	cur->next = nw;
	nw->data = item;

	curSize++;
	return true;
}
template<typename T>
bool AVRLinkedList<T>::add(T item){
	return pushTop(item);
}
template<typename T>
bool AVRLinkedList<T>::pushTop(T item){
	if(curSize >= maxNodes) return false;

	LN<T>* nw = popFree();
	nw->next = root;
	root = nw;
	nw->data = item;
	if(curSize == 0) last = nw;

	curSize++;
	return true;
}
template<typename T>
bool AVRLinkedList<T>::pushBottom(T item){
	if(curSize >= maxNodes) return false;

	LN<T>* nw = popFree();
	if(curSize == 0) root = nw;
	else last->next = nw;
	last = nw;
	nw->data = item;

	curSize++;
	return true;
}
template<typename T>
bool AVRLinkedList<T>::set(uint16_t index, T item){
	if(index >= curSize) return false;
	LN<T> *node = getNode(index);
	node->data = item;
	return true;
}
template<typename T>
T AVRLinkedList<T>::get(uint16_t index){
	if(index >= curSize || index < 0) return T();
	else if(index == curSize-1) return last->data;
	return getNode(index)->data;
}
template<typename T>
T AVRLinkedList<T>::remove(uint16_t index){
	if(curSize <= 0) return T();
	else if(index >= curSize || index < 0) return T();
	else if(index == 0) return popTop();
	LN<T> *del, *pre;

	pre = getNode(index-1);
	del = pre->next;
	pre->next = del->next;
	if(del == last) last = pre;
	T tmp = del->data;

	curSize--;
	pushFree(del);
	return tmp;
}
template<typename T>
T AVRLinkedList<T>::popTop(){
	if(curSize <= 0) return T();

	LN<T> *del = root;
	root = root->next;
	T tmp = del->data;

	curSize--;
	pushFree(del);
	return tmp;
}
template<typename T>
T AVRLinkedList<T>::popBottom(){
	return remove(curSize-1);
}
template<typename T>
void AVRLinkedList<T>::clear(){
	if(curSize == 0) return;
	LN<T>* tmp;
	for (int i = 0; i < curSize; i++){
		tmp = root;
		root = root->next;
		pushFree(tmp);
	}
	root = 0;
	last = 0;
	curSize = 0;
}
#endif

