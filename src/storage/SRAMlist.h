#ifndef SRAMLIST_H
#define SRAMLIST_H
/*
	AVR's are very susceptable to memory fragmentation due to the very limited
	amount of RAM, so a Linked List Implementation using a constant number of
	Nodes and a free list is necessary for stability
*/
namespace{
	template<typename T>
	struct Node{
		T data;
		Node<T> *next;
	};
}
template<typename T>
class SRAMlist : public List<T>{
public:
	explicit SRAMlist(size_t numberOfNodes);
	~SRAMlist();
	size_t size();
	size_t maxSize();
	bool add(size_t index, T item);
	bool add(T item);
	bool pushTop(T item);
	bool pushBottom(T item);
	bool set(size_t index, T item);
	T get(size_t index);
	T remove(size_t index);
	T popTop();
	T popBottom();
	void clear();
private:
	SRAMlist(const SRAMlist&);
	size_t curSize, maxNodes;
	Node<T> *root, *last;
	Node<T> *freeRoot;
	void* raw;

	void pushFree(Node<T>* node);
	Node<T>* popFree();
	Node<T>* getNode(size_t index);
};
template<typename T>
SRAMlist<T>::SRAMlist(size_t numberOfNodes)
		:curSize(0), maxNodes(numberOfNodes), root(0), last(0){
	raw = malloc( maxNodes*sizeof(Node<T>) );
	if(raw == 0) maxNodes = 0; //not enough memory; disable the list
	freeRoot = (Node<T>*) raw;
	for(size_t i = 0; i < maxNodes-1; i++){
		(freeRoot+i)->next = (freeRoot+(i+1));
	}
}
template<typename T>
SRAMlist<T>::~SRAMlist(){
	free(raw);
}
template<typename T>
void SRAMlist<T>::pushFree(Node<T>* node){
	node->next = freeRoot;
	freeRoot = node;
}
template<typename T>
Node<T>* SRAMlist<T>::popFree(){
	Node<T>* freed = freeRoot;
	freeRoot = freeRoot->next;
	return freed;
}
template<typename T> inline
Node<T>* SRAMlist<T>::getNode(size_t index){
	if(index > curSize) return 0;
	Node<T> *cur = root;
	for(size_t i=0; i<index; i++) cur = cur->next;
	return cur;
}
//-- public from here on --//
template<typename T>
size_t SRAMlist<T>::size(){
	return curSize;
}
template<typename T>
size_t SRAMlist<T>::maxSize(){
	return maxNodes;
}
template<typename T>
bool SRAMlist<T>::add(size_t index, T item){
	if(curSize >= maxNodes) return false;

	if(index == 0) return pushTop(item);
	else if(index == curSize) return pushBottom(item);
	else if(index > curSize) return false;

	Node<T>* cur = getNode(index-1);
	Node<T>* nw = popFree();
	nw->next = cur->next;
	cur->next = nw;
	nw->data = item;

	curSize++;
	return true;
}
template<typename T>
bool SRAMlist<T>::add(T item){
	return pushTop(item);
}
template<typename T>
bool SRAMlist<T>::pushTop(T item){
	if(curSize >= maxNodes) return false;

	Node<T>* nw = popFree();
	nw->next = root;
	root = nw;
	nw->data = item;
	if(curSize == 0) last = nw;

	curSize++;
	return true;
}
template<typename T>
bool SRAMlist<T>::pushBottom(T item){
	if(curSize >= maxNodes) return false;

	Node<T>* nw = popFree();
	if(curSize == 0) root = nw;
	else last->next = nw;
	last = nw;
	nw->data = item;

	curSize++;
	return true;
}
template<typename T>
bool SRAMlist<T>::set(size_t index, T item){
	if(index >= curSize) return false;
	Node<T> *node = getNode(index);
	node->data = item;
	return true;
}
template<typename T>
T SRAMlist<T>::get(size_t index){
	if(index >= curSize || index < 0) return T();
	else if(index == curSize-1) return last->data;
	return getNode(index)->data;
}
template<typename T>
T SRAMlist<T>::remove(size_t index){
	if(curSize <= 0) return T();
	else if(index >= curSize || index < 0) return T();
	else if(index == 0) return popTop();
	Node<T> *del, *pre;

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
T SRAMlist<T>::popTop(){
	if(curSize <= 0) return T();

	Node<T> *del = root;
	root = root->next;
	T tmp = del->data;

	curSize--;
	pushFree(del);
	return tmp;
}
template<typename T>
T SRAMlist<T>::popBottom(){
	return remove(curSize-1);
}
template<typename T>
void SRAMlist<T>::clear(){
	if(curSize == 0) return;
	for (size_t i = 0; i < curSize; i++){
		Node<T>* tmp = root;
		root = root->next;
		pushFree(tmp);
	}
	root = 0;
	last = 0;
	curSize = 0;
}
#endif

