#ifndef EEPROMLIST_H
#define EEPROMLIST_H

#include "math/Waypoint.h"
#include "storage/EEPROMconfig.h"
#include "storage/EEPROMsubsystem.h"
#include "util/byteConv.h"
//make sure this uses oldest first

/*
Change conversion unions over to just good old byte arrays and cast

*/

namespace{
    //node is EEaddr next EEaddr prev EE_LIST_TYPE data
    struct eeNodePtr{
    private:
        template<typename T> 
        void write(T newData, EEaddr address){
            byte data[sizeof(T)];
            *((T*)data) = newData;
            for(int i=0; i<sizeof(T); i++){
                eeprom::safeWrite(address+i, data[i]);
            }
        }
        template<typename T>
        T read(EEaddr address){
            byte data[sizeof(T)];
            for(int i=0; i<sizeof(T); i++){
                data[i] = eeprom::safeRead(address+i);
            }
            return *((T*)data);
        }
    public:
        EEaddr addr;
        eeNodePtr() : addr(0) {}
        eeNodePtr(EEaddr a): addr(a) {}
        eeNodePtr getNext(){
            return read<EEaddr>(addr);
        }
        void setNext(eeNodePtr New){
            write<EEaddr>(New.addr, addr);
            return;
        }
        eeNodePtr getPrev(){
            return read<EEaddr>(addr + sizeof(EEaddr));
        }
        void setPrev(eeNodePtr New){
            write<EEaddr>(New.addr, addr + sizeof(EEaddr));
            return;
        }
        EE_LIST_TYPE getData(){
            return read<EE_LIST_TYPE>(addr + 2*sizeof(EEaddr));
        }
        void setData(EE_LIST_TYPE New){
            write<EE_LIST_TYPE>(New, addr + 2*sizeof(EEaddr));
            return;
        }
    };
    uint16_t NODE_SIZE = (sizeof(EE_LIST_TYPE)+2*sizeof(EEaddr));
    uint16_t MAX_NODES = EE_LIST_LENGTH / NODE_SIZE;
    EEaddr FREE_TERM = 0x0000; //loose end address value of free list
    EEaddr DATA_TERM = 0xffff; //loose end address value of data list
}
void runEEListTest(){
    TEST(EE_LIST_START);
    TEST(EE_LIST_LENGTH);
    TEST(NODE_SIZE);
    TEST(MAX_NODES);
    Serial.print("\n");
    
    eeNodePtr node(512);
    Waypoint point(1234.f, 4567.f, (uint16_t) 1337);
        
    node.setData(point);
    node.setNext(12);
    node.setPrev(34);
    
    Waypoint ret = node.getData();
    TEST( (node.getNext()).addr );
    TEST( (node.getPrev()).addr );
    TEST( ret.degLatitude()     );
    TEST( ret.degLongitude()    );
    TEST( ret.getExtra()        );
    Serial.print("\n");
}
#if false
class EEPROMlist : public List<EE_LIST_TYPE>{
    /**
     * doubly linked list in arduino EEprom
     * keeps track of free list and data list in fixed region of memory
     * it has to be ablo to "recover" from reboots, so the loose addresses
     * are a tell of if that node terminates the free list of the data list
     */
public:
    static EEPROMlist* getInstance(){
        if(m_instance == NULL) m_instance = new EEPROMlist();
        return m_instance;
    }
    uint16_t     size();
    uint16_t     maxSize();
    bool         add(uint16_t index, EE_LIST_TYPE item);
    bool         add(EE_LIST_TYPE item);
    bool         pushTop(EE_LIST_TYPE item);
    bool         pushBottom(EE_LIST_TYPE item);
    bool         set(uint16_t index, EE_LIST_TYPE item);
    EE_LIST_TYPE get(uint16_t index);
    EE_LIST_TYPE remove(uint16_t index);
    EE_LIST_TYPE popTop();
    EE_LIST_TYPE popBottom();
    void         clear();
private:
    int curSize;
    eeNodePtr dataroot, datalast;
    eeNodePtr freeRoot, freeLast;
    static EEPROMlist* m_instance;
    EEPROMlist();
    EEPROMlist(const EEPROMlist&);
    bool       readList();
    void       constructList();
    void       pushFree(eeNodePtr* eeNodePtr);
    eeNodePtr* popFree();
    eeNodePtr* getNode(uint16_t index);
};
EEPROMlist::EEPROMlist(): curSize(0) {
    if(!readList(EE_LIST_START)) constructList(EE_LIST_START);
}
/**
 * try and deduce the roots af lasts of free and data lists
 * return true if every node is accounted for
 * return false if a valid list could not be salvaged
 */
bool EEPROMlist::readList(EEaddr start){
    for(int i=0; i<MAX_NODES; i++){
        eeNodePtr node(EE_LIST_START + i*NODE_SIZE);
        switch(node.getNext()){
            case FREE_TERM:
                if(freelast.addr != 0) return false;
                freelast = node;
                break;
            case DATA_TERM:
                if(datalast.addr != 0) return false;
                datalast = node;
                break;
        }
        switch(node.getPrev()){
            case FREE_TERM:
                if(freeroot.addr != 0) return false;
                freeroot = node;
                break;
            case DATA_TERM:
                if(dataroot.addr != 0) return false;
                dataroot = node;
                break;
        }
    }
    
    //check to make sure we found all our end nodes
    if (freelast.addr == 0 || datalast.addr == 0 ||
        freeroot.addr == 0 || dataroot.addr == 0 ) return false;
    
    //traverse both lists to see if the addresses are valid
    uint16_t found = 2; //the two roots don't get counted otherwise
    eeNodePtr cur = freeroot;
    while (cur.addr != freelast.addr) {
        eeNodePtr next = cur.getNext();
        if(next.getPrev() != cur.addr) return false;
        cur = next;
        found++;   
        if(found > MAX_NODES) return false;
    }
    cur = dataroot;
    while (cur.addr != datalast.addr) {
        eeNodePtr next = cur.getNext();
        if(next.getPrev() != cur.addr) return false;
        cur = next;
        found++;   
        if(found > MAX_NODES) return false;
    }
    
    //did we find the right number?
    return (found == MAX_NODES);
}
/**
 * Construct a brand new list, 100% free
 * This will necessarily obliderate all existing data
 */
void EEPROMlist::constructList(EEaddr start){
    for (int i = 0; i < MAX_NODES-1; i++){
        (freeRoot+i)->next = (freeRoot+(i+1));
    }
}
void EEPROMlist::pushFree(eeNodePtr* eeNodePtr){
    eeNodePtr->next = freeRoot;
    freeRoot = eeNodePtr;
}
eeNodePtr* EEPROMlist::popFree(){
    eeNodePtr* freed = freeRoot;
    freeRoot = freeRoot->next;
    return freed;
} inline
eeNodePtr* EEPROMlist::getNode(uint16_t index){
    if(index > curSize) return false;
    eeNodePtr *cur = root;
    for(int i=0; i<index; i++) cur = cur->next;
    return cur;
}
//-- public from here on --//
uint16_t EEPROMlist::size(){
    return curSize;
}
uint16_t EEPROMlist::maxSize(){
    return MAX_NODES;
}
bool EEPROMlist::add(uint16_t index, EE_LIST_TYPE item){
    if(curSize >= MAX_NODES) return false;

    if(index == 0) return pushTop(item);
    else if(index == curSize) return pushBottom(item);
    else if(index > curSize) return false;

    eeNodePtr* cur = getNode(index-1);
    eeNodePtr* nw = popFree();
    nw->next = cur->next;
    cur->next = nw;
    nw->data = item;

    curSize++;
    return true;
}
bool EEPROMlist::add(EE_LIST_TYPE item){
    return pushTop(item);
}
bool EEPROMlist::pushTop(EE_LIST_TYPE item){
    if(curSize >= MAX_NODES) return false;

    eeNodePtr* nw = popFree();
    nw->next = root;
    root = nw;
    nw->data = item;
    if(curSize == 0) last = nw;

    curSize++;
    return true;
}
bool EEPROMlist::pushBottom(EE_LIST_TYPE item){
    if(curSize >= MAX_NODES) return false;

    eeNodePtr* nw = popFree();
    if(curSize == 0) root = nw;
    else last->next = nw;
    last = nw;
    nw->data = item;

    curSize++;
    return true;
}
bool EEPROMlist::set(uint16_t index, EE_LIST_TYPE item){
    if(index >= curSize) return false;
    eeNodePtr *eeNodePtr = getNode(index);
    eeNodePtr->data = item;
    return true;
}
EE_LIST_TYPE EEPROMlist::get(uint16_t index){
    if(index >= curSize || index < 0) return EE_LIST_TYPE();
    else if(index == curSize-1) return last->data;
    return getNode(index)->data;
}
EE_LIST_TYPE EEPROMlist::remove(uint16_t index){
    if(curSize <= 0) return EE_LIST_TYPE();
    else if(index >= curSize || index < 0) return EE_LIST_TYPE();
    else if(index == 0) return popTop();
    eeNodePtr *del, *pre;

    pre = getNode(index-1);
    del = pre->next;
    pre->next = del->next;
    if(del == last) last = pre;
    EE_LIST_TYPE tmp = del->data;

    curSize--;
    pushFree(del);
    return tmp;
}
EE_LIST_TYPE EEPROMlist::popTop(){
    if(curSize <= 0) return EE_LIST_TYPE();

    eeNodePtr *del = root;
    root = root->next;
    EE_LIST_TYPE tmp = del->data;

    curSize--;
    pushFree(del);
    return tmp;
}
EE_LIST_TYPE EEPROMlist::popBottom(){
    return remove(curSize-1);
}
void EEPROMlist::clear(){
    if(curSize == 0) return;
    eeNodePtr* tmp;
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
#endif
