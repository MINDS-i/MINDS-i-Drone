#ifndef EEPROMLIST_H
#define EEPROMLIST_H

#include "math/Waypoint.h"
#include "storage/EEPROMconfig.h"
#include "storage/EEPROMsubsystem.h"
#include "util/byteConv.h"

namespace{
    //node is EEaddr next EEaddr prev EE_LIST_TYPE data
    struct eeNodePtr{
    private:
        template<typename T>
        void write(T newData, EEaddr address){
            if(address < EEaddrStart) return;
            if(address >= EE_MAX) return;
            byte data[sizeof(T)];
            *((T*)data) = newData;
            for(int i=0; i<sizeof(T); i++){
                eeprom::safeWrite(address+i, data[i]);
            }
        }
        template<typename T>
        T read(EEaddr address){
            if(address < EEaddrStart) return T();
            if(address >= EE_MAX) return T();
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
    const uint16_t NODE_SIZE = (sizeof(EE_LIST_TYPE)+2*sizeof(EEaddr));
    const uint16_t MAX_NODES = EE_LIST_LENGTH / NODE_SIZE;
    const EEaddr FREE_TERM = 0x0000; //loose end address value of free list
    const EEaddr DATA_TERM = 0xffff; //loose end address value of data list
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
    eeNodePtr dataRoot, dataLast;
    eeNodePtr freeRoot, freeLast;
    static EEPROMlist* m_instance;
    EEPROMlist();
    EEPROMlist(const EEPROMlist&);
    bool      readList();
    void      constructList();
    void      pushFree(eeNodePtr eeNodePtr);
    eeNodePtr popFree();
    eeNodePtr getNode(uint16_t index);
};
EEPROMlist* EEPROMlist::m_instance = NULL;
EEPROMlist::EEPROMlist(): curSize(0) {
    if(!readList()) constructList();
}
/**
 * try and deduce the roots af lasts of free and data lists
 * return true if every node is accounted for
 * return false if a valid list could not be salvaged
 */
bool EEPROMlist::readList(){
    for(int i=0; i<MAX_NODES; i++){
        eeNodePtr node(EE_LIST_START + i*NODE_SIZE);
        switch(node.getNext().addr){
            case FREE_TERM:
                if(freeLast.addr != 0) return false;
                freeLast = node;
                break;
            case DATA_TERM:
                if(dataLast.addr != 0) return false;
                dataLast = node;
                break;
        }
        switch(node.getPrev().addr){
            case FREE_TERM:
                if(freeRoot.addr != 0) return false;
                freeRoot = node;
                break;
            case DATA_TERM:
                if(dataRoot.addr != 0) return false;
                dataRoot = node;
                break;
        }
    }

    //check to make sure we found all our end nodes
    if (freeLast.addr == 0 || dataLast.addr == 0 ||
        freeRoot.addr == 0 || dataRoot.addr == 0 ) return false;

    //traverse both lists to see if the addresses are valid
    uint16_t found = 2; //the two roots don't get counted otherwise
    eeNodePtr cur = freeRoot;
    while (cur.addr != freeLast.addr) {
        eeNodePtr next = cur.getNext();
        if(next.getPrev().addr != cur.addr) return false;
        cur = next;
        found++;
        if(found > MAX_NODES) return false;
    }
    cur = dataRoot;
    while (cur.addr != dataLast.addr) {
        eeNodePtr next = cur.getNext();
        if(next.getPrev().addr != cur.addr) return false;
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
void EEPROMlist::constructList(){
    dataRoot.addr = DATA_TERM;
    dataLast.addr = DATA_TERM;
    freeRoot.addr = EE_LIST_START;
    freeLast.addr = FREE_TERM;

    eeNodePtr here = freeRoot;
    eeNodePtr prev(FREE_TERM);
    for (int i = 0; i < MAX_NODES-1; i++){
        eeNodePtr next = eeNodePtr(here.addr+NODE_SIZE);
        here.setNext(next);
        here.setPrev(prev);
        prev = here;
        here = next;
    }
    freeLast = prev;
    freeLast.setNext(FREE_TERM);
    curSize = 0;
}
void EEPROMlist::pushFree(eeNodePtr node){
    node.setNext(freeRoot);
    node.setPrev(FREE_TERM);
    freeRoot.setPrev(node);
    freeRoot = node;
}
eeNodePtr EEPROMlist::popFree(){
    eeNodePtr freed = freeRoot;
    freeRoot = freeRoot.getNext();
    freeRoot.setPrev(FREE_TERM);
    return freed;
} inline
eeNodePtr EEPROMlist::getNode(uint16_t index){
    //use the previous chain if that is shorter
    if(index > curSize) return false;
    eeNodePtr cur = dataRoot;
    for(int i=0; i<index; i++) cur = cur.getNext();
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

    eeNodePtr curNode = getNode(index-1);
    eeNodePtr newNode = popFree();

    newNode.setNext(curNode.getNext());
    newNode.setPrev(curNode);
    newNode.setData(item);
    curNode.setNext(newNode);
    curSize++;

    return true;
}
bool EEPROMlist::add(EE_LIST_TYPE item){
    return pushTop(item);
}
bool EEPROMlist::pushTop(EE_LIST_TYPE item){
    if(curSize >= MAX_NODES) return false;

    eeNodePtr newNode = popFree();
    newNode.setNext(dataRoot);
    newNode.setPrev(DATA_TERM);
    newNode.setData(item);
    dataRoot.setPrev(newNode);
    dataRoot = newNode;

    if(curSize == 0) dataLast = newNode;
    curSize++;

    return true;
}
bool EEPROMlist::pushBottom(EE_LIST_TYPE item){
    if(curSize >= MAX_NODES) return false;

    eeNodePtr newNode = popFree();
    newNode.setData(item);
    newNode.setPrev(dataLast);
    newNode.setNext(DATA_TERM);
    if(curSize == 0) dataRoot = newNode;
    else dataLast.setNext(newNode);
    dataLast = newNode;
    curSize++;

    return true;
}
bool EEPROMlist::set(uint16_t index, EE_LIST_TYPE item){
    if(index >= curSize || index < 0) return false;
    eeNodePtr node = getNode(index);
    node.setData(item);
    return true;
}
EE_LIST_TYPE EEPROMlist::get(uint16_t index){
    if(index >= curSize || index < 0) return EE_LIST_TYPE();
    return (getNode(index)).getData();
}
EE_LIST_TYPE EEPROMlist::remove(uint16_t index){
    if(curSize <= 0) return EE_LIST_TYPE();
    else if(index >= curSize || index < 0) return EE_LIST_TYPE();
    else if(index == 0) return popTop();
    else if(index == curSize-1) return popBottom();

    eeNodePtr before, del, after;

    before = getNode(index-1);
    del    = before.getNext();
    after  = del.getNext();

    before.setNext(after);
    after.setPrev(before);
    EE_LIST_TYPE data = del.getData();

    curSize--;
    pushFree(del);

    return data;
}
EE_LIST_TYPE EEPROMlist::popTop(){
    if(curSize <= 0) return EE_LIST_TYPE();

    eeNodePtr del = dataRoot;
    dataRoot = dataRoot.getNext();
    dataRoot.setPrev(DATA_TERM);
    EE_LIST_TYPE data = del.getData();
    curSize--;
    pushFree(del);

    return data;
}
EE_LIST_TYPE EEPROMlist::popBottom(){
    if(curSize <= 0) return EE_LIST_TYPE();

    eeNodePtr del = dataLast;
    dataLast = dataLast.getPrev();
    dataLast.setNext(DATA_TERM);
    EE_LIST_TYPE data = del.getData();
    curSize--;
    pushFree(del);

    return data;
}
void EEPROMlist::clear(){
    if(curSize == 0) return;
    eeNodePtr tmp;
    for (int i = 0; i < curSize; i++){
        tmp = dataRoot;
        dataRoot = dataRoot.getNext();
        pushFree(tmp);
    }
    dataRoot = DATA_TERM;
    dataLast = DATA_TERM;
    curSize = 0;
}
#endif
