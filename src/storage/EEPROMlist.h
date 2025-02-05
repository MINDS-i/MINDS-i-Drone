#ifndef EEPROMLIST_H
#define EEPROMLIST_H

#include "math/Waypoint.h"
#include "storage/EEPROMconfig.h"
#include "storage/EEPROMsubsystem.h"
#include "util/byteConv.h"

// disable strict aliasing for now, its the only way this code can work
#pragma GCC diagnostic ignored "-Wstrict-aliasing"

namespace EENode {
// node is EEaddr next; EEaddr prev; EE_LIST_TYPE data;
struct eeNodePtr {
  private:
    template <typename T> void write(T newData, EEaddr address) {
        if (address < EE_LIST_START) {
            return;
        }
        if (address >= EE_MAX) {
            return;
        }
        byte data[sizeof(T)];
        *((T*)data) = newData;
        for (size_t i = 0; i < sizeof(T); i++) {
            eeprom::safeWrite(address + i, data[i]);
        }
    }
    template <typename T> T read(EEaddr address) {
        if (address < EE_LIST_START) {
            return T();
        }
        if (address >= EE_MAX) {
            return T();
        }
        byte data[sizeof(T)];
        for (size_t i = 0; i < sizeof(T); i++) {
            data[i] = eeprom::safeRead(address + i);
        }
        return *((T*)data);
    }

  public:
    EEaddr addr;
    eeNodePtr() : addr(0) {}
    eeNodePtr(EEaddr a) : addr(a) {}
    eeNodePtr getNext() { return read<EEaddr>(addr); }
    void setNext(eeNodePtr New) {
        write<EEaddr>(New.addr, addr);
        return;
    }
    eeNodePtr getPrev() { return read<EEaddr>(addr + sizeof(EEaddr)); }
    void setPrev(eeNodePtr New) {
        write<EEaddr>(New.addr, addr + sizeof(EEaddr));
        return;
    }
    EE_LIST_TYPE getData() { return read<EE_LIST_TYPE>(addr + 2 * sizeof(EEaddr)); }
    void setData(EE_LIST_TYPE New) {
        write<EE_LIST_TYPE>(New, addr + 2 * sizeof(EEaddr));
        return;
    }
};
const uint16_t NODE_SIZE = (sizeof(EE_LIST_TYPE) + 2 * sizeof(EEaddr));
const uint16_t MAX_NODES = EE_LIST_LENGTH / NODE_SIZE;
const EEaddr FREE_TERM = 0x0000; // loose end address value of free list
const EEaddr DATA_TERM = 0xffff; // loose end address value of data list
} // namespace EENode
using namespace EENode;
class EEPROMlist : public List<EE_LIST_TYPE> {
    /**
     * doubly linked list in arduino EEprom
     * keeps track of free list and data list in fixed region of memory
     * it has to be ablo to "recover" from reboots, so the loose addresses
     * are a tell of if that node terminates the free list of the data list
     */
  public:
    static EEPROMlist* getInstance() {
        if (m_instance == NULL) {
            m_instance = new EEPROMlist();
        }
        return m_instance;
    }
    uint16_t size();
    uint16_t maxSize();
    bool add(uint16_t index, EE_LIST_TYPE item);
    bool add(EE_LIST_TYPE item);
    bool pushTop(EE_LIST_TYPE item);
    bool pushBottom(EE_LIST_TYPE item);
    bool set(uint16_t index, EE_LIST_TYPE item);
    EE_LIST_TYPE get(uint16_t index);
    EE_LIST_TYPE remove(uint16_t index);
    EE_LIST_TYPE popTop();
    EE_LIST_TYPE popBottom();
    void clear();

  private:
    uint16_t curSize;
    eeNodePtr dataRoot, dataLast;
    eeNodePtr freeRoot, freeLast;
    static EEPROMlist* m_instance;
    EEPROMlist();
    EEPROMlist(const EEPROMlist&);
    bool readList();
    void constructList();
    void pushFree(eeNodePtr eeNodePtr);
    eeNodePtr popFree();
    eeNodePtr getNode(uint16_t index);
    friend void runEEListTest();
};
EEPROMlist* EEPROMlist::m_instance = NULL;
EEPROMlist::EEPROMlist() : curSize(0) {
    bool foundList = readList();
    if (!foundList) {
        constructList();
    }
}
/**
 * try and deduce the roots af lasts of free and data lists
 * return true if every node is accounted for
 * return false if a valid list could not be salvaged
 */

bool EEPROMlist::readList() {
    dataRoot.addr = 0;
    dataLast.addr = 0;
    freeRoot.addr = 0;
    freeLast.addr = 0;

    // search for end nodes
    for (uint16_t i = 0; i < MAX_NODES; i++) {
        eeNodePtr node(EE_LIST_START + i * NODE_SIZE);
        switch (node.getNext().addr) {
        case FREE_TERM:
            if (freeLast.addr != 0) {
                FAIL("Repeat freeLast Node");
            }
            freeLast = node;
            break;
        case DATA_TERM:
            if (dataLast.addr != 0) {
                FAIL("Repeat dataLast Node");
            }
            dataLast = node;
            break;
        }
        switch (node.getPrev().addr) {
        case FREE_TERM:
            if (freeRoot.addr != 0) {
                FAIL("Repeat freeRoot Node");
            }
            freeRoot = node;
            break;
        case DATA_TERM:
            if (dataRoot.addr != 0) {
                FAIL("Repeat dataRoot Node");
            }
            dataRoot = node;
            break;
        }
    }

    // check data end nodes
    if (dataRoot.addr == 0 && dataLast.addr == 0) {
        // data list was empty
        dataRoot.addr = DATA_TERM;
        dataLast.addr = DATA_TERM;
    } else if (dataRoot.addr == 0 || dataLast.addr == 0) {
        // only one endpoint found; bad list
        FAIL("Odd number of data Endpoints");
    }

    // check free end nodes
    if (freeRoot.addr == 0 && freeLast.addr == 0) {
        // free list was empty
        freeRoot.addr = FREE_TERM;
        freeLast.addr = FREE_TERM;
    } else if (freeRoot.addr == 0 || freeLast.addr == 0) {
        // only one endpoint found; bad list
        FAIL("Odd number of free Endpoints");
    }

    // traverse both lists to see if the addresses are valid
    uint16_t found = 0;

    eeNodePtr cur = freeRoot;
    if (freeRoot.addr != FREE_TERM) {
        found++; // first node not counted below
    }
    while (cur.addr != freeLast.addr) {
        eeNodePtr next = cur.getNext();
        if (next.getPrev().addr != cur.addr) {
            FAIL("Broken freelist Chain");
        }
        cur = next;
        found++;
        if (found > MAX_NODES) {
            FAIL("Chain too long (free)");
        }
    }

    cur = dataRoot;
    if (dataRoot.addr != DATA_TERM) {
        found++; // first node not counted below
        curSize++;
    }
    while (cur.addr != dataLast.addr) {
        eeNodePtr next = cur.getNext();
        if (next.getPrev().addr != cur.addr) {
            FAIL("Broken dataList Chain");
        }
        cur = next;
        found++;
        curSize++;
        if (found > MAX_NODES) {
            FAIL("Chain too long (data)");
        }
    }

    if (found < MAX_NODES) {
        FAIL("Chain too short");
    }

    return true;
}
/**
 * Construct a brand new list, 100% free
 * This will necessarily obliderate all existing data
 */
void EEPROMlist::constructList() {
    dataRoot.addr = DATA_TERM;
    dataLast.addr = DATA_TERM;
    freeRoot.addr = EE_LIST_START;
    freeLast.addr = FREE_TERM;

    eeNodePtr here = freeRoot;
    eeNodePtr prev(FREE_TERM);
    for (uint16_t i = 0; i < MAX_NODES; i++) {
        eeNodePtr next = eeNodePtr(here.addr + NODE_SIZE);
        here.setNext(next);
        here.setPrev(prev);
        prev = here;
        here = next;
    }
    freeLast = prev;
    freeLast.setNext(FREE_TERM);
    curSize = 0;
}
void EEPROMlist::pushFree(eeNodePtr node) {
    node.setNext(freeRoot);
    node.setPrev(FREE_TERM);
    if (freeRoot.addr != FREE_TERM) {
        freeRoot.setPrev(node);
    }

    freeRoot = node;
    if (freeLast.addr == FREE_TERM) {
        freeLast = node;
    }
}
eeNodePtr EEPROMlist::popFree() {
    if (freeLast.addr == FREE_TERM) {
        return eeNodePtr(EENULL);
    }

    eeNodePtr freed = freeLast;
    if (freeLast.addr == freeRoot.addr) {
        freeLast.addr = FREE_TERM;
        freeRoot.addr = FREE_TERM;
    } else {
        freeLast = freeLast.getPrev();
        freeLast.setNext(FREE_TERM);
    }
    return freed;
}
inline eeNodePtr EEPROMlist::getNode(uint16_t index) {
    if (index > curSize) {
        return eeNodePtr(EENULL);
    }

    eeNodePtr cur;
    if (index <= curSize / 2) {
        cur = dataRoot;
        for (uint16_t i = 0; i < index; i++) {
            cur = cur.getNext();
        }
    } else {
        cur = dataLast;
        for (uint16_t i = 0; i < (curSize - index - 1); i++) {
            cur = cur.getPrev();
        }
    }

    return cur;
}
//-- public from here on --//
uint16_t EEPROMlist::size() { return curSize; }
uint16_t EEPROMlist::maxSize() { return MAX_NODES; }
bool EEPROMlist::add(uint16_t index, EE_LIST_TYPE item) {
    if (curSize >= MAX_NODES) {
        return false;
    }

    if (index == 0) {
        return pushTop(item);
    } else if (index == curSize) {
        return pushBottom(item);
    } else if (index > curSize) {
        return false;
    }

    eeNodePtr preNode = getNode(index - 1);
    eeNodePtr postNode = preNode.getNext();
    eeNodePtr newNode = popFree();

    newNode.setNext(postNode);
    newNode.setPrev(preNode);
    newNode.setData(item);

    preNode.setNext(newNode);
    postNode.setPrev(newNode);

    curSize++;
    return true;
}
bool EEPROMlist::add(EE_LIST_TYPE item) { return pushTop(item); }
bool EEPROMlist::set(uint16_t index, EE_LIST_TYPE item) {
    if (index >= curSize) {
        return false;
    }
    eeNodePtr node = getNode(index);
    node.setData(item);
    return true;
}
EE_LIST_TYPE EEPROMlist::get(uint16_t index) {
    if (index >= curSize) {
        return EE_LIST_TYPE();
    }
    return (getNode(index)).getData();
}
EE_LIST_TYPE EEPROMlist::remove(uint16_t index) {
    if (curSize <= 0) {
        return EE_LIST_TYPE();
    } else if (index >= curSize) {
        return EE_LIST_TYPE();
    } else if (index == 0) {
        return popTop();
    } else if (index == curSize - 1) {
        return popBottom();
    }

    eeNodePtr before, del, after;

    before = getNode(index - 1);
    del = before.getNext();
    after = del.getNext();

    before.setNext(after);
    after.setPrev(before);
    EE_LIST_TYPE data = del.getData();

    curSize--;
    pushFree(del);

    return data;
}
bool EEPROMlist::pushTop(EE_LIST_TYPE item) {
    if (curSize >= MAX_NODES) {
        return false;
    }

    eeNodePtr newNode = popFree();
    newNode.setNext(dataRoot);
    newNode.setPrev(DATA_TERM);
    newNode.setData(item);
    if (dataRoot.addr != DATA_TERM) {
        dataRoot.setPrev(newNode);
    }
    dataRoot = newNode;

    if (curSize == 0) {
        dataLast = dataRoot;
    }
    curSize++;

    return true;
}
bool EEPROMlist::pushBottom(EE_LIST_TYPE item) {
    if (curSize >= MAX_NODES) {
        return false;
    }

    eeNodePtr newNode = popFree();
    newNode.setData(item);
    newNode.setPrev(dataLast);
    newNode.setNext(DATA_TERM);
    if (curSize == 0) {
        dataRoot = newNode;
    } else {
        dataLast.setNext(newNode);
    }
    dataLast = newNode;
    curSize++;

    return true;
}
EE_LIST_TYPE EEPROMlist::popTop() {
    if (curSize <= 0) {
        return EE_LIST_TYPE();
    }

    eeNodePtr del = dataRoot;
    if (curSize > 1) {
        dataRoot = dataRoot.getNext();
        dataRoot.setPrev(DATA_TERM);
    } else {
        dataRoot.addr = DATA_TERM;
        dataLast.addr = DATA_TERM;
    }
    EE_LIST_TYPE data = del.getData();
    curSize--;
    pushFree(del);

    return data;
}
EE_LIST_TYPE EEPROMlist::popBottom() {
    if (curSize <= 0) {
        return EE_LIST_TYPE();
    }

    eeNodePtr del = dataLast;
    if (curSize != 1) { // if this is not the last element
        dataLast = dataLast.getPrev();
        dataLast.setNext(DATA_TERM);
    } else {
        dataRoot.addr = DATA_TERM;
        dataLast.addr = DATA_TERM;
    }
    EE_LIST_TYPE data = del.getData();
    curSize--;
    pushFree(del);

    return data;
}
void EEPROMlist::clear() {
    if (curSize == 0) {
        return;
    }
    eeNodePtr tmp;
    for (uint16_t i = 0; i < curSize; i++) {
        tmp = dataRoot;
        dataRoot = dataRoot.getNext();
        pushFree(tmp);
    }
    dataRoot = DATA_TERM;
    dataLast = DATA_TERM;
    curSize = 0;
}
#if DEBUG
void runEEListTest() {
    EEPROMlist* list = (EEPROMlist::getInstance());

    TEST(EE_LIST_START);
    TEST(EE_LIST_LENGTH);
    TEST(NODE_SIZE);
    TEST(MAX_NODES);
    Serial.print("\n");
    TEST(list->curSize);
    Serial.print("\n");

    eeNodePtr here = eeNodePtr(EE_LIST_START);
    for (uint16_t i = 0; i < MAX_NODES; i++) {
        uint16_t This = here.addr;
        uint16_t Next = here.getNext().addr;
        uint16_t Prev = here.getPrev().addr;
        TEST(This);
        TEST(Next);
        TEST(Prev);
        if (This == list->freeLast.addr) {
            Serial.print(" <- Free End");
        }
        if (This == list->freeRoot.addr) {
            Serial.print(" <- Free Start");
        }
        if (This == list->dataLast.addr) {
            Serial.print(" <- Data End");
        }
        if (This == list->dataRoot.addr) {
            Serial.print(" <- Data Start");
        }
        Serial.print("\n");
        here = eeNodePtr(here.addr + NODE_SIZE);
    }
}
#endif
#pragma GCC diagnostic pop
#endif
