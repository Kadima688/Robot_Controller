#pragma once
#include <mutex>
#include<iostream>
template <typename T>
struct mListNode
{
    T val;
    mListNode *next;
    mListNode *pre;
    mListNode() : next(nullptr), pre(nullptr) {}
    mListNode(T value) : next(nullptr), pre(nullptr), val(value) {}
};
template <typename T>
inline mListNode<T> *DeleteNode(mListNode<T> *node)
{
    if (node->pre)
        node->pre->next = node->next;
    if (node->next)
        node->next->pre = node->pre;
    return node;
}
template <typename T>
inline BOOL BackInsertNode(mListNode<T> *srcNode, mListNode<T> *Node)
{
    if (srcNode == nullptr || Node == nullptr || srcNode->next == nullptr)
        return FALSE;
    Node->next = srcNode->next;
    Node->pre = srcNode;
    srcNode->next->pre = Node;
    srcNode->next = Node;
    return TRUE;
}
template <typename T>
inline BOOL FrontInsertNode(mListNode<T> *srcNode, mListNode<T> *Node)
{
    if (srcNode == nullptr || Node == nullptr || srcNode->pre == nullptr)
        return FALSE;
    Node->pre = srcNode->pre;
    Node->next = srcNode;
    srcNode->pre->next = Node;
    srcNode->pre = Node;
    return TRUE;
}

class FBIdManager
{
private:
    mListNode<int> *Freehead;
    mListNode<int> *Freetail;
    mListNode<int> *Usedhead;
    mListNode<int> *Usedtail;
    std::mutex UsedMtx;
    std::mutex FreeMtx;

    mListNode<int> ListData[255];
    bool Data[255];

public:
    FBIdManager();
    int GetIndex();
    ~FBIdManager();
    void FreeID(int id);
    bool Judge(int id);
};

inline FBIdManager::FBIdManager()
{
    Freehead = new mListNode<int>(-1);
    Freetail = new mListNode<int>(-1);
    Usedhead = new mListNode<int>(-1);
    Usedtail = new mListNode<int>(-1);
    Usedhead->next = Usedtail;
    Usedtail->pre = Usedhead;

    Freehead->next = Freetail;
    Freetail->pre = Freehead;
    for (int i = 0; i < 255; i++)
    {
        ListData[i].val = i;
        FrontInsertNode(Freetail, &ListData[i]);
    }
    
}
inline int FBIdManager::GetIndex()
{
    FreeMtx.lock();
    if (Freehead->next == Freetail&&Freetail->pre==Freehead)
    {
        FreeMtx.unlock();
        UsedMtx.lock();
        int id = Usedtail->pre->val;
        mListNode<int> *temp = &ListData[id];
        DeleteNode(temp);
        BackInsertNode(Usedhead, temp);
        UsedMtx.unlock();
        std::cout<<"sadfsdafsad"<<std::endl;

        return id;
    }
    else
    {
        int id = Freehead->next->val;
        mListNode<int> *temp = &ListData[id];
        DeleteNode(temp);
        FreeMtx.unlock();
        UsedMtx.lock();
        BackInsertNode(Usedhead, temp);
        UsedMtx.unlock();

        return id;
    }
}
inline bool FBIdManager::Judge(int id)
{
    if (Data[id])
    {
        UsedMtx.lock();
        mListNode<int> *temp = &ListData[id];
        DeleteNode(temp);
        BackInsertNode(Usedhead, temp);
        UsedMtx.unlock();
        return true;
    }
    else
    {
        return false;
    }
}
inline void FBIdManager::FreeID(int id)
{
    if (Data[id])
    {
        UsedMtx.lock();
        mListNode<int> *temp = &ListData[id];
        DeleteNode(temp);
        UsedMtx.unlock();
        FreeMtx.lock();
        FrontInsertNode(Freetail, temp);
        FreeMtx.unlock();

    }
}

inline FBIdManager::~FBIdManager()
{
    if (Freehead)
    {
        delete Freehead;
    }
    if (Freetail)
    {
        delete Freetail;
    }
    if (Usedhead)
    {
        delete Usedhead;
    }
    if (Usedtail)
    {
        delete Usedtail;
    }
}
