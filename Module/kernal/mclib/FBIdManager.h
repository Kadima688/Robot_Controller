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
