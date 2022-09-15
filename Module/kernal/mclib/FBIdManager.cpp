#include"FBIdManager.h"

FBIdManager::FBIdManager()
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
int FBIdManager::GetIndex()
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
bool FBIdManager::Judge(int id)
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
void FBIdManager::FreeID(int id)
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

FBIdManager::~FBIdManager()
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
