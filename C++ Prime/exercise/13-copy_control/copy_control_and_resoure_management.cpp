//
// Created by zxj on 2023/3/20.
//
#include <string>
#include <vector>
#include <new>


//exercise 13.22
class HasPtr{
public:
    HasPtr(const std::string &s=std::string()):
            ps(new std::string(s)),i(10){}
    HasPtr &operator=(const HasPtr& hasPtr){
        ps=hasPtr.ps;
        i=hasPtr.i;
        return *this;
    }


    ~HasPtr(){
        delete ps;
    }
    HasPtr(const HasPtr &hasPtr):
            ps(new std::string(*hasPtr.ps)),i(hasPtr.i)
    {

    }
private:
    std::string *ps;
    int i;
};


//exercise 13.28
class TreeNode{
public:
    TreeNode(){};
    TreeNode(const std::string &s=std::string())
        :value(s),left(nullptr),right(nullptr),count(new int){}
    TreeNode(const TreeNode& t)
        :value(t.value),left(t.left),right(t.right),count(t.count){
        ++*count;
    }
    ~TreeNode();
    TreeNode& operator=(const TreeNode&);

private:
    std::string value;
    int *count;
    TreeNode *left;
    TreeNode *right;
};

TreeNode& TreeNode::operator=(const TreeNode &t) {
    ++*t.count;
    if(--*count==0){
        if(left){
            delete left;
            left= nullptr;
        }
        if(right){
            delete right;
            right= nullptr;
        }
        delete count;
    }
    left=t.left;
    right=t.right;
    count=t.count;
    value=t.value;

    return *this;
}

TreeNode::~TreeNode() {
    if(--*count==0){
        if(left){
            delete left;
            left= nullptr;
        }
        if(right){
            delete right;
            right= nullptr;
        }
        delete count;
    }
}

class BinStrTree{
public:
    BinStrTree():root(nullptr){}
    ~BinStrTree(){
        if (root){
            delete root;
        }
    }
    BinStrTree(const BinStrTree& b):root(b.root){};
    BinStrTree &operator=(const BinStrTree& b){
        auto c=b.root;
        if (root){
            delete root;
        }
        root=c;
        return *this;
    }

private:
    TreeNode *root;
};




