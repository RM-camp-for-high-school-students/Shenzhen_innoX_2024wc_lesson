#pragma once
#include <vector>
#include <memory>
#include "Dir.h"
#include "Blocks/Block_Base.h"

class Block_Base;

using std::vector;
using std::shared_ptr;
using std::string;
using std::make_shared;

class Game
{
public:
    vector<shared_ptr<Block_Base>> blocks;
    int row, col, moved, lastIndex;
    char** map;
    Game(int _row, int _col, int _moved);
    void clearMap();
    void calcState();
    void printState();
    string toString();
    shared_ptr<Game> copy();
    int estimate();
};