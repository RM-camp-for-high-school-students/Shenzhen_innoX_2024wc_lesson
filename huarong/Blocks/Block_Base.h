#pragma once
#include "../Game.h"

class Game;

using std::vector;
using std::shared_ptr;
using std::make_shared;

class Block_Base
{
protected:
    shared_ptr<Game> game;
    int row, col;
public:
    Block_Base(shared_ptr<Game> _game, int _row, int _col);
    bool move(Dir dir);
    virtual void fill() = 0;
    virtual bool movable(Dir dir) = 0;
    virtual shared_ptr<Block_Base> copy(shared_ptr<Game> _game) = 0;
    virtual int estimate() = 0;
};