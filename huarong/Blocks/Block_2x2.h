#include "Block_Base.h"

class Block_2x2 : public Block_Base
{
public:
    Block_2x2(shared_ptr<Game> _game, int _row, int _col) : Block_Base(_game, _row, _col){};
    void fill();
    bool movable(Dir dir);
    shared_ptr<Block_Base> copy(shared_ptr<Game> _game);
    int estimate();
};