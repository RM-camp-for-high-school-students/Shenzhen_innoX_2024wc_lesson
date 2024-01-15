#include "Block_Base.h"

Block_Base::Block_Base(shared_ptr<Game> _game, int _row, int _col)
{
    game = _game;
    row = _row;
    col = _col;
}

bool Block_Base::move(Dir dir)
{
    if (!movable(dir)) return false;
    switch (dir)
    {
    case UP:
        row--;
        break;
    case LEFT:
        col--;
        break;
    case DOWN:
        row++;
        break;
    case RIGHT:
        col++;
        break;
    default:
        break;
    }
    game->moved++;
    game->calcState();
    return true;
}