#include "Block_1x1.h"

void Block_1x1::fill()
{
    game->map[row][col] = '1';
}

bool Block_1x1::movable(Dir dir)
{
    switch (dir)
    {
    case UP:
        if (row == 0) return false;
        return game->map[row - 1][col] == '0';
    case DOWN:
        if (row == game->row - 1) return false;
        return game->map[row + 1][col] == '0';
    case LEFT:
        if (col == 0) return false;
        return game->map[row][col - 1] == '0';
    case RIGHT:
        if (col == game->col - 1) return false;
        return game->map[row][col + 1] == '0';
    default:
        return false;
    }
}

shared_ptr<Block_Base> Block_1x1::copy(shared_ptr<Game> _game)
{
    Block_1x1 t(_game, row, col);
    return make_shared<Block_1x1>(t);
}

int Block_1x1::estimate()
{
    return 0;
}