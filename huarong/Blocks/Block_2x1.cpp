#include "Block_2x1.h"

void Block_2x1::fill()
{
    game->map[row][col] = '4';
    game->map[row + 1][col] = '4';
}

bool Block_2x1::movable(Dir dir)
{
    switch (dir)
    {
    case UP:
        if (row == 0) return false;
        return game->map[row - 1][col] == '0';
    case DOWN:
        if (row == game->row - 2) return false;
        return game->map[row + 2][col] == '0';
    case LEFT:
        if (col == 0) return false;
        return game->map[row][col - 1] == '0' && game->map[row + 1][col - 1] == '0';
    case RIGHT:
        if (col == game->col - 1) return false;
        return game->map[row][col + 1] == '0' && game->map[row + 1][col + 1] == '0';
    default:
        return false;
    }
}

shared_ptr<Block_Base> Block_2x1::copy(shared_ptr<Game> _game)
{
    Block_2x1 t(_game, row, col);
    return make_shared<Block_2x1>(t);
}

int Block_2x1::estimate()
{
    return 0;
}