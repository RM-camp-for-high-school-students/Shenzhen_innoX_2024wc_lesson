#include "Block_1x2.h"

void Block_1x2::fill()
{
    game->map[row][col] = '3';
    game->map[row][col + 1] = '3';
}

bool Block_1x2::movable(Dir dir)
{
    switch (dir)
    {
    case UP:
        if (row == 0) return false;
        return game->map[row - 1][col] == '0' && game->map[row - 1][col + 1] == '0';
    case DOWN:
        if (row == game->row - 1) return false;
        return game->map[row + 1][col] == '0' && game->map[row + 1][col + 1] == '0';
    case LEFT:
        if (col == 0) return false;
        return game->map[row][col - 1] == '0';
    case RIGHT:
        if (col == game->col - 2) return false;
        return game->map[row][col + 2] == '0';
    default:
        return false;
    }
}

shared_ptr<Block_Base> Block_1x2::copy(shared_ptr<Game> _game)
{
    Block_1x2 t(_game, row, col);
    return make_shared<Block_1x2>(t);
}

int Block_1x2::estimate()
{
    return 0;
}